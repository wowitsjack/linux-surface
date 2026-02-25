// SPDX-License-Identifier: GPL-2.0
/*
 * surface_s2idle_fix.c - Fix s2idle death sleep on Surface Laptop 5
 *
 * Intel INTC1055 pinctrl power-gating during s2idle corrupts pin 213's
 * PADCFG0 RXINV bit. The phantom edge fires a spurious SCI on GPE 0x52,
 * and the ACPI handler calls pm_system_cancel_wakeup(), permanently
 * poisoning the wakeup framework. The system never wakes from s2idle.
 *
 * Fix:
 *  - Mask GPE 0x52 during s2idle, unmask on resume
 *  - ACPI wakeup handler + resume_early catch and restore RXINV corruption
 *  - Passive KEY_POWER observer distinguishes real vs spurious wakes
 *  - Post-resume failsafe re-suspends on spurious wake (max 10 retries)
 *  - RXSTATE polling re-suspends after power-button-wake-with-lid-closed
 *    (GPIO settles slowly, 5-30s, logind won't re-trigger)
 *  - Background RXSTATE polling catches lid close events that the broken
 *    GPIO edge path misses after s2idle cycles
 *
 * Work functions that call pm_suspend() set failsafe_in_progress to
 * prevent PM_SUSPEND_PREPARE from deadlocking on cancel_delayed_work_sync().
 */

#define pr_fmt(fmt) "surface_s2idle_fix: " fmt

#include <linux/module.h>
#include <linux/acpi.h>
#include <linux/io.h>
#include <linux/suspend.h>
#include <linux/dmi.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/platform_device.h>

/* Pin 213, INTC1055 community */
#define LID_PADCFG0_PHYS       0xfd6a09a0
#define LID_PADCFG1_PHYS       0xfd6a09a4
#define PADCFG_MAP_SIZE        8

#define PADCFG0_RXINV          BIT(23)
#define PADCFG0_GPIROUTSCI     BIT(19)
#define PADCFG0_GPIORXDIS      BIT(8)
#define PADCFG0_GPIORXSTATE    BIT(1)

#define LID_GPE                0x52

#define LID_FAILSAFE_DELAY_MS   2000
#define FAILSAFE_MAX_RETRIES    10
#define LID_RESYNC_INTERVAL_MS  1000
#define LID_RESYNC_MAX_POLLS    120   /* 1s * 120 = 2 minutes */
#define LID_POLL_INTERVAL_MS    2000

static void __iomem *lid_padcfg_base;
static u32 saved_padcfg0;
static u32 saved_padcfg1;
static bool padcfg_saved;
static bool gpe52_was_enabled;
static int sci_irq;

static struct delayed_work lid_failsafe_work;
static struct delayed_work lid_resync_work;
static struct delayed_work lid_poll_work;
static bool lid_was_closed_at_suspend;
static bool failsafe_in_progress;	/* guards cancel_delayed_work_sync */
static bool lid_poll_active;
static int last_poll_rxstate;
static unsigned int failsafe_suspends;
static unsigned int resync_polls_remaining;

/* Stats */
static unsigned int suspend_cycles;
static unsigned int padcfg_restores;
static unsigned int wakeup_handler_calls;
static unsigned int early_restores;
static unsigned int lid_resyncs;
static unsigned int lid_close_suspends;
static unsigned int gpio_recovery_polls;

/* Power button input handler (passive observer for KEY_POWER) */
static atomic_t power_button_seen = ATOMIC_INIT(0);

static int pwr_connect(struct input_handler *handler, struct input_dev *dev,
		       const struct input_device_id *id)
{
	struct input_handle *handle;
	int err;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "surface_s2idle_pwr";

	err = input_register_handle(handle);
	if (err)
		goto err_free;

	err = input_open_device(handle);
	if (err)
		goto err_unregister;

	pr_info("input connected: %s\n", dev->name);
	return 0;

err_unregister:
	input_unregister_handle(handle);
err_free:
	kfree(handle);
	return err;
}

static void pwr_disconnect(struct input_handle *handle)
{
	pr_info("input disconnected: %s\n", handle->dev->name);
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static void pwr_event(struct input_handle *handle,
		      unsigned int type, unsigned int code, int value)
{
	if (type == EV_KEY && code == KEY_POWER && value == 1) {
		atomic_set(&power_button_seen, 1);
		pr_info("power button press detected\n");
	}
}

static const struct input_device_id pwr_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			 INPUT_DEVICE_ID_MATCH_KEYBIT,
		.evbit  = { [BIT_WORD(EV_KEY)] = BIT_MASK(EV_KEY) },
		.keybit = { [BIT_WORD(KEY_POWER)] = BIT_MASK(KEY_POWER) },
	},
	{ },
};

MODULE_DEVICE_TABLE(input, pwr_ids);

static struct input_handler pwr_handler = {
	.event      = pwr_event,
	.connect    = pwr_connect,
	.disconnect = pwr_disconnect,
	.name       = "surface_s2idle_pwr",
	.id_table   = pwr_ids,
	.passive_observer = true,
};

/* ACPI wakeup handler: check/fix PADCFG corruption on every SCI */
static bool lid_wake_handler(void *context)
{
	u32 current_padcfg0;

	wakeup_handler_calls++;

	if (!lid_padcfg_base || !padcfg_saved)
		return false;

	current_padcfg0 = readl(lid_padcfg_base);

	if ((current_padcfg0 & PADCFG0_RXINV) !=
	    (saved_padcfg0 & PADCFG0_RXINV)) {
		writel(saved_padcfg1, lid_padcfg_base + 4);
		wmb();
		writel(saved_padcfg0, lid_padcfg_base);
		wmb();

		acpi_clear_gpe(NULL, LID_GPE);

		padcfg_restores++;
		pr_info("PADCFG0 RXINV corrupted: was 0x%08x, restored "
			"0x%08x (cycle #%u)\n",
			current_padcfg0, saved_padcfg0, suspend_cycles);
	}

	return false;
}

/*
 * resume_early: double-check PADCFG after surface_gpe's resume_noirq,
 * before the ACPI button driver reads _LID in normal resume.
 */
static int s2idle_fix_resume_early(struct device *dev)
{
	u32 padcfg0;

	if (!lid_padcfg_base || !padcfg_saved)
		return 0;

	padcfg0 = readl(lid_padcfg_base);

	if ((padcfg0 & PADCFG0_RXINV) != (saved_padcfg0 & PADCFG0_RXINV)) {
		writel(saved_padcfg1, lid_padcfg_base + 4);
		wmb();
		writel(saved_padcfg0, lid_padcfg_base);
		wmb();

		acpi_clear_gpe(NULL, LID_GPE);

		early_restores++;
		pr_info("early resume: PADCFG0 RXINV corrected "
			"0x%08x -> 0x%08x\n", padcfg0, saved_padcfg0);
	} else {
		pr_info("early resume: PADCFG0 OK (0x%08x)\n", padcfg0);
	}

	/* Unmask before ACPI button driver reads _LID */
	if (gpe52_was_enabled) {
		acpi_mask_gpe(NULL, LID_GPE, FALSE);
		acpi_set_gpe(NULL, LID_GPE, ACPI_GPE_ENABLE);
		gpe52_was_enabled = false;
		pr_info("early resume: unmasked GPE 0x52\n");
	}

	return 0;
}

static const struct dev_pm_ops s2idle_fix_pm_ops = {
	.resume_early = s2idle_fix_resume_early,
};

static struct platform_driver s2idle_fix_plat_driver = {
	.driver = {
		.name = "surface_s2idle_fix",
		.pm = &s2idle_fix_pm_ops,
	},
};

static struct platform_device *s2idle_fix_pdev;

/*
 * Background RXSTATE polling: GPIO edge events break after s2idle cycles,
 * so poll the register directly and call pm_suspend() on lid close.
 */
static void lid_poll_fn(struct work_struct *work)
{
	u32 padcfg0;
	int rxstate;

	if (!lid_padcfg_base || !lid_poll_active)
		return;

	padcfg0 = readl(lid_padcfg_base);
	rxstate = !!(padcfg0 & PADCFG0_GPIORXSTATE);

	if (rxstate != last_poll_rxstate) {
		pr_info("lid poll: RXSTATE %d->%d (PADCFG0=0x%08x)\n",
			last_poll_rxstate, rxstate, padcfg0);

		if (rxstate == 1 && last_poll_rxstate == 0) {
			lid_close_suspends++;
			last_poll_rxstate = rxstate;
			pr_info("lid poll: lid closed detected, "
				"suspending directly\n");
			lid_was_closed_at_suspend = true;
			failsafe_in_progress = true;
			pm_suspend(PM_SUSPEND_TO_IDLE);
			failsafe_in_progress = false;
			return;
		}

		last_poll_rxstate = rxstate;
	}

	if (lid_poll_active)
		schedule_delayed_work(&lid_poll_work,
			msecs_to_jiffies(LID_POLL_INTERVAL_MS));
}

/*
 * Lid resync: after power-button wake with lid closed, poll RXSTATE
 * until the GPIO settles, then re-suspend if still closed.
 */
static void lid_resync_fn(struct work_struct *work)
{
	u32 padcfg0;
	bool rxstate;

	if (!lid_padcfg_base)
		return;

	padcfg0 = readl(lid_padcfg_base);
	rxstate = !!(padcfg0 & PADCFG0_GPIORXSTATE);
	gpio_recovery_polls++;

	if (rxstate) {
		lid_resyncs++;
		pr_info("lid resync: RXSTATE=1 (closed) after %u polls "
			"(%us), re-suspending\n",
			gpio_recovery_polls,
			(gpio_recovery_polls * LID_RESYNC_INTERVAL_MS) / 1000);

		failsafe_in_progress = true;
		pm_suspend(PM_SUSPEND_TO_IDLE);
		failsafe_in_progress = false;
		return;
	}

	if (resync_polls_remaining > 0) {
		resync_polls_remaining--;
		if ((gpio_recovery_polls % 10) == 0)
			pr_info("lid resync: still RXSTATE=0 after %u polls "
				"(%us)\n",
				gpio_recovery_polls,
				(gpio_recovery_polls *
				 LID_RESYNC_INTERVAL_MS) / 1000);

		schedule_delayed_work(&lid_resync_work,
				      msecs_to_jiffies(LID_RESYNC_INTERVAL_MS));
	} else {
		pr_info("lid resync: GPIO never settled to closed after "
			"%us, assuming lid is open\n",
			(gpio_recovery_polls *
			 LID_RESYNC_INTERVAL_MS) / 1000);
	}
}

/* Post-resume failsafe: 2s after wake, decide stay-awake vs re-suspend */
static void lid_failsafe_fn(struct work_struct *work)
{
	if (atomic_read(&power_button_seen)) {
		atomic_set(&power_button_seen, 0);
		failsafe_suspends = 0;

		if (lid_was_closed_at_suspend) {
			pr_info("failsafe: power button wake, lid was "
				"closed, starting resync\n");
			gpio_recovery_polls = 0;
			resync_polls_remaining = LID_RESYNC_MAX_POLLS;
			schedule_delayed_work(&lid_resync_work,
				msecs_to_jiffies(LID_RESYNC_INTERVAL_MS));
		} else {
			pr_info("failsafe: power button wake, lid was "
				"open, staying awake\n");
			lid_was_closed_at_suspend = false;
		}
		return;
	}

	if (!lid_was_closed_at_suspend) {
		pr_info("failsafe: lid was open at suspend, "
			"staying awake\n");
		failsafe_suspends = 0;
		lid_was_closed_at_suspend = false;
		return;
	}

	if (failsafe_suspends >= FAILSAFE_MAX_RETRIES) {
		pr_warn("failsafe: max retries (%u) reached, giving up\n",
			FAILSAFE_MAX_RETRIES);
		failsafe_suspends = 0;
		lid_was_closed_at_suspend = false;
		return;
	}

	pr_info("failsafe: spurious wake, lid closed, "
		"forcing s2idle (%u/%u)\n",
		failsafe_suspends + 1, FAILSAFE_MAX_RETRIES);
	failsafe_suspends++;

	failsafe_in_progress = true;
	pm_suspend(PM_SUSPEND_TO_IDLE);
	failsafe_in_progress = false;
}

/* PM notifier: GPE masking + PADCFG save/restore */
static int s2idle_pm_notify(struct notifier_block *nb,
			    unsigned long action, void *data)
{
	u32 padcfg0;

	if (!lid_padcfg_base)
		return NOTIFY_DONE;

	switch (action) {
	case PM_SUSPEND_PREPARE:
		/* Skip cancels when we're the caller, avoids workqueue deadlock */
		if (!failsafe_in_progress) {
			lid_poll_active = false;
			cancel_delayed_work_sync(&lid_poll_work);
			cancel_delayed_work_sync(&lid_resync_work);
			cancel_delayed_work_sync(&lid_failsafe_work);
			failsafe_suspends = 0;
		}

		atomic_set(&power_button_seen, 0);

		/* Save PADCFG, mask out volatile GPIORXSTATE */
		padcfg0 = readl(lid_padcfg_base);
		saved_padcfg0 = padcfg0 & ~PADCFG0_GPIORXSTATE;
		saved_padcfg1 = readl(lid_padcfg_base + 4);
		padcfg_saved = true;
		suspend_cycles++;

		/* Don't clobber lid state on our own re-suspends */
		if (!failsafe_in_progress && !lid_was_closed_at_suspend)
			lid_was_closed_at_suspend =
				!!(padcfg0 & PADCFG0_GPIORXSTATE);

		/* Mask GPE 0x52 for the duration of s2idle */
		acpi_mask_gpe(NULL, LID_GPE, TRUE);
		gpe52_was_enabled = true;

		pr_info("suspend #%u: masked GPE 0x52, PADCFG0=0x%08x "
			"RXINV=%d RXSTATE=%d lid_closed=%d "
			"failsafe_in_progress=%d\n",
			suspend_cycles, padcfg0,
			!!(padcfg0 & PADCFG0_RXINV),
			!!(padcfg0 & PADCFG0_GPIORXSTATE),
			lid_was_closed_at_suspend,
			failsafe_in_progress);
		break;

	case PM_POST_SUSPEND:
		/* Final PADCFG safety check (resume_early already ran) */
		if (padcfg_saved) {
			padcfg0 = readl(lid_padcfg_base);
			if ((padcfg0 & PADCFG0_RXINV) !=
			    (saved_padcfg0 & PADCFG0_RXINV)) {
				writel(saved_padcfg1, lid_padcfg_base + 4);
				wmb();
				writel(saved_padcfg0, lid_padcfg_base);
				wmb();
				acpi_clear_gpe(NULL, LID_GPE);
				padcfg_restores++;
				pr_info("post-suspend: PADCFG0 STILL "
					"corrupted, restored "
					"(was 0x%08x, wrote 0x%08x)\n",
					padcfg0, saved_padcfg0);
			}
		}

		/* Unmask fallback if resume_early missed it */
		if (gpe52_was_enabled) {
			acpi_mask_gpe(NULL, LID_GPE, FALSE);
			acpi_set_gpe(NULL, LID_GPE, ACPI_GPE_ENABLE);
			gpe52_was_enabled = false;
			pr_info("post-suspend: GPE 0x52 unmask (fallback)\n");
		}

		/* Failsafe check in 2s */
		schedule_delayed_work(&lid_failsafe_work,
				      msecs_to_jiffies(LID_FAILSAFE_DELAY_MS));

		/* Restart RXSTATE polling */
		padcfg0 = readl(lid_padcfg_base);
		last_poll_rxstate = !!(padcfg0 & PADCFG0_GPIORXSTATE);
		lid_poll_active = true;
		schedule_delayed_work(&lid_poll_work,
				      msecs_to_jiffies(LID_POLL_INTERVAL_MS));

		padcfg0 = readl(lid_padcfg_base);
		pr_info("resume #%u: PADCFG0=0x%08x RXSTATE=%d "
			"lid_closed=%d handler_calls=%u restores=%u "
			"early_restores=%u\n",
			suspend_cycles, padcfg0,
			!!(padcfg0 & PADCFG0_GPIORXSTATE),
			lid_was_closed_at_suspend,
			wakeup_handler_calls, padcfg_restores,
			early_restores);

		padcfg_saved = false;
		wakeup_handler_calls = 0;
		break;
	}

	return NOTIFY_DONE;
}

static struct notifier_block s2idle_pm_nb = {
	.notifier_call = s2idle_pm_notify,
	.priority = INT_MAX,
};

/* DMI matching */
static const struct dmi_system_id surface_ids[] = {
	{
		.ident = "Surface Laptop 5",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "Microsoft Corporation"),
			DMI_MATCH(DMI_PRODUCT_NAME, "Surface Laptop 5"),
		},
	},
	{ }
};

/* Init / Exit */
static int __init surface_s2idle_fix_init(void)
{
	u32 initial_padcfg0;
	int err;

	if (!dmi_check_system(surface_ids)) {
		pr_err("not a supported Surface device, aborting\n");
		return -ENODEV;
	}

	lid_padcfg_base = ioremap(LID_PADCFG0_PHYS, PADCFG_MAP_SIZE);
	if (!lid_padcfg_base) {
		pr_err("failed to ioremap PADCFG at 0x%08x\n",
		       LID_PADCFG0_PHYS);
		return -ENOMEM;
	}

	initial_padcfg0 = readl(lid_padcfg_base);

	if (!(initial_padcfg0 & PADCFG0_GPIROUTSCI)) {
		pr_err("PADCFG0=0x%08x: GPIROUTSCI not set, wrong pin?\n",
		       initial_padcfg0);
		iounmap(lid_padcfg_base);
		lid_padcfg_base = NULL;
		return -ENODEV;
	}

	sci_irq = acpi_gbl_FADT.sci_interrupt;

	/* Platform driver for resume_early */
	err = platform_driver_register(&s2idle_fix_plat_driver);
	if (err) {
		pr_err("platform_driver_register failed: %d\n", err);
		iounmap(lid_padcfg_base);
		lid_padcfg_base = NULL;
		return err;
	}

	s2idle_fix_pdev = platform_device_register_simple(
		"surface_s2idle_fix", -1, NULL, 0);
	if (IS_ERR(s2idle_fix_pdev)) {
		err = PTR_ERR(s2idle_fix_pdev);
		pr_err("platform_device_register_simple failed: %d\n", err);
		platform_driver_unregister(&s2idle_fix_plat_driver);
		iounmap(lid_padcfg_base);
		lid_padcfg_base = NULL;
		return err;
	}

	INIT_DELAYED_WORK(&lid_failsafe_work, lid_failsafe_fn);
	INIT_DELAYED_WORK(&lid_resync_work, lid_resync_fn);
	INIT_DELAYED_WORK(&lid_poll_work, lid_poll_fn);

	err = input_register_handler(&pwr_handler);
	if (err)
		pr_warn("power button handler failed: %d "
			"(failsafe will use max-retry fallback)\n", err);

	acpi_register_wakeup_handler(sci_irq, lid_wake_handler, NULL);
	register_pm_notifier(&s2idle_pm_nb);

	/* Start RXSTATE polling */
	last_poll_rxstate = !!(initial_padcfg0 & PADCFG0_GPIORXSTATE);
	lid_poll_active = true;
	schedule_delayed_work(&lid_poll_work,
			      msecs_to_jiffies(LID_POLL_INTERVAL_MS));

	pr_info("loaded: SCI=%d PADCFG0=0x%08x RXINV=%d RXSTATE=%d "
		"pwr_handler=%s resume_early=YES\n",
		sci_irq, initial_padcfg0,
		!!(initial_padcfg0 & PADCFG0_RXINV),
		!!(initial_padcfg0 & PADCFG0_GPIORXSTATE),
		err ? "FAILED" : "OK");
	pr_info("  GPE 0x52 masked during s2idle, failsafe %ums, "
		"resync %ums interval (max %u polls), "
		"lid poll %ums\n",
		LID_FAILSAFE_DELAY_MS,
		LID_RESYNC_INTERVAL_MS, LID_RESYNC_MAX_POLLS,
		LID_POLL_INTERVAL_MS);

	return 0;
}

static void __exit surface_s2idle_fix_exit(void)
{
	lid_poll_active = false;
	cancel_delayed_work_sync(&lid_poll_work);
	cancel_delayed_work_sync(&lid_resync_work);
	cancel_delayed_work_sync(&lid_failsafe_work);

	unregister_pm_notifier(&s2idle_pm_nb);
	acpi_unregister_wakeup_handler(lid_wake_handler, NULL);
	input_unregister_handler(&pwr_handler);

	platform_device_unregister(s2idle_fix_pdev);
	platform_driver_unregister(&s2idle_fix_plat_driver);

	if (gpe52_was_enabled) {
		acpi_mask_gpe(NULL, LID_GPE, FALSE);
		acpi_set_gpe(NULL, LID_GPE, ACPI_GPE_ENABLE);
		gpe52_was_enabled = false;
	}

	if (lid_padcfg_base) {
		iounmap(lid_padcfg_base);
		lid_padcfg_base = NULL;
	}

	pr_info("unloaded: %u cycles, %u restores, %u early_restores, "
		"%u resyncs, %u lid_close_suspends\n",
		suspend_cycles, padcfg_restores, early_restores,
		lid_resyncs, lid_close_suspends);
}

module_init(surface_s2idle_fix_init);
module_exit(surface_s2idle_fix_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Surface Linux Debug");
MODULE_DESCRIPTION("Fix s2idle death sleep on Surface Laptop 5");
MODULE_ALIAS("dmi:*:svnMicrosoftCorporation:pnSurfaceLaptop5:*");
