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
#include <linux/mc146818rtc.h>
#include <linux/interrupt.h>

#define RTC_IRQ		8	/* Standard CMOS RTC IRQ */

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
#define EC_KEEPALIVE_SECS       5     /* RTC wakeup interval during s2idle */

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
static bool lid_resync_active;
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
static unsigned int keepalive_wakes;
static unsigned int lid_open_wakes;

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
 * suspend_noirq: GPE 0x52 is now managed entirely by surface_gpe.
 * We leave the wake mask alone so lid-open triggers native wake.
 */
static int s2idle_fix_suspend_noirq(struct device *dev)
{
	/* GPE 0x52 wake mask left alone: surface_gpe manages it */
	pr_info("suspend_noirq: PADCFG save complete\n");
	return 0;
}

/*
 * resume_noirq: fix PADCFG corruption BEFORE any GPE re-enablement.
 * This runs before surface_gpe's resume and before the GPE subsystem
 * re-enables anything, so the pin state is clean when the SCI fires.
 */
static int s2idle_fix_resume_noirq(struct device *dev)
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

		padcfg_restores++;
		pr_info("resume_noirq: PADCFG0 RXINV corrected "
			"0x%08x -> 0x%08x\n", padcfg0, saved_padcfg0);
	} else {
		pr_info("resume_noirq: PADCFG0 OK (0x%08x)\n", padcfg0);
	}

	return 0;
}

/*
 * RTC keepalive via direct CMOS port I/O.
 *
 * During the s2idle loop, the RTC device is in noirq/suspended state,
 * so rtc_set_alarm() may fail or silently not re-enable AIE. Bypass
 * the RTC class entirely and hit the CMOS registers directly. The
 * CMOS RTC is on the LPC bus and is always accessible regardless of
 * CPU C-state or device suspend state.
 */
static void set_keepalive_alarm_cmos(void)
{
	unsigned char sec, min, hr, regb;
	unsigned int carry;
	unsigned long flags;

	spin_lock_irqsave(&rtc_lock, flags);

	/* Wait for update-in-progress to clear */
	while (CMOS_READ(RTC_FREQ_SELECT) & RTC_UIP)
		cpu_relax();

	/* Read current time (BCD) */
	sec = CMOS_READ(RTC_SECONDS);
	min = CMOS_READ(RTC_MINUTES);
	hr  = CMOS_READ(RTC_HOURS);

	/* BCD -> binary */
	sec = bcd2bin(sec);
	min = bcd2bin(min);
	hr  = bcd2bin(hr);

	/* Add keepalive interval */
	sec += EC_KEEPALIVE_SECS;
	carry = sec / 60;
	sec %= 60;
	min += carry;
	carry = min / 60;
	min %= 60;
	hr = (hr + carry) % 24;

	/* Write alarm registers (BCD) */
	CMOS_WRITE(bin2bcd(sec), RTC_SECONDS_ALARM);
	CMOS_WRITE(bin2bcd(min), RTC_MINUTES_ALARM);
	CMOS_WRITE(bin2bcd(hr),  RTC_HOURS_ALARM);

	/* Enable alarm interrupt (AIE) */
	regb = CMOS_READ(RTC_CONTROL);
	if (!(regb & RTC_AIE))
		CMOS_WRITE(regb | RTC_AIE, RTC_CONTROL);

	/* Clear any pending alarm flag */
	(void)CMOS_READ(RTC_INTR_FLAGS);

	spin_unlock_irqrestore(&rtc_lock, flags);
}

static void cancel_keepalive_alarm_cmos(void)
{
	unsigned char regb;
	unsigned long flags;

	spin_lock_irqsave(&rtc_lock, flags);

	/* Disable alarm interrupt */
	regb = CMOS_READ(RTC_CONTROL);
	if (regb & RTC_AIE)
		CMOS_WRITE(regb & ~RTC_AIE, RTC_CONTROL);

	/* Clear pending flag */
	(void)CMOS_READ(RTC_INTR_FLAGS);

	spin_unlock_irqrestore(&rtc_lock, flags);
}

/*
 * LPS0 s2idle device ops: these hooks run inside acpi_s2idle_prepare_late()
 * AFTER acpi_enable_all_wakeup_gpes() has already written the hardware
 * GPE enable registers. This is the ONLY place we can definitively disable
 * GPE 0x52, since acpi_s2idle_prepare() (called before dpm_suspend_noirq)
 * enables all wake GPEs at hardware level.
 *
 * Also sets an RTC keepalive alarm to prevent the EC from power-gating
 * the PCH. The EC has an internal timer; Windows prevents it by cycling
 * SSH D0<->D3 every ~190ms. We use a 25s RTC alarm instead.
 */
static void s2idle_lps0_prepare(void)
{
	/*
	 * GPE 0x52 left ENABLED: surface_gpe manages it as a proper
	 * wake GPE. Lid-open fires GPE 0x52 -> native wake.
	 * If PADCFG corruption causes a spurious GPE, the failsafe
	 * detects lid-still-closed and re-suspends.
	 */

	/*
	 * Suppress RTC wake so the keepalive alarm stays internal to the
	 * s2idle loop (fires lps0_check, not a full resume). This keeps
	 * the EC alive without churning through full suspend/resume cycles.
	 */
	acpi_disable_event(ACPI_EVENT_RTC, 0);
	acpi_clear_event(ACPI_EVENT_RTC);
	disable_irq_wake(RTC_IRQ);
	disable_irq_nosync(RTC_IRQ);

	set_keepalive_alarm_cmos();

	pr_info("lps0_prepare: GPE 0x52 enabled (native lid wake), "
		"RTC suppressed, keepalive alarm %ds\n", EC_KEEPALIVE_SECS);
}

static void s2idle_lps0_check(void)
{
	u32 padcfg0;

	keepalive_wakes++;

	/* Fix PADCFG corruption on every wake during s2idle loop */
	if (lid_padcfg_base && padcfg_saved) {
		padcfg0 = readl(lid_padcfg_base);
		if ((padcfg0 & PADCFG0_RXINV) !=
		    (saved_padcfg0 & PADCFG0_RXINV)) {
			writel(saved_padcfg1, lid_padcfg_base + 4);
			wmb();
			writel(saved_padcfg0, lid_padcfg_base);
			wmb();
			padcfg_restores++;
			pr_info("lps0_check: PADCFG0 RXINV corrected "
				"(wake #%u)\n", keepalive_wakes);
		}
	}
	acpi_clear_gpe(NULL, LID_GPE);
	acpi_clear_event(ACPI_EVENT_RTC);

	/* Re-arm keepalive for the next cycle */
	set_keepalive_alarm_cmos();

	/*
	 * Backup lid-open detection via RXSTATE polling. With GPE 0x52
	 * enabled, lid-open should wake natively. This is a safety net
	 * in case the native wake fails for any reason.
	 */
	if (lid_padcfg_base && padcfg_saved) {
		padcfg0 = readl(lid_padcfg_base);
		if (lid_was_closed_at_suspend &&
		    !(padcfg0 & PADCFG0_GPIORXSTATE)) {
			lid_open_wakes++;
			lid_was_closed_at_suspend = false;
			pr_info("lps0_check: lid opened (RXSTATE=0), "
				"triggering wake #%u\n", lid_open_wakes);
			pm_system_wakeup();
		}
	}
}

static void s2idle_lps0_restore(void)
{
	u32 padcfg0;

	cancel_keepalive_alarm_cmos();

	/* Re-enable RTC wake path */
	enable_irq(RTC_IRQ);
	enable_irq_wake(RTC_IRQ);
	acpi_clear_event(ACPI_EVENT_RTC);
	acpi_enable_event(ACPI_EVENT_RTC, 0);

	/* Fix PADCFG corruption before re-enabling GPE */
	if (lid_padcfg_base && padcfg_saved) {
		padcfg0 = readl(lid_padcfg_base);
		if ((padcfg0 & PADCFG0_RXINV) !=
		    (saved_padcfg0 & PADCFG0_RXINV)) {
			writel(saved_padcfg1, lid_padcfg_base + 4);
			wmb();
			writel(saved_padcfg0, lid_padcfg_base);
			wmb();
			padcfg_restores++;
			pr_info("lps0_restore: PADCFG0 RXINV corrected\n");
		}
	}
	pr_info("lps0_restore: keepalive wakes=%u lid_open_wakes=%u\n",
		keepalive_wakes, lid_open_wakes);
	keepalive_wakes = 0;
	lid_open_wakes = 0;
}

static struct acpi_s2idle_dev_ops s2idle_lps0_ops = {
	.prepare = s2idle_lps0_prepare,
	.check   = s2idle_lps0_check,
	.restore = s2idle_lps0_restore,
};

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

	return 0;
}

static const struct dev_pm_ops s2idle_fix_pm_ops = {
	.suspend_noirq = s2idle_fix_suspend_noirq,
	.resume_noirq  = s2idle_fix_resume_noirq,
	.resume_early  = s2idle_fix_resume_early,
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

	/* Don't race with lid_resync or failsafe, they handle suspend */
	if (lid_resync_active || failsafe_in_progress)
		goto reschedule;

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

reschedule:
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
		lid_resync_active = false;
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
		lid_resync_active = false;
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
			lid_resync_active = true;
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
		return;
	}

	/* Check CURRENT lid state, not just historical */
	if (lid_padcfg_base) {
		u32 now = readl(lid_padcfg_base);
		if (!(now & PADCFG0_GPIORXSTATE)) {
			pr_info("failsafe: lid is currently open "
				"(RXSTATE=0), staying awake\n");
			failsafe_suspends = 0;
			lid_was_closed_at_suspend = false;
			return;
		}
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
			lid_resync_active = false;
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

		/*
		 * Do NOT mask GPE 0x52: with surface_gpe loaded, it
		 * properly manages the lid GPE. Leaving it enabled gives
		 * native lid-open wake. If PADCFG corruption causes a
		 * spurious wake, the failsafe handles re-suspend.
		 */

		pr_info("suspend #%u: GPE 0x52 left enabled, "
			"PADCFG0=0x%08x RXINV=%d RXSTATE=%d "
			"lid_closed=%d failsafe_in_progress=%d\n",
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
	int err, lps0_err;
	bool pwr_ok;

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

	pr_info("RTC keepalive: %ds interval via direct CMOS I/O\n",
		EC_KEEPALIVE_SECS);

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
	pwr_ok = !err;
	if (err)
		pr_warn("power button handler failed: %d "
			"(failsafe will use max-retry fallback)\n", err);

	acpi_register_wakeup_handler(sci_irq, lid_wake_handler, NULL);
	register_pm_notifier(&s2idle_pm_nb);

	/* LPS0 hooks: the definitive GPE disable, runs AFTER wake GPEs
	 * are enabled in acpi_s2idle_prepare(), giving us the last word */
	lps0_err = acpi_register_lps0_dev(&s2idle_lps0_ops);
	if (lps0_err)
		pr_warn("LPS0 registration failed: %d "
			"(falling back to suspend_noirq only)\n", lps0_err);
	else
		pr_info("LPS0 s2idle hooks registered\n");

	/* Start RXSTATE polling */
	last_poll_rxstate = !!(initial_padcfg0 & PADCFG0_GPIORXSTATE);
	lid_poll_active = true;
	schedule_delayed_work(&lid_poll_work,
			      msecs_to_jiffies(LID_POLL_INTERVAL_MS));

	pr_info("loaded: SCI=%d PADCFG0=0x%08x RXINV=%d RXSTATE=%d "
		"pwr_handler=%s resume_early=YES lps0=%s\n",
		sci_irq, initial_padcfg0,
		!!(initial_padcfg0 & PADCFG0_RXINV),
		!!(initial_padcfg0 & PADCFG0_GPIORXSTATE),
		pwr_ok ? "OK" : "FAILED",
		lps0_err ? "FAILED" : "OK");
	pr_info("  GPE 0x52 left enabled (native lid wake via surface_gpe), "
		"failsafe %ums, resync %ums interval (max %u polls), "
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

	acpi_unregister_lps0_dev(&s2idle_lps0_ops);
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
