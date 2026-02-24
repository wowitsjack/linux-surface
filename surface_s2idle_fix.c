// SPDX-License-Identifier: GPL-2.0
/*
 * surface_s2idle_fix.c - Fix s2idle death sleep on Surface Laptop 5
 *
 * ROOT CAUSE:
 *   GPE 0x52 (lid switch, pin 213) causes spurious SCI during s2idle.
 *   The Intel INTC1055 pinctrl power-gates during idle, corrupting
 *   the PADCFG0 RXINV bit on pin 213. This generates a false edge
 *   on GPE 0x52, triggering a spurious SCI. The ACPI s2idle wake
 *   handler processes this as a non-wake event and calls
 *   pm_system_cancel_wakeup(), poisoning the wakeup framework.
 *   Subsequent legitimate wake events (RTC alarm, etc.) are ignored,
 *   resulting in permanent sleep ("death sleep").
 *
 * FIX STRATEGY:
 *   1. Before s2idle: save PADCFG0/1 for pin 213, mask GPE 0x52
 *      using acpi_mask_gpe() so it survives acpi_enable_all_wakeup_gpes()
 *   2. Register an ACPI wakeup handler that checks/restores PADCFG0
 *      if RXINV was corrupted, returns false for normal wake processing
 *   3. After resume: restore PADCFG if corrupted, unmask GPE 0x52
 *   4. Force lid state re-evaluation by calling \_GPE._L52, so the
 *      kernel/logind see the correct lid position after resume.
 *      Without this, the system can stay awake with lid closed.
 *
 * Kernel: 6.18.x (surface-linux)
 * Hardware: Microsoft Surface Laptop 5 (ADL-U, INTC1055 pinctrl)
 */

#define pr_fmt(fmt) "surface_s2idle_fix: " fmt

#include <linux/module.h>
#include <linux/acpi.h>
#include <linux/io.h>
#include <linux/suspend.h>
#include <linux/dmi.h>

/* ------------------------------------------------------------------ */
/* Lid GPIO physical addresses (pin 213, community INTC1055, pad S0)  */
/* ------------------------------------------------------------------ */
#define LID_PADCFG0_PHYS       0xfd6a09a0
#define LID_PADCFG1_PHYS       0xfd6a09a4
#define PADCFG_MAP_SIZE        8

#define PADCFG0_RXINV          BIT(23)
#define PADCFG0_GPIROUTSCI     BIT(19)
#define PADCFG0_GPIOTXDIS      BIT(9)
#define PADCFG0_GPIORXSTATE    BIT(1)

#define LID_GPE                0x52

/* ------------------------------------------------------------------ */
/* Module state                                                        */
/* ------------------------------------------------------------------ */
static void __iomem *lid_padcfg_base;
static u32 saved_padcfg0;
static u32 saved_padcfg1;
static bool padcfg_saved;
static bool gpe52_was_enabled;
static int sci_irq;

/* GPE method path for lid state re-evaluation on resume */
#define LID_GPE_METHOD  "\\_GPE._L52"

/* Stats */
static unsigned int suspend_cycles;
static unsigned int padcfg_restores;
static unsigned int wakeup_handler_calls;
static unsigned int lid_resyncs;

/* ------------------------------------------------------------------ */
/* Wakeup handler                                                      */
/*                                                                     */
/* Called from acpi_check_wakeup_handlers() inside acpi_s2idle_wake()  */
/* on ANY SCI during s2idle.                                           */
/*                                                                     */
/* We check/fix PADCFG corruption but return false so normal wake      */
/* processing continues (RTC alarm, power button, etc. all work).      */
/* Since GPE 0x52 is masked during s2idle, it cannot generate          */
/* the spurious SCI that triggers the death spiral.                    */
/* ------------------------------------------------------------------ */
static bool lid_wake_handler(void *context)
{
	u32 current_padcfg0;

	wakeup_handler_calls++;

	if (!lid_padcfg_base || !padcfg_saved)
		return false;

	current_padcfg0 = readl(lid_padcfg_base);

	if ((current_padcfg0 & PADCFG0_RXINV) !=
	    (saved_padcfg0 & PADCFG0_RXINV)) {
		/* PADCFG0 RXINV bit was corrupted by power gating.
		 * Restore it before anyone re-enables GPE 0x52. */
		writel(saved_padcfg1, lid_padcfg_base + 4);
		wmb();
		writel(saved_padcfg0, lid_padcfg_base);
		wmb();

		/* Clear any pending GPE 0x52 from the corruption */
		acpi_clear_gpe(NULL, LID_GPE);

		padcfg_restores++;
		pr_info("PADCFG0 RXINV corrupted: was 0x%08x, restored 0x%08x "
			"(cycle #%u)\n",
			current_padcfg0, saved_padcfg0, suspend_cycles);
	}

	/* Return false: let normal ACPI wake processing handle
	 * the actual wake event (RTC alarm, power button, etc.) */
	return false;
}

/* ------------------------------------------------------------------ */
/* PM notifier: mask GPE 0x52 before s2idle, unmask + resync after     */
/* ------------------------------------------------------------------ */
static int s2idle_pm_notify(struct notifier_block *nb,
			    unsigned long action, void *data)
{
	u32 padcfg0;

	if (!lid_padcfg_base)
		return NOTIFY_DONE;

	switch (action) {
	case PM_SUSPEND_PREPARE:
		/* Save PADCFG state (mask out volatile GPIORXSTATE) */
		padcfg0 = readl(lid_padcfg_base);
		saved_padcfg0 = padcfg0 & ~PADCFG0_GPIORXSTATE;
		saved_padcfg1 = readl(lid_padcfg_base + 4);
		padcfg_saved = true;
		suspend_cycles++;

		/* Mask GPE 0x52 to prevent spurious SCI from
		 * PADCFG RXINV corruption during s2idle.
		 * acpi_mask_gpe() sets a persistent flag that prevents
		 * acpi_enable_all_wakeup_gpes() from re-enabling it.
		 * This is the same mechanism as sysfs "echo disable". */
		acpi_mask_gpe(NULL, LID_GPE, TRUE);
		gpe52_was_enabled = true;

		pr_info("suspend #%u: masked GPE 0x52, "
			"PADCFG0=0x%08x RXINV=%d\n",
			suspend_cycles, padcfg0,
			!!(padcfg0 & PADCFG0_RXINV));
		break;

	case PM_POST_SUSPEND:
		/* Restore PADCFG if corrupted (belt-and-suspenders,
		 * the wakeup handler may have already done this) */
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
				pr_info("resume: PADCFG0 restored "
					"(was 0x%08x, wrote 0x%08x)\n",
					padcfg0, saved_padcfg0);
			}
		}

		/* Unmask GPE 0x52 for normal lid operation */
		if (gpe52_was_enabled) {
			acpi_mask_gpe(NULL, LID_GPE, FALSE);
			acpi_set_gpe(NULL, LID_GPE, ACPI_GPE_ENABLE);
			gpe52_was_enabled = false;
		}

		/*
		 * Force lid state re-evaluation. GPE 0x52 was masked
		 * during sleep, so the kernel has a stale lid state.
		 * Call the GPE _L52 method directly, which triggers:
		 *   _L52() -> Notify(LID0, 0x80) -> button driver
		 *          -> reads _LID -> sends SW_LID input event
		 * This ensures logind sees the correct lid position
		 * and re-suspends if the lid is still closed.
		 */
		{
			acpi_status as;
			as = acpi_evaluate_object(NULL, LID_GPE_METHOD,
						  NULL, NULL);
			if (ACPI_SUCCESS(as)) {
				lid_resyncs++;
				pr_info("resume #%u: lid state resynced "
					"via %s\n",
					suspend_cycles, LID_GPE_METHOD);
			} else {
				pr_warn("resume #%u: %s failed: %s\n",
					suspend_cycles, LID_GPE_METHOD,
					acpi_format_exception(as));
			}
		}

		pr_info("resume #%u: unmasked+enabled GPE 0x52, "
			"handler_calls=%u padcfg_restores=%u "
			"lid_resyncs=%u\n",
			suspend_cycles, wakeup_handler_calls,
			padcfg_restores, lid_resyncs);

		padcfg_saved = false;
		wakeup_handler_calls = 0;
		break;
	}

	return NOTIFY_DONE;
}

static struct notifier_block s2idle_pm_nb = {
	.notifier_call = s2idle_pm_notify,
	.priority = INT_MAX,  /* Run before other PM notifiers */
};

/* ------------------------------------------------------------------ */
/* DMI matching                                                        */
/* ------------------------------------------------------------------ */
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

/* ------------------------------------------------------------------ */
/* Module init/exit                                                    */
/* ------------------------------------------------------------------ */
static int __init surface_s2idle_fix_init(void)
{
	u32 initial_padcfg0;

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

	/* Register wakeup handler for PADCFG fixup on wake */
	acpi_register_wakeup_handler(sci_irq, lid_wake_handler, NULL);

	/* PM notifier to disable/enable GPE 0x52 around s2idle */
	register_pm_notifier(&s2idle_pm_nb);

	pr_info("loaded: SCI=%d PADCFG0=0x%08x RXINV=%d\n",
		sci_irq, initial_padcfg0,
		!!(initial_padcfg0 & PADCFG0_RXINV));
	pr_info("  GPE 0x52 will be masked during s2idle to prevent "
		"spurious wake death spiral\n");

	return 0;
}

static void __exit surface_s2idle_fix_exit(void)
{
	unregister_pm_notifier(&s2idle_pm_nb);
	acpi_unregister_wakeup_handler(lid_wake_handler, NULL);

	/* Make sure GPE 0x52 is enabled if we're being unloaded
	 * during a suspend cycle (unlikely but safe) */
	if (gpe52_was_enabled) {
		acpi_mask_gpe(NULL, LID_GPE, FALSE);
		acpi_set_gpe(NULL, LID_GPE, ACPI_GPE_ENABLE);
		gpe52_was_enabled = false;
	}

	if (lid_padcfg_base) {
		iounmap(lid_padcfg_base);
		lid_padcfg_base = NULL;
	}

	pr_info("unloaded: %u suspend cycles, %u padcfg restores, "
		"%u lid resyncs\n",
		suspend_cycles, padcfg_restores, lid_resyncs);
}

module_init(surface_s2idle_fix_init);
module_exit(surface_s2idle_fix_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Surface Linux Debug");
MODULE_DESCRIPTION("Fix s2idle death sleep on Surface Laptop 5 "
		   "(mask lid GPE 0x52 during s2idle to prevent "
		   "PADCFG RXINV corruption spurious wake)");
MODULE_ALIAS("dmi:*:svnMicrosoftCorporation:pnSurfaceLaptop5:*");
