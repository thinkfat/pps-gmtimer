/*
 * pps-gmtimer.c -- PPS client driver using OMAP Timers
 *
 * Copyright (C) 2014  Daniel Drown <dan-android@drown.org>
 * Copyright (C) 2020  Matthias Welwarsky <matthias@welwarsky.de>
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#define MODULE_NAME "pps-gmtimer"
#define pr_fmt(fmt) MODULE_NAME ": " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pps_kernel.h>
#include <linux/clocksource.h>
#include <linux/workqueue.h>
#include <linux/notifier.h>
#include <linux/reboot.h>

#include <clocksource/timer-ti-dm.h>
#include <linux/platform_data/dmtimer-omap.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Matthias Welwarsky");
MODULE_DESCRIPTION("PPS input driver");
MODULE_VERSION("0.1.0");

#define TIMER_TYPE_CAPTURE	1
#define TIMER_TYPE_PPS		2

struct pps_gmtimer_data
{
	struct platform_device *pdev;
    const struct omap_dm_timer_ops *timer_ops;
    struct omap_dm_timer *capture_timer;
    struct omap_dm_timer *pps_timer;
    const char *timer_name;
    const char *pps_timer_name;
    int use_tclkin;
    uint32_t frequency;
    unsigned int capture;
    unsigned int match;
    uint32_t count_at_interrupt;
    uint32_t count_at_match;
    uint32_t count_at_capture;
    uint32_t next_match;
    int32_t diff_at_match;
    bool primed;
    struct pps_event_time ts;
    struct timespec64 delta;
    struct pps_device *pps;
    struct pps_source_info info;
    struct clocksource clksrc;
    struct work_struct offload;
    struct notifier_block reboot_notifier;
};

/* kobject *******************/
static ssize_t timer_name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct pps_gmtimer_data *pdata = dev_get_drvdata(dev);
    return sprintf(buf, "%s\n", pdata->timer_name);
}

static DEVICE_ATTR(timer_name, S_IRUGO, timer_name_show, NULL);

static ssize_t stats_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct pps_gmtimer_data *pdata = dev_get_drvdata(dev);
    return sprintf(buf, "capture: %u\nmatch: %u\n", pdata->capture, pdata->match);
}

static DEVICE_ATTR(stats, S_IRUGO, stats_show, NULL);

static ssize_t interrupt_delta_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct pps_gmtimer_data *pdata = dev_get_drvdata(dev);
    return sprintf(buf, "%lld.%09ld\n", (long long)pdata->delta.tv_sec, pdata->delta.tv_nsec);
}

static DEVICE_ATTR(interrupt_delta, S_IRUGO, interrupt_delta_show, NULL);

static ssize_t pps_ts_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct pps_gmtimer_data *pdata = dev_get_drvdata(dev);
    return sprintf(buf, "%lld.%09ld\n", (long long)pdata->ts.ts_real.tv_sec, pdata->ts.ts_real.tv_nsec);
}

static DEVICE_ATTR(pps_ts, S_IRUGO, pps_ts_show, NULL);

static ssize_t count_at_interrupt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct pps_gmtimer_data *pdata = dev_get_drvdata(dev);
    return sprintf(buf, "%u\n", pdata->count_at_interrupt);
}

static DEVICE_ATTR(count_at_interrupt, S_IRUGO, count_at_interrupt_show, NULL);

static ssize_t capture_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct pps_gmtimer_data *pdata = dev_get_drvdata(dev);
    return sprintf(buf, "%u\n",
                   __omap_dm_timer_read(pdata->capture_timer, OMAP_TIMER_CAPTURE_REG, pdata->capture_timer->posted));
}

static DEVICE_ATTR(capture, S_IRUGO, capture_show, NULL);

static ssize_t ctrlstatus_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct pps_gmtimer_data *pdata = dev_get_drvdata(dev);
    return sprintf(buf, "%x\n",
                   __omap_dm_timer_read(pdata->capture_timer, OMAP_TIMER_CTRL_REG, pdata->capture_timer->posted));
}

static DEVICE_ATTR(ctrlstatus, S_IRUGO, ctrlstatus_show, NULL);

static ssize_t timer_counter_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct pps_gmtimer_data *pdata = dev_get_drvdata(dev);
    unsigned int current_count = 0;

    current_count = pdata->timer_ops->read_counter(pdata->capture_timer);
    return sprintf(buf, "%u\n", current_count);
}

static DEVICE_ATTR(timer_counter, S_IRUGO, timer_counter_show, NULL);

static ssize_t timer_match_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct pps_gmtimer_data *pdata = dev_get_drvdata(dev);
    return sprintf(buf, "%u\n", pdata->count_at_match);
}

static DEVICE_ATTR(timer_match, S_IRUGO, timer_match_show, NULL);

static ssize_t timer_diff_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct pps_gmtimer_data *pdata = dev_get_drvdata(dev);
    return sprintf(buf, "%i %08x %08x\n", pdata->diff_at_match, 
		    pdata->count_at_match, pdata->count_at_capture);
}

static DEVICE_ATTR(timer_diff, S_IRUGO, timer_diff_show, NULL);

static struct attribute *attrs[] = {
    &dev_attr_timer_counter.attr,
    &dev_attr_ctrlstatus.attr,
    &dev_attr_capture.attr,
    &dev_attr_count_at_interrupt.attr,
    &dev_attr_pps_ts.attr,
    &dev_attr_interrupt_delta.attr,
    &dev_attr_stats.attr,
    &dev_attr_timer_name.attr,
    &dev_attr_timer_match.attr,
    &dev_attr_timer_diff.attr,
    NULL,
};

static struct attribute_group attr_group = {
    .attrs = attrs,
};

/*
 * Trigger a reload on two timers (almost) simultaneously
 *
 * Ideally, we need the capture and the pps pwm timer to start
 * exactly synchronized so that the internal pps events
 * are aligned with the externally visible pps pulse. However, the
 * dmtimers don't provide for cross-triggering so we do the best we
 * can in software.
 */
static void pps_gmtimer_sync(struct pps_gmtimer_data *pdata)
{
	uint32_t cnt;

	void __iomem *pps_cnt = pdata->pps_timer->func_base + _OMAP_TIMER_COUNTER_OFFSET;
	void __iomem *cpt_cnt = pdata->capture_timer->func_base + _OMAP_TIMER_COUNTER_OFFSET;
	uint32_t load_val = 0xFFFFFFFF - pdata->frequency + 1;

	/* read the capture counter until it changes */
	cnt = readl_relaxed(cpt_cnt);
	while (readl_relaxed(cpt_cnt) == cnt)
		;
	/* two back-to-back writes to both TCNT registers, can't do better */
	writel_relaxed(0, cpt_cnt);
	writel_relaxed(load_val + 2, pps_cnt);
}

/* timers ********************/
static irqreturn_t pps_gmtimer_interrupt(int irq, void *data)
{
    struct pps_gmtimer_data *pdata = data;
	unsigned int irq_status;

	irq_status = pdata->timer_ops->read_status(pdata->capture_timer);
	/* ack the interrupt immediately to not miss capture events */
	__omap_dm_timer_write_status(pdata->capture_timer, irq_status);

	/* MATCH event triggers the virtual PPS event capture */
	if (irq_status & OMAP_TIMER_INT_MATCH) {
		struct system_time_snapshot snap;
		uint32_t cam = pdata->next_match;
		uint32_t ps_per_hz;

		if (pdata->primed) {
			/* use ktime_get_snapshot() as it delivers a synchronized raw cycle count from the clocksource */
			ktime_get_snapshot(&snap);
			pdata->ts.ts_real = ktime_to_timespec64(snap.real);
			pdata->count_at_interrupt = snap.cycles;
			// use picoseconds per hz to avoid floating point and limit the rounding error
			ps_per_hz = 1000000000 / (pdata->frequency / 1000);
			pdata->delta.tv_sec = 0;
			pdata->delta.tv_nsec = ((pdata->count_at_interrupt - cam) * ps_per_hz) / 1000;
			pps_sub_ts(&pdata->ts, pdata->delta);
			pps_event(pdata->pps, &pdata->ts, PPS_CAPTUREASSERT, NULL);

			pdata->match++;
		}

		/* next match in one second */
		pdata->count_at_match = cam;
		pdata->next_match = cam + pdata->frequency;
		__omap_dm_timer_write(pdata->capture_timer,
				OMAP_TIMER_MATCH_REG, pdata->next_match, pdata->capture_timer->posted);
	}

	if (irq_status & OMAP_TIMER_INT_CAPTURE) {
	   pdata->count_at_capture = __omap_dm_timer_read(pdata->capture_timer,
			OMAP_TIMER_CAPTURE_REG, pdata->capture_timer->posted);

		pdata->capture++;

		/* prime the counters, possibly sync them */
		if (pdata->capture > 60 && pdata->primed == 0) {
			pdata->count_at_match = pdata->count_at_capture = 0;
			pdata->next_match = pdata->count_at_match + pdata->frequency;
			pdata->match = pdata->capture;

			__omap_dm_timer_write(pdata->capture_timer,
					OMAP_TIMER_MATCH_REG, pdata->next_match, pdata->capture_timer->posted);

			if (pdata->pps_timer != NULL)
				pps_gmtimer_sync(pdata);
			else
				__omap_dm_timer_write(pdata->capture_timer,
						OMAP_TIMER_TRIGGER_REG, 0, pdata->capture_timer->posted);

			/* kernel may now use the clocksource */
			schedule_work(&pdata->offload);
			pdata->primed = 1;

			pr_debug("match primed: %u cap:%u\n", pdata->next_match, pdata->count_at_match);
		} else {
			uint32_t last_cam = pdata->count_at_match;
			int32_t cycle_diff, diff_raw;

			cycle_diff = diff_raw = last_cam - pdata->count_at_capture;

			if (cycle_diff < 0 && abs(cycle_diff) > pdata->frequency / 2)
				cycle_diff += pdata->frequency;
			if  (cycle_diff > 0 && abs(cycle_diff) > pdata->frequency / 2)
				cycle_diff -= pdata->frequency;

			pdata->diff_at_match = cycle_diff;

			pr_debug("capture: cam:%u cap:%u diff:%i raw:%i/%x\n", last_cam,
					pdata->count_at_capture, cycle_diff, diff_raw, diff_raw);
		}
	}

    return IRQ_HANDLED;
}

static void pps_gmtimer_setup_pwm(struct pps_gmtimer_data *pdata, struct omap_dm_timer *timer)
{
	const struct omap_dm_timer_ops *timer_ops =  pdata->timer_ops;
	uint32_t ns_per_hz = 100000 / (pdata->frequency / 1000);
	uint32_t load_val = 0xFFFFFFFF - pdata->frequency + 1;

	timer_ops->set_prescaler(timer, -1);
	timer_ops->set_load(timer, 1, load_val);
	timer_ops->set_match(timer, 1, load_val + 1000 * ns_per_hz);
	timer_ops->set_pwm(timer, 0, 1, OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);
}

static void pps_gmtimer_setup_capture(struct pps_gmtimer_data *pdata, struct omap_dm_timer *timer)
{
    const struct omap_dm_timer_ops *timer_ops =  pdata->timer_ops;
    u32 ctrl;

    /*
     * NOTE: the OMAP dmtimer framework doesn't support capture mode!
     * Do the timer setup manually.
     */
    timer_ops->enable(timer);

    ctrl = __omap_dm_timer_read(timer, OMAP_TIMER_CTRL_REG, timer->posted);

    /* disable prescaler */
    ctrl &= ~(OMAP_TIMER_CTRL_PRE | (0x07 << 2));

    /* autoreload with 0 */
    ctrl |= OMAP_TIMER_CTRL_AR;
    __omap_dm_timer_write(timer, OMAP_TIMER_LOAD_REG, 0, timer->posted);

    /* setup compare event to match counter in 1 second */
    ctrl |= OMAP_TIMER_CTRL_CE;
    __omap_dm_timer_write(timer, OMAP_TIMER_MATCH_REG, pdata->frequency, timer->posted);
    pdata->next_match = pdata->frequency;
    pdata->count_at_match = 0;
    pdata->primed = 0;

    /* enable capture on rising edge */
    ctrl |= OMAP_TIMER_CTRL_TCM_LOWTOHIGH | OMAP_TIMER_CTRL_GPOCFG;

    /* load counter with 0 */
    __omap_dm_timer_write(timer, OMAP_TIMER_COUNTER_REG, 0, timer->posted);
    /* set the configuration but don't start yet */
    __omap_dm_timer_write(timer, OMAP_TIMER_CTRL_REG, ctrl, timer->posted);

    timer_ops->disable(timer);

    /* Save the context */
    timer->context.tclr = ctrl;
    timer->context.tldr = 0;
    timer->context.tcrr = 0;
    timer->context.tmar = pdata->frequency;
}

static void pps_gmtimer_set_clksrc(struct pps_gmtimer_data *pdata,
		struct omap_dm_timer *timer, const char *clksrc)
{
    struct clk *gt_fclk;
    struct clk *parent;
    int ret;

    gt_fclk = pdata->timer_ops->get_fclk(timer);
    /* retrieve the virtual timer clock "timerN_fck" */
    gt_fclk = clk_get_parent(gt_fclk);
    if (IS_ERR(gt_fclk)) {
	    pr_err("cannot get timerN_fck\n");
	    return;
    }
    parent = clk_get(&timer->pdev->dev, clksrc);
    if (IS_ERR(parent)) {
	    pr_err("parent timer %s not found\n", clksrc);
	    return;
    }
    pr_debug("%s rate %luHz\n", clksrc, clk_get_rate(parent));
    ret = clk_set_parent(gt_fclk, parent);
    if (ret < 0)
	    pr_err("cannot reparent clock to %s (%i)\n", clksrc, ret);
    clk_put(parent);
}

static void pps_gmtimer_enable_irq(struct pps_gmtimer_data *pdata)
{
    unsigned int interrupt_mask;

    interrupt_mask = OMAP_TIMER_INT_CAPTURE | OMAP_TIMER_INT_MATCH;
    __omap_dm_timer_int_enable(pdata->capture_timer, interrupt_mask);
    pdata->capture_timer->context.tier = interrupt_mask;
    pdata->capture_timer->context.twer = interrupt_mask;
}

static struct omap_dm_timer *pps_gmtimer_get_timer_by_dn(
		struct pps_gmtimer_data *pdata, struct device_node *timer_dn,
		const char **timer_name)
{
    struct omap_dm_timer *timer;

    /* check if the node is indeed a timer */
    if (of_property_read_string_index(timer_dn, "ti,hwmods", 0, timer_name) < 0) {
        pr_err("ti,hwmods property missing?\n");
        return ERR_PTR(-ENODEV);
    }

    timer = pdata->timer_ops->request_by_node(timer_dn);
    if (timer == NULL) {
    	pr_err("timer request_by_node failed\n");
    	return ERR_PTR(-ENODEV);
    }

    return timer;
}

static int pps_gmtimer_init_timer(struct pps_gmtimer_data *pdata,
		struct omap_dm_timer *timer, const char *timer_name, int timer_type)
{
    struct clk *gt_fclk;

	if (pdata->use_tclkin)
		pps_gmtimer_set_clksrc(pdata, timer, "tclkin_ck");
	else
		pr_info("using system clock\n");

	/* override timer errata and enable posted mode */
	__omap_dm_timer_override_errata(timer, OMAP_TIMER_ERRATA_I103_I767);
	__omap_dm_timer_enable_posted(timer);

    switch (timer_type) {
    case TIMER_TYPE_CAPTURE:
    	if (devm_request_irq(&pdata->pdev->dev,
    			timer->irq, pps_gmtimer_interrupt, IRQF_TIMER, MODULE_NAME, pdata)) {
            pr_err("cannot register IRQ %d for timer %s\n", timer->irq, timer_name);
            /* don't leak timers if interrupt registration fails */
            pdata->timer_ops->free(timer);
            return -EIO;
        }

        /* get clock frequency from the mandatory capture timer */
        gt_fclk = pdata->timer_ops->get_fclk(timer);
        pdata->frequency = clk_get_rate(gt_fclk);
        pr_info("timer name=%s rate=%uHz\n", timer_name, pdata->frequency);
        pps_gmtimer_setup_capture(pdata, timer);
        break;

    case TIMER_TYPE_PPS:
    	pps_gmtimer_setup_pwm(pdata, timer);
    	break;
    }

    return 0;
}

static void pps_gmtimer_cleanup_timers(struct pps_gmtimer_data *pdata)
{
    if (pdata->capture_timer) {
    	struct omap_dm_timer *timer = pdata->capture_timer;
    	pps_gmtimer_set_clksrc(pdata, timer, "sys_clkin_ck");
        pdata->timer_ops->set_int_disable(timer,
    		OMAP_TIMER_INT_CAPTURE | OMAP_TIMER_INT_MATCH);
        devm_free_irq(&pdata->pdev->dev, timer->irq, pdata);
        pdata->timer_ops->stop(pdata->capture_timer);
        pdata->timer_ops->free(pdata->capture_timer);
        pdata->capture_timer = NULL;
    }

    if (pdata->pps_timer) {
    	pps_gmtimer_set_clksrc(pdata, pdata->pps_timer, "sys_clkin_ck");
        pdata->timer_ops->stop(pdata->pps_timer);
        pdata->timer_ops->free(pdata->pps_timer);
    	pdata->pps_timer = NULL;
    }

    pr_info("Exiting.\n");
}

/* clocksource ***************/

static void clocksource_work(struct work_struct *work) {
	struct pps_gmtimer_data *pdata = container_of(work, struct pps_gmtimer_data, offload);
	struct clocksource *cs = &pdata->clksrc;

	/* set a very high rating to force selection of this clocksource */
	pr_debug("clocksource rating changed\n");

	clocksource_change_rating(cs, 401);
}

static u64 pps_gmtimer_read_cycles(struct clocksource *cs)
{
	struct pps_gmtimer_data *pdata = container_of(cs, struct pps_gmtimer_data, clksrc);
    return (u64)__omap_dm_timer_read_counter(pdata->capture_timer, pdata->capture_timer->posted);
}

static void pps_gmtimer_clocksource_init(struct pps_gmtimer_data *pdata)
{
	pdata->clksrc.name = pdata->timer_name;
	pdata->clksrc.read = pps_gmtimer_read_cycles;
	pdata->clksrc.mask = CLOCKSOURCE_MASK(32);
	pdata->clksrc.flags = CLOCK_SOURCE_IS_CONTINUOUS;
	/* clocksource is unusable until 1PPS input is stable */
	pdata->clksrc.rating = 1;

	if (clocksource_register_hz(&pdata->clksrc, pdata->frequency)) {
		pr_err("Could not register clocksource %s\n", pdata->clksrc.name);
		return;
	}

	pr_info("clocksource: %s at %u Hz\n", pdata->clksrc.name, pdata->frequency);
}

/* module ********************/

static int pps_gmtimer_prepare_reboot(struct notifier_block *n,
		unsigned long what, void *data)
{
	struct pps_gmtimer_data *pdata = container_of(n,
			struct pps_gmtimer_data, reboot_notifier);

	clocksource_unregister(&pdata->clksrc);
	pps_gmtimer_cleanup_timers(pdata);

	return 0;
}

static int of_get_pps_gmtimer_pdata(struct device_node *np,
		struct pps_gmtimer_data *pdata)
{
    struct device_node *timer_dn;
    struct platform_device *timer_pdev;
    struct dmtimer_platform_data *timer_pdata;
    struct omap_dm_timer *timer;
    const char *timer_name;

    /* Get the capture timer */
    timer_dn = of_parse_phandle(np, "timer", 0);
    if (!timer_dn) {
        pr_err("Unable to parse device node\n");
        return -ENODEV;
    }

    timer_pdev = of_find_device_by_node(timer_dn);
    if (!timer_pdev) {
        pr_err("Unable to find Timer pdev\n");
        goto fail2;
    }

    timer_pdata = dev_get_platdata(&timer_pdev->dev);
    if (!timer_pdata) {
        pr_err("dmtimer pdata structure NULL\n");
        goto fail2;
    }

    pdata->timer_ops = timer_pdata->timer_ops;

    /* sanity check for timer_ops completeness */
    if (!pdata->timer_ops || !pdata->timer_ops->request_by_node ||
        !pdata->timer_ops->free ||
        !pdata->timer_ops->enable ||
        !pdata->timer_ops->disable ||
        !pdata->timer_ops->get_fclk ||
        !pdata->timer_ops->start ||
        !pdata->timer_ops->stop ||
        !pdata->timer_ops->set_load ||
        !pdata->timer_ops->set_match ||
        !pdata->timer_ops->set_pwm ||
        !pdata->timer_ops->set_prescaler ||
        !pdata->timer_ops->write_counter)
    {
        pr_err("Incomplete dmtimer pdata structure\n");
        goto fail2;
    }

    timer = pps_gmtimer_get_timer_by_dn(pdata, timer_dn, &timer_name);
    if (IS_ERR(timer))
    	goto fail2;

    pdata->capture_timer = timer;
    pdata->timer_name = timer_name;

    /* done with the capture timer */
    of_node_put(timer_dn);

    /* Get the optional pps output timer */
    timer_dn = of_parse_phandle(np, "ppstimer", 0);
    if (timer_dn == NULL) {
        pr_info("no ppstimer defined, going without\n");
        return 0;
    }

    timer = pps_gmtimer_get_timer_by_dn(pdata, timer_dn, &timer_name);
    /* done with the device_node, release it */
    of_node_put(timer_dn);
    /* check if we got the timer */
    if (IS_ERR(timer)) {
    	pr_info("ppstimer invalid\n");
    	return 0;
    }

    pdata->pps_timer = timer;
    pdata->pps_timer_name = timer_name;

    return 0;

fail2:
    of_node_put(timer_dn);
    return -ENODEV;
}

static const struct of_device_id pps_gmtimer_dt_ids[] = {
    {
        .compatible = "pps-gmtimer",
    },
    {/* sentinel */}};
MODULE_DEVICE_TABLE(of, pps_gmtimer_dt_ids);

static int pps_gmtimer_probe(struct platform_device *pdev)
{
    struct pps_gmtimer_data *pdata;
    struct pinctrl *pinctrl;

    pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
    if (!pdata)
        return -ENOMEM;

    pdata->pdev = pdev;

    /* establish pinmux before initializing the timer */
    pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
    if (IS_ERR(pinctrl))
        pr_warning("pins are not configured from the driver\n");

    pdata->use_tclkin = of_property_read_bool(pdev->dev.of_node, "use-tclkin");

    /* fill in config from OF */
    if (of_get_pps_gmtimer_pdata(pdev->dev.of_node, pdata) < 0)
    	goto out_free;

    if (pps_gmtimer_init_timer(pdata,
    		pdata->capture_timer, pdata->timer_name, TIMER_TYPE_CAPTURE) < 0)
    	goto out_cleanup;

    if (pdata->pps_timer != NULL)
		pps_gmtimer_init_timer(pdata,
				pdata->pps_timer, pdata->pps_timer_name, TIMER_TYPE_PPS);

    /*
     * at this point, all configured timers are configured
     * but not yet running
     */

    platform_set_drvdata(pdev, pdata);

    if (sysfs_create_group(&pdev->dev.kobj, &attr_group))
        pr_err("sysfs_create_group failed\n");

    INIT_WORK(&pdata->offload, clocksource_work);

    /* PPS registration */
    pdata->info.mode = PPS_CAPTUREASSERT | PPS_ECHOASSERT | PPS_CANWAIT | PPS_TSFMT_TSPEC;
    pdata->info.owner = THIS_MODULE;
    snprintf(pdata->info.name, PPS_MAX_NAME_LEN - 1, "%s", pdata->timer_name);

    pdata->pps = pps_register_source(&pdata->info, PPS_CAPTUREASSERT);
    if (pdata->pps == NULL) {
        pr_err("failed to register %s as PPS source\n", pdata->timer_name);
        goto out_cleanup;
    }

    /* reboot notifier */
    pdata->reboot_notifier.notifier_call = pps_gmtimer_prepare_reboot;
    pdata->reboot_notifier.priority = 0;
    register_reboot_notifier(&pdata->reboot_notifier);

	/* enable interrupts */
	pps_gmtimer_enable_irq(pdata);
	/* start the timers */
	pdata->timer_ops->start(pdata->capture_timer);
	if (pdata->pps_timer) {
		pdata->timer_ops->start(pdata->pps_timer);
		pps_gmtimer_sync(pdata);
	}

    /* clocksource registration */
	pps_gmtimer_clocksource_init(pdata);

    return 0;

out_cleanup:
	pps_gmtimer_cleanup_timers(pdata);
out_free:
	devm_kfree(&pdev->dev, pdata);
	return -ENODEV;
}

static int pps_gmtimer_remove(struct platform_device *pdev)
{
    struct pps_gmtimer_data *pdata = platform_get_drvdata(pdev);

	clocksource_unregister(&pdata->clksrc);
	pps_gmtimer_cleanup_timers(pdata);
	pps_unregister_source(pdata->pps);
	sysfs_remove_group(&pdev->dev.kobj, &attr_group);
	unregister_reboot_notifier(&pdata->reboot_notifier);
	devm_kfree(&pdev->dev, pdata);
	platform_set_drvdata(pdev, NULL);

    return 0;
}

static struct platform_driver pps_gmtimer_driver = {
    .probe = pps_gmtimer_probe,
    .remove = pps_gmtimer_remove,
    .driver = {
        .name = MODULE_NAME,
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(pps_gmtimer_dt_ids),
    },
};

module_platform_driver(pps_gmtimer_driver);
