/*
 * Copyright (c) 2014-2015, Alucard24@XDA
 * Copyright (c) 2015, Tom G. <roboter972@gmail.com>
 *
 * Based on the cpu-boost-implementation:
 * Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <mach/cpufreq.h>

enum {
	INPUT_BOOST_MS = 40,
	MAX_CPUS = CONFIG_NR_CPUS,
	MIN_INPUT_INTERVAL = 150
};

static unsigned int input_boost_ms = INPUT_BOOST_MS;
module_param(input_boost_ms, uint, S_IRUGO | S_IWUSR);

static unsigned int nr_boost_cpus = MAX_CPUS;
module_param(nr_boost_cpus, uint, S_IRUGO | S_IWUSR);

static unsigned int min_input_interval = MIN_INPUT_INTERVAL;
module_param(min_input_interval, uint, S_IRUGO | S_IWUSR);

static unsigned long input_boost_freqs[] = {
	1026000, /* CPU0 */
	1026000, /* CPU1 */
	1026000, /* CPU2 */
	1026000  /* CPU3 */
};

static struct workqueue_struct *touch_boost_wq;
static struct delayed_work input_boost_rem;
static struct work_struct input_boost_work;

static void do_input_boost_rem(struct work_struct *work)
{
	unsigned int i;

	for_each_possible_cpu(i)
		set_cpu_min_lock(i, 0);
}

static void do_input_boost(struct work_struct *work)
{
	unsigned int i;

	cancel_delayed_work_sync(&input_boost_rem);

	if (nr_boost_cpus < 1)
		nr_boost_cpus = 1;
	else if (nr_boost_cpus > MAX_CPUS)
		nr_boost_cpus = MAX_CPUS;

	for (i = 0; i < nr_boost_cpus; i++) {
		struct cpufreq_policy policy;
		unsigned int cur = 0;

		set_cpu_min_lock(i, input_boost_freqs[i]);

		if (cpu_online(i)) {
			cur = cpufreq_quick_get(i);
			if (cur < input_boost_freqs[i] && cur > 0) {
				policy.cpu = i;
				cpufreq_driver_target(&policy,
					input_boost_freqs[i], CPUFREQ_RELATION_L);
			}
		}
	}

	queue_delayed_work_on(0, touch_boost_wq, &input_boost_rem,
					msecs_to_jiffies(input_boost_ms));
}

static void touchboost_input_event(struct input_handle *handle,
			unsigned int type, unsigned int code, int value)
{
	u64 now, last_input_time = 0;
	unsigned int i;

	for (i = 0; i < nr_boost_cpus; i++) {
		if (input_boost_freqs[i] == 0)
			return;
	}

	now = ktime_to_us(ktime_get());

	if (now - last_input_time < min_input_interval * USEC_PER_MSEC)
		return;

	if (work_pending(&input_boost_work))
		return;

	queue_work(touch_boost_wq, &input_boost_work);

	last_input_time = ktime_to_us(ktime_get());
}

static int touchboost_input_connect(struct input_handler *handler,
		struct input_dev *dev, const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = handler->name;

	error = input_register_handle(handle);
	if (error)
		goto err2;

	error = input_open_device(handle);
	if (error)
		goto err1;

	return 0;

err1:
	input_unregister_handle(handle);
err2:
	kfree(handle);
	return error;
}

static void touchboost_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id touchboost_ids[] = {
	/* Multi-touch touchscreen */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			INPUT_DEVICE_ID_MATCH_ABSBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		.absbit = { [BIT_WORD(ABS_MT_POSITION_X)] =
			BIT_MASK(ABS_MT_POSITION_X) |
			BIT_MASK(ABS_MT_POSITION_Y) },
	},
	/* Touchpad */
	{
		.flags = INPUT_DEVICE_ID_MATCH_KEYBIT |
			INPUT_DEVICE_ID_MATCH_ABSBIT,
		.keybit = { [BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH) },
		.absbit = { [BIT_WORD(ABS_X)] =
			BIT_MASK(ABS_X) | BIT_MASK(ABS_Y) },
	},
	{ },
};

static struct input_handler touchboost_input_handler = {
	.event          = touchboost_input_event,
	.connect        = touchboost_input_connect,
	.disconnect     = touchboost_input_disconnect,
	.name           = "alu_t_boost",
	.id_table       = touchboost_ids,
};

static int set_input_boost_freqs(const char *buf, const struct kernel_param *kp)
{
	unsigned long val[4];
	unsigned int i;
	int ret;

	ret = sscanf(buf, "%lu %lu %lu %lu", &val[0], &val[1], &val[2], &val[3]);

	if (ret < 1 || ret > 4)
		return -EINVAL;

	for (i = 0; i < 4; i++)
		input_boost_freqs[i] = val[i];

	return 0;
}

static int get_input_boost_freqs(char *buf, const struct kernel_param *kp)
{
	return snprintf(buf, PAGE_SIZE, "%lu \t%lu \t%lu \t%lu\n",
					input_boost_freqs[0],
					input_boost_freqs[1],
					input_boost_freqs[2],
					input_boost_freqs[3]);
}

static struct kernel_param_ops param_ops_input_boost_freqs = {
	.set = set_input_boost_freqs,
	.get = get_input_boost_freqs,
};

module_param_cb(input_boost_freqs, &param_ops_input_boost_freqs, NULL,
							S_IRUGO | S_IWUSR);

static int touch_boost_init(void)
{
	int ret;

	touch_boost_wq = alloc_workqueue("touch_boost_wq", WQ_HIGHPRI, 0);
	if (!touch_boost_wq)
		return -ENOMEM;

	INIT_WORK(&input_boost_work, do_input_boost);
	INIT_DELAYED_WORK(&input_boost_rem, do_input_boost_rem);

	ret = input_register_handler(&touchboost_input_handler);
	if (ret) {
		pr_err("Failed to register touchboost input handler!\n");
		return ret;
	}

	return ret;
}

late_initcall(touch_boost_init);
