/*
 * Copyright (c) 2012, The Linux Foundation. All rights reserved.
 * Copyright (c) 2015, Tom G. <roboter972@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define pr_fmt(fmt) "msm_thermal: " fmt

#include <linux/cpufreq.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/msm_thermal.h>
#include <linux/msm_tsens.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/workqueue.h>
#include <mach/cpufreq.h>

static DEFINE_SPINLOCK(thermal_lock);

#define TSENS_FAILED_RESCHEDULE_MS	100

enum throttling_status {
	UNTHROTTLED,
	PHASE1,
	PHASE2,
};

static enum throttling_status tstat = UNTHROTTLED;
static unsigned int prev_tstat = UNTHROTTLED;
static uint32_t max_freq = MSM_CPUFREQ_NO_LIMIT;

static struct msm_thermal_data msm_thermal_info;
static struct workqueue_struct *msm_thermal_wq;
static struct delayed_work check_temp_work;
static struct kobject *msm_thermal_kobject;

static void check_temp(struct work_struct *work)
{
	struct tsens_device tsens_dev;
	unsigned long temp;
	unsigned int sampling_rate;
	int i, ret;

	tsens_dev.sensor_num = msm_thermal_info.sensor_id;

	ret = tsens_get_temp(&tsens_dev, &temp);
	if (ret) {
		pr_err("Failed to read TSENS sensor data!\n");
		queue_delayed_work(msm_thermal_wq, &check_temp_work,
				msecs_to_jiffies(TSENS_FAILED_RESCHEDULE_MS));
		return;
	}

	switch (tstat) {
	case UNTHROTTLED:
		if (temp >= msm_thermal_info.allowed_low_temp) {
			max_freq = msm_thermal_info.allowed_low_freq;
			tstat = PHASE1;
		}
		break;
	case PHASE1:
		if (temp < msm_thermal_info.allowed_low_rel_temp) {
			max_freq = MSM_CPUFREQ_NO_LIMIT;
			tstat = UNTHROTTLED;
		} else if (temp >= msm_thermal_info.allowed_high_temp) {
			max_freq = msm_thermal_info.allowed_high_freq;
			tstat = PHASE2;
		}
		break;
	case PHASE2:
		if (temp < msm_thermal_info.allowed_high_rel_temp) {
			max_freq = msm_thermal_info.allowed_low_freq;
			tstat = PHASE1;
		} else if (temp >= msm_thermal_info.shutdown_temp) {
			spin_lock(&thermal_lock);
			emergency_sync();
			kernel_power_off();
			spin_unlock(&thermal_lock);
		}
		break;
	}

	if (tstat != prev_tstat) {
		for_each_possible_cpu(i) {
			msm_cpufreq_set_freq_limits(i,
					MSM_CPUFREQ_NO_LIMIT, max_freq);

			ret = cpufreq_update_policy(i);
			if (ret)
				pr_debug("Failed to update \
					CPU policy for CPU%d!\n", i);
		}
		prev_tstat = tstat;
	}

	switch (tstat) {
	case UNTHROTTLED:
		sampling_rate = msm_thermal_info.poll_ms;
		break;
	default:
		sampling_rate = msm_thermal_info.throttle_poll_ms;
		break;
	}

	queue_delayed_work(msm_thermal_wq, &check_temp_work,
					msecs_to_jiffies(sampling_rate));
}

/******************************** SYSFS START ********************************/
#define show_one(file_name, object)					\
static ssize_t show_##file_name						\
(struct kobject *kobj, struct attribute *attr, char *buf)		\
{									\
	return snprintf(buf, PAGE_SIZE, "%u\n", msm_thermal_info.object); \
}

#define store_one(file_name, object)					\
static ssize_t store_##file_name					\
(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count) \
{									\
	unsigned int input;						\
	int ret;							\
									\
	ret = sscanf(buf, "%u", &input);				\
	if (ret != 1)							\
		return -EINVAL;						\
									\
	msm_thermal_info.object = input;				\
									\
	return count;							\
}

#define global_attr_rw(_name)						\
static struct global_attr _name =					\
__ATTR(_name, S_IRUGO | S_IWUSR, show_##_name, store_##_name)

show_one(shutdown_temp, shutdown_temp);
show_one(allowed_high_temp, allowed_high_temp);
show_one(allowed_high_rel_temp, allowed_high_rel_temp);
show_one(allowed_high_freq, allowed_high_freq);
show_one(allowed_low_temp, allowed_low_temp);
show_one(allowed_low_rel_temp, allowed_low_rel_temp);
show_one(allowed_low_freq, allowed_low_freq);
show_one(throttle_poll_ms, throttle_poll_ms);
show_one(poll_ms, poll_ms);

store_one(shutdown_temp, shutdown_temp);
store_one(allowed_high_temp, allowed_high_temp);
store_one(allowed_high_rel_temp, allowed_high_rel_temp);
store_one(allowed_high_freq, allowed_high_freq);
store_one(allowed_low_temp, allowed_low_temp);
store_one(allowed_low_rel_temp, allowed_low_rel_temp);
store_one(allowed_low_freq, allowed_low_freq);
store_one(throttle_poll_ms, throttle_poll_ms);
store_one(poll_ms, poll_ms);

global_attr_rw(shutdown_temp);
global_attr_rw(allowed_high_temp);
global_attr_rw(allowed_high_rel_temp);
global_attr_rw(allowed_high_freq);
global_attr_rw(allowed_low_temp);
global_attr_rw(allowed_low_rel_temp);
global_attr_rw(allowed_low_freq);
global_attr_rw(throttle_poll_ms);
global_attr_rw(poll_ms);

static struct attribute *msm_thermal_attributes[] = {
	&shutdown_temp.attr,
	&allowed_high_temp.attr,
	&allowed_high_rel_temp.attr,
	&allowed_high_freq.attr,
	&allowed_low_temp.attr,
	&allowed_low_rel_temp.attr,
	&allowed_low_freq.attr,
	&throttle_poll_ms.attr,
	&poll_ms.attr,
	NULL,
};

static struct attribute_group msm_thermal_attr_group = {
	.attrs = msm_thermal_attributes,
};
/********************************* SYSFS END *********************************/

int __devinit msm_thermal_init(struct msm_thermal_data *pdata)
{
	int rc;

	if (!pdata || pdata->sensor_id >= TSENS_MAX_SENSORS)
		return -EINVAL;

	memcpy(&msm_thermal_info, pdata, sizeof(struct msm_thermal_data));

	msm_thermal_wq = alloc_workqueue("msm_thermal",
					WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if (!msm_thermal_wq) {
		pr_err("Workqueue allocation failed!");
		return -ENOMEM;
	}

	INIT_DELAYED_WORK(&check_temp_work, check_temp);
	queue_delayed_work(msm_thermal_wq, &check_temp_work, 0);

	msm_thermal_kobject = kobject_create_and_add("msm_thermal",
							kernel_kobj);
	if (!msm_thermal_kobject) {
		pr_err("Sysfs kobj creation failed!");
		return -ENOMEM;
	}

	rc = sysfs_create_group(msm_thermal_kobject, &msm_thermal_attr_group);
	if (rc) {
		pr_err("Sysfs group creation failed!");
		kobject_put(msm_thermal_kobject);
		return rc;
	}

	pr_info("Initialized!\n");

	return 0;
}

static int __devinit msm_thermal_dev_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct msm_thermal_data data;
	int ret;
	char *key = NULL;

	memset(&data, 0, sizeof(struct msm_thermal_data));

	key = "qcom,sensor-id";
	ret = of_property_read_u32(node, key, &data.sensor_id);
	if (ret)
		goto fail;
	if (data.sensor_id >= TSENS_MAX_SENSORS) {
		data.sensor_id = 7;
		pr_warn("Tsens sensor-id out of range, defaulting to %u\n",
							data.sensor_id);
	}

	key = "qcom,poll-ms";
	ret = of_property_read_u32(node, key, &data.poll_ms);
	if (ret)
		goto fail;

	key = "qcom,throttle_poll-ms";
	ret = of_property_read_u32(node, key, &data.throttle_poll_ms);
	if (ret)
		goto fail;

	key = "qcom,shutdown_temp";
	ret = of_property_read_u32(node, key, &data.shutdown_temp);
	if (ret)
		goto fail;

	key = "qcom,allowed_high_temp";
	ret = of_property_read_u32(node, key, &data.allowed_high_temp);
	if (ret)
		goto fail;

	key = "qcom,allowed_high_rel_temp";
	ret = of_property_read_u32(node, key, &data.allowed_high_rel_temp);
	if (ret)
		goto fail;

	key = "qcom,allowed_high_freq";
	ret = of_property_read_u32(node, key, &data.allowed_high_freq);
	if (ret)
		goto fail;

	key = "qcom,allowed_low_temp";
	ret = of_property_read_u32(node, key, &data.allowed_low_temp);
	if (ret)
		goto fail;

	key = "qcom,allowed_low_rel_temp";
	ret = of_property_read_u32(node, key, &data.allowed_low_rel_temp);
	if (ret)
		goto fail;

	key = "qcom,allowed_low_freq";
	ret = of_property_read_u32(node, key, &data.allowed_low_freq);
	if (ret)
		goto fail;

	msm_thermal_init(&data);

	pr_info("Probed!\n");

	return 0;

fail:
	pr_err("%s: Failed reading node=%s, key=%s\n",
					__func__, node->full_name, key);
	return -EINVAL;
}

static struct of_device_id msm_thermal_match_table[] = {
	{.compatible = "qcom,msm-thermal"},
	{},
};

static struct platform_driver msm_thermal_device_driver = {
	.probe = msm_thermal_dev_probe,
	.driver = {
		.name = "msm-thermal",
		.owner = THIS_MODULE,
		.of_match_table = msm_thermal_match_table,
	},
};

int __init msm_thermal_device_init(void)
{
	return platform_driver_register(&msm_thermal_device_driver);
}

late_initcall(msm_thermal_device_init);
