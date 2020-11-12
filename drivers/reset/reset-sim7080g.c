/*
 * GPIO-based Reset Driver for SIMCom SIM7080G
 *
 * Copyright (c) 2020 Advaly System, Inc. All Rights Reserved.
 *
 * Based on: reset-ec25.c
 *   Copyright (c) 2018 Atmark Techno, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/regulator/consumer.h>

struct sim7080g_reset_data {
	struct reset_controller_dev rcdev;

	struct regulator *vbat;

	unsigned int gpio_pwrkey;
	unsigned int gpio_status;

	bool active_low_pwrkey;

	struct device *dev;
};

#define SIM7080G_PWRKEY_POWER_ON_ASSERT_TIME_MS		(1000)
#define SIM7080G_PWRKEY_POWER_OFF_ASSERT_TIME_MS	(1200)
#define SIM7080G_POWER_OFF_TO_ON_WAIT_TIME_MS	(2000)

#define to_sim7080g_reset_data(_rcdev) \
	container_of(_rcdev, struct sim7080g_reset_data, rcdev)

static void sim7080g_reset(unsigned int gpio, bool active_low, unsigned long delay_ms)
{
	gpio_set_value_cansleep(gpio, !active_low);
	mdelay(delay_ms);
	gpio_set_value_cansleep(gpio, active_low);
}

static int sim7080g_power_off(struct sim7080g_reset_data *drvdata)
{
	int ret;

	/* We should check the status pin this time */
	if (regulator_is_enabled(drvdata->vbat)) {
		ret = regulator_disable(drvdata->vbat);
		if (ret)
			return ret;
	}

	return 0;
}

static int sim7080g_power_on(struct sim7080g_reset_data *drvdata)
{
	int ret;

	sim7080g_power_off(drvdata);
	mdelay(SIM7080G_POWER_OFF_TO_ON_WAIT_TIME_MS);

	ret = regulator_enable(drvdata->vbat);
	if (ret)
		return ret;

	sim7080g_reset(drvdata->gpio_pwrkey, drvdata->active_low_pwrkey,
				SIM7080G_PWRKEY_POWER_ON_ASSERT_TIME_MS);

	return 0;
}

static ssize_t sim7080g_power_ctrl_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct platform_device *pdev= to_platform_device(dev);
	struct sim7080g_reset_data *drvdata = platform_get_drvdata(pdev);
	int val, ret;

	ret = kstrtoint(buf, 0, &val);
	if (ret)
		return ret;

	if (val)
		ret = sim7080g_power_on(drvdata);
	else
		ret = sim7080g_power_off(drvdata);

	return ret ? : count;
}
static DEVICE_ATTR(sim7080g_power_ctrl, S_IWUSR, NULL, sim7080g_power_ctrl_store);

static struct attribute *sim7080g_reset_attrs[] = {
	&dev_attr_sim7080g_power_ctrl.attr,
	NULL
};

static struct attribute_group sim7080g_reset_attr_group = {
	.name = NULL, /* put in device directory */
	.attrs = sim7080g_reset_attrs,
};

static int of_sim7080g_reset_xlate(struct reset_controller_dev *rcdev,
				const struct of_phandle_args *reset_spec)
{
	if (WARN_ON(reset_spec->args_count != 0))
		return -EINVAL;

	return 0;
}

static int sim7080g_reset_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct sim7080g_reset_data *drvdata;
	enum of_gpio_flags flags;
	unsigned long gpio_flags;
	int ret;

	drvdata = devm_kzalloc(&pdev->dev, sizeof(*drvdata), GFP_KERNEL);
	if (drvdata == NULL)
		return -ENOMEM;

	drvdata->vbat = devm_regulator_get(&pdev->dev, "vbat");
	if (IS_ERR(drvdata->vbat)) {
		ret = PTR_ERR(drvdata->vbat);
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "vbat property missing\n");

		return ret;
	}

	if (of_gpio_named_count(np, "gpio-pwrkey") != 1) {
		dev_err(&pdev->dev,
			"gpio-pwrkey property missing, or not a single gpio\n");
		return -EINVAL;
	}

	drvdata->gpio_pwrkey = of_get_named_gpio_flags(np,
						"gpio-pwrkey", 0, &flags);
	if (drvdata->gpio_pwrkey == -EPROBE_DEFER) {
		return drvdata->gpio_pwrkey;
	} else if (!gpio_is_valid(drvdata->gpio_pwrkey)) {
		dev_err(&pdev->dev, "invalid pwrkey gpio: %d\n", drvdata->gpio_pwrkey);
		return drvdata->gpio_pwrkey;
	}

	/* The status pin may be necessary in the future. */
	if (of_gpio_named_count(np, "gpio-status") == 1) {
		drvdata->gpio_status = of_get_named_gpio_flags(np, "gpio-status", 0, &flags);
		if (drvdata->gpio_status == -EPROBE_DEFER) {
			return drvdata->gpio_status;
		} else if (!gpio_is_valid(drvdata->gpio_status)) {
			dev_err(&pdev->dev, "invalid status gpio: %d\n", drvdata->gpio_status);
			return drvdata->gpio_status;
		}

		ret = devm_gpio_request_one(&pdev->dev, drvdata->gpio_status, GPIOF_IN, NULL);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to request gpio %d: %d\n",
				drvdata->gpio_status, ret);
			return ret;
		}
	}

	drvdata->active_low_pwrkey = flags & OF_GPIO_ACTIVE_LOW;
	if (drvdata->active_low_pwrkey)
		gpio_flags = GPIOF_OUT_INIT_HIGH;
	else
		gpio_flags = GPIOF_OUT_INIT_LOW;

	ret = devm_gpio_request_one(&pdev->dev, drvdata->gpio_pwrkey, gpio_flags, NULL);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to request gpio %d: %d\n",
			drvdata->gpio_pwrkey, ret);
		return ret;
	}

	ret = regulator_enable(drvdata->vbat);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable vbat regulator: %d\n",
									ret);
		return ret;
	}

	sim7080g_power_on(drvdata);

	drvdata->dev = &pdev->dev;
	platform_set_drvdata(pdev, drvdata);

	drvdata->rcdev.of_node = np;
	drvdata->rcdev.owner = THIS_MODULE;
	drvdata->rcdev.nr_resets = 1;
	drvdata->rcdev.of_xlate = of_sim7080g_reset_xlate;
	reset_controller_register(&drvdata->rcdev);

	return sysfs_create_group(&pdev->dev.kobj, &sim7080g_reset_attr_group);
}

static int sim7080g_reset_remove(struct platform_device *pdev)
{
	struct sim7080g_reset_data *drvdata = platform_get_drvdata(pdev);

	reset_controller_unregister(&drvdata->rcdev);

	sysfs_remove_group(&pdev->dev.kobj, &sim7080g_reset_attr_group);

	return 0;
}

static struct of_device_id sim7080g_reset_dt_ids[] = {
	{ .compatible = "sim7080g-reset" },
	{ }
};

static struct platform_driver sim7080g_reset_driver = {
	.probe = sim7080g_reset_probe,
	.remove = sim7080g_reset_remove,
	.driver = {
		.name = "sim7080g-reset",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(sim7080g_reset_dt_ids),
	},
};

static int __init sim7080g_reset_init(void)
{
	return platform_driver_register(&sim7080g_reset_driver);
}
arch_initcall(sim7080g_reset_init);

static void __exit sim7080g_reset_exit(void)
{
	platform_driver_unregister(&sim7080g_reset_driver);
}
module_exit(sim7080g_reset_exit);

MODULE_AUTHOR("Takumi Ando <t-ando@advaly.co.jp>");
MODULE_DESCRIPTION("SIMCom SIM7080G Reset Controller");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:sim7080g-reset");
MODULE_DEVICE_TABLE(of, sim7080g_reset_dt_ids);
