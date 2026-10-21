// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2026 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * GPIO/PWM based system fan driver.
 *
 * Matches devicetree nodes with compatible = "gpio5-pwm", e.g.:
 *
 *  fan_controller {
 *		compatible = "gpio5-pwm";
 *		pwms = <&pmxr2230_pwm_2 0 40000>;
 *		gpio5-pwm-state = <1>;
 *		gpio5-pwm-duty-ns = <10000>;
 *		gpio5-pwm-period-ns = <40000>;
 *		qcom,fan-en-gpio = <&tlmm 55 0>;
 *		qcom,fan-fg-gpio = <&tlmm 3 0x2004>;
 *		pinctrl-names = "default", "sleep";
 *		pinctrl-0 = <&fan_pwm_ctrl_default
 *		     &fan_en_default
 *		     &fan_fg_default>;
 *		pinctrl-1 = <&fan_pwm_ctrl_sleep
 *		     &fan_en_sleep
 *		     &fan_fg_sleep>;
 *	};
 *
 * Sysfs (class "gpio5_pwm2"):
 *   /sys/class/gpio5_pwm2/state  (rw) - 1 enable / 0 disable
 *   /sys/class/gpio5_pwm2/duty   (rw) - duty cycle in ns, 0..period
 *   /sys/class/gpio5_pwm2/speed  (ro) - measured fan speed in RPM
 */

#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pwm.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#define FAN_CLASS_NAME		"gpio5_pwm2"
#define FAN_DEFAULT_PERIOD_NS	40000U
#define FAN_DEFAULT_DUTY_NS	10000U
#define FAN_PULSES_PER_REV	2U
#define FAN_SAMPLE_INTERVAL_MS	1000U
#define FAN_POWER_ON_DELAY_MS	20U
#define FAN_POWER_OFF_DELAY_MS	20U

struct qcom_fan_pwm {
	struct device		*dev;
	struct pwm_device	*pwm;
	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pinctrl_default;
	struct pinctrl_state	*pinctrl_sleep;
	int			en_gpio;
	int			fg_gpio;
	int			fg_irq;
	struct class		*class;

	struct mutex		lock;
	bool			enabled;
	u32			period_ns;
	u32			duty_ns;

	atomic_t		pulse_count;
	unsigned int		speed_rpm;
	struct delayed_work	speed_work;
};

/* Single instance is expected on a given board. */
static struct qcom_fan_pwm *g_fan;

static int qcom_fan_pwm_select_pinctrl(struct qcom_fan_pwm *fan,
					       bool enable)
{
	struct pinctrl_state *state;

	state = enable ? fan->pinctrl_default : fan->pinctrl_sleep;
	return pinctrl_select_state(fan->pinctrl, state);
}

static int qcom_fan_pwm_apply(struct qcom_fan_pwm *fan, bool enable)
{
	struct pwm_state pstate;
	u32 duty = fan->duty_ns;
	bool was_enabled = fan->enabled;
	int ret;

	/*
	 * The Qualcomm LPG converts the (duty_cycle / period) ratio into a
	 * fixed-width PWM register value:
	 *
	 *   pwm_value = (u16)((duty_ns << pwm_size) / period_ns)
	 *
	 * When duty_cycle == period the result is exactly 2^pwm_size. For the
	 * larger pwm_size values the LPG picks for better resolution this
	 * overflows the u16 and wraps back to 0, so a "full speed" request
	 * paradoxically programs 0% duty and STOPS the fan. Keep the applied
	 * duty strictly below the period (one step of margin) so a maxed-out
	 * duty still spins the fan at (near) full speed.
	 */
	if (duty >= fan->period_ns)
		duty = fan->period_ns - max(fan->period_ns / 256, 1U);

	/* Select the active pin mux before changing the power or PWM state. */
	if (enable && !was_enabled) {
		ret = qcom_fan_pwm_select_pinctrl(fan, true);
		if (ret) {
			dev_err(fan->dev, "failed to select default pinctrl: %d\n",
				ret);
			return ret;
		}

		/* Power must be stable before the motor PWM is enabled. */
		if (fan->en_gpio >= 0)
			gpio_set_value_cansleep(fan->en_gpio, 1);
		msleep(FAN_POWER_ON_DELAY_MS);
	}

	pwm_get_state(fan->pwm, &pstate);
	pstate.enabled = enable;
	pstate.period = fan->period_ns;
	pstate.duty_cycle = enable ? duty : 0;
	pstate.polarity = PWM_POLARITY_NORMAL;

	ret = pwm_apply_state(fan->pwm, &pstate);
	if (ret) {
		dev_err(fan->dev, "failed to apply pwm state: %d\n", ret);
		if (enable && !was_enabled) {
			/* Do not leave 5V enabled if PWM startup failed. */
			if (fan->en_gpio >= 0)
				gpio_set_value_cansleep(fan->en_gpio, 0);
			qcom_fan_pwm_select_pinctrl(fan, false);
		}
		return ret;
	}

	if (!enable) {
		if (was_enabled) {
			/* Let motor energy decay before removing its 5V supply. */
			msleep(FAN_POWER_OFF_DELAY_MS);
			if (fan->en_gpio >= 0)
				gpio_set_value_cansleep(fan->en_gpio, 0);
		}
		fan->enabled = false;

		ret = qcom_fan_pwm_select_pinctrl(fan, false);
		if (ret) {
			dev_err(fan->dev, "failed to select sleep pinctrl: %d\n",
				ret);
			return ret;
		}
	} else {
		fan->enabled = true;
	}

	return 0;
}

static irqreturn_t qcom_fan_pwm_fg_isr(int irq, void *data)
{
	struct qcom_fan_pwm *fan = data;

	atomic_inc(&fan->pulse_count);

	return IRQ_HANDLED;
}

static void qcom_fan_pwm_speed_work(struct work_struct *work)
{
	struct qcom_fan_pwm *fan = container_of(to_delayed_work(work),
						 struct qcom_fan_pwm, speed_work);
	unsigned int pulses = atomic_xchg(&fan->pulse_count, 0);

	if (fan->enabled)
		fan->speed_rpm = (pulses * 60000U) /
				 (FAN_PULSES_PER_REV * FAN_SAMPLE_INTERVAL_MS);
	else
		fan->speed_rpm = 0;

	schedule_delayed_work(&fan->speed_work,
			      msecs_to_jiffies(FAN_SAMPLE_INTERVAL_MS));
}

static ssize_t state_show(const struct class *class,
			  const struct class_attribute *attr,
			   char *buf)
{
	struct qcom_fan_pwm *fan = g_fan;

	if (!fan)
		return -ENODEV;

	return sysfs_emit(buf, "%d\n", fan->enabled ? 1 : 0);
}

static ssize_t state_store(const struct class *class,
			   const struct class_attribute *attr,
			    const char *buf, size_t count)
{
	struct qcom_fan_pwm *fan = g_fan;
	unsigned int val;
	int ret;

	if (!fan)
		return -ENODEV;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;

	if (val != 0 && val != 1)
		return -EINVAL;

	mutex_lock(&fan->lock);
	ret = qcom_fan_pwm_apply(fan, val == 1);
	mutex_unlock(&fan->lock);

	return ret ? ret : count;
}
static CLASS_ATTR_RW(state);

static ssize_t duty_show(const struct class *class,
			 const struct class_attribute *attr,
			  char *buf)
{
	struct qcom_fan_pwm *fan = g_fan;

	if (!fan)
		return -ENODEV;

	return sysfs_emit(buf, "%u\n", fan->duty_ns);
}

static ssize_t duty_store(const struct class *class,
			  const struct class_attribute *attr,
			   const char *buf, size_t count)
{
	struct qcom_fan_pwm *fan = g_fan;
	unsigned int val;
	int ret;

	if (!fan)
		return -ENODEV;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;

	if (val > fan->period_ns)
		val = fan->period_ns;

	mutex_lock(&fan->lock);
	fan->duty_ns = val;
	if (fan->enabled)
		ret = qcom_fan_pwm_apply(fan, true);
	mutex_unlock(&fan->lock);

	return ret ? ret : count;
}
static CLASS_ATTR_RW(duty);

static ssize_t speed_show(const struct class *class,
			  const struct class_attribute *attr,
			   char *buf)
{
	struct qcom_fan_pwm *fan = g_fan;

	if (!fan)
		return -ENODEV;

	return sysfs_emit(buf, "%u\n", fan->speed_rpm);
}
static CLASS_ATTR_RO(speed);

static int qcom_fan_pwm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct qcom_fan_pwm *fan;
	int ret;

	fan = devm_kzalloc(dev, sizeof(*fan), GFP_KERNEL);
	if (!fan)
		return -ENOMEM;

	fan->dev = dev;
	mutex_init(&fan->lock);
	atomic_set(&fan->pulse_count, 0);

	fan->pwm = devm_pwm_get(dev, NULL);
	if (IS_ERR(fan->pwm))
		return dev_err_probe(dev, PTR_ERR(fan->pwm),
				     "failed to get pwm\n");

	/* Pinctrl states are described by DT and selected by the driver. */
	fan->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(fan->pinctrl))
		return dev_err_probe(dev, PTR_ERR(fan->pinctrl),
				     "failed to get fan pinctrl\n");

	fan->pinctrl_default = pinctrl_lookup_state(fan->pinctrl,
						    PINCTRL_STATE_DEFAULT);
	if (IS_ERR(fan->pinctrl_default))
		return dev_err_probe(dev, PTR_ERR(fan->pinctrl_default),
				     "failed to get default pinctrl state\n");

	fan->pinctrl_sleep = pinctrl_lookup_state(fan->pinctrl, "sleep");
	if (IS_ERR(fan->pinctrl_sleep))
		return dev_err_probe(dev, PTR_ERR(fan->pinctrl_sleep),
				     "failed to get sleep pinctrl state\n");

	fan->period_ns = FAN_DEFAULT_PERIOD_NS;
	fan->duty_ns = FAN_DEFAULT_DUTY_NS;
	of_property_read_u32(np, "gpio5-pwm-period-ns", &fan->period_ns);
	of_property_read_u32(np, "gpio5-pwm-duty-ns", &fan->duty_ns);

	if (fan->duty_ns > fan->period_ns)
		fan->duty_ns = fan->period_ns;

	fan->en_gpio = of_get_named_gpio(np, "qcom,fan-en-gpio", 0);
	if (fan->en_gpio < 0 && fan->en_gpio != -ENOENT)
		return dev_err_probe(dev, fan->en_gpio,
				     "failed to get fan-en-gpio\n");
	if (gpio_is_valid(fan->en_gpio)) {
		ret = devm_gpio_request_one(dev, fan->en_gpio,
					     GPIOF_OUT_INIT_LOW, "fan-en");
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to request fan-en-gpio\n");
	}

	fan->fg_gpio = of_get_named_gpio(np, "qcom,fan-fg-gpio", 0);
	if (fan->fg_gpio < 0 && fan->fg_gpio != -ENOENT)
		return dev_err_probe(dev, fan->fg_gpio,
				     "failed to get fan-fg-gpio\n");
	if (gpio_is_valid(fan->fg_gpio)) {
		ret = devm_gpio_request_one(dev, fan->fg_gpio, GPIOF_IN,
					     "fan-fg");
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to request fan-fg-gpio\n");

		fan->fg_irq = gpio_to_irq(fan->fg_gpio);
		if (fan->fg_irq < 0)
			return dev_err_probe(dev, fan->fg_irq,
					     "failed to map fan-fg-gpio irq\n");

		ret = devm_request_irq(dev, fan->fg_irq, qcom_fan_pwm_fg_isr,
					IRQF_TRIGGER_FALLING, "fan-fg", fan);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to request fan-fg irq\n");
	} else {
		fan->fg_irq = -ENXIO;
	}

	fan->class = class_create(FAN_CLASS_NAME);
	if (IS_ERR(fan->class))
		return dev_err_probe(dev, PTR_ERR(fan->class),
				     "failed to create sysfs class\n");

	ret = class_create_file(fan->class, &class_attr_state);
	if (ret)
		goto err_destroy_class;
	ret = class_create_file(fan->class, &class_attr_duty);
	if (ret)
		goto err_remove_state;
	ret = class_create_file(fan->class, &class_attr_speed);
	if (ret)
		goto err_remove_duty;

	platform_set_drvdata(pdev, fan);
	g_fan = fan;

	INIT_DELAYED_WORK(&fan->speed_work, qcom_fan_pwm_speed_work);
	schedule_delayed_work(&fan->speed_work,
			      msecs_to_jiffies(FAN_SAMPLE_INTERVAL_MS));

	/*
	 * Power up with the fan OFF by default. The devicetree
	 * "gpio5-pwm-state" property is intentionally ignored: user space
	 * decides when to spin the fan up via /sys/class/gpio5_pwm2/state.
	 */
	mutex_lock(&fan->lock);
	ret = qcom_fan_pwm_apply(fan, false);
	mutex_unlock(&fan->lock);
	if (ret) {
		cancel_delayed_work_sync(&fan->speed_work);
		g_fan = NULL;
		class_remove_file(fan->class, &class_attr_speed);
		goto err_remove_duty;
	}

	dev_info(dev, "qcom-fan-pwm registered, period=%uns duty=%uns\n",
		 fan->period_ns, fan->duty_ns);

	return 0;

err_remove_duty:
	class_remove_file(fan->class, &class_attr_duty);
err_remove_state:
	class_remove_file(fan->class, &class_attr_state);
err_destroy_class:
	class_destroy(fan->class);
	return ret;
}

static int qcom_fan_pwm_remove(struct platform_device *pdev)
{
	struct qcom_fan_pwm *fan = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&fan->speed_work);

	mutex_lock(&fan->lock);
	qcom_fan_pwm_apply(fan, false);
	mutex_unlock(&fan->lock);

	class_remove_file(fan->class, &class_attr_speed);
	class_remove_file(fan->class, &class_attr_duty);
	class_remove_file(fan->class, &class_attr_state);
	class_destroy(fan->class);
	g_fan = NULL;

	return 0;
}
#if 0
static int __maybe_unused qcom_fan_pwm_suspend(struct device *dev)
{
	struct qcom_fan_pwm *fan = dev_get_drvdata(dev);

	/* Always stop the fan when the system enters suspend. */
	mutex_lock(&fan->lock);
	qcom_fan_pwm_apply(fan, false);
	mutex_unlock(&fan->lock);

	return 0;
}

/*
 * Resume deliberately leaves the fan OFF; user space re-enables it via the
 * "state" sysfs node when needed, matching the default-off boot behaviour.
 */
static SIMPLE_DEV_PM_OPS(qcom_fan_pwm_pm_ops, qcom_fan_pwm_suspend, NULL);
#endif

static void qcom_fan_pwm_shutdown(struct platform_device *pdev)
{
	struct qcom_fan_pwm *fan = platform_get_drvdata(pdev);

	mutex_lock(&fan->lock);
	qcom_fan_pwm_apply(fan, false);
	mutex_unlock(&fan->lock);

	pr_err("qcom_fan_pwm_shutdown\n");
}

static const struct of_device_id qcom_fan_pwm_of_match[] = {
	{ .compatible = "gpio5-pwm" },
	{ }
};
MODULE_DEVICE_TABLE(of, qcom_fan_pwm_of_match);

static struct platform_driver qcom_fan_pwm_driver = {
	.driver = {
		.name = "qcom-fan-pwm",
		.of_match_table = qcom_fan_pwm_of_match,
		//.pm = &qcom_fan_pwm_pm_ops,
	},
	.probe = qcom_fan_pwm_probe,
	.remove = qcom_fan_pwm_remove,
	.shutdown = qcom_fan_pwm_shutdown,
};
module_platform_driver(qcom_fan_pwm_driver);

MODULE_DESCRIPTION("Qualcomm GPIO/PWM based system fan driver");
MODULE_LICENSE("GPL");
