#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of_irq.h>
#include <linux/power_supply.h>

struct hall_switch_dev {
	struct input_dev *input;
	struct device *dev;
	struct workqueue_struct *hall_workqueue;
	struct delayed_work report_work;
	int gpio;
	int irq;
	int last_state;
};

static void hall_switch_report_status(struct hall_switch_dev *hall)
{
	int state;

	state = gpio_get_value(hall->gpio);
	state = !state; // 反转电平

	if(hall->last_state != state) {
		hall->last_state = state;
		input_report_switch(hall->input, SW_LID, state);  // 1: 盖上，0: 打开
		input_sync(hall->input);
		pr_info("hall_switch: report state = %d\n", state);
	}else {
		pr_info("hall_switch: ignore repeated state = %d\n", state);
	}
	
}

static void hall_switch_report_func(struct work_struct *work)
{
    struct hall_switch_dev *hall = container_of(work,
											struct hall_switch_dev, report_work.work);
	hall_switch_report_status(hall);
}

static irqreturn_t hall_irq_handler(int irq, void *dev_id)
{
	struct hall_switch_dev *hall = dev_id;

	if(hall->hall_workqueue){
		mod_delayed_work(hall->hall_workqueue, &hall->report_work,msecs_to_jiffies(200));
	}else {
		hall_switch_report_status(hall);
	}
	pm_wakeup_event(hall->dev, 1000);

	return IRQ_HANDLED;
}

static int hall_switch_probe(struct platform_device *pdev)
{
	struct hall_switch_dev *hall;
	struct device *dev = &pdev->dev;
	int ret;

	hall = devm_kzalloc(dev, sizeof(*hall), GFP_KERNEL);
	if (!hall)
		return -ENOMEM;

	// 从设备树中解析 GPIO
	hall->gpio = of_get_named_gpio(dev->of_node, "hall-gpio", 0);
	if (!gpio_is_valid(hall->gpio)) {
		dev_err(dev, "invalid gpio\n");
		return -EINVAL;
	}

	ret = devm_gpio_request_one(dev, hall->gpio, GPIOF_IN, "hall-gpio");
	if (ret) {
		dev_err(dev, "failed to request gpio\n");
		return ret;
	}

	hall->irq = gpio_to_irq(hall->gpio);
	if (hall->irq < 0){
		dev_err(dev, "get irq no err %d\n",hall->irq);
		return -EINVAL;
	}

	// 初始化 input_dev
	hall->input = devm_input_allocate_device(dev);
	if (!hall->input)
		return -ENOMEM;

	hall->input->name = "hall_switch";
	hall->input->phys = "hall/input0";
	hall->input->id.bustype = BUS_HOST;
	input_set_capability(hall->input, EV_SW, SW_LID);

	ret = input_register_device(hall->input);
	if (ret){
		dev_err(dev, "input_register_device failed\n");
		input_free_device(hall->input);
		return ret;
	}

	hall->last_state = -1;

	hall->hall_workqueue = create_singlethread_workqueue("hall_switch_wq");
	if (!hall->hall_workqueue) {
		dev_err(dev, "create workqueue fail\n");
	} else {
		INIT_DELAYED_WORK(&hall->report_work, hall_switch_report_func);
	}

	// 使能中断唤醒功能
	hall->dev = &pdev->dev;
	ret = device_init_wakeup(hall->dev, true);
	if (ret) {
		dev_err(dev, "failed to init wakeup: %d\n", ret);
		goto err_init;
	}

	ret = devm_request_threaded_irq(dev, hall->irq, NULL,
				hall_irq_handler,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"hall-switch", hall);
	if (ret) {
		dev_err(dev, "request irq failed\n");
		goto err_init;
	}

	platform_set_drvdata(pdev, hall);

	// 启用 IRQ 唤醒功能
	enable_irq_wake(hall->irq);

	hall_switch_report_status(hall);
	pr_info("hall_switch: probe done\n");

	return 0;

err_init:
	destroy_workqueue(hall->hall_workqueue);
	return ret;
}

static int hall_switch_remove(struct platform_device *pdev)
{
	struct hall_switch_dev *hall = platform_get_drvdata(pdev);
	// 禁用 IRQ 唤醒功能
	disable_irq_wake(hall->irq);

	// 禁用设备唤醒功能
	device_init_wakeup(hall->dev, false);

	if (hall->hall_workqueue){
		cancel_delayed_work_sync(&hall->report_work);
		destroy_workqueue(hall->hall_workqueue);
		hall->hall_workqueue = NULL;
	}

	pr_info("hall_switch: removed\n");
	return 0;
}

#ifdef CONFIG_PM
static int hall_switch_suspend(struct device *dev)
{
	struct hall_switch_dev *hall =  dev_get_drvdata(dev);
    pr_info("%s last_state=%d enter\n",__func__,hall->last_state);
	hall->last_state = -1;
	return 0;
}

static int hall_switch_resume(struct device *dev)
{
	struct hall_switch_dev *hall = dev_get_drvdata(dev);;
    pr_info("%s last_state=%d enter\n",__func__,hall->last_state);
	hall->last_state = -1;
	return 0;
}

static const struct dev_pm_ops hall_switch_dev_pm_ops = {
	.suspend = hall_switch_suspend,
	.resume = hall_switch_resume,
};
#endif

static const struct of_device_id hall_switch_dt_ids[] = {
	{ .compatible = "qcom,hall-switch" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, hall_switch_dt_ids);

static struct platform_driver hall_switch_driver = {
	.probe = hall_switch_probe,
	.remove = hall_switch_remove,
	.driver = {
		.name = "hall_switch",
		.of_match_table = hall_switch_dt_ids,
#ifdef CONFIG_PM
		.pm = &hall_switch_dev_pm_ops,
#endif
	},
};
module_platform_driver(hall_switch_driver);

MODULE_AUTHOR("Q");
MODULE_DESCRIPTION("Hall Sensor Driver reporting SW_LID");
MODULE_LICENSE("GPL");
