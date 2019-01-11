/*
 * GPIO Driver for TW686X chips
 */
#include <linux/bitops.h>
#include <linux/gpio/driver.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "../media/pci/tw686x/tw686x.h"

#define DRIVER_NAME "gpio_tw686x"

struct tw686x_gpio_chip{
};

static int gpio_tw686x_probe(struct platform_device *pdev)
{
	struct pci_dev *pcidev = to_pci_dev(pdev->dev.parent);
	struct tw686x_gpio_chip *tw686x_gpio;

	tw686x_gpio = devm_kzalloc(&pdev->dev, sizeof(*tw686x_gpio), GFP_KERNEL);
	if (!tw686x_gpio)
		return -ENOMEM;

#if 0
	struct exar_gpio_chip *exar_gpio;
	u32 first_pin, ngpios;
	void __iomem *p;
	int index, ret;

	/*
	 * The UART driver must have mapped region 0 prior to registering this
	 * device - use it.
	 */
	p = pcim_iomap_table(pcidev)[0];
	if (!p)
		return -ENOMEM;

	ret = device_property_read_u32(&pdev->dev, "exar,first-pin",
				       &first_pin);
	if (ret)
		return ret;

	ret = device_property_read_u32(&pdev->dev, "ngpios", &ngpios);
	if (ret)
		return ret;

	exar_gpio = devm_kzalloc(&pdev->dev, sizeof(*exar_gpio), GFP_KERNEL);
	if (!exar_gpio)
		return -ENOMEM;

	mutex_init(&exar_gpio->lock);

	index = ida_simple_get(&ida_index, 0, 0, GFP_KERNEL);

	sprintf(exar_gpio->name, "exar_gpio%d", index);
	exar_gpio->gpio_chip.label = exar_gpio->name;
	exar_gpio->gpio_chip.parent = &pdev->dev;
	exar_gpio->gpio_chip.direction_output = exar_direction_output;
	exar_gpio->gpio_chip.direction_input = exar_direction_input;
	exar_gpio->gpio_chip.get_direction = exar_get_direction;
	exar_gpio->gpio_chip.get = exar_get_value;
	exar_gpio->gpio_chip.set = exar_set_value;
	exar_gpio->gpio_chip.base = -1;
	exar_gpio->gpio_chip.ngpio = ngpios;
	exar_gpio->regs = p;
	exar_gpio->index = index;
	exar_gpio->first_pin = first_pin;

	ret = devm_gpiochip_add_data(&pdev->dev,
				     &exar_gpio->gpio_chip, exar_gpio);
	if (ret)
		goto err_destroy;

	platform_set_drvdata(pdev, exar_gpio);

	return 0;

err_destroy:
	ida_simple_remove(&ida_index, index);
	mutex_destroy(&exar_gpio->lock);
	return ret;
#endif
}

static int gpio_tw686x_remove(struct platform_device *pdev)
{
	struct tw686x_gpio_chip *tw686x_gpio = platform_get_drvdata(pdev);

	//ida_simple_remove(&ida_index, exar_gpio->index);
	//mutex_destroy(&exar_gpio->lock);

	return 0;
}

static struct platform_driver gpio_tw686x_driver = {
	.probe	= gpio_tw686x_probe,
	.remove	= gpio_tw686x_remove,
	.driver	= {
		.name = DRIVER_NAME,
	},
};

module_platform_driver(gpio_tw686x_driver);

MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_DESCRIPTION("TW686X GPIO driver");
MODULE_AUTHOR("Robert Middleton <robert.middleton@rm5248.com>");
MODULE_LICENSE("GPL");
