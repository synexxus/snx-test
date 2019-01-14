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
	struct tw686x_dev *parent;
	struct gpio_chip gpio_chip;
	struct mutex lock;
};

static int tw686x_gpio_register_num(unsigned int offset)
{
	if (offset < 16)
		return GPIO_REG;
	if (offset > 15 && offset < 32)
		return GPIO_BANK2;

	return GPIO_BANK3;
}

static u32 tw686x_direction_bit(unsigned int offset)
{
	if (offset < 16)
		return BIT(offset) << 16;
	if (offset > 15 && offset < 32) 
		return BIT(offset - 16) << 16;

	return BIT(offset - 32) << 16;
}

static u32 tw686x_value_bit(unsigned int offset)
{
	if (offset < 16)
		return BIT(offset);
	if (offset > 15 && offset < 32) 
		return BIT(offset - 16);

	return BIT(offset - 32);
}

static void tw686x_set_gpio(struct gpio_chip *chip, int val,
			unsigned int offset)
{
	struct tw686x_gpio_chip *tw686x_gpio = gpiochip_get_data(chip);
	u32 temp;

	mutex_lock(&tw686x_gpio->lock);
	temp = reg_read(tw686x_gpio->parent, tw686x_gpio_register_num(offset));
	printk( "set_gpio reg read 0x%X\n", temp );
	temp &= ~tw686x_value_bit(offset);
	if (val)
		temp |= tw686x_value_bit(offset);
	printk("offset %d is bit 0x%X\n", offset, tw686x_value_bit(offset));
	printk( "gpio setting reg %x to value %x\n", tw686x_gpio_register_num(offset), temp);
	reg_write(tw686x_gpio->parent, tw686x_gpio_register_num(offset), temp);
	mutex_unlock(&tw686x_gpio->lock);
}

static int tw686x_set_direction(struct gpio_chip *chip, int direction,
			      unsigned int offset)
{
	struct tw686x_gpio_chip *tw686x_gpio = gpiochip_get_data(chip);
	u32 temp;

	mutex_lock(&tw686x_gpio->lock);
	temp = reg_read(tw686x_gpio->parent, tw686x_gpio_register_num(offset));
	temp &= ~tw686x_direction_bit(offset);
	if (direction)
		temp |= tw686x_direction_bit(offset);
	reg_write(tw686x_gpio->parent, tw686x_gpio_register_num(offset), temp);
	mutex_unlock(&tw686x_gpio->lock);

	return 0;
}


static int tw686x_direction_output(struct gpio_chip *chip, unsigned int offset,
				 int value)
{
	tw686x_set_gpio(chip, value, offset);
	return tw686x_set_direction(chip, 0, offset);
}

static int tw686x_direction_input(struct gpio_chip *chip, unsigned int offset)
{
	return tw686x_set_direction(chip, 1, offset);
}

static int tw686x_get_direction(struct gpio_chip *chip, unsigned int offset)
{
	struct tw686x_gpio_chip *tw686x_gpio = gpiochip_get_data(chip);
	int temp;

	mutex_lock(&tw686x_gpio->lock);
	temp = reg_read(tw686x_gpio->parent, tw686x_gpio_register_num(offset));
	printk( "get direction temp is 0x%x\n", temp );
	temp &= tw686x_direction_bit(offset);
	mutex_unlock(&tw686x_gpio->lock);
	
	return !!temp;
}

static int tw686x_get_value(struct gpio_chip *chip, unsigned int offset)
{
	struct tw686x_gpio_chip *tw686x_gpio = gpiochip_get_data(chip);
	int temp;

	mutex_lock(&tw686x_gpio->lock);
	temp = reg_read(tw686x_gpio->parent, tw686x_gpio_register_num(offset));
	temp &= tw686x_value_bit(offset);
	mutex_unlock(&tw686x_gpio->lock);
	
	return !!temp;
}

static void tw686x_set_value(struct gpio_chip *chip, unsigned int offset,
			   int value)
{
	tw686x_set_gpio(chip, value, offset);
}

static int gpio_tw686x_probe(struct platform_device *pdev)
{
	struct pci_dev *pcidev = to_pci_dev(pdev->dev.parent);
	struct tw686x_gpio_chip *tw686x_gpio;
	int ret;

	printk( "----------------------PROBE\n");
	tw686x_gpio = devm_kzalloc(&pdev->dev, sizeof(*tw686x_gpio), GFP_KERNEL);
	if (!tw686x_gpio)
		return -ENOMEM;


	mutex_init(&tw686x_gpio->lock);

	tw686x_gpio->parent = pci_get_drvdata(pcidev);
	tw686x_gpio->gpio_chip.label = "tw686x_gpio";
	tw686x_gpio->gpio_chip.parent = &pdev->dev;
	tw686x_gpio->gpio_chip.direction_output = tw686x_direction_output;
	tw686x_gpio->gpio_chip.direction_input = tw686x_direction_input;
	tw686x_gpio->gpio_chip.get_direction = tw686x_get_direction;
	tw686x_gpio->gpio_chip.get = tw686x_get_value;
	tw686x_gpio->gpio_chip.set = tw686x_set_value;
	tw686x_gpio->gpio_chip.base = -1;
	tw686x_gpio->gpio_chip.ngpio = 42;

	ret = devm_gpiochip_add_data(&pdev->dev,
				     &tw686x_gpio->gpio_chip, tw686x_gpio);
	if (ret)
		goto err_destroy;

	platform_set_drvdata(pdev, tw686x_gpio);

	return 0;

err_destroy:
	mutex_destroy(&tw686x_gpio->lock);
	return ret;

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

	mutex_destroy(&tw686x_gpio->lock);

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
