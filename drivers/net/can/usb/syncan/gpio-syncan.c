#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/usb.h>
#include <linux/slab.h>
#include <linux/gpio/driver.h>

#include "syncan-common.h"

#define GPIOSYNCAN_USB_DRIVER_NAME "syncan-gpio"
#define MAX_TX_URBS 6
#define MAX_RX_URBS 6

#define SYNCAN_GPIO_ENDPOINT	2
#define GPIO_EP_SIZE		16

/* Table of devices that work with this driver */
static struct usb_device_id syncan_gpio_usb_table[] = {
	{USB_DEVICE_INTERFACE_NUMBER(USB_SYN_VENDOR_ID, USB_SYN_PRODUCT_ID, 1)},
	{} /* Terminating entry */
};

struct gpio_data{
	u32 pin_value;
	u32 pin_direction;
};

struct syncan_gpio_priv;

struct syncan_urb_context {
	struct syncan_gpio_priv *priv;
	u8 echo_index;
};

struct syncan_gpio_priv{
	struct usb_device *udev;
	struct usb_interface *interface;

	struct gpio_chip chip;

	union syncan_configure_packet config_buffer;
	struct gpio_data current_setting;
	u8 num_gpio;

	/* USB info */
	struct usb_anchor tx_submitted;
	struct syncan_urb_context tx_contexts[MAX_TX_URBS];
	struct usb_anchor rx_submitted;
};

static int syncan_gpio_ctrl_command_out(struct syncan_gpio_priv *up,
				 u8 cmd, u16 subcmd, u16 datalen)
{
	return usb_control_msg(up->udev,
			       usb_sndctrlpipe(up->udev, 0),
			       cmd,
			       USB_DIR_OUT | USB_TYPE_VENDOR |
						USB_RECIP_DEVICE,
			       subcmd,
			       0,
			       &up->config_buffer,
			       datalen,
			       CTL_PIPE_TIMEOUT);
}

static int syncan_gpio_device_request_in(struct syncan_gpio_priv *up,
				  u8 cmd, u16 subcmd, u16 datalen)
{
	return usb_control_msg(up->udev,
			       usb_rcvctrlpipe(up->udev, 0),
			       cmd,
			       USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			       subcmd,
			       0,
			       &up->config_buffer,
			       datalen,
			       CTL_PIPE_TIMEOUT);
}

static int syncan_gpio_get_direction(struct gpio_chip *chip, unsigned offset)
{
	struct syncan_gpio_priv *priv;

	priv = gpiochip_get_data(chip);

	return !!(priv->current_setting.pin_direction & BIT(offset));
}

static void syncan_gpio_read_callback(struct urb *urb)
{
	struct gpio_data *gpio_data;
	struct syncan_gpio_priv *priv = urb->context;
	int ret;

	/* check URB status */
	switch (urb->status) {
	case 0:
		break;
	case -ENOENT:
	case -EPIPE:
	case -EPROTO:
	case -ESHUTDOWN:
	case -ETIME:
		/* urb is not resubmitted -> free dma data */
		usb_free_coherent(priv->udev,
				  GPIO_EP_SIZE,
				  urb->transfer_buffer,
				  urb->transfer_dma);
		//netdev_dbg(up->netdev, "not resubmitting urb; status: %d\n",
		//	   urb->status);
		return;
	default:
		goto resubmit;
	}

	gpio_data = (struct gpio_data*)(urb->transfer_buffer);
	priv->current_setting.pin_direction = gpio_data->pin_direction;

resubmit:
	/* resubmit urb when done */
	usb_fill_bulk_urb(urb, priv->udev,
			  usb_rcvbulkpipe(priv->udev,
					  SYNCAN_GPIO_ENDPOINT),
			  urb->transfer_buffer,
			  GPIO_EP_SIZE,
			  syncan_gpio_read_callback,
			  priv);

	usb_anchor_urb(urb, &priv->rx_submitted);
	ret = usb_submit_urb(urb, GFP_KERNEL);

	if (ret < 0) {
		usb_unanchor_urb(urb);
		usb_free_coherent(priv->udev,
				  GPIO_EP_SIZE,
				  urb->transfer_buffer,
				  urb->transfer_dma);
	}
}

static void syncan_gpio_write_gpio_callback(struct urb *urb)
{
	struct syncan_gpio_priv *priv;
	struct syncan_urb_context *context = urb->context;

	/* get the urb context */
	if (WARN_ON_ONCE(!context))
		return;

	/* free up our allocated buffer */
	usb_free_coherent(urb->dev,
			  sizeof(struct gpio_data),
			  urb->transfer_buffer,
			  urb->transfer_dma);

	priv = context->priv;
	if (WARN_ON_ONCE(!priv))
		return;

	/* transmission failed */
	if (urb->status) {
		dev_warn(&(priv->udev->dev),
			    "failed to transmit USB message to device: %d\n",
			     urb->status);
	}
	context->echo_index = MAX_TX_URBS;
}

static int syncan_gpio_write_data(struct syncan_gpio_priv *priv)
{
	struct urb *urb = NULL;
	struct gpio_data *data = NULL;
	struct syncan_urb_context *context = NULL;
	int i, ret;

	for (i = 0; i < MAX_TX_URBS; i++) {
		if (priv->tx_contexts[i].echo_index == MAX_TX_URBS) {
			context = &priv->tx_contexts[i];
			break;
		}
	}

	if (!context)
		goto nomem;

	urb = usb_alloc_urb(0, GFP_ATOMIC);
	if (!urb)
		goto nomem;

	data = usb_alloc_coherent(priv->udev,
			       sizeof(struct gpio_data),
			       GFP_ATOMIC,
			       &urb->transfer_dma);
	if (!data)
		goto nomem;

	/* build the urb */
	usb_fill_bulk_urb(urb, priv->udev,
			  usb_sndbulkpipe(priv->udev,
					  SYNCAN_GPIO_ENDPOINT),
			  data,
			  sizeof(struct gpio_data),
			  syncan_gpio_write_gpio_callback,
			  context);
	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	data->pin_value = priv->current_setting.pin_value;
	data->pin_direction = priv->current_setting.pin_direction;

	/* transmit it */
	usb_anchor_urb(urb, &priv->tx_submitted);
	ret = usb_submit_urb(urb, GFP_ATOMIC);

	if (ret) {
		/* on error, clean up */
		usb_unanchor_urb(urb);
		context->echo_index = MAX_TX_URBS;
	}

	/* release ref, as we do not need the urb anymore */
	usb_free_urb(urb);

	return ret;

nomem:
	if (urb)
		usb_free_urb(urb);
	return -ENOMEM;

}

static int syncan_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct syncan_gpio_priv *priv;

	priv = gpiochip_get_data(chip);

	priv->current_setting.pin_direction |= BIT(offset);

	return syncan_gpio_write_data(priv);
}

static int syncan_gpio_direction_output(struct gpio_chip *chip, unsigned offset, int value)
{
	struct syncan_gpio_priv *priv;

	priv = gpiochip_get_data(chip);

	priv->current_setting.pin_direction &= ~BIT(offset);
	if (value)
		priv->current_setting.pin_value |= BIT(offset);
	else
		priv->current_setting.pin_value &= ~BIT(offset);

	return syncan_gpio_write_data(priv);
}

static int syncan_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct syncan_gpio_priv *priv;

	priv = gpiochip_get_data(chip);

	return !!(priv->current_setting.pin_value & BIT(offset));
}

static void syncan_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct syncan_gpio_priv *priv;

	priv = gpiochip_get_data(chip);

	priv->current_setting.pin_direction &= ~BIT(offset);
	if (value)
		priv->current_setting.pin_value |= BIT(offset);
	else
		priv->current_setting.pin_value &= ~BIT(offset);

	syncan_gpio_write_data(priv);
}

static void syncan_gpio_cleanup_rx_urbs(struct syncan_gpio_priv *up, struct urb **urbs)
{
	int i;

	for (i = 0; i < MAX_RX_URBS; i++) {
		if (urbs[i]) {
			usb_unanchor_urb(urbs[i]);
			usb_free_coherent(up->udev,
					  GPIO_EP_SIZE,
					  urbs[i]->transfer_buffer,
					  urbs[i]->transfer_dma);
			usb_free_urb(urbs[i]);
		}
	}

	memset(urbs, 0, sizeof(*urbs) * MAX_RX_URBS);
}

static int syncan_gpio_prepare_and_anchor_rx_urbs(struct syncan_gpio_priv *priv,
					   struct urb **urbs)
{
	int i;

	memset(urbs, 0, sizeof(*urbs) * MAX_RX_URBS);

	for (i = 0; i < MAX_RX_URBS; i++) {
		void *buf;

		urbs[i] = usb_alloc_urb(0, GFP_KERNEL);
		if (!urbs[i])
			goto err;

		buf = usb_alloc_coherent(priv->udev,
					 GPIO_EP_SIZE,
					 GFP_KERNEL, &urbs[i]->transfer_dma);
		if (!buf) {
			/* cleanup this urb */
			usb_free_urb(urbs[i]);
			urbs[i] = NULL;
			goto err;
		}

		usb_fill_bulk_urb(urbs[i], priv->udev,
				  usb_rcvbulkpipe(priv->udev,
						  SYNCAN_GPIO_ENDPOINT),
				  buf,
				  GPIO_EP_SIZE,
				  syncan_gpio_read_callback,
				  priv);

		urbs[i]->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

		usb_anchor_urb(urbs[i], &priv->rx_submitted);
	}
	return 0;

err:
	/* cleanup other unsubmitted urbs */
	syncan_gpio_cleanup_rx_urbs(priv, urbs);
	return -ENOMEM;
}

static int syncan_gpio_submit_rx_urbs(struct syncan_gpio_priv *priv, 
		struct urb **urbs)
{
	int i, ret;

	/* Iterate over all urbs to submit. On success remove the urb
	 * from the list.
	 */
	for (i = 0; i < MAX_RX_URBS; i++) {
		ret = usb_submit_urb(urbs[i], GFP_KERNEL);
		if (ret) {
			dev_err(&(priv->udev->dev),
				   "could not submit urb; code: %d\n",
				   ret);
			goto err;
		}

		/* Anchor URB and drop reference, USB core will take
		 * care of freeing it
		 */
		usb_free_urb(urbs[i]);
		urbs[i] = NULL;
	}
	return 0;

err:
	/* Cleanup unsubmitted urbs */
	syncan_gpio_cleanup_rx_urbs(priv, urbs);

	/* Kill urbs that are already submitted */
	usb_kill_anchored_urbs(&priv->rx_submitted);

	return ret;
}

static void unlink_all_urbs(struct syncan_gpio_priv *priv)
{
	int i;

	usb_kill_anchored_urbs(&priv->rx_submitted);

	usb_kill_anchored_urbs(&priv->tx_submitted);

	for (i = 0; i < MAX_TX_URBS; i++)
		priv->tx_contexts[i].echo_index = MAX_TX_URBS;
}

static int syncan_gpio_usb_probe(struct usb_interface *intf,
                          const struct usb_device_id *id)
{
	struct syncan_gpio_priv *priv;
	int err, i, ret;
	struct urb *urbs[MAX_RX_URBS];

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		goto nomem;

	ret = 0;

	priv->udev = usb_get_dev(interface_to_usbdev(intf));
	priv->interface = intf;

	usb_set_intfdata(intf, priv);

	init_usb_anchor(&priv->rx_submitted);
	init_usb_anchor(&priv->tx_submitted);

	ret = syncan_gpio_device_request_in(priv, SYNCAN_GET_GPIO_INFO, 
		0, 
		sizeof(struct gpio_information));
	if (ret < 0)
		goto err_bad_dev;

	priv->num_gpio = priv->config_buffer.gpio_info.num_gpio;
	priv->chip.label = "syncan-gpio";
	priv->chip.base = -1;
	priv->chip.ngpio = priv->config_buffer.gpio_info.num_gpio;
	priv->chip.get_direction = syncan_gpio_get_direction;
	priv->chip.direction_input = syncan_gpio_direction_input;
	priv->chip.direction_output = syncan_gpio_direction_output;
	priv->chip.get = syncan_gpio_get;
	priv->chip.set = syncan_gpio_set;

	priv->chip.owner = THIS_MODULE;
	priv->chip.parent = &intf->dev;

	ret = gpiochip_add_data(&priv->chip, priv);
	if (ret < 0)
		goto err_bad_gpio;

	for (i = 0; i < MAX_TX_URBS; i++) {
		priv->tx_contexts[i].priv = priv;
		priv->tx_contexts[i].echo_index = MAX_TX_URBS;
	}

	/* Driver is ready to receive data. Submit RX URBS */
	ret = syncan_gpio_prepare_and_anchor_rx_urbs(priv, urbs);
	if (ret)
		goto err_bad_usb;
	ret = syncan_gpio_submit_rx_urbs(priv, urbs);
	if (ret)
		goto err_bad_usb;

	ret = syncan_gpio_ctrl_command_out(priv, SYNCAN_UPDATE_GPIO_STATUS, 0, 0);
	if (ret < 0)
		goto err_bad_dev;

	return ret;

nomem:
	ret = -ENOMEM;
	return ret;

err_bad_usb:
	unlink_all_urbs(priv);

err_bad_gpio:
err_bad_dev:
	kfree(priv);
	ret = -ENODEV;
	return ret;
}

static void syncan_gpio_usb_disconnect(struct usb_interface *intf)
{
	struct syncan_gpio_priv *priv = usb_get_intfdata(intf);

	usb_set_intfdata(intf, NULL);

	if (priv) {
		unlink_all_urbs(priv);
		gpiochip_remove(&priv->chip);
		kfree(priv);
	}
}

/* usb specific object needed to register this driver with the usb subsystem */
static struct usb_driver syncan_gpio_usb_driver = {
	.name = GPIOSYNCAN_USB_DRIVER_NAME,
	.disconnect = syncan_gpio_usb_disconnect,
	.probe = syncan_gpio_usb_probe,
	.id_table = syncan_gpio_usb_table,
};

static int __init syncan_gpio_usb_init(void)
{
	int err;

	/* register this driver with the USB subsystem */
	err = usb_register(&syncan_gpio_usb_driver);
	if (err)
		pr_err("%s: usb_register failed (err %d)\n",
			GPIOSYNCAN_USB_DRIVER_NAME, err);

	return err;
}

static void __exit syncan_gpio_usb_exit(void)
{
	/* deregister this driver with the USB subsystem */
	usb_deregister(&syncan_gpio_usb_driver);

	pr_info("%s: interfaces driver unloaded\n",
		GPIOSYNCAN_USB_DRIVER_NAME);
}

module_init(syncan_gpio_usb_init);
module_exit(syncan_gpio_usb_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Robert Middleton");
MODULE_DESCRIPTION("Synexxus USB To CAN converter(GPIO support)");
MODULE_VERSION("0.1");
