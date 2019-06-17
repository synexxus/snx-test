#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/usb.h>

#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/can/error.h>
#include <linux/can/led.h>

#include "syncan-common.h"

#define SYNCAN_USB_DRIVER_NAME "syncan"
#define MAX_RX_URBS			20
#define MAX_TX_URBS			20
#define RX_BUFFER_SIZE			64
#define SYNCAN_CMD_TIMEOUT		1000

/* DLC field contains both the data length and RTR/IDE bits */
#define DLC_DATA_LEN_MASK		0x0F
#define DLC_RTR_BIT			BIT(4)
#define DLC_IDE_BIT			BIT(5)

#define ERRFLAG_RX_WARNING		BIT(0)
#define ERRFLAG_RX_BUS_PASSIVE		BIT(1)
#define ERRFLAG_TX_WARNING		BIT(2)
#define ERRFLAG_TX_BUS_PASSIVE		BIT(3)
#define ERRFLAG_TX_BUS_OFF		BIT(4)
#define ERRFLAG_RX_OVERFLOW		BIT(5)
#define ERRFLAG_TX_OVERFLOW		BIT(6)

#define CONFIGURE_CMD_PHASE2_TIMESEL	BIT(0)
#define CONFIGURE_CMD_SAMPLE3TIME	BIT(1)
#define CONFIGURE_CMD_DEFAULT_QUANTA	BIT(2)

#define OPEN_NORMAL		0
#define OPEN_LOOPBACK 		1
#define OPEN_LISTEN_ONLY 	2

#define FEATURE_SUPPORT_TX_ALLOWED      (0x01 << 0)
#define FEATURE_SUPPORT_FIRMWARE_UPDATE (0x01 << 1)
#define FEATURE_SUPPORT_INTERNAL_BUILD  (0x01 << 2)
#define FEATURE_SUPPORT_RELEASE_BUILD   (0x01 << 3)

/* Endpoints */
#define SYNCAN_DATA_ENDPOINT	1
#define DATA_EP_SIZE		64
#define CONFIG_EP_SIZE		32

struct syncan_priv;

struct syncan_urb_context {
	struct syncan_priv *priv;
	u32 echo_index;
	u8 dlc;
};

struct __packed syncan_rx_frame {
	__le32 reserved0; /* Timestamp potentially */
	__le32 can_id; /* EID + SID */
	u8 data[8];
	u8 dlc; /* upper 4 bits contain flags */
	u8 errflags;
	u8 reserved1;
	u8 reserved2;
};

struct __packed syncan_tx_frame {
	__le32 reserved0;
	__le32 can_id; /* EID + SID */
	u8 data[8];
	u8 dlc; /* Upper 4 bits contain flags */
	u8 reserved1;
	u8 reserved2;
	u8 reserved3;
};

struct syncan_priv {
	struct can_priv can; /* must be the first member */

	struct usb_device *udev;
	struct net_device *netdev;

	atomic_t active_tx_urbs;
	struct usb_anchor tx_submitted;
	struct syncan_urb_context tx_contexts[MAX_TX_URBS];

	struct usb_anchor rx_submitted;

	struct can_berr_counter bec;

	union syncan_configure_packet config_buffer;

	spinlock_t echo_skb_lock;
};

/* Table of devices that work with this driver */
static struct usb_device_id syncan_usb_table[] = {
	{USB_DEVICE_INTERFACE_NUMBER(USB_SYN_VENDOR_ID, USB_SYN_PRODUCT_ID, 0)},
	{} /* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, syncan_usb_table);

static const struct can_bittiming_const syncan_bittiming_const = {
	.name = "syncan",
	.tseg1_min = 1,
	.tseg1_max = 8,
	.tseg2_min = 1,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 128,
	.brp_inc = 2,
};

static int syncan_ctrl_command_out(struct syncan_priv *up,
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

static int syncan_device_request_in(struct syncan_priv *up,
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

static int syncan_open_cmd(struct syncan_priv* priv, int open_type)
{
	priv->config_buffer.open_mode = open_type;
	return syncan_ctrl_command_out(priv, SYNCAN_OPEN, 0, 1);
}

static int syncan_close_cmd(struct syncan_priv* priv)
{
	return syncan_ctrl_command_out(priv, SYNCAN_CLOSE, 0, 0);
}

static void syncan_rx_can_err_msg(struct syncan_priv *priv,
				struct syncan_rx_frame *rx_frame)
{
	struct can_frame *cf;
	struct sk_buff *skb;
	struct net_device_stats *stats = &priv->netdev->stats;
	int newstate = CAN_STATE_ERROR_ACTIVE;

	skb = alloc_can_err_skb(priv->netdev, &cf);
	if (!skb)
		return;

	if (rx_frame->errflags & ERRFLAG_RX_WARNING) {
		cf->can_id |= CAN_ERR_CRTL;
		cf->data[1] |= CAN_ERR_CRTL_RX_WARNING;
		newstate = CAN_STATE_ERROR_WARNING;
	}
	if( rx_frame->errflags & ERRFLAG_RX_BUS_PASSIVE ){
		cf->can_id |= CAN_ERR_CRTL;
		cf->data[1] |= CAN_ERR_CRTL_RX_PASSIVE;
		newstate = CAN_STATE_ERROR_PASSIVE;
	}
	if( rx_frame->errflags & ERRFLAG_TX_WARNING ){
		cf->can_id |= CAN_ERR_CRTL;
		cf->data[1] |= CAN_ERR_CRTL_TX_WARNING;
		newstate = CAN_STATE_ERROR_WARNING;
	}
	if( rx_frame->errflags & ERRFLAG_TX_BUS_PASSIVE ){
		cf->can_id |= CAN_ERR_CRTL;
		cf->data[1] |= CAN_ERR_CRTL_TX_PASSIVE;
		newstate = CAN_STATE_ERROR_PASSIVE;
	}
	if( rx_frame->errflags & ERRFLAG_TX_BUS_OFF ){
		cf->can_id |= CAN_ERR_BUSOFF;
		newstate = CAN_STATE_BUS_OFF;
	}
	if( rx_frame->errflags & ERRFLAG_RX_OVERFLOW ){
		cf->can_id |= CAN_ERR_CRTL;
		cf->data[1] |= CAN_ERR_CRTL_RX_OVERFLOW;
	}

	priv->can.state = newstate;

	priv->bec.txerr = rx_frame->data[0];
	priv->bec.rxerr = rx_frame->data[1];

	stats->rx_packets++;
	stats->rx_bytes += cf->can_dlc;
	netif_rx(skb);
}

static void syncan_rx_can_msg(struct syncan_priv *priv,
				struct syncan_rx_frame *rx_frame)
{
	struct can_frame *cf;
	struct sk_buff *skb;
	struct net_device_stats *stats = &priv->netdev->stats;

	if (rx_frame->errflags) {
		/* Error message frame */
		syncan_rx_can_err_msg(priv, rx_frame);
	}else {
		skb = alloc_can_skb(priv->netdev, &cf);
		if (!skb)
			return;

		cf->can_dlc = rx_frame->dlc & DLC_DATA_LEN_MASK;
		if (rx_frame->dlc & DLC_IDE_BIT) {
			u32 tmp_id = le32_to_cpu(rx_frame->can_id);
			cf->can_id = (tmp_id & CAN_SFF_MASK) << 18;
			cf->can_id |= (tmp_id & (0x3FFFF << 11)) >> 11;
			cf->can_id |= CAN_EFF_FLAG;
		} else {
			cf->can_id = le32_to_cpu(rx_frame->can_id & CAN_SFF_MASK);
		}

		if (rx_frame->dlc & DLC_RTR_BIT)
			cf->can_id |= CAN_RTR_FLAG;

		memcpy(cf->data, rx_frame->data, cf->can_dlc & DLC_DATA_LEN_MASK);

		stats->rx_packets++;
		stats->rx_bytes += (cf->can_dlc & DLC_DATA_LEN_MASK);
		netif_rx(skb);
	}
}

static void syncan_read_bulk_callback(struct urb *urb)
{
	int ret;
	int pos;
	struct syncan_priv *up = urb->context;
	struct net_device *netdev = up->netdev;

	if (!netif_device_present(netdev))
		return;

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
		usb_free_coherent(up->udev,
				  DATA_EP_SIZE,
				  urb->transfer_buffer,
				  urb->transfer_dma);
		netdev_dbg(up->netdev, "not resubmitting urb; status: %d\n",
			   urb->status);
		return;
	default:
		goto resubmit;
	}

	/* sanity check */
	if (!netif_device_present(netdev))
		return;

	/* iterate over input */
	pos = 0;
	while (pos < urb->actual_length) {
		struct syncan_rx_frame *rx_frame;

		rx_frame = (struct syncan_rx_frame *)(urb->transfer_buffer + pos);

		syncan_rx_can_msg(up, rx_frame);

		pos += sizeof(struct syncan_rx_frame);
	}

resubmit:
	/* resubmit urb when done */
	usb_fill_bulk_urb(urb, up->udev,
			  usb_rcvbulkpipe(up->udev,
					  SYNCAN_DATA_ENDPOINT),
			  urb->transfer_buffer,
			  DATA_EP_SIZE,
			  syncan_read_bulk_callback,
			  up);

	usb_anchor_urb(urb, &up->rx_submitted);
	ret = usb_submit_urb(urb, GFP_KERNEL);

	if (ret < 0) {
		netdev_err(up->netdev,
			   "failed resubmitting read bulk urb: %d\n",
			   ret);

		usb_unanchor_urb(urb);
		usb_free_coherent(up->udev,
				  DATA_EP_SIZE,
				  urb->transfer_buffer,
				  urb->transfer_dma);

		if (ret == -ENODEV)
			netif_device_detach(netdev);
	}
}

static void syncan_cleanup_rx_urbs(struct syncan_priv *up, struct urb **urbs)
{
	int i;

	for (i = 0; i < MAX_RX_URBS; i++) {
		if (urbs[i]) {
			usb_unanchor_urb(urbs[i]);
			usb_free_coherent(up->udev,
					  DATA_EP_SIZE,
					  urbs[i]->transfer_buffer,
					  urbs[i]->transfer_dma);
			usb_free_urb(urbs[i]);
		}
	}

	memset(urbs, 0, sizeof(*urbs) * MAX_RX_URBS);
}

static int syncan_prepare_and_anchor_rx_urbs(struct syncan_priv *priv,
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
					 DATA_EP_SIZE,
					 GFP_KERNEL, &urbs[i]->transfer_dma);
		if (!buf) {
			/* cleanup this urb */
			usb_free_urb(urbs[i]);
			urbs[i] = NULL;
			goto err;
		}

		usb_fill_bulk_urb(urbs[i], priv->udev,
				  usb_rcvbulkpipe(priv->udev,
						  SYNCAN_DATA_ENDPOINT),
				  buf,
				  DATA_EP_SIZE,
				  syncan_read_bulk_callback,
				  priv);

		urbs[i]->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

		usb_anchor_urb(urbs[i], &priv->rx_submitted);
	}
	return 0;

err:
	/* cleanup other unsubmitted urbs */
	syncan_cleanup_rx_urbs(priv, urbs);
	return -ENOMEM;
}

static int syncan_submit_rx_urbs(struct syncan_priv *up, struct urb **urbs)
{
	int i, ret;

	/* Iterate over all urbs to submit. On success remove the urb
	 * from the list.
	 */
	for (i = 0; i < MAX_RX_URBS; i++) {
		ret = usb_submit_urb(urbs[i], GFP_KERNEL);
		if (ret) {
			netdev_err(up->netdev,
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
	syncan_cleanup_rx_urbs(up, urbs);

	/* Kill urbs that are already submitted */
	usb_kill_anchored_urbs(&up->rx_submitted);

	return ret;
}

static int syncan_open(struct net_device *netdev)
{
	struct syncan_priv *priv = netdev_priv(netdev);
	int err;
	int ret;
	struct urb *urbs[MAX_RX_URBS];
	int openmode = OPEN_NORMAL;

	ret = syncan_prepare_and_anchor_rx_urbs(priv, urbs);
	if (ret)
		return ret;
		//goto err_contexts;

	/* common open */
	err = open_candev(netdev);
	if (err)
		return err;

	//can_led_event(netdev, CAN_LED_EVENT_OPEN);

	if (priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK)
                openmode = OPEN_LOOPBACK;
	if (priv->can.ctrlmode & CAN_CTRLMODE_LISTENONLY)
                openmode = OPEN_LISTEN_ONLY;

	err = syncan_open_cmd(priv, openmode);
	if (err < 0) {
		if (err == -ENODEV)
			netif_device_detach(priv->netdev);

		netdev_warn(netdev, "couldn't start device: %d\n",
			 err);

		close_candev(netdev);

		return err;
	}

	/* Driver is ready to receive data. Submit RX URBS */
	ret = syncan_submit_rx_urbs(priv, urbs);
	if (ret)
		return ret;

	netif_start_queue(netdev);
	priv->can.state = CAN_STATE_ERROR_ACTIVE;

	return 0;
}

static void unlink_all_urbs(struct syncan_priv *priv)
{
	int i;

	usb_kill_anchored_urbs(&priv->rx_submitted);

	usb_kill_anchored_urbs(&priv->tx_submitted);
	//atomic_set(&priv->active_tx_urbs, 0);

	for (i = 0; i < MAX_TX_URBS; i++)
		priv->tx_contexts[i].echo_index = MAX_TX_URBS;
}

static int syncan_close(struct net_device *netdev)
{
	struct syncan_priv *priv = netdev_priv(netdev);
	int err = 0;

	/* Send CLOSE command to CAN controller */
	err = syncan_close_cmd(priv);
	if (err < 0)
		netdev_warn(netdev, "couldn't stop device");

	priv->can.state = CAN_STATE_STOPPED;

	netif_stop_queue(netdev);

	/* Stop polling */
	unlink_all_urbs(priv);

	close_candev(netdev);

	//can_led_event(netdev, CAN_LED_EVENT_STOP);

	return err;
}

static void syncan_write_txframe_callback(struct urb *urb)
{
	struct syncan_priv *priv;
	struct syncan_urb_context *context = urb->context;
	struct net_device *netdev;
	unsigned long flags;

	/* get the urb context */
	if (WARN_ON_ONCE(!context))
		return;

	/* free up our allocated buffer */
	usb_free_coherent(urb->dev,
			  sizeof(struct syncan_tx_frame),
			  urb->transfer_buffer,
			  urb->transfer_dma);

	priv = context->priv;
	if (WARN_ON_ONCE(!priv))
		return;

	/* sanity check */
	if (!netif_device_present(priv->netdev))
		return;

	netdev = priv->netdev;
	netdev->stats.tx_packets++;
	netdev->stats.tx_bytes += context->dlc;

	/* transmission failed (USB - the device will not send a TX complete) */
	if (urb->status) {
		netdev_warn(priv->netdev,
			    "failed to transmit USB message to device: %d\n",
			     urb->status);

		/* update counters an cleanup */
		spin_lock_irqsave(&priv->echo_skb_lock, flags);
		can_free_echo_skb(priv->netdev, context->echo_index);
		spin_unlock_irqrestore(&priv->echo_skb_lock, flags);

		priv->netdev->stats.tx_dropped++;

		/* release context and restart the queue if necessary */
		//if (!ucan_release_context(up, context))
		//	netdev_err(up->netdev,
		//		   "urb failed, failed to release context\n");
	}
	context->echo_index = MAX_TX_URBS;
}

static struct urb *syncan_prepare_tx_urb(struct syncan_priv *up,
				       struct syncan_urb_context *context,
				       struct can_frame *cf)
{
	struct urb *urb;
	struct syncan_tx_frame *m;

	/* create a URB, and a buffer for it, and copy the data to the URB */
	urb = usb_alloc_urb(0, GFP_ATOMIC);
	if (!urb) {
		netdev_err(up->netdev, "no memory left for URBs\n");
		return NULL;
	}

	m = usb_alloc_coherent(up->udev,
			       sizeof(struct syncan_tx_frame),
			       GFP_ATOMIC,
			       &urb->transfer_dma);
	if (!m) {
		netdev_err(up->netdev, "no memory left for USB buffer\n");
		usb_free_urb(urb);
		return NULL;
	}

	/* build the USB message */
	m->reserved0 = 0;
	m->reserved1 = 0;
	m->reserved2 = 0;
	m->reserved3 = 0;
	m->dlc = cf->can_dlc;
	m->data[0] = cf->data[0];
	m->data[1] = cf->data[1];
	m->data[2] = cf->data[2];
	m->data[3] = cf->data[3];
	m->data[4] = cf->data[4];
	m->data[5] = cf->data[5];
	m->data[6] = cf->data[6];
	m->data[7] = cf->data[7];
	m->can_id = cf->can_id & CAN_SFF_MASK;
	if (cf->can_id & CAN_EFF_FLAG) {
		m->dlc |= DLC_IDE_BIT;
		m->can_id = (cf->can_id & (CAN_SFF_MASK << 18)) >> 18;
		m->can_id |= (cf->can_id & (0x3FFFF)) << 11;
	}
	if (cf->can_id & CAN_RTR_FLAG) {
		m->dlc |= DLC_RTR_BIT;
	}

	context->dlc = cf->can_dlc;

	/* build the urb */
	usb_fill_bulk_urb(urb, up->udev,
			  usb_sndbulkpipe(up->udev,
					  SYNCAN_DATA_ENDPOINT),
			  m,
			  sizeof(struct syncan_tx_frame),
			  syncan_write_txframe_callback,
			  context);
	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	return urb;
}

static netdev_tx_t syncan_start_xmit(struct sk_buff *skb,
				      struct net_device *netdev)
{
	struct net_device_stats *stats = &netdev->stats;
	struct urb *urb;
	struct syncan_urb_context *context = NULL;
	struct can_frame *cf = (struct can_frame *)skb->data;
	struct syncan_priv *priv = netdev_priv(netdev);
	int ret, i;
	unsigned long flags;
	u8 echo_index;

	if (can_dropped_invalid_skb(netdev, skb))
		return NETDEV_TX_OK;

	for (i = 0; i < MAX_TX_URBS; i++) {
		if (priv->tx_contexts[i].echo_index == MAX_TX_URBS) {
			context = &priv->tx_contexts[i];
			break;
		}
	}

	if (!context)
		goto nomem;

	context->echo_index = i;
	echo_index = i;

	/* prepare urb for transmission */
	urb = syncan_prepare_tx_urb(priv, context, cf);
	if (!urb)
		goto nomem;

	/* put the skb on can loopback stack */
	spin_lock_irqsave(&priv->echo_skb_lock, flags);
	can_put_echo_skb(skb, priv->netdev, echo_index);
	spin_unlock_irqrestore(&priv->echo_skb_lock, flags);

	/* transmit it */
	usb_anchor_urb(urb, &priv->tx_submitted);
	ret = usb_submit_urb(urb, GFP_ATOMIC);

	/* cleanup urb */
	if (ret) {
		/* on error, clean up */
		usb_unanchor_urb(urb);
		//ucan_clean_up_tx_urb(up, urb);
		//if (!ucan_release_context(up, context))
		//	netdev_err(up->netdev,
		//		   "xmit err: failed to release context\n");

		/* remove the skb from the echo stack - this also
		 * frees the skb
		 */
		spin_lock_irqsave(&priv->echo_skb_lock, flags);
		can_free_echo_skb(priv->netdev, context->echo_index);
		spin_unlock_irqrestore(&priv->echo_skb_lock, flags);

		context->echo_index = MAX_TX_URBS;

		if (ret == -ENODEV) {
			netif_device_detach(priv->netdev);
		} else {
			netdev_warn(priv->netdev,
				    "xmit err: failed to submit urb %d\n",
				    ret);
			priv->netdev->stats.tx_dropped++;
		}
		return NETDEV_TX_OK;
	}

	netif_trans_update(netdev);

	/* release ref, as we do not need the urb anymore */
	usb_free_urb(urb);

	return NETDEV_TX_OK;

nomem:
	if (urb)
		usb_free_urb(urb);
	dev_kfree_skb(skb);
	stats->tx_dropped++;

	return NETDEV_TX_OK;
}

static const struct net_device_ops syncan_netdev_ops = {
	.ndo_open = syncan_open,
	.ndo_stop = syncan_close,
	.ndo_start_xmit = syncan_start_xmit,
	.ndo_change_mtu = can_change_mtu,
};

static int syncan_set_mode(struct net_device *netdev, enum can_mode mode)
{
	struct syncan_priv *priv = netdev_priv(netdev);
	int err = 0;
	int openmode = OPEN_NORMAL;

	if (!priv)
		return -ENODEV;

	if (priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK)
                openmode = OPEN_LOOPBACK;
	if (priv->can.ctrlmode & CAN_CTRLMODE_LISTENONLY)
                openmode = OPEN_LISTEN_ONLY;

	switch (mode) {
	case CAN_MODE_START:
		err = syncan_open_cmd(priv, openmode);
		break;
	case CAN_MODE_STOP:
		err = syncan_close_cmd(priv);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return err;
}

static int syncan_get_berr_counter(const struct net_device *netdev,
				     struct can_berr_counter *bec)
{
	struct syncan_priv *priv = netdev_priv(netdev);

	bec->txerr = priv->bec.txerr;
	bec->rxerr = priv->bec.rxerr;

	return 0;
}

static int syncan_set_bittiming(struct net_device *netdev)
{
	struct syncan_priv *priv = netdev_priv(netdev);
	struct can_bittiming* timing;

	timing = &priv->can.bittiming;

	priv->config_buffer.speed_config.speed = cpu_to_le32(timing->bitrate);
	priv->config_buffer.speed_config.flags = CONFIGURE_CMD_PHASE2_TIMESEL;
	if (priv->can.ctrlmode & CAN_CTRLMODE_3_SAMPLES) 
		priv->config_buffer.speed_config.flags |= CONFIGURE_CMD_SAMPLE3TIME;
	priv->config_buffer.speed_config.tq1 = timing->phase_seg1 & 0xFF;
	priv->config_buffer.speed_config.tq2 = timing->phase_seg2 & 0xFF;
	priv->config_buffer.speed_config.propSegTQ = timing->prop_seg & 0xFF;
	priv->config_buffer.speed_config.syncJumpWidth = timing->sjw & 0xFF;

	return syncan_ctrl_command_out(priv, 
		SYNCAN_SET_SPEED, 0, 
		sizeof(priv->config_buffer.speed_config) );
}

static int syncan_usb_probe(struct usb_interface *intf,
                          const struct usb_device_id *id)
{
	struct usb_device *usbdev = interface_to_usbdev(intf);
	struct net_device *netdev;
	struct syncan_priv *priv;
	int err, i;

	netdev = alloc_candev(sizeof(struct syncan_priv), MAX_TX_URBS);
	if (!netdev) {
		dev_err(&intf->dev, "Couldn't alloc candev\n");
		return -ENOMEM;
	}

	priv = netdev_priv(netdev);

	priv->udev = usbdev;
	priv->netdev = netdev;

	priv->can.state = CAN_STATE_STOPPED;
	priv->can.clock.freq = 80000000 / 2;
	priv->can.bittiming_const = &syncan_bittiming_const;
	priv->can.do_set_bittiming = syncan_set_bittiming;
	priv->can.do_set_mode = syncan_set_mode;
	priv->can.do_get_berr_counter = syncan_get_berr_counter;
	priv->can.ctrlmode_supported = CAN_CTRLMODE_LOOPBACK |
				      CAN_CTRLMODE_LISTENONLY;

	netdev->netdev_ops = &syncan_netdev_ops;

	spin_lock_init(&priv->echo_skb_lock);

	init_usb_anchor(&priv->rx_submitted);

	init_usb_anchor(&priv->tx_submitted);
	atomic_set(&priv->active_tx_urbs, 0);

	for (i = 0; i < MAX_TX_URBS; i++) {
		priv->tx_contexts[i].priv = priv;
		priv->tx_contexts[i].echo_index = MAX_TX_URBS;
	}

/*
	priv->cmd_msg_buffer = devm_kzalloc(&intf->dev, sizeof(struct usb_8dev_cmd_msg),
					    GFP_KERNEL);
	if (!priv->cmd_msg_buffer)
		goto cleanup_candev;
*/

	usb_set_intfdata(intf, priv);

	SET_NETDEV_DEV(netdev, &intf->dev);

	//mutex_init(&priv->usb_8dev_cmd_lock);

	err = register_candev(netdev);
	if (err) {
		netdev_err(netdev,
			"couldn't register CAN device: %d\n", err);
		goto cleanup_candev;
	}


	err = syncan_device_request_in(priv, SYNCAN_GET_DEVICE_INFO, 0,
		sizeof(struct device_information));
	if (err < 0) {
		netdev_err(netdev, "can't get firmware version\n");
		goto cleanup_unregister_candev;
	}

	netdev_info(netdev, "firmware version: %d.%d.",
		(priv->config_buffer.device_info.firmware_version & 0xFF00) >> 8,
		(priv->config_buffer.device_info.firmware_version & 0x00FF) );
	if (priv->config_buffer.device_info.feature_support &
		FEATURE_SUPPORT_INTERNAL_BUILD) {
		netdev_info(netdev, "  Internal firmware build." );
	} else {
		netdev_info(netdev, "  Release firmware build." );
	}

	if (priv->config_buffer.device_info.feature_support & FEATURE_SUPPORT_TX_ALLOWED) {
		netdev_info(netdev, "  TX Allowed.");
	} else {
		netdev_info(netdev, "  RX only.");
	}

	return 0;

cleanup_unregister_candev:
	unregister_netdev(priv->netdev);

cleanup_candev:
	free_candev(netdev);

	return err;
}

static void syncan_usb_disconnect(struct usb_interface *intf)
{
	struct syncan_priv *priv = usb_get_intfdata(intf);

	usb_set_intfdata(intf, NULL);

	if (priv) {
		netdev_info(priv->netdev, "device disconnected\n");

		unregister_netdev(priv->netdev);
		free_candev(priv->netdev);
		unlink_all_urbs(priv);
	}
}

/* usb specific object needed to register this driver with the usb subsystem */
static struct usb_driver syncan_usb_driver = {
	.name = SYNCAN_USB_DRIVER_NAME,
	.disconnect = syncan_usb_disconnect,
	.probe = syncan_usb_probe,
	.id_table = syncan_usb_table,
};


static int __init syncan_usb_init(void)
{
	int err;

	/* register this driver with the USB subsystem */
	err = usb_register(&syncan_usb_driver);
	if (err)
		pr_err("%s: usb_register failed (err %d)\n",
			SYNCAN_USB_DRIVER_NAME, err);

	return err;
}

static void __exit syncan_usb_exit(void)
{
	/* deregister this driver with the USB subsystem */
	usb_deregister(&syncan_usb_driver);

	pr_info("%s: interfaces driver unloaded\n",
		SYNCAN_USB_DRIVER_NAME);
}

module_init(syncan_usb_init);
module_exit(syncan_usb_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Robert Middleton");
MODULE_DESCRIPTION("Synexxus USB To CAN converter");
MODULE_VERSION("0.1");

