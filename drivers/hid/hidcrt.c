#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/usb.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/completion.h>
#include <linux/uaccess.h>
#include <linux/proc_fs.h>

#include "hidcrt-ioctl.h"

#define HIDCRTOUCH_VENDOR_ID_FTDI 		0x0403
#define HIDCRTOUCH_PRODUCT_ID_SYN_FTDI 		0x6080
#define HIDCRTOUCH_PRODUCT_ID_SYN_CORRECT 	0x6080

#define FTDI_READ_LEN 	64

#define MAX_TX_URBS		10

/* TODO need to figure out why we need this */
#define CUSTOM_HR 1280
#define CUSTOM_VR 800

#define FTDI_RESET 		0x00
#define FTDI_MODEM_CTRL		0x01
#define FTDI_SET_FLOW_CTRL	0x02
#define FTDI_SET_BAUD		0x03
#define FTDI_SET_DATA		0x04
#define FTDI_GET_MODEM_STAT	0x05
#define FTDI_SET_EVENT_CHAR	0x06
#define FTDI_SET_ERROR_CHAR	0x07
#define FTDI_SET_LAT_TIMER	0x09
#define FTDI_GET_LAT_TIMER	0x0A
#define FTDI_SET_BIT_MODE	0x0B
#define FTDI_GET_BIT_MODE	0x0C

#define FTDI_DATA_IN_EP		0x01
#define FTDI_DATA_OUT_EP	0x02

#define HIDCRT_SOF		0xC0
#define HIDCRT_EOF		0xC1

#define HIDCRT_REG_RESIST_ERR	0x00
#define HIDCRT_REG_RESIST_STAT1	0x01
#define HIDCRT_REG_RESIST_STAT2	0x02
#define HIDCRT_REG_XPOS_MSB	0x03
#define HIDCRT_REG_XPOS_LSB	0x04
#define HIDCRT_REG_YPOS_MSB	0x05
#define HIDCRT_REG_YPOS_LSB	0x06
#define HIDCRT_REG_PRESS_MSB	0x07
#define HIDCRT_REG_PRESS_LSB	0x08
#define HIDCRT_REG_RESIST_FIFO_STAT	0x09
#define HIDCRT_REG_TRIGGER	0x41
#define HIDCRT_REG_HORIZ_RES	0x4A
#define HIDCRT_REG_XCALIB1_MSB	0x4F
#define HIDCRT_REG_XCALIB1_LSB	0x50
#define HIDCRT_REG_XCALIB2_MSB	0x51
#define HIDCRT_REG_XCALIB2_LSB	0x52
#define HIDCRT_REG_XCALIB3_MSB	0x53
#define HIDCRT_REG_XCALIB3_LSB	0x54
#define HIDCRT_REG_YCALIB1_MSB	0x55
#define HIDCRT_REG_YCALIB1_LSB	0x56
#define HIDCRT_REG_YCALIB2_MSB	0x57
#define HIDCRT_REG_YCALIB2_LSB	0x58
#define HIDCRT_REG_YCALIB3_MSB	0x59
#define HIDCRT_REG_YCALIB3_LSB	0x5A
#define HIDCRT_REG_XCALIB_CONST_MSB	0x5B
#define HIDCRT_REG_XCALIB_CONST_LSB	0x5C
#define HIDCRT_REG_YCALIB_CONST_MSB	0x5D
#define HIDCRT_REG_YCALIB_CONST_LSB	0x5E

#define RESISTIVE_TOUCHING	BIT(7)
#define CALIBRATION_MODE	BIT(6)

#define CALIBRATION_ERR		BIT(6)

#define SCREEN_MULTI_TOUCH 0x01
#define SCREEN_SINGLE_TOUCH 0x02
#define SCREEN_NO_SCREEN 0x00 

#define HIDCRT_MAX_BYTES_TRANSFER	(80)
#define HIDCRT_DATA_BUFFER_LEN		256
#define HIDCRT_MAX_REGISTER_NUM		0x71

#define HIDCRT_STATUS_POSITIVE_ACK		0xC2
#define HIDCRT_STATUS_SAMPLE_OUT_OF_RANGE	0xE0
#define HIDCRT_STATUS_INVALID_WRITE		0xE1
#define HIDCRT_STATUS_DATA_OUT_OF_RANGE		0xE2
#define HIDCRT_STATUS_REG_ADDR_OUT_OF_RANGE	0xE3
#define HIDCRT_STATUS_PARITY_ERROR		0xE5

#define CANADIAN_PACER_MODE_MANAGED		0
#define CANADIAN_PACER_MODE_UNMANAGED		1

struct hidcrt_priv{
	struct usb_device *udev;
	struct input_dev *inputdev;
	struct completion command_completion;

	struct usb_anchor tx_submitted;
	//struct syncan_urb_context tx_contexts[MAX_TX_URBS];
	struct usb_anchor rx_submitted;

	int number;
	u8 reg_values[HIDCRT_MAX_REGISTER_NUM];
	u8 response_status;
};

static int hidcrt_write_register(struct hidcrt_priv *priv, u8 reg_address, 
		u8* in_data, u8 len);
static int hidcrt_read_register(struct hidcrt_priv *priv, u8 reg_address, 
		u8 len);

#define HIDCRTOUCHL_MAX_DEVICES 50
static struct hidcrt_priv *extensions[HIDCRTOUCHL_MAX_DEVICES];
static struct proc_dir_entry *proc_entry;
static u8 hidcrt_last_touched;

static int hidcrtouchl_open(struct inode *inode, struct file *file)
{

        file->private_data = extensions[iminor(inode)];
        return 0;
}

static int hidcrtouchl_release(struct inode *inode, struct file *file)
{
        return 0;
}

static long hidcrtouchl_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct hidcrt_priv *priv = file->private_data;
	int ret;
	u8 reg_value;
	int touched;
	struct hidcrt_calibration_registers calib_regs;
	u8 register_write[16];

	if (!priv) {
		ret = -EINVAL;
		goto ioctl_done;
	}
	
	ret = 0;
	switch (cmd) {
	case HIDCRT_START_CALIBRATION:
		reg_value = CALIBRATION_MODE;
		hidcrt_write_register(priv, HIDCRT_REG_TRIGGER, &reg_value, 1);
		break;
	case HIDCRT_STOP_CALIBRATION:
		hidcrt_read_register(priv, HIDCRT_REG_RESIST_ERR, 4);
		if (priv->reg_values[HIDCRT_REG_RESIST_ERR] & CALIBRATION_ERR) {
			ret = -EAGAIN;
			printk( "calibration had error\n");
		} else {
			printk("calibration was good\n");
		}
		break;
	case HIDCRT_WAIT_CALIB_POINT:
		touched = 0;
		do {
			hidcrt_read_register(priv, HIDCRT_REG_RESIST_STAT1, 1);
			if (priv->reg_values[HIDCRT_REG_RESIST_STAT1] & 
				RESISTIVE_TOUCHING) {
				touched = 1;
			}
			if (!(priv->reg_values[HIDCRT_REG_RESIST_STAT1] & 
				RESISTIVE_TOUCHING) &&
				touched == 1) {
				touched = 2;
			}
		} while(touched != 2);
		
/*
			do {    
				sr1=ReadMMRegister(DevEx,0x01);
				if((sr1 & (1 << 7))) {
					touched=1;
				}
				if(!(sr1 & (1 << 7)) && touched){
					touched=2;
				}
			} while (touched!=2);
*/
		break;
	case HIDCRT_READ_REGISTERS:
		hidcrt_read_register(priv, HIDCRT_REG_XCALIB1_MSB, 16);
		calib_regs.xcal1 = priv->reg_values[HIDCRT_REG_XCALIB1_MSB] << 8 |
			priv->reg_values[HIDCRT_REG_XCALIB1_LSB];
		calib_regs.xcal2 = priv->reg_values[HIDCRT_REG_XCALIB2_MSB] << 8 |
			priv->reg_values[HIDCRT_REG_XCALIB2_LSB];
		calib_regs.xcal3 = priv->reg_values[HIDCRT_REG_XCALIB3_MSB] << 8 |
			priv->reg_values[HIDCRT_REG_XCALIB3_LSB];
		calib_regs.ycal1 = priv->reg_values[HIDCRT_REG_YCALIB1_MSB] << 8 |
			priv->reg_values[HIDCRT_REG_YCALIB1_LSB];
		calib_regs.ycal2 = priv->reg_values[HIDCRT_REG_YCALIB2_MSB] << 8 |
			priv->reg_values[HIDCRT_REG_YCALIB2_LSB];
		calib_regs.ycal3 = priv->reg_values[HIDCRT_REG_YCALIB3_MSB] << 8 |
			priv->reg_values[HIDCRT_REG_YCALIB3_LSB];
		calib_regs.cxcal = priv->reg_values[HIDCRT_REG_XCALIB_CONST_MSB] << 8 |
			priv->reg_values[HIDCRT_REG_XCALIB_CONST_LSB];
		calib_regs.cycal = priv->reg_values[HIDCRT_REG_YCALIB_CONST_MSB] << 8 |
			priv->reg_values[HIDCRT_REG_YCALIB_CONST_LSB];
		if (copy_to_user((void*)arg, (void*)&calib_regs, sizeof(calib_regs) ) != 0 ) {
			printk( "Unable to copy calib to user space\n" );
			ret = -EIO;
		}
		break;
	case HIDCRT_WRITE_REGISTERS:
		if (copy_from_user((void*)&calib_regs, (void*)arg, sizeof(calib_regs)) != 0) {
			printk( "Unable to copy calib from user space\n");
			ret = -EIO;
			goto ioctl_done;
		}
		register_write[0] = (calib_regs.xcal1 & 0xFF00) >> 8;
		register_write[1] = (calib_regs.xcal1 & 0x00FF);
		register_write[2] = (calib_regs.xcal2 & 0xFF00) >> 8;
		register_write[3] = (calib_regs.xcal2 & 0x00FF);
		register_write[4] = (calib_regs.xcal3 & 0xFF00) >> 8;
		register_write[5] = (calib_regs.xcal3 & 0x00FF);
		register_write[6] = (calib_regs.ycal1 & 0xFF00) >> 8;
		register_write[7] = (calib_regs.ycal1 & 0x00FF);
		register_write[8] = (calib_regs.ycal2 & 0xFF00) >> 8;
		register_write[9] = (calib_regs.ycal2 & 0x00FF);
		register_write[10] = (calib_regs.ycal3 & 0xFF00) >> 8;
		register_write[11] = (calib_regs.ycal3 & 0x00FF);
		register_write[12] = (calib_regs.cxcal & 0xFF00) >> 8;
		register_write[13] = (calib_regs.cxcal & 0x00FF);
		register_write[14] = (calib_regs.cycal & 0xFF00) >> 8;
		register_write[15] = (calib_regs.cycal & 0x00FF);
		hidcrt_write_register(priv, HIDCRT_REG_XCALIB1_MSB, register_write, 16);
		break;
	case HIDCRT_RESET_CALIB_DEFAULT:
	default:
		ret = -EINVAL;
	}

ioctl_done:
	return ret;
}

static ssize_t proc_read(struct file *file, char __user *userbuf, size_t len, loff_t *ppos){
	if (len < 1)
		return 0;

	if (*ppos == 1)
		return 0;

	if (copy_to_user(userbuf, &hidcrt_last_touched, 1))
		return -EFAULT;

	*ppos = 1;
	return 1;
}

static struct file_operations calib_fops = {
        .owner = THIS_MODULE,
        .open = hidcrtouchl_open,
        .release = hidcrtouchl_release,
        .unlocked_ioctl = hidcrtouchl_ioctl
};

static struct file_operations proc_fops = {
	.owner = THIS_MODULE,
	.read = proc_read,
};

static struct usb_class_driver calib_class = {
	.name = "hidcrtouchl_calib%d",
	.fops = &calib_fops,
	.minor_base = 0, /* This is probably ignored */
};

static int hidcrt_set_canadian_pacer_proc_mode(struct hidcrt_priv *priv, int mode)
{
        /* 
         * Write SOF 0x18 0x00 0x00 0x00 EOF for managed mode on the PIC
         * Write SOF 0x18 0x01 0x00 0x00 EOF for no managed mode on the PIC
	 *
	 * The PIC on the canadian pacer controls who gets the current touch
	 * information, since we can have multiple hosts plugged into one display,
	 * we only want the data to go to one of them at a time.
	 */
	u8 *write_buffer;
	int actual_length;
	int ret;

	write_buffer = kmalloc(6, GFP_KERNEL);
	if (!write_buffer)
		return -ENOMEM;

	write_buffer[0] = HIDCRT_SOF;
	write_buffer[1] = 0x18;
	if (mode == CANADIAN_PACER_MODE_MANAGED) {
		write_buffer[2] = 0x00;
	} else if (mode == CANADIAN_PACER_MODE_UNMANAGED) {
		write_buffer[2] = 0x01;
	}
	write_buffer[3] = 0x00;
	write_buffer[4] = 0x00;
	write_buffer[5] = HIDCRT_EOF;

	ret = usb_bulk_msg(priv->udev, 
		usb_sndbulkpipe(priv->udev,
			FTDI_DATA_OUT_EP),
		write_buffer,
		6,
		&actual_length,
		0);

	kfree(write_buffer);
	return ret;
}

static void hidcrt_do_mouse_from_registers(struct hidcrt_priv *priv)
{
	u16 ypos, xpos, stat;
	bool touching = !!(priv->reg_values[HIDCRT_REG_RESIST_STAT1] & 
		RESISTIVE_TOUCHING);
	
	ypos = (priv->reg_values[HIDCRT_REG_YPOS_MSB] << 8) |
		priv->reg_values[HIDCRT_REG_YPOS_LSB];
	xpos = (priv->reg_values[HIDCRT_REG_XPOS_MSB] << 8) |
		priv->reg_values[HIDCRT_REG_XPOS_LSB];
	stat = (priv->reg_values[HIDCRT_REG_PRESS_MSB] << 8) |
		priv->reg_values[HIDCRT_REG_PRESS_LSB];

	/* 
	 * For some reason, we sometimes get values of 0 for the xpos and ypos.
	 * Suspect bad firmware, but don't feel like investigating at this point.
	 * Ignore touch location if either xpos or ypos is 0.
	 */
	if (ypos == 0 || xpos == 0)
		goto input_done;

	if (touching) {
		input_report_abs(priv->inputdev, ABS_X, xpos);
		input_report_abs(priv->inputdev, ABS_Y, ypos);
		hidcrt_last_touched = priv->number;
	}

input_done:
	input_report_abs(priv->inputdev, ABS_MT_PRESSURE, touching);
	input_report_key(priv->inputdev, BTN_TOUCH, touching);
	input_sync(priv->inputdev);
	return;
}

/*
 * Process one read from the FTDI at a time.  This is not ideal(as we may loose some data),
 * however it seems that the firmware kinda sucks and gives us spurious data back.
 * This is what the original driver does, so it seems to work better.
 */
static void hidcrt_process_data_buffer(struct hidcrt_priv *priv, u8 *data, int data_len)
{
	int x;
	u8 *sof_location = NULL;
	int payload_size;
	int reg_addr;

	/* Find how long the packet to process is */
	for (x = 0; x < data_len; x++)
	{
		if (data[x] == HIDCRT_SOF){
			sof_location = data + x;
			break;
		}
	}

	if (sof_location == NULL) {
		goto resubmit;
	}

	reg_addr = (sof_location[1] << 4) | sof_location[2];
	payload_size = (sof_location[3] << 4) | sof_location[4];

	if (reg_addr < 0 || reg_addr > HIDCRT_MAX_REGISTER_NUM) {
		pr_err( "invalid register number %d\n", reg_addr );
		goto resubmit;
	}

	/* Skip address and payload size bytes */
	for (x = 5; x < (payload_size * 2) + 5; x += 2 ){
		priv->reg_values[reg_addr++] = (sof_location[x] << 4) | sof_location[x+1];
		if (reg_addr > HIDCRT_MAX_REGISTER_NUM)
			break;
	}

	complete(&priv->command_completion);
	hidcrt_do_mouse_from_registers(priv);

resubmit:
	return;
}

static void hidcrt_bulk_read_callback(struct urb *urb)
{
	struct hidcrt_priv *priv = urb->context;
	u8 *data;
	int ret;

	/* check URB status */
	switch (urb->status) {
	case 0:
		break;
	case -ENOENT:
	case -EPIPE:
	case -ESHUTDOWN:
		/* urb is not resubmitted -> free dma data */
		usb_free_coherent(priv->udev,
				  FTDI_READ_LEN,
				  urb->transfer_buffer,
				  urb->transfer_dma);
		return;
	default:
		goto resubmit;
	}

	data = urb->transfer_buffer;

	/* The FTDI send two status bytes at the start of the message */
	if (urb->actual_length > 2){
		/* Do any processing on our data that we need */
		hidcrt_process_data_buffer(priv, data, urb->actual_length - 2);
	}

resubmit:
	/* resubmit urb when done */
	usb_fill_bulk_urb(urb, priv->udev,
			  usb_rcvbulkpipe(priv->udev,
					  FTDI_DATA_IN_EP),
			  urb->transfer_buffer,
			  FTDI_READ_LEN,
			  hidcrt_bulk_read_callback,
			  priv);

	usb_anchor_urb(urb, &priv->rx_submitted);
	ret = usb_submit_urb(urb, GFP_KERNEL);

	if (ret < 0) {
		usb_unanchor_urb(urb);
		usb_free_coherent(priv->udev,
				  FTDI_READ_LEN,
				  urb->transfer_buffer,
				  urb->transfer_dma);
	}
}

static int hidcrt_start_bulk_read(struct hidcrt_priv* priv)
{
	struct urb *urb = NULL;
	u8 *data = NULL;
	int ret;

	urb = usb_alloc_urb(0, GFP_ATOMIC);
	if (!urb)
		goto nomem;

	data = usb_alloc_coherent(priv->udev,
			       FTDI_READ_LEN,
			       GFP_ATOMIC,
			       &urb->transfer_dma);
	if (!data)
		goto nomem;

	usb_fill_bulk_urb(urb, priv->udev,
			  usb_rcvbulkpipe(priv->udev,
					  FTDI_DATA_IN_EP),
			  data,
			  FTDI_READ_LEN,
			  hidcrt_bulk_read_callback,
			  priv);


	usb_anchor_urb(urb, &priv->rx_submitted);

	ret = usb_submit_urb(urb, GFP_KERNEL);
	if (ret)
		goto err;

	/* Anchor URB and drop reference, USB core will take
	 * care of freeing it
	 */
	usb_free_urb(urb);

	return 0;

err:
	usb_unanchor_urb(urb);
	usb_free_coherent(priv->udev,
			  FTDI_READ_LEN,
			  urb->transfer_buffer,
			  urb->transfer_dma);
nomem:
	if (urb)
		usb_free_urb(urb);

	return -ENOMEM;
}

static int hidcrt_device_out_vendor_transfer(struct hidcrt_priv* priv, 
		u8 request, 
		u16 value) {
	/*Send a control transfer*/
	return usb_control_msg (priv->udev,
		usb_sndctrlpipe(priv->udev, 0),
		request,
		USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_OUT,
		value,
		0,
		NULL,
		0,
		0);
}

static int hidcrt_device_in_vendor_transfer(struct hidcrt_priv* priv, 
	u8 request, u16 value, u16 *data) {
	return usb_control_msg(priv->udev,
		usb_rcvctrlpipe(priv->udev,0),
		request,
		USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_IN,
		value,
		0,
		(unsigned char *) data,
		2,
		0);
}

static int hidcrt_init_uart(struct hidcrt_priv* priv)
{
	if( hidcrt_device_out_vendor_transfer(priv, FTDI_RESET, 0) < 0 )
		goto ret_err;

	if( hidcrt_device_out_vendor_transfer(priv, FTDI_SET_LAT_TIMER, 0x10) < 0 )
		goto ret_err;

	if( hidcrt_device_out_vendor_transfer(priv, FTDI_SET_DATA, 0x108) < 0 )
		goto ret_err;

	if( hidcrt_device_out_vendor_transfer(priv, FTDI_SET_FLOW_CTRL, 0x0) < 0 )
		goto ret_err;

	if( hidcrt_device_out_vendor_transfer(priv, FTDI_SET_BAUD, 0x1A) < 0 )
		goto ret_err;

/*
	if( hidcrt_device_out_vendor_transfer(priv, FTDI_SET_EVENT_CHAR, 0x1A) < 0 )
		goto ret_err;
*/

	return 0;

ret_err:
	return -EIO;
/*
WatchError(hidcrtouchl_device_out_vendor_transfer(DevEx, 0x00, 0x00));
WatchError(hidcrtouchl_device_out_vendor_transfer(DevEx, 0x09, 0x10));
WatchError(hidcrtouchl_device_out_vendor_transfer(DevEx, 0x09, 0x10));
WatchError(hidcrtouchl_device_in_vendor_transfer(DevEx, 0x09, 0x10,&data));
WatchError(hidcrtouchl_device_out_vendor_transfer(DevEx, 0x04, 0x108));
WatchError(hidcrtouchl_device_out_vendor_transfer(DevEx, 0x01, 0x101));
WatchError(hidcrtouchl_device_out_vendor_transfer(DevEx, 0x01, 0x200));
WatchError(hidcrtouchl_device_out_vendor_transfer(DevEx, 0x02, 0x0));
WatchError(hidcrtouchl_device_out_vendor_transfer(DevEx, 0x03, 0x1a));
WatchError(hidcrtouchl_device_out_vendor_transfer(DevEx, 0x01, 0x200));
WatchError(hidcrtouchl_device_out_vendor_transfer(DevEx, 0x01, 0x100));
WatchError(hidcrtouchl_device_out_vendor_transfer(DevEx, 0x04, 0x108));
WatchError(hidcrtouchl_device_out_vendor_transfer(DevEx, 0x01, 0x100));
WatchError(hidcrtouchl_device_out_vendor_transfer(DevEx, 0x02, 0x00));
WatchError(hidcrtouchl_device_out_vendor_transfer(DevEx, 0x03, 0x1a));
WatchError(hidcrtouchl_device_out_vendor_transfer(DevEx, 0x01, 0x200));
WatchError(hidcrtouchl_device_out_vendor_transfer(DevEx, 0x01, 0x101));
WatchError(hidcrtouchl_device_out_vendor_transfer(DevEx, 0x04, 0x108));
WatchError(hidcrtouchl_device_out_vendor_transfer(DevEx, 0x02, 0x00));
WatchError(hidcrtouchl_device_out_vendor_transfer(DevEx, 0x01, 0x101));
WatchError(hidcrtouchl_device_out_vendor_transfer(DevEx, 0x06, 0x11A));
WatchError(hidcrtouchl_device_out_vendor_transfer(DevEx, 0x03, 0x1A));
WatchError(hidcrtouchl_device_out_vendor_transfer(DevEx, 0x01, 0x200));
WatchError(hidcrtouchl_device_out_vendor_transfer(DevEx, 0x01, 0x101));
WatchError(hidcrtouchl_device_out_vendor_transfer(DevEx, 0x04, 0x108));
WatchError(hidcrtouchl_device_out_vendor_transfer(DevEx, 0x02, 0x00));
WatchError(hidcrtouchl_device_out_vendor_transfer(DevEx, 0x01, 0x101));
WatchError(hidcrtouchl_device_out_vendor_transfer(DevEx, 0x06, 0x1a));
WatchError(hidcrtouchl_device_out_vendor_transfer(DevEx, 0x01, 0x100));
WatchError(hidcrtouchl_device_out_vendor_transfer(DevEx, 0x01, 0x200));
*/
}

static void hidcrt_write_data_callback(struct urb *urb)
{
	struct hidcrt_priv *priv = urb->context;

	/* get the urb context */
	if (WARN_ON_ONCE(!priv))
		return;

	/* free up our allocated buffer */
	usb_free_coherent(urb->dev,
			  HIDCRT_MAX_BYTES_TRANSFER,
			  urb->transfer_buffer,
			  urb->transfer_dma);

	/* transmission failed */
	if (urb->status) {
		dev_warn(&(priv->udev->dev),
			    "failed to transmit USB message to device: %d\n",
			     urb->status);
	}
}

static int hidcrt_read_register(struct hidcrt_priv *priv, u8 reg_address, 
		u8 len)
{
	int ret;
	int status;
	u8 *data;
	struct urb *urb = NULL;

	urb = usb_alloc_urb(0, GFP_ATOMIC);
	if (!urb)
		goto nomem;

	data = usb_alloc_coherent(priv->udev,
			       HIDCRT_MAX_BYTES_TRANSFER,
			       GFP_ATOMIC,
			       &urb->transfer_dma);
	if (!data)
		goto nomem;

	data[0] = HIDCRT_SOF;
	data[1] = (reg_address & 0xF0) >> 4;
	data[2] = reg_address & 0x0F;
	data[3] = (len & 0xF0) >> 4;
	data[4] = len & 0x0F;
	data[5] = HIDCRT_EOF;

	/* build the urb */
	usb_fill_bulk_urb(urb, priv->udev,
			  usb_sndbulkpipe(priv->udev,
					  FTDI_DATA_OUT_EP),
			  data,
			  6,
			  hidcrt_write_data_callback,
			  priv);
	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	/* transmit it */
	usb_anchor_urb(urb, &priv->tx_submitted);
	ret = usb_submit_urb(urb, GFP_ATOMIC);

	if (ret) {
		/* on error, clean up */
		usb_unanchor_urb(urb);
	}

	/* release ref, as we do not need the urb anymore */
	usb_free_urb(urb);

	/* Wait for our internal buffer to be updated */
	status = wait_for_completion_timeout(&priv->command_completion, 
			msecs_to_jiffies(300));
	
	if (status == 0)
		ret = -ETIMEDOUT;
	else if (status < 0)
		ret = status;
	else
		ret = priv->response_status;

	return ret;

nomem:
	if (urb)
		usb_free_urb(urb);
	return -ENOMEM;

}

static int hidcrt_write_register(struct hidcrt_priv *priv, u8 reg_address, 
		u8* in_data, u8 len)
{
	int x;
	int loc;
	int ret;
	int status;
	u8 *data;
	struct urb *urb = NULL;

	urb = usb_alloc_urb(0, GFP_ATOMIC);
	if (!urb)
		goto nomem;

	data = usb_alloc_coherent(priv->udev,
			       HIDCRT_MAX_BYTES_TRANSFER,
			       GFP_ATOMIC,
			       &urb->transfer_dma);
	if (!data)
		goto nomem;

	data[0] = HIDCRT_SOF;
	data[1] = (reg_address & 0xF0) >> 4;
	data[1] |= BIT(3); /* Indicates this is a write command */
	data[2] = reg_address & 0x0F;
	loc = 3;
	for( x = 0; x < len; x++ ){
		data[loc++] = (in_data[x] & 0xF0) >> 4;
		data[loc++] = (in_data[x] & 0x0F);
	}
	data[loc] = HIDCRT_EOF;

	/* build the urb */
	usb_fill_bulk_urb(urb, priv->udev,
			  usb_sndbulkpipe(priv->udev,
					  FTDI_DATA_OUT_EP),
			  data,
			  loc,
			  hidcrt_write_data_callback,
			  priv);
	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	/* transmit it */
	usb_anchor_urb(urb, &priv->tx_submitted);
	ret = usb_submit_urb(urb, GFP_ATOMIC);

	if (ret) {
		/* on error, clean up */
		usb_unanchor_urb(urb);
	}

	/* release ref, as we do not need the urb anymore */
	usb_free_urb(urb);

	status = wait_for_completion_timeout(&priv->command_completion, 
			msecs_to_jiffies(300));
	
	if (status == 0)
		ret = -ETIMEDOUT;
	else if (status < 0)
		ret = status;
	else
		ret = priv->response_status;

	return ret;

nomem:
	if (urb)
		usb_free_urb(urb);
	return -ENOMEM;

}

static int hidcrt_set_resolution(struct hidcrt_priv *priv, u16 xres, u16 yres)
{
	u8 data_bytes[4];

	xres = cpu_to_be16(xres);
	yres = cpu_to_be16(yres);

	data_bytes[0] = (xres & 0xFF00) >> 8;
	data_bytes[1] = (xres & 0x00FF);
	data_bytes[2] = (yres & 0xFF00) >> 8;
	data_bytes[3] = (yres & 0x00FF);

	return hidcrt_write_register(priv, HIDCRT_REG_HORIZ_RES, data_bytes, 4);
}

static int hidcrtouchl_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
	struct usb_device *usbdev = interface_to_usbdev(intf);
	struct hidcrt_priv *priv = NULL;
	size_t name_len;
	int ret = 0;
	int x;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		goto nomem;

	priv->udev = usbdev;
	usb_set_intfdata(intf, priv);
	
	init_completion(&priv->command_completion);
	init_usb_anchor(&priv->rx_submitted);
	init_usb_anchor(&priv->tx_submitted);

	if (hidcrt_init_uart(priv) < 0)
		goto err_init_uart;

	name_len = strlen(usbdev->product);
	priv->number = usbdev->product[name_len - 1];
	/* Ensure that this number is between ASCII 0 and 9 */
	if (priv->number < '0' || priv->number > '9')
		priv->number = 0;

	/* Initialize our input_dev information */
	priv->inputdev = input_allocate_device();
	if (priv->inputdev == NULL)
		goto nomem_input;

	priv->inputdev->name = "CRTOUCH Input Device";
        priv->inputdev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
        priv->inputdev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input_set_abs_params(priv->inputdev, ABS_X, 0, CUSTOM_HR, 0, 0);
        input_set_abs_params(priv->inputdev, ABS_Y, 0, CUSTOM_VR, 0, 0);
        input_set_abs_params(priv->inputdev, ABS_PRESSURE, 0, 1, 0, 0);

	/* Register our input device */
        ret = input_register_device(priv->inputdev);
	if (ret < 0) {
		input_free_device(priv->inputdev);
		goto err;
	}

	/* Register calibration device */
	ret = usb_register_dev(intf, &calib_class);
	if (ret < 0)
		goto err_usb_register;


	extensions[intf->minor] = priv;

	ret = hidcrt_start_bulk_read(priv);
	if (ret)
		goto err_usb_read;

	if (hidcrt_set_canadian_pacer_proc_mode(priv, CANADIAN_PACER_MODE_MANAGED) < 0 )
		goto err_usb_read;

	/* Set the resolution of the CRTouch */
	hidcrt_set_resolution(priv, CUSTOM_HR, CUSTOM_VR);

	/* Try a few times to read the resolution registers */
	for (x = 0; x < 10; x++) {
		ret = hidcrt_read_register(priv, HIDCRT_REG_HORIZ_RES, 4);
		if (ret < 0) {
			/* Can't read registers for some reason, try again */
			continue;
		} else {
			break;
		}
	}

	if (x == 10) {
		pr_warn( "could not get resolution" );
	}

	/* Now let's try and read our status registers */
	for (x = 0; x < 10; x++) {
		ret = hidcrt_read_register(priv, 0, 4);
		if (ret < 0) {
			/* Can't read registers for some reason, try again */
			continue;
		} else {
			break;
		}
	}

	if (x == 10) {
		pr_warn( "could not status register" );
	}

	switch (priv->reg_values[HIDCRT_REG_RESIST_STAT2] & 0x0F){
	case SCREEN_MULTI_TOUCH:
		pr_info("Multi-touch screen\n");
		break;
	case SCREEN_SINGLE_TOUCH:
		pr_info("Single-touch screen\n");
		break;
	case SCREEN_NO_SCREEN:
		pr_info("No screen detected!\n");
		break;
	}

	if (hidcrt_set_canadian_pacer_proc_mode(priv, CANADIAN_PACER_MODE_UNMANAGED) < 0 )
		goto err_usb_read;

	return 0;

nomem_input:
	kfree(priv);

nomem:
	return -ENOMEM;

err_usb_read:
	usb_deregister_dev(intf, &calib_class);

err_usb_register:
	input_unregister_device(priv->inputdev);

err_init_uart:
err:
	kfree(priv);
	return ret;
}

static void hidcrtouchl_disconnect(struct usb_interface *intf)
{
	struct hidcrt_priv *priv = usb_get_intfdata(intf);

	usb_set_intfdata(intf, NULL);

	if (priv) {
		complete_all(&priv->command_completion);
		usb_kill_anchored_urbs(&priv->rx_submitted);
		usb_kill_anchored_urbs(&priv->tx_submitted);
		input_unregister_device(priv->inputdev);
		usb_deregister_dev(intf, &calib_class);
		kfree(priv);
	}
}

static struct usb_device_id id_table [] = {
	{ USB_DEVICE(HIDCRTOUCH_VENDOR_ID_FTDI, HIDCRTOUCH_PRODUCT_ID_SYN_FTDI) },
	{ USB_DEVICE(HIDCRTOUCH_VENDOR_ID_FTDI, HIDCRTOUCH_PRODUCT_ID_SYN_CORRECT) },
	{ },
};

MODULE_DEVICE_TABLE (usb, id_table);

static struct usb_driver hidcrtouchl_driver = {
        .name           = "hidcrtouchl",
        .probe          = hidcrtouchl_probe,
        .disconnect     = hidcrtouchl_disconnect,
        .id_table       = id_table,
};

static int __init hidcrtouchl_init(void)
{
	int err;

	hidcrt_last_touched = -1;
	proc_entry = NULL;
	memset(extensions, 0, sizeof(extensions));
	err = usb_register(&hidcrtouchl_driver);
	if (err)
		pr_err( "hidcrt: usb register failed(err %d)\n",
			err);

	if (!err)
		proc_entry = proc_create( "hidcrt_last_touched", 0660, NULL, &proc_fops);

	return err;
}

static void hidcrtouchl_exit(void){
	usb_deregister(&hidcrtouchl_driver);
	proc_remove(proc_entry);
}

module_init(hidcrtouchl_init);
module_exit(hidcrtouchl_exit);

MODULE_AUTHOR("Robert Middleton <rmiddleton@synexxus.com>");
MODULE_DESCRIPTION("CRTouch HID over FTDI");
MODULE_LICENSE("GPL");
