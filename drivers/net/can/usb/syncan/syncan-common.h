#ifndef SYNCAN_COMMON_H
#define SYNCAN_COMMON_H

/* vendor and product id */
#define USB_SYN_VENDOR_ID		0x04D8 /* Microchip */
#define USB_SYN_PRODUCT_ID		0xED91 /* Assigned by Microchip */

#define CTL_PIPE_TIMEOUT 1000

enum syncan_cmd {
	SYNCAN_RESET = 1,
	SYNCAN_OPEN,
	SYNCAN_CLOSE,
	SYNCAN_SET_SPEED,
	SYNCAN_GET_DEVICE_INFO,
	SYNCAN_ERASE_FLASH,
	SYNCAN_GET_GPIO_INFO,
	SYNCAN_UPDATE_GPIO_STATUS,
};

struct speed_configuration{
	u32 speed; /* Speed in khz */
	u32 flags; /* Flags for speed configuration */
	u8 tq1; /* Time quantum 1 (values: 1-8) */
	u8 tq2; /* Time quantum 2 (values: 1-8) */
	u8 propSegTQ; /* Propogation segment time quantum (values: 1-8) */
	u8 syncJumpWidth; /* Sync Jump Width (values: 1-4) */
};

struct device_information{
	u32 feature_support;
	u16 firmware_version;
};

struct gpio_information{
	u8 num_gpio;
	u8 gpio_flags;
	u16 reserved;
};

union syncan_configure_packet {
	u8 data[32];
	u8 open_mode;
	struct device_information device_info;
	struct speed_configuration speed_config;
	struct gpio_information gpio_info;
};


#endif
