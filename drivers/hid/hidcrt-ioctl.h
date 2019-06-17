#ifndef HIDCRT_IOCTL
#define HIDCRT_IOCTL

#include <linux/ioctl.h>

struct hidcrt_calibration_registers {
	u16 xcal1;
        u16 xcal2;
	u16 xcal3;
	u16 ycal1;
	u16 ycal2;
	u16 ycal3;
	u16 cxcal;
	u16 cycal;
};

#define HIDCRT_START_CALIBRATION	_IO('E',0)
#define HIDCRT_STOP_CALIBRATION		_IO('E',1)
#define HIDCRT_WAIT_CALIB_POINT		_IO('E',2)
#define HIDCRT_READ_REGISTERS		_IOR('E',3, struct hidcrt_calibration_registers)
#define HIDCRT_WRITE_REGISTERS		_IOW('E',4, struct hidcrt_calibration_registers)
#define HIDCRT_RESET_CALIB_DEFAULT	_IO('E',5)

#endif
