/*
 * 
 */

#ifndef XPLDRV_H
#define XPLDRV_H

#include <linux/ioctl.h>

#define DEVICE_NAME		    "xpldrv"
#define MAJOR_NUM           100

struct ioctl_reg
{
	unsigned long reg_addr;
	unsigned long reg_data;
};

#define IOCTL_XPL_PERI_WR 	_IOWR(MAJOR_NUM, 0, struct ioctl_reg)
#define IOCTL_XPL_PERI_RD 	_IOWR(MAJOR_NUM, 1, struct ioctl_reg)
#define IOCTL_DDR3_WR32     _IOWR(MAJOR_NUM, 2, struct ioctl_reg)
#define IOCTL_DDR3_RD32     _IOWR(MAJOR_NUM, 3, struct ioctl_reg)


#endif // XPLDRV_H
