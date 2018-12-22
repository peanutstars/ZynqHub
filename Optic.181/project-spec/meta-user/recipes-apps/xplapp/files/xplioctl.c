#include <inttypes.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include "xpldrv.h"
#include "xplioctl.h"
#include "debug.h"


#define XPL_DRIVER      "/dev/xpldrv"

static int xplfd = NOFD;


int xpl_open()
{
    int rv = IERROR;

    if (xplfd >= 0)
        return IOK;

    xplfd = open(XPL_DRIVER, O_RDWR);
    if (xplfd < 0) {
        ERR("Failed to open %s\n", XPL_DRIVER);
    } else {
        rv = IOK;
    }

    return rv;
}

int xpl_close()
{
    if (xplfd >= 0) {
        close(xplfd);
        xplfd = NOFD;
    }
    return IOK;
}

int reg_read32(uint32_t offset, uint32_t* pData)
{
    struct ioctl_reg reg;
    reg.reg_addr = offset;

    if (ioctl(xplfd, IOCTL_XPL_PERI_RD, &reg) != 0){
        return IERROR;
    }
    *pData = reg.reg_data;
    return IOK;
}


int reg_write32(uint32_t offset, uint32_t data)
{
    struct ioctl_reg reg;
    reg.reg_addr = offset;
    reg.reg_data = data;

    if (ioctl(xplfd, IOCTL_XPL_PERI_WR, &reg) != 0){
        return IERROR;
    }
    return IOK;
}

int ddr_read32(uint32_t offset, uint32_t *pData)
{
    struct ioctl_reg reg;
    reg.reg_addr = offset;

    if (ioctl(xplfd, IOCTL_DDR3_RD32, &reg) != 0){
        return IERROR;
    }
    *pData = reg.reg_data;
    return IOK;
}

int ddr_write32(uint32_t offset, uint32_t data)
{
    struct ioctl_reg reg;
    reg.reg_addr = offset;
    reg.reg_data = data;

    if (ioctl(xplfd, IOCTL_DDR3_WR32, &reg) != 0){
        return IERROR;
    }
    return IOK;
}

