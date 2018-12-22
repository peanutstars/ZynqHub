#ifndef __XPLIOCTL_H__
#define __XPLIOCTL_H__

int xpl_open();
int xpl_close();
int reg_read32(uint32_t offset, uint32_t* pData);
int reg_write32(uint32_t offset, uint32_t data);
int ddr_read32(uint32_t offset, uint32_t *pData);
int ddr_write32(uint32_t offset, uint32_t data);

#endif /* __XPLIOCTL_H__ */
