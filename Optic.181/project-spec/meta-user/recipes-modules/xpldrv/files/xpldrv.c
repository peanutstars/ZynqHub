
/*
 *  XPL Module Driver.
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/stat.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/ioport.h>
//#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include "xpldrv.h"
#include "debug.h"


MODULE_LICENSE("GPL");
MODULE_AUTHOR("HSLee");

#ifndef VM_RESERVED
#define VM_RESERVED  (VM_DONTEXPAND | VM_DONTDUMP)
#endif 

#define USE_XPL_PERI_BASE_ADDR		1
#define USE_XPL_DDR_BASE_ADDR       1

#if USE_XPL_PERI_BASE_ADDR
static unsigned long xpl_peri_base_addr     = 0x43000000;
static int xpl_peri_size                    = 0x00010000;
static void* xpl_peri_base                  = NULL;
#endif 
#if USE_XPL_DDR_BASE_ADDR
static unsigned long ddr3_base_addr         = 0x60000000;
static int ddr3_size                        = 0x00100000;
static void* ddr3_base                      = NULL;
#endif

module_param(xpl_peri_base_addr,  ulong, S_IRUSR);
MODULE_PARM_DESC(xpl_peri_base_addr, "xpl periperal base address");
#if USE_XPL_DDR_BASE_ADDR
module_param(ddr3_base_addr,  ulong, S_IRUSR);
MODULE_PARM_DESC(ddr3_base_addr, "ddr base address");
#endif

struct mmap_info 
{
	char *data;
	int reference;
};


/* 
 * This is called whenever a process attempts to open the device file 
 */
static int xpl_open(struct inode *inode, struct file *file)
{
	printk(KERN_INFO "device_open(%p)\n", file);
	
	try_module_get(THIS_MODULE);
	return 0;
}

static int xpl_release(struct inode *inode, struct file *file)
{
	printk(KERN_INFO "device_release(%p,%p)\n", inode, file);


	module_put(THIS_MODULE);
	return 0;
}

/* 
 * This function is called whenever a process which has already opened the
 * device file attempts to read from it.
 */
static ssize_t xpl_read(struct file *file,	/* see include/linux/fs.h   */
			   char __user * buffer,	/* buffer to be
							 * filled with data */
			   size_t length,	/* length of the buffer     */
			   loff_t * offset)
{
	return 0;
}

/* 
 * This function is called when somebody tries to
 * write into our device file. 
 */
static ssize_t
xpl_write(struct file *file,
	     const char __user * buffer, size_t length, loff_t * offset)
{
	return 0;
}

void mmap_open(struct vm_area_struct *vma)
{
	struct mmap_info *info = (struct mmap_info *)vma->vm_private_data;
	info->reference++;
}

void mmap_close(struct vm_area_struct *vma)
{
		struct mmap_info *info = (struct mmap_info*)vma->vm_private_data;
		info->reference--;
}
static int mmap_fault(struct vm_fault *vmf)
{
    struct vm_area_struct *vma = vmf->vma;
	struct page *page;
	struct mmap_info *info;
	info = (struct mmap_info *)vma->vm_private_data;
	if (!info->data)
	{
		printk("no data\n");
		return 0;
	}
	// get the page
	page = virt_to_page(info->data);
	
	get_page(page);
	vmf->page = page;
	
	return 0;
}

struct vm_operations_struct mmap_vm_ops = 
{
		.open  = mmap_open,
		.close = mmap_close,
		.fault = mmap_fault,
		//.nopage  = mmap_nopage,
};

int xpl_mmap(struct file *filep,  struct vm_area_struct *vma)
{
	vma->vm_ops = &mmap_vm_ops;
	vma->vm_flags |= VM_RESERVED;
	vma->vm_private_data = filep->private_data;
	mmap_open(vma);
	return 0;
}
/* 
 * This function is called whenever a process tries to do an ioctl on our
 * device file. We get two extra parameters (additional to the inode and file
 * structures, which all device functions get): the number of the ioctl called
 * and the parameter given to the ioctl function.
 *
 * If the ioctl is write or read/write (meaning output is returned to the
 * calling process), the ioctl call returns the output of this function.
 *
 */
long xpl_ioctl(
		 struct file *file,	/* ditto */
		 unsigned int ioctl_num,	/* number and param for ioctl */
		 unsigned long ioctl_param)
{
	struct ioctl_reg stReg;
    void __user *argp = (void __user *)ioctl_param;
	
	/* 
	 * Switch according to the ioctl called 
	 */
	switch (ioctl_num) 
	{
	#if USE_XPL_PERI_BASE_ADDR
		case IOCTL_XPL_PERI_WR:
			if (copy_from_user(&stReg, argp, sizeof(struct ioctl_reg))){
				return -EACCES;
            }
			writel(stReg.reg_data, xpl_peri_base + stReg.reg_addr);
			break;

		case IOCTL_XPL_PERI_RD:
			if (copy_from_user(&stReg, argp, sizeof(struct ioctl_reg))){
				return -EACCES;
            }
			DBG("Addr : 0x%08lX\n", stReg.reg_addr);
			mdelay(10);
			stReg.reg_data = readl(xpl_peri_base + stReg.reg_addr);
			if (copy_to_user(argp, &stReg, sizeof(struct ioctl_reg))) {
				return -EFAULT;
			}
			break;
    #endif //
    #if USE_XPL_DDR_BASE_ADDR
		case IOCTL_DDR3_WR32:
			if (copy_from_user(&stReg, argp, sizeof(struct ioctl_reg))){
				return -EACCES;
            }
			writel(stReg.reg_data, ddr3_base + stReg.reg_addr);
			break;

		case IOCTL_DDR3_RD32:
			if (copy_from_user(&stReg, argp, sizeof(struct ioctl_reg))){
				return -EACCES;
            }
			stReg.reg_data = readl(ddr3_base + stReg.reg_addr);
			if (copy_to_user(argp, &stReg, sizeof(struct ioctl_reg))) {
				return -EFAULT;
			}
			break;
    #endif

	}
	return 0;
}

/* 
 * This structure will hold the functions to be called
 * when a process does something to the device we
 * created. Since a pointer to this structure is kept in
 * the devices table, it can't be local to
 * init_module. NULL is for unimplemented functions. 
 */
const static struct file_operations xpldrv_fops = {
	.read           = xpl_read,
	.write          = xpl_write,
	.unlocked_ioctl = xpl_ioctl,
	.open           = xpl_open,
	.release        = xpl_release,
	.mmap           = xpl_mmap,
};

//static struct miscdevice xpldrv_dev = {
//	.minor          = MISC_DYNAMIC_MINOR,
//	.name           = DEVICE_NAME,
//	.fops           = &xpldrv_fops,
//};

static void check_vdma_version(void)
{
    uint32_t *version = (uint32_t *)(xpl_peri_base+0x2C);
    uint32_t regv = *version;
    printk(KERN_INFO "VDMA VERSION: %d.%d%d\n", 
            regv>>28, (regv>>24)&0xF, (regv>>20)&0xF);
}

static void check_ddr_memory(void)
{
    uint32_t *mem = (uint32_t *)ddr3_base;
    int i, limit = ddr3_size/16;
    int fg_fail = 0;
    for (i=0; i<limit; i++) {
        *(mem+i) = i;
    }
    for (i=0; i<limit; i++) {
        if (*(mem+i) != i) {
            printk(KERN_ERR "Memroy Error at %p:%p\n",
                    ddr3_base+(i<<2), mem+i);
            fg_fail = 1;
        }
    }
    if (fg_fail == 0) {
        printk(KERN_INFO "VIDEO Meory Okay.\n");
    }
}

static int __init xpl_init(void)
{
	int ret_val;
	struct resource * pstRes;

	/* 
	 * Register the character device (atleast try) 
	 */
//	ret_val = misc_register(&xpldrv_dev);
	ret_val = register_chrdev(MAJOR_NUM, DEVICE_NAME, &xpldrv_fops);

	/* 
	 * Negative values signify an error 
	 */
	if (ret_val < 0) {
		printk(KERN_ALERT "%s failed with %d\n",
		       "Sorry, registering the character device ", ret_val);
		return ret_val;
	}
	printk(KERN_INFO "XPL Driver\n=============\n");
#if USE_XPL_PERI_BASE_ADDR
/*
    // Already defined the address of the xpl peri area from vdma in the dts.
	printk(KERN_INFO "xpl_peri_base_addr = 0x%08lx\n", xpl_peri_base_addr);

	pstRes = request_mem_region(xpl_peri_base_addr, xpl_peri_size, DEVICE_NAME);
	if (pstRes == NULL) {
		printk(KERN_ERR "requst_mem_region(0x%08x, 0x%08x) failed\n", 
			(int)xpl_peri_base_addr, (int)xpl_peri_size);
		return -1;
	}
*/
	xpl_peri_base = ioremap(xpl_peri_base_addr, xpl_peri_size);
	if (xpl_peri_base == NULL)
		printk(KERN_ERR "xpl_peri_base ioremap failed\n");

    printk(KERN_INFO "XPL Base IOREMAP:%08X ->  %p\n", xpl_peri_base_addr, xpl_peri_base);
    check_vdma_version();

#endif
#if USE_XPL_DDR_BASE_ADDR
	pstRes = request_mem_region(ddr3_base_addr, ddr3_size, "pl_ddr3");
    if (pstRes == NULL) {
        printk(KERN_ERR "requst_mem_region(0x%08x, 0x%08x) failed\n", 
                (int)ddr3_base_addr, (int)ddr3_size);
        return -1;
    }
	ddr3_base = ioremap(ddr3_base_addr, ddr3_size);
	if (ddr3_base == NULL) {
		printk(KERN_ERR "ddr_base ioremap failed\n");
		release_mem_region(ddr3_base_addr, ddr3_size);
        return -1;
    }
	printk(KERN_INFO "ddr3_base = %p\n", ddr3_base);
	check_ddr_memory();
#endif 		
	return 0;
}

static void __exit xpl_exit(void)
{

	printk(KERN_INFO "Goodbye, XPL Driver\n");

	/* 
	 * Unregister the device 
	 */
	unregister_chrdev(MAJOR_NUM, DEVICE_NAME);
//	misc_deregister (&xpldrv_dev);
	
#if USE_XPL_DDR_BASE_ADDR
	if (ddr3_base)
	{
		iounmap(ddr3_base);
		release_mem_region(ddr3_base_addr, ddr3_size);
	}
#endif
	
#if USE_XPL_PERI_BASE_ADDR
	if (xpl_peri_base)
	{
		iounmap(xpl_peri_base);
		release_mem_region(xpl_peri_base_addr, xpl_peri_size);
        xpl_peri_base = NULL;
	}
#endif 
}


module_init(xpl_init);
module_exit(xpl_exit);

