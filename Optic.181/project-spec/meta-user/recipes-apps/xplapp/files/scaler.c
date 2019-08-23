
#include <stdio.h>
#include <string.h>
#include "xilinx/xiltypes.h"
#include "xplioctl.h"
#include "debug.h"
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <linux/fb.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#define HPS_SCALER_BASE_ADDR            	          	(0x82000000)

#define HPS_SCALER0_IO32_BASE_ADDR            	    	(0x00000000)
#define HPS_SCALER1_IO32_BASE_ADDR            	    	(0x00040000)
#define HPS_SCALER2_IO32_BASE_ADDR            	    	(0x00080000)


#define HPS_SCALER0_IO_VERTICAL_SCALE_FACTOR_REG	  	(volatile u32*)(HPS_SCALER0_IO32_BASE_ADDR + (0x0000<<2))
#define HPS_SCALER0_IO_VERTICAL_LANCZOS_COEFF_REG    	(volatile u32*)(HPS_SCALER0_IO32_BASE_ADDR + (0x4000<<2))
#define HPS_SCALER0_IO_HORIZONTAL_SCALE_FACTOR0_REG  	(volatile u32*)(HPS_SCALER0_IO32_BASE_ADDR + (0x8000<<2))
#define HPS_SCALER0_IO_HORIZONTAL_SCALE_FACTOR1_REG  	(volatile u32*)(HPS_SCALER0_IO32_BASE_ADDR + (0xA000<<2))
#define HPS_SCALER0_IO_HORIZONTAL_LANCZOS_COEFF_REG  	(volatile u32*)(HPS_SCALER0_IO32_BASE_ADDR + (0xC000<<2))
#define HPS_SCALER0_IO_HV_SCALER_CONTROL_REG		    (volatile u32*)(HPS_SCALER0_IO32_BASE_ADDR + (0xE000<<2))

#define HPS_SCALER1_IO_VERTICAL_SCALE_FACTOR_REG	  	(volatile u32*)(HPS_SCALER1_IO32_BASE_ADDR + (0x0000<<2))
#define HPS_SCALER1_IO_VERTICAL_LANCZOS_COEFF_REG    	(volatile u32*)(HPS_SCALER1_IO32_BASE_ADDR + (0x4000<<2))
#define HPS_SCALER1_IO_HORIZONTAL_SCALE_FACTOR0_REG  	(volatile u32*)(HPS_SCALER1_IO32_BASE_ADDR + (0x8000<<2))
#define HPS_SCALER1_IO_HORIZONTAL_SCALE_FACTOR1_REG  	(volatile u32*)(HPS_SCALER1_IO32_BASE_ADDR + (0xA000<<2))
#define HPS_SCALER1_IO_HORIZONTAL_LANCZOS_COEFF_REG  	(volatile u32*)(HPS_SCALER1_IO32_BASE_ADDR + (0xC000<<2))
#define HPS_SCALER1_IO_HV_SCALER_CONTROL_REG		    (volatile u32*)(HPS_SCALER1_IO32_BASE_ADDR + (0xE000<<2))

#define HPS_SCALER2_IO_VERTICAL_SCALE_FACTOR_REG	  	(volatile u32*)(HPS_SCALER2_IO32_BASE_ADDR + (0x0000<<2))
#define HPS_SCALER2_IO_VERTICAL_LANCZOS_COEFF_REG    	(volatile u32*)(HPS_SCALER2_IO32_BASE_ADDR + (0x4000<<2))
#define HPS_SCALER2_IO_HORIZONTAL_SCALE_FACTOR0_REG  	(volatile u32*)(HPS_SCALER2_IO32_BASE_ADDR + (0x8000<<2))
#define HPS_SCALER2_IO_HORIZONTAL_SCALE_FACTOR1_REG  	(volatile u32*)(HPS_SCALER2_IO32_BASE_ADDR + (0xA000<<2))
#define HPS_SCALER2_IO_HORIZONTAL_LANCZOS_COEFF_REG  	(volatile u32*)(HPS_SCALER2_IO32_BASE_ADDR + (0xC000<<2))
#define HPS_SCALER2_IO_HV_SCALER_CONTROL_REG		    (volatile u32*)(HPS_SCALER2_IO32_BASE_ADDR + (0xE000<<2))


// The bit map of Vertical Scale Factor Register
#define HPS_SCALER_VSF_START_LINE   (0x200)
#define HPS_SCALER_VSF_LAST_LINE    (0x100)
#define HPS_SCALER_VSF_LOAD_LINE    (0x080)
#define HPS_SCALER_VSF_SHIFT_LINE   (0x040)
#define HPS_SCALER_VSF_VALID_LINE   (0x020)
#define HPS_SCALER_VSF_FRACTION     (0x01F)

// The bit map of Horizontal Scale Factor Register
#define HPS_SCALER_HSF_START_PIXEL  (0x001)
#define HPS_SCALER_HSF_LAST_PIXEL   (0x002)
#define HPS_SCALER_HSF_LOAD_PIXEL   (0x004)
#define HPS_SCALER_HSF_SHIFT_PIXEL  (0x008)
#define HPS_SCALER_HSF_CTRL_DONE    (0x010)

#define HPS_SCALER_HSF_VALID_PIXEL  (0x001)
#define HPS_SCALER_HSF_FRACTION     (0x01F)

//  The bit map of Scaler Control Register
#define HPS_SCALER_CTRL_VER_RESET   (0x001)
#define HPS_SCALER_CTRL_HOR_RESET   (0x002)
// FULL_RANGE => 1 : RGB(0~255), YUV(0~255)
//               0 : RGB(16~235), Y(16~235), UV(16~240)
#define HPS_SCALER_CTRL_FULL_RANGE  (0x010)

#define HPS_SCALE_BIT	            (16)
#define HPS_FRAC_BIT                (5)


#define SCALER1_DEVICE_ID			0
#define SCALER2_DEVICE_ID			1

static uint32_t *spdAddr;
static int      spdfd;
static size_t vlength = 0x1000000;

void 	HPS_WriteReg(volatile unsigned int *offset, unsigned int data);
u32 	HPS_ReadReg(volatile unsigned int *offset);
void 	HPS_Reset(int Ch, int reset);
void 	HPS_SetPixelDataRamge(int range);
void 	HPS_BitShiftLeft (u32* data1, u32* data0, int shift);
int 	HPS_ScaleDown(int Ch, int HOR_IN_RES,  int HOR_IN_CSTART, int HOR_IN_CSIZE,
 					  int VER_IN_RES,  int VER_IN_CSTART, int VER_IN_CSIZE,
 					  int HOR_OUT_RES, int VER_OUT_RES,   int PIXEL_PER_CLOCK);

int 	HPS_ScaleUp(int Ch, int HOR_IN_RES,  int HOR_IN_CSTART, int HOR_IN_CSIZE,
 					int VER_IN_RES,  int VER_IN_CSTART, int VER_IN_CSIZE,
 					int HOR_OUT_RES, int VER_OUT_RES,   int PIXEL_PER_CLOCK);

void	HPS_ScaleUpDown(int Ch, int SrcW, int SrcH, int DstW, int DstH);
int	 	HPS_ChangeRes(u16 inW, u16 inH, u16 outW, u16 outH);

int 	HPS_Init();

//--------------------------------------------------------------------------------------------------------------
void HPS_Open()
{
	spdfd = open("/dev/mem", O_RDWR | O_SYNC);
  	if (spdfd < 0) {
      	fprintf(stderr, "%s", strerror(errno));
  	}

  	spdAddr = mmap(NULL, vlength, PROT_READ | PROT_WRITE, MAP_SHARED, spdfd, HPS_SCALER_BASE_ADDR);
  	// memset(spdAddr, 0x00, vlength / 4);
  	xil_printf("[SCALER] : HPS Open %x\n", spdAddr);
}
//--------------------------------------------------------------------------------------------------------------
void HPS_Close()
{
	
  	munmap(spdAddr, vlength);
  	close(spdfd);
  	xil_printf("[SCALER] : HPS close\n");
}
//--------------------------------------------------------------------------------------------------------------
void HPS_WriteReg (volatile unsigned int *offset, unsigned int data)
{
  	// xil_printf("HPS write (%x) : %x\n", (unsigned)offset, data);
  	*(spdAddr + ((unsigned int )offset/4)) = data;

	//*(volatile unsigned int *)offset = data;
}
//--------------------------------------------------------------------------------------------------------------
u32 HPS_ReadReg (volatile unsigned int *offset)
{
	//return (*(volatile unsigned int *)offset);
  	// xil_printf("HPS read %x (%x)\n", (unsigned int)spdAddr, (unsigned)offset);
  	return *(spdAddr + ((unsigned int)offset/4));
}
//--------------------------------------------------------------------------------------------------------------
void HPS_Reset(int Ch, int reset)
{
	u32 Data;

	if(Ch == 1)
	{
		Data  =   HPS_ReadReg(HPS_SCALER1_IO_HV_SCALER_CONTROL_REG);
		Data &= ~(HPS_SCALER_CTRL_VER_RESET | HPS_SCALER_CTRL_HOR_RESET);

		if (reset)
			HPS_WriteReg(HPS_SCALER1_IO_HV_SCALER_CONTROL_REG, Data | HPS_SCALER_CTRL_VER_RESET | HPS_SCALER_CTRL_HOR_RESET);
		else
			HPS_WriteReg(HPS_SCALER1_IO_HV_SCALER_CONTROL_REG, Data);

	}
	else
	{

		Data  =   HPS_ReadReg(HPS_SCALER0_IO_HV_SCALER_CONTROL_REG);
		Data &= ~(HPS_SCALER_CTRL_VER_RESET | HPS_SCALER_CTRL_HOR_RESET);

		if (reset)
			HPS_WriteReg(HPS_SCALER0_IO_HV_SCALER_CONTROL_REG, Data | HPS_SCALER_CTRL_VER_RESET | HPS_SCALER_CTRL_HOR_RESET);
		else
			HPS_WriteReg(HPS_SCALER0_IO_HV_SCALER_CONTROL_REG, Data);
	}
}
//--------------------------------------------------------------------------------------------------------------
void HPS_SetPixelDataRange(int range)
{
	u32 Data;

	Data = HPS_ReadReg(HPS_SCALER0_IO_HV_SCALER_CONTROL_REG);

	if (range == 0)		// 0 : RGB(16~235), Y(16~235), UV(16~240)
		HPS_WriteReg(HPS_SCALER0_IO_HV_SCALER_CONTROL_REG, Data & ~HPS_SCALER_CTRL_FULL_RANGE);
	else				// 1 : RGB(0~255), YUV(0~255)
		HPS_WriteReg(HPS_SCALER0_IO_HV_SCALER_CONTROL_REG, Data |  HPS_SCALER_CTRL_FULL_RANGE);

}
//--------------------------------------------------------------------------------------------------------------
void HPS_BitShiftLeft (u32* data1, u32* data0, int shift)
{
	*data1 <<= shift;
	*data1  |= (*data0 >> (32-shift)) & ~(-1 << shift);
	*data0 <<= shift;
}
//--------------------------------------------------------------------------------------------------------------
int HPS_ScaleDown (int Ch, int HOR_IN_RES, int HOR_IN_CSTART, int HOR_IN_CSIZE,
 					int VER_IN_RES, int VER_IN_CSTART, int VER_IN_CSIZE,
 					int HOR_OUT_RES, int VER_OUT_RES, int PIXEL_PER_CLOCK)
{
    unsigned int HOR_SCF, VER_SCF, acc;
    unsigned int *io_addr;
    unsigned int *io_addr1;
    unsigned int io_data;
    unsigned int io_data1;


    int i, j, yco, iacc, ico, xoc, xsc, xic, align, pzx;
    int xoc_max;
    int line_bgn, line_end, load_pxd, shift_pxd, ctrl_done;
    int pzx_msel[4];
    int frac_val[4];
    int frac_vld[4];
    int pxd_vld[4];
    int pi_cnt, pl_cnt, li_cnt;

    HOR_SCF = (HOR_IN_RES << HPS_SCALE_BIT) / HOR_OUT_RES;
    VER_SCF = (VER_IN_RES << HPS_SCALE_BIT) / VER_OUT_RES;


	xil_printf("[Scaler] : %d Scaler Down [%d %d %d] [%d %d %d] [%d %d %d]\r\n",
		Ch, HOR_IN_RES,  HOR_IN_CSTART, HOR_IN_CSIZE,
		VER_IN_RES,  VER_IN_CSTART, VER_IN_CSIZE,
		HOR_OUT_RES, VER_OUT_RES,   PIXEL_PER_CLOCK);

	///////////////////////////////
    // Vertical Scale Factor setup
    ///////////////////////////////

    if(Ch == 1)
		io_addr = HPS_SCALER1_IO_VERTICAL_SCALE_FACTOR_REG;
	else
		io_addr = HPS_SCALER0_IO_VERTICAL_SCALE_FACTOR_REG;

    // Pre-skip lines
    io_data = HPS_SCALER_VSF_LOAD_LINE | HPS_SCALER_VSF_SHIFT_LINE;

    for(i = 0; i < VER_IN_CSTART; i++)
    {
        HPS_WriteReg(io_addr++, io_data);
    }

    // preload 2 lines
    li_cnt = 0;

    io_data = HPS_SCALER_VSF_LOAD_LINE | HPS_SCALER_VSF_SHIFT_LINE;

    for(i = 0; i < 2; i++)
    {
        HPS_WriteReg(io_addr++, io_data);
        li_cnt++;
    }

    acc = 0;
    yco = 0;

    for (i = 0; i < VER_OUT_RES; i++)
    {
        iacc = acc >> (HPS_SCALE_BIT - HPS_FRAC_BIT);
        ico  = acc >> HPS_SCALE_BIT;

        while (yco != ico)
        {
            io_data = ((li_cnt < (VER_IN_RES - VER_IN_CSTART)) ? HPS_SCALER_VSF_LOAD_LINE : 0)
                    | HPS_SCALER_VSF_SHIFT_LINE;

            HPS_WriteReg(io_addr++, io_data);
            yco++;
            li_cnt++;
        }

        io_data = ((i == 0) ? HPS_SCALER_VSF_START_LINE : 0)
        	    | ((i == VER_OUT_RES-1) ? HPS_SCALER_VSF_LAST_LINE : 0)
        	    | ((li_cnt < (VER_IN_RES-VER_IN_CSTART)) ? HPS_SCALER_VSF_LOAD_LINE : 0)
        	    | HPS_SCALER_VSF_SHIFT_LINE
        	    | HPS_SCALER_VSF_VALID_LINE
        	    | (iacc & HPS_SCALER_VSF_FRACTION);

        HPS_WriteReg(io_addr++, io_data);
        acc += VER_SCF;
        yco++;
        li_cnt++;
    }

    // Skip Lines
    if (VER_IN_RES > (VER_IN_CSTART+li_cnt))
    {
    	io_data = HPS_SCALER_VSF_LOAD_LINE | HPS_SCALER_VSF_SHIFT_LINE;
    	for (i = 0; i < VER_IN_RES-(VER_IN_CSTART+li_cnt); i++)
    	{
			HPS_WriteReg(io_addr++, io_data);
        	li_cnt++;
    	}
    }

    // done flag
    io_data = 0;
    HPS_WriteReg(io_addr++, io_data);

	//////////////////////////////////
    // Horizontal Scale Down Factor setup
	//////////////////////////////////
    if(Ch == 1)
	{
		io_addr  = HPS_SCALER1_IO_HORIZONTAL_SCALE_FACTOR0_REG;
		io_addr1 = HPS_SCALER1_IO_HORIZONTAL_SCALE_FACTOR1_REG;
	}else
	{
		io_addr  = HPS_SCALER0_IO_HORIZONTAL_SCALE_FACTOR0_REG;
		io_addr1 = HPS_SCALER0_IO_HORIZONTAL_SCALE_FACTOR1_REG;
	}

    pl_cnt = 0;

    // Pre-skip pixels
    io_data  = HPS_SCALER_HSF_LOAD_PIXEL | HPS_SCALER_HSF_SHIFT_PIXEL;
    io_data1 = 0;

    for (i = 0; i < (HOR_IN_CSTART+PIXEL_PER_CLOCK-1)/PIXEL_PER_CLOCK; i++)
    {
    	HPS_WriteReg(io_addr++,  io_data);
        if (PIXEL_PER_CLOCK == 4)
        	HPS_WriteReg(io_addr1++, io_data1);
        pl_cnt += PIXEL_PER_CLOCK;
    }

    pi_cnt = (((HOR_IN_CSTART+PIXEL_PER_CLOCK-1)/PIXEL_PER_CLOCK)*PIXEL_PER_CLOCK)-HOR_IN_CSTART;
    pzx = pi_cnt;

    // preload 2 pixels
    io_data  = HPS_SCALER_HSF_LOAD_PIXEL | HPS_SCALER_HSF_SHIFT_PIXEL;
    io_data1 = 0;

    for(i = 0; i < 2/PIXEL_PER_CLOCK; i++)
    {
        HPS_WriteReg (io_addr++,  io_data);
        if (PIXEL_PER_CLOCK == 4)
        	HPS_WriteReg (io_addr1++, io_data1);
        pi_cnt += PIXEL_PER_CLOCK;
        pl_cnt += PIXEL_PER_CLOCK;
    }

    acc = 0;
    xic = 0;
    xoc = 0;
    align = 0;
    ctrl_done = 0;
    xoc_max = ((HOR_OUT_RES+PIXEL_PER_CLOCK-1)/PIXEL_PER_CLOCK)*PIXEL_PER_CLOCK;

	while (xoc < xoc_max)
    {
    	line_bgn  = 0;
    	line_end  = 0;
    	load_pxd  = 0;
    	shift_pxd = 0;

    	for(i = 0; i < 4; i++)
    	{
    		pzx_msel[i] = 0;
    		frac_vld[i] = 0;
    		frac_val[i] = 0;
    		pxd_vld[i]	= 0;
    	}

    	for (i = PIXEL_PER_CLOCK-1; i >= 0; i--)
    	{
    		if (xoc < xoc_max)
    		{
		        iacc = acc >> (HPS_SCALE_BIT - HPS_FRAC_BIT);
		        xsc  = acc >> HPS_SCALE_BIT;

		        load_pxd |= (pi_cnt < (HOR_IN_RES-HOR_IN_CSTART)) ? 1 : 0;
				shift_pxd = 1;

				if (xic == xsc)
		        {
		        	line_bgn |= (align == 0 && (xoc/PIXEL_PER_CLOCK) == 0) ? 1 : 0;
		        	line_end |= (align == 0 && (xoc/PIXEL_PER_CLOCK) == (xoc_max/PIXEL_PER_CLOCK-1)) ? 1 : 0;
		        	pzx_msel[align] = pzx + i;
		        	frac_val[align] = iacc & HPS_SCALER_HSF_FRACTION;
		        	frac_vld[align] = 1;
					pxd_vld[align] = 1;

		        	xoc++;
		        	acc += HOR_SCF;
		        	align++;

		        	if (align >= PIXEL_PER_CLOCK)
		        		align = 0;
		        }

		        xic ++;
		        pi_cnt++;

			} // if
	 	} // for

	 	io_data  = 0;
	 	io_data1 = 0;

	 	// pxd_vld
	 	for (i = PIXEL_PER_CLOCK-1; i >= 0; i--) {
	 		HPS_BitShiftLeft(&io_data1, &io_data, 1);
	 		io_data |= pxd_vld[i] & 0x1;
	 	}

	 	// frac_vld
	 	for (i = PIXEL_PER_CLOCK-1; i >= 0; i--)
	 	{
	 		HPS_BitShiftLeft(&io_data1, &io_data, 1);
	 		io_data |= frac_vld[i] & 0x1;
	 	}

	 	// frac_val
	 	for (i = PIXEL_PER_CLOCK-1; i >= 0; i--)
	 	{
	 		HPS_BitShiftLeft(&io_data1, &io_data, 5);
	 		io_data |= frac_val[i] & 0x1F;
	 	}

	 	// pzx_msel
	 	for (i = PIXEL_PER_CLOCK-1; i >= 0; i--)
	 	{
	 		HPS_BitShiftLeft(&io_data1, &io_data, 2);
	 		io_data |= pzx_msel[i] & 0x3;
	 	}

		// {ctrl_done, shift_pxd, load_pxd, line_end, line_bgn}
	 	HPS_BitShiftLeft(&io_data1, &io_data, 5);

	 	if (ctrl_done)
	 		io_data |= HPS_SCALER_HSF_CTRL_DONE;

	 	if (shift_pxd)
	 		io_data |= HPS_SCALER_HSF_SHIFT_PIXEL;

	 	if ( load_pxd)
	 		io_data |= HPS_SCALER_HSF_LOAD_PIXEL;

	 	if ( line_end)
	 		io_data |= HPS_SCALER_HSF_LAST_PIXEL;

	 	if ( line_bgn)
	 		io_data |= HPS_SCALER_HSF_START_PIXEL;

		HPS_WriteReg(io_addr++, io_data);

        if (PIXEL_PER_CLOCK == 4) HPS_WriteReg(io_addr1++, io_data1);

	 	if (load_pxd) pl_cnt += PIXEL_PER_CLOCK;

    } // while

	// Post-Skip pixels
    io_data  = HPS_SCALER_HSF_LOAD_PIXEL | HPS_SCALER_HSF_SHIFT_PIXEL;
    io_data1 = 0;

	while (pl_cnt < ((HOR_IN_RES+PIXEL_PER_CLOCK-1)/PIXEL_PER_CLOCK)*PIXEL_PER_CLOCK)
	{
		HPS_WriteReg(io_addr++, io_data);

        if (PIXEL_PER_CLOCK == 4)
        	HPS_WriteReg(io_addr1++, io_data1);

        pl_cnt += PIXEL_PER_CLOCK;
	}	// while

    // done flag
    io_data  = HPS_SCALER_HSF_CTRL_DONE;
    io_data1 = 0;
    HPS_WriteReg(io_addr++, io_data);

    if (PIXEL_PER_CLOCK == 4)
    	HPS_WriteReg(io_addr1++, io_data1);

    return 1;
}
//--------------------------------------------------------------------------------------------------------------
int HPS_ScaleUp(int Ch, int HOR_IN_RES,  int HOR_IN_CSTART, int HOR_IN_CSIZE,
 				 int VER_IN_RES,  int VER_IN_CSTART, int VER_IN_CSIZE,
 				 int HOR_OUT_RES, int VER_OUT_RES,   int PIXEL_PER_CLOCK )
{
    unsigned int HOR_SCF, VER_SCF, acc, acc1;
    volatile unsigned int *io_addr;
	volatile unsigned int *io_addr1;
    unsigned int io_data;
	unsigned int io_data1;
    int i, j, yco, yco1, iacc, iacc1, ico, ico1, xoc, xsc, xic, pzx;
    int xoc_max;
    int line_bgn, line_end, load_pxd, shift_pxd, ctrl_done;
    int pzx_msel[4];
    int frac_val[4];
    int frac_vld[4];
    int pxd_vld[4];
    int pi_cnt, pl_cnt, li_cnt, li_cnt1;

    HOR_SCF = (HOR_IN_RES << HPS_SCALE_BIT) / HOR_OUT_RES;
    VER_SCF = (VER_IN_RES << HPS_SCALE_BIT) / VER_OUT_RES;


    xil_printf("[Scaler] : %d Scaler Up [%d %d %d] [%d %d %d] [%d %d %d]\r\n",
    		Ch, HOR_IN_RES,  HOR_IN_CSTART, HOR_IN_CSIZE,
    		VER_IN_RES,  VER_IN_CSTART, VER_IN_CSIZE,
    		HOR_OUT_RES, VER_OUT_RES,   PIXEL_PER_CLOCK);

    // Vertical Scale Factor setup

    if(Ch == 1)
    	io_addr = HPS_SCALER1_IO_VERTICAL_SCALE_FACTOR_REG;
    else
    	io_addr = HPS_SCALER0_IO_VERTICAL_SCALE_FACTOR_REG;


    // Pre-skip lines
    io_data = HPS_SCALER_VSF_LOAD_LINE | HPS_SCALER_VSF_SHIFT_LINE;

    for(i = 0; i < VER_IN_CSTART; i++)
    {
        HPS_WriteReg(io_addr++, io_data);
    }

    li_cnt  = 0;
    li_cnt1 = 1;

	// preload 2 lines
	io_data = HPS_SCALER_VSF_LOAD_LINE | HPS_SCALER_VSF_SHIFT_LINE;

	for(i = 0; i < 2; i++)
	{
	    HPS_WriteReg(io_addr++, io_data);
	    li_cnt++;
	    li_cnt1++;
	}

	acc  = 0;     // current
	yco  = 0;

	acc1 = VER_SCF; // next
	yco1 = 0;

	// First line
	iacc  = acc  >> (HPS_SCALE_BIT - HPS_FRAC_BIT);
	iacc1 = acc1 >> (HPS_SCALE_BIT - HPS_FRAC_BIT);
	ico   = acc  >> HPS_SCALE_BIT;
	ico1  = acc1 >> HPS_SCALE_BIT;

	io_data = HPS_SCALER_VSF_START_LINE
	        | HPS_SCALER_VSF_LOAD_LINE
	        | HPS_SCALER_VSF_VALID_LINE;
	HPS_WriteReg(io_addr++, io_data);

    acc  += VER_SCF;
    acc1 += VER_SCF;
    li_cnt++;

    for (i = 1; i < VER_OUT_RES; i++)
    {
        iacc  = acc  >> (HPS_SCALE_BIT - HPS_FRAC_BIT);  // current
        iacc1 = acc1 >> (HPS_SCALE_BIT - HPS_FRAC_BIT); // next
        ico   = acc  >> HPS_SCALE_BIT;
        ico1  = acc1 >> HPS_SCALE_BIT;

        io_data = ((li_cnt < (VER_IN_RES-VER_IN_CSTART)) ? ((yco != ico) ? HPS_SCALER_VSF_LOAD_LINE : 0) : 0)
        	    | ((yco1 != ico1) ? HPS_SCALER_VSF_SHIFT_LINE : 0)
        	    | ((i == VER_OUT_RES-1) ? HPS_SCALER_VSF_LAST_LINE : 0)
        	    | HPS_SCALER_VSF_VALID_LINE
                | (iacc & HPS_SCALER_VSF_FRACTION);

        HPS_WriteReg(io_addr++, io_data);

        if (yco != ico)
        {
            yco++;
            li_cnt++;
        }

        if (yco1 != ico1)
        {
        	yco1++;
        	li_cnt1++;
        }

        acc  += VER_SCF;
        acc1 += VER_SCF;
    }

	// Skip Lines
    if (VER_IN_RES > (VER_IN_CSTART+li_cnt))
    {
    	io_data = HPS_SCALER_VSF_LOAD_LINE | HPS_SCALER_VSF_SHIFT_LINE;
    	for (i = 0; i < VER_IN_RES-(VER_IN_CSTART+li_cnt); i++)
    	{
			HPS_WriteReg(io_addr++, io_data);
        	li_cnt++;
    	}
    }

    // done flag
    io_data = 0;
    HPS_WriteReg(io_addr++, io_data);

	/////////////////////////////////
    // Horizontal Scale UP Factor setup
	/////////////////////////////////

    if(Ch == 1)
    {
    	io_addr  = HPS_SCALER1_IO_HORIZONTAL_SCALE_FACTOR0_REG;
    	io_addr1 = HPS_SCALER1_IO_HORIZONTAL_SCALE_FACTOR1_REG;
    }else
    {
    	io_addr  = HPS_SCALER0_IO_HORIZONTAL_SCALE_FACTOR0_REG;
    	io_addr1 = HPS_SCALER0_IO_HORIZONTAL_SCALE_FACTOR1_REG;
    }

    pl_cnt = 0;

    // Pre-skip pixels
    io_data  = HPS_SCALER_HSF_LOAD_PIXEL | HPS_SCALER_HSF_SHIFT_PIXEL;
    io_data1 = 0;

    for (i = 0; i < (HOR_IN_CSTART+PIXEL_PER_CLOCK-1)/PIXEL_PER_CLOCK; i++)
    {
    	HPS_WriteReg(io_addr++,  io_data);
        if (PIXEL_PER_CLOCK == 4) HPS_WriteReg(io_addr1++, io_data1);
        pl_cnt += PIXEL_PER_CLOCK;
    }

    pi_cnt = (((HOR_IN_CSTART+PIXEL_PER_CLOCK-1)/PIXEL_PER_CLOCK)*PIXEL_PER_CLOCK)-HOR_IN_CSTART;
    pzx = pi_cnt;

    // preload 2 pixels
    io_data  = HPS_SCALER_HSF_LOAD_PIXEL | HPS_SCALER_HSF_SHIFT_PIXEL;
    io_data1 = 0;
    for(i = 0; i < 2/PIXEL_PER_CLOCK; i++)
    {
        HPS_WriteReg(io_addr++,  io_data);
        if (PIXEL_PER_CLOCK == 4) HPS_WriteReg(io_addr1++, io_data1);
        pi_cnt += PIXEL_PER_CLOCK;
        pl_cnt += PIXEL_PER_CLOCK;
    }

   	acc = 0;
    xic = 0;
    xoc = 0;
    ctrl_done = 0;
    xoc_max = ((HOR_OUT_RES+PIXEL_PER_CLOCK-1) / PIXEL_PER_CLOCK) * PIXEL_PER_CLOCK;

    // Load first
    io_data  = HPS_SCALER_HSF_LOAD_PIXEL | HPS_SCALER_HSF_SHIFT_PIXEL;
    io_data1 = 0;
    HPS_WriteReg (io_addr++, io_data);

    if (PIXEL_PER_CLOCK == 4)
    	HPS_WriteReg(io_addr1++, io_data1);

    pi_cnt += PIXEL_PER_CLOCK;
    pl_cnt += PIXEL_PER_CLOCK;
    pzx = pi_cnt - 3;

	while (xoc < xoc_max)
    {
    	// initialize
    	line_bgn = 0;
    	line_end = 0;
    	load_pxd = 0;
    	shift_pxd = 0;

    	for(i = 0; i < 4; i++)
    	{
    		pzx_msel[i] = 0;
    		frac_vld[i] = 0;
    		frac_val[i] = 0;
    		pxd_vld[i]	= 0;
    	}

    	for (i = 0; i < PIXEL_PER_CLOCK; i++)
    	{
    		if (xoc < xoc_max)
    		{
		        iacc = acc >> (HPS_SCALE_BIT - HPS_FRAC_BIT);
		        xsc  = acc >> HPS_SCALE_BIT;

		        line_bgn |= (xoc == 0) ? 1 : 0;
		        line_end |= ((xoc/PIXEL_PER_CLOCK) == (xoc_max/PIXEL_PER_CLOCK-1)) ? 1 : 0;

				if (xic != xsc)
		        {
		        	if (xsc >= (pi_cnt - 2))
		        	{
			        	load_pxd  |= (pi_cnt < (HOR_IN_RES-HOR_IN_CSTART)) ? 1 : 0;
						shift_pxd |= 1;

						for (j = 0; j < i; j++) pzx_msel[j] += PIXEL_PER_CLOCK;

			        	pi_cnt += PIXEL_PER_CLOCK;
			        	pl_cnt += PIXEL_PER_CLOCK;
			        	pzx += PIXEL_PER_CLOCK;
			       	} // if

			       	xic++;
			       	pzx--;

		        } // if

		        pzx_msel[i] = pzx;
			    frac_val[i] = iacc & HPS_SCALER_HSF_FRACTION;
			    frac_vld[i] = 1;
			    pxd_vld[i] = (xoc < HOR_OUT_RES) ? 1 : 0;
			    xoc++;
			    acc += HOR_SCF;

			} // if

	 	} // for

	 	io_data  = 0;
	 	io_data1 = 0;

	 	// pxd_vld
	 	for (i = PIXEL_PER_CLOCK-1; i >= 0; i--)
	 	{
	 		HPS_BitShiftLeft(&io_data1, &io_data, 1);
	 		io_data |= pxd_vld[i] & 0x1;
	 	}

	 	// frac_vld
	 	for (i = PIXEL_PER_CLOCK-1; i >= 0; i--)
	 	{
	 		HPS_BitShiftLeft(&io_data1, &io_data, 1);
	 		io_data |= frac_vld[i] & 0x1;
	 	}

	 	// frac_val
	 	for (i = PIXEL_PER_CLOCK-1; i >= 0; i--)
	 	{
	 		HPS_BitShiftLeft(&io_data1, &io_data, 5);
	 		io_data |= frac_val[i] & 0x1F;
	 	}

	 	// pzx_msel
	 	for (i = PIXEL_PER_CLOCK-1; i >= 0; i--)
	 	{
	 		HPS_BitShiftLeft(&io_data1, &io_data, 2);
	 		io_data |= pzx_msel[i] & 0x3;
	 	}

		// {ctrl_done, shift_pxd, load_pxd, line_end, line_bgn}
	 	HPS_BitShiftLeft(&io_data1, &io_data, 5);

	 	if (ctrl_done)
	 		io_data |= HPS_SCALER_HSF_CTRL_DONE;

	 	if (shift_pxd)
	 		io_data |= HPS_SCALER_HSF_SHIFT_PIXEL;

	 	if ( load_pxd)
	 		io_data |= HPS_SCALER_HSF_LOAD_PIXEL;

	 	if ( line_end)
	 		io_data |= HPS_SCALER_HSF_LAST_PIXEL;

	 	if ( line_bgn)
	 		io_data |= HPS_SCALER_HSF_START_PIXEL;

		HPS_WriteReg(io_addr++, io_data);

        if (PIXEL_PER_CLOCK == 4) HPS_WriteReg(io_addr1++, io_data1);

    } // while

    // Post-Skip pixels
    io_data  = HPS_SCALER_HSF_LOAD_PIXEL | HPS_SCALER_HSF_SHIFT_PIXEL;
    io_data1 = 0;
	while (pl_cnt < ((HOR_IN_RES + PIXEL_PER_CLOCK - 1) / PIXEL_PER_CLOCK) * PIXEL_PER_CLOCK)
	{
		HPS_WriteReg(io_addr++,  io_data);

        if (PIXEL_PER_CLOCK == 4)
        	HPS_WriteReg(io_addr1++, io_data1);

        pl_cnt += PIXEL_PER_CLOCK;

	} // while

	// done flag
    io_data  = HPS_SCALER_HSF_CTRL_DONE;
    io_data1 = 0;

    HPS_WriteReg(io_addr++,  io_data);

    if (PIXEL_PER_CLOCK == 4)
    	HPS_WriteReg(io_addr1++, io_data1);

    return 1;
}
//--------------------------------------------------------------------------------------------------------------
void HPS_ScaleUpDown(int Ch, int SrcW, int SrcH, int DstW, int DstH)
{
	xil_printf("[Scaler] : (%d, %d) -> (%d, %d)\r\n", SrcW, SrcH, DstW, DstH);

	if (SrcW >= DstW)
		HPS_ScaleDown(Ch, SrcW, 0, SrcW, SrcH, 0, SrcH, DstW, DstH, 1);
	else
		HPS_ScaleUp(Ch, SrcW, 0, SrcW, SrcH, 0, SrcH, DstW, DstH, 1);
}
//--------------------------------------------------------------------------------------------------------------
int HPS_Init(int Id, int mode)
{
	int i;
	HPS_Open();

	for(i = 0; i < 10000000; i ++)
	{
	
	}

	switch(Id)
	{
		case SCALER1_DEVICE_ID:
			HPS_Reset(0, 1);
			for(i = 0; i < 100000000; i ++)
			{
			}
			
			if(mode == 0)
			{
				HPS_ScaleUpDown(0, 1920, 1536, 1280, 1024);
			}else
			{
				HPS_ScaleUpDown(0, 1920, 1080, 1920, 1080);
			}
			HPS_Reset(0, 0);
			break;
			
		case SCALER2_DEVICE_ID:
			HPS_Reset(1, 1);
			for(i = 0; i < 100000000; i ++)
			{
			}
			if(mode == 0)   // 5:4
		  	{
				HPS_ScaleUpDown(1, 1920, 1536, 1280, 1024);
		  	}else           // Full HD
		  	{
				HPS_ScaleUpDown(1, 1920, 1080, 1920, 1080);
		  	}
			HPS_Reset(1, 0);
			break;
			
		default:
			break;
	
	}

  	xil_printf("-----------------------------------------------------------\r\n");
  	xil_printf("[SCALER] : HPS_SCALER0_IO_VERTICAL_SCALE_FACTOR_REG = %x\r\n", HPS_ReadReg(HPS_SCALER0_IO_VERTICAL_SCALE_FACTOR_REG));
  	xil_printf("[SCALER] : HPS_SCALER0_IO_VERTICAL_LANCZOS_COEFF_REG = %x\r\n", HPS_ReadReg(HPS_SCALER0_IO_VERTICAL_LANCZOS_COEFF_REG));
  	xil_printf("[SCALER] : HPS_SCALER0_IO_HORIZONTAL_SCALE_FACTOR0_REG = %x\r\n", HPS_ReadReg(HPS_SCALER0_IO_HORIZONTAL_SCALE_FACTOR0_REG));
  	xil_printf("[SCALER] : HPS_SCALER0_IO_HORIZONTAL_SCALE_FACTOR1_REG = %x\r\n", HPS_ReadReg(HPS_SCALER0_IO_HORIZONTAL_SCALE_FACTOR1_REG));
  	xil_printf("[SCALER] : HPS_SCALER0_IO_HORIZONTAL_LANCZOS_COEFF_REG = %x\r\n", HPS_ReadReg(HPS_SCALER0_IO_HORIZONTAL_LANCZOS_COEFF_REG));
  	xil_printf("[SCALER] : HPS_SCALER0_IO_HV_SCALER_CONTROL_REG = %x\r\n", HPS_ReadReg(HPS_SCALER0_IO_HV_SCALER_CONTROL_REG));

  	for(int i = 0; i < (1930); i++)
  	{
  		if((i <50) || (i > 1900))
  		{
  			if((i%8) == 0)
  				xil_printf("\r\n");
  			xil_printf("[%d=%x] ", (i), HPS_ReadReg(HPS_SCALER0_IO_VERTICAL_SCALE_FACTOR_REG + (i)));
  		}
  	}
  	xil_printf("\r\n\r\n");
  	for(int i = 0; i < (1100); i++)
  	{
  		if((i <50) || (i > 1030))
  		{
  			if((i%8) == 0)
  				xil_printf("\r\n");
  			xil_printf("[%d:%x] ", ( i), HPS_ReadReg(HPS_SCALER0_IO_HORIZONTAL_SCALE_FACTOR0_REG + (i)));
  		}
  	}
  	xil_printf("\r\n-----------------------------------------------------------\r\n");

  	for(int i = 0; i < (1566); i++)
  	{
  		if((i <50) || (i > 1506))
  		{
  			if((i%8) == 0)
  				xil_printf("\r\n");
  			xil_printf("[%d=%x] ", (i), HPS_ReadReg(HPS_SCALER1_IO_VERTICAL_SCALE_FACTOR_REG + (i)));
  		}
  	}
  	xil_printf("\r\n\r\n");
  	for(int i = 0; i < (894); i++)
  	{
  		if((i <50) || (i > 834))
  		{
  			if((i%8) == 0)
  				xil_printf("\r\n");
  			xil_printf("[%d:%x] ", (i), HPS_ReadReg(HPS_SCALER1_IO_HORIZONTAL_SCALE_FACTOR0_REG + (i)));
  		}
  	}
  	xil_printf("\r\n-----------------------------------------------------------\r\n");


  HPS_Close();
  return 0;
}
