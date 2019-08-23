/*
* Copyright (C) 2013 - 2016  Xilinx, Inc.  All rights reserved.
*
* Permission is hereby granted, free of charge, to any person
* obtaining a copy of this software and associated documentation
* files (the "Software"), to deal in the Software without restriction,
* including without limitation the rights to use, copy, modify, merge,
* publish, distribute, sublicense, and/or sell copies of the Software,
* and to permit persons to whom the Software is furnished to do so,
* subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* Use of the Software is limited solely to applications:
* (a) running on a Xilinx device, or (b) that interact
* with a Xilinx device through a bus or interconnect.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL XILINX  BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
* CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of the Xilinx shall not be used
* in advertising or otherwise to promote the sale, use or other dealings in this
* Software without prior written authorization from Xilinx.
*
*/

#include <inttypes.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>
#include "xplioctl.h"
#include "debug.h"

struct Config {
    int vdma_init;
    int core_init;
    int rear1_init;
    int rear2_init;
    int res_init;
    
    uint32_t vout_fd;
	uint32_t cap_fd;
	int 	 rear1_res;
	int		 rear2_res;

	uint32_t xOffset;
	uint32_t yOffset;

    uint32_t fb_mem;
	uint32_t fb_mem_cap;

};
typedef struct Config Config;


#define VDMA1_FRAME_DEVICE_ID		0
#define VDMA3_FRAME_DEVICE_ID		2
#define VDMA4_FRAME_DEVICE_ID		1
#define VDMA5_FRAME_DEVICE_ID		3
#define VDMA6_FRAME_DEVICE_ID		4
#define VDMA7_FRAME_DEVICE_ID		5
#define VDMA8_FRAME_DEVICE_ID		6
#define VDMA9_FRAME_DEVICE_ID		7

#define XVTC1_DEVICE_ID				0
#define XVTC2_DEVICE_ID				1
#define XVTC5_DEVICE_ID				3
#define XVTC4_DEVICE_ID				2


#define SCALER1_DEVICE_ID			0
#define SCALER2_DEVICE_ID			1


extern int Vdma_Init(uint32_t *menu_addr);
extern int Vdma_Stop(int Id);
extern int Vdma_SetActive(int Id, int hsize, int vsize, int hoffset, int voffset, uint32_t *vdma_base);
extern int Vtc_Init(int Id, int Mode);
extern int Vtc_Enable(int Id, int Mode);
extern int HPS_Init(int Id, int mode);

void check_xpl_interface(void);

//--------------------------------------------------------------------------
void reset_fpga_core(void)
{
    int fd;
    size_t vlength = 0x100000;
    uint32_t *vaddr;
	int i = 0;


    fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        fprintf(stderr, "%s", strerror(errno));
        exit(EXIT_FAILURE);
    }
    vaddr = mmap(NULL, vlength, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x80000000);

    printf("Reset FPGA CORE, Virtual Address = %p.\n", vaddr);
    *(vaddr) = 0x00000001;
    *(vaddr + 0x2000/4) = 0x00000001;
    
    for(i = 0; i < 10000; i ++)
    {
    
    }
    
    *(vaddr) = *(vaddr) & 0xFFFFFFFE;
    *(vaddr + 0x2000/4) = *(vaddr + 0x2000/4) & 0xFFFFFFFE;


    for(i = 0; i < 100000000; i ++)
    {
    
    }
    
	printf("RX REset        = %x\n", *(vaddr));
	printf("RX REset        = %x\n", *(vaddr + 1));
	printf("RX REset        = %x\n", *(vaddr + 2));
	printf("TX REset        = %x\n", *(vaddr + 0x2000/4));

    munmap(vaddr, vlength);
}
//--------------------------------------------------------------------------
void initialize(Config *cfg)
{
    xpl_open();
    printf("xpl_open OK\n");
    check_xpl_interface();
    printf("FB memory address: 0x%08X\n", cfg->fb_mem);

    if (cfg->core_init) {
        reset_fpga_core();
    }
    
    if (cfg->vdma_init) 
	{
		// VTC Disable
		Vtc_Enable(XVTC1_DEVICE_ID, 0);
		Vtc_Enable(XVTC2_DEVICE_ID, 0);
		Vtc_Enable(XVTC5_DEVICE_ID, 0);
		Vtc_Enable(XVTC4_DEVICE_ID, 0);
		
		// VDMA Stop
		Vdma_Stop(VDMA1_FRAME_DEVICE_ID);
		Vdma_Stop(VDMA3_FRAME_DEVICE_ID);
		Vdma_Stop(VDMA4_FRAME_DEVICE_ID);
		Vdma_Stop(VDMA5_FRAME_DEVICE_ID);
		Vdma_Stop(VDMA6_FRAME_DEVICE_ID);
		Vdma_Stop(VDMA7_FRAME_DEVICE_ID);
		Vdma_Stop(VDMA8_FRAME_DEVICE_ID);
		Vdma_Stop(VDMA9_FRAME_DEVICE_ID);
		
		// Scaler Init
		HPS_Init(SCALER1_DEVICE_ID, 1);
		HPS_Init(SCALER2_DEVICE_ID, 1);
		
		// VTC Init
		Vtc_Init(XVTC1_DEVICE_ID, 1);
		Vtc_Init(XVTC5_DEVICE_ID, 1);
		
		// VTC Enable
		Vtc_Enable(XVTC1_DEVICE_ID, 1);
		Vtc_Enable(XVTC2_DEVICE_ID, 1);
		Vtc_Enable(XVTC5_DEVICE_ID, 1);
		Vtc_Enable(XVTC4_DEVICE_ID, 1);
		
		// VDMA Init
    	Vdma_SetActive(VDMA1_FRAME_DEVICE_ID, 1920, 1080, 0, 0, 0);
		Vdma_SetActive(VDMA3_FRAME_DEVICE_ID, 1920, 1080, 0, 0, &cfg->fb_mem);
		Vdma_SetActive(VDMA9_FRAME_DEVICE_ID, 1920, 1080, 0, 0, 0);
		Vdma_SetActive(VDMA7_FRAME_DEVICE_ID, 1920, 1080, 0, 0, 0);
		Vdma_SetActive(VDMA8_FRAME_DEVICE_ID, 720,   480, 0, 0, 0);
				
		Vdma_SetActive(VDMA4_FRAME_DEVICE_ID, 1920, 1080, 0, 0, 0);
		Vdma_SetActive(VDMA5_FRAME_DEVICE_ID, 1536,  864, cfg->xOffset, cfg->yOffset, 0);
		Vdma_SetActive(VDMA6_FRAME_DEVICE_ID, 720,   480, 0, 0, 0);
    }

	if(cfg->rear1_init)
	{
		// VTC Disable
		Vtc_Enable(XVTC1_DEVICE_ID, 0);
		
		// VDMA STOP
		Vdma_Stop(VDMA1_FRAME_DEVICE_ID);
		Vdma_Stop(VDMA3_FRAME_DEVICE_ID);

		// Scale Init
		HPS_Init(SCALER1_DEVICE_ID, cfg->rear1_res);		
		
		// VTC Init
		Vtc_Init(XVTC1_DEVICE_ID, cfg->rear1_res);

		// VTC Enable
		Vtc_Enable(XVTC1_DEVICE_ID, 1);
		if(cfg->rear1_res == 1)
		{
			Vdma_SetActive(VDMA1_FRAME_DEVICE_ID, 1920, 1080, 0, 0, 0);
			Vdma_SetActive(VDMA3_FRAME_DEVICE_ID, 1920, 1080, 0, 0, &cfg->fb_mem);
		}else{
			Vdma_SetActive(VDMA1_FRAME_DEVICE_ID, 1920, 1536, 0, 0, 0);
			Vdma_SetActive(VDMA3_FRAME_DEVICE_ID, 1920, 1536, 0, 0, &cfg->fb_mem);
		}
	}
	
	if(cfg->rear2_init)
	{
		// VTC Disable
		Vtc_Enable(XVTC5_DEVICE_ID, 0);
		
		// VDMA STOP
		Vdma_Stop(VDMA9_FRAME_DEVICE_ID);

		// Scale Init
		HPS_Init(SCALER2_DEVICE_ID, cfg->rear2_res);		
		
		// VTC Init
		Vtc_Init(XVTC5_DEVICE_ID, cfg->rear2_res);

		// VTC Enable
		Vtc_Enable(XVTC5_DEVICE_ID, 1);
		if(cfg->rear2_res == 1)
		{
			Vdma_SetActive(VDMA9_FRAME_DEVICE_ID, 1920, 1080, 0, 0, 0);
		}else
		{
			Vdma_SetActive(VDMA9_FRAME_DEVICE_ID, 1920, 1536, 0, 0, 0);
		}
	}
	
	if(cfg->res_init)
	{
		Vdma_SetActive(VDMA5_FRAME_DEVICE_ID, 1536,  864, cfg->xOffset, cfg->yOffset, 0);
	}
}
//--------------------------------------------------------------------------
void finalize(void)
{
    usleep(1000);
    xpl_close();
}
//--------------------------------------------------------------------------
void check_xpl_interface(void)
{
    uint32_t regv;
    int major, minor;

    reg_read32(0x2C, &regv);
    major = regv >> 28;
    minor = (regv >> 20) & 0xFF;
    printf("VDMA Core Version %X.%02X\n", major, minor);
    if (major == 6 && minor == 0x20)
        return;

    fprintf(stderr, "VDMA Core Version is not matched.\n");
    exit(1);
}
//--------------------------------------------------------------------------
void usage(void)
{
    const char *doc = \
    "  usage: xplinit [-i] [-d <fb_num>]i [-r]\n" \
    "    options:\n" \
    "      -i             do initialization.\n" \
    "      -d <fb_num>    set fb to oup video, fb_num is range 0 to 7.\n" \
    "      -r             reset fpga core.\n" \
    "\n";

    fprintf(stderr, "%s", doc);
    exit(EXIT_FAILURE);
}
//--------------------------------------------------------------------------
int options(Config *cfg, int argc, char **argv)
{
    int opt;
    uint32_t _fb_mem[8] = {\
        0x40000000, 0x41000000,
        0x42000000, 0x43000000,
        0x44000000, 0x45000000,
        0x46000000, 0x47000000,
    };
	

    while ((opt = getopt(argc, argv, "hidr:m:n:x:y:")) != -1) 
	{
        switch (opt) {
            case 'h':
                usage();
                break;

            case 'i':
                cfg->core_init = 1;
                cfg->vdma_init = 1;
                break;

            case 'd':
                cfg->vout_fd = (uint32_t) atoi(optarg);
                break;

            case 'r':
                cfg->core_init = 1;
                break;

			case 'm':
				cfg->rear1_init = 1;
				cfg->rear1_res = (uint32_t) atoi(optarg);
				if(cfg->rear1_res > 1)
					cfg->rear1_res = 1;
				break;

			case 'n':
				cfg->rear2_init = 1;
				cfg->rear2_res = (uint32_t) atoi(optarg);
				if(cfg->rear2_res > 1)
					cfg->rear2_res = 1;
				break;

			case 'x':
				cfg->xOffset = (uint32_t) atoi(optarg);
				cfg->res_init = 1;
				break;

			case 'y':
				cfg->yOffset = (uint32_t) atoi(optarg);
				cfg->res_init = 1;
				break;

            default: 
                usage();
        }
    }
    if (cfg->vout_fd >= 8) {
        fprintf(stderr, "FB port is ranged from 0 to 7.\n");
        usage();
    }
	cfg->cap_fd = 3;		// 영상 Capture 는 항상 FB3을 사용한다.
    cfg->fb_mem = _fb_mem[cfg->vout_fd];
	cfg->fb_mem_cap = _fb_mem[cfg->cap_fd];

    fprintf(stderr, "Options, core_rst:%d vdma_rst:%d video_out_fd:%d(%x)\n", cfg->core_init, cfg->vdma_init, cfg->vout_fd, cfg->fb_mem);

    return 0;
}
//--------------------------------------------------------------------------
int main(int argc, char **argv)
{
    Config cfg = { 0, };
    
    options(&cfg, argc, argv);

	printf("VDMA & FB Test Program - Optilogic a %s %s\n", __DATE__, __TIME__);

    initialize(&cfg);

    finalize();
    return 0;
}
