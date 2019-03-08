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
#include "xplioctl.h"
#include "debug.h"

extern int activate_vdma_0(int base, int hsize, int vsize, uint32_t *fb_base);
extern int activate_vdma_1(int base, int hsize, int vsize, uint32_t *vdma1_base);
extern int activate_vdma_2(int base, int hsize, int vsize, uint32_t *vdma2_base);
extern int activate_vdma_3(int base, int hsize, int vsize, uint32_t *fb_base);
extern int activate_vdma_4(int base, int hsize, int vsize, uint32_t *vdma4_base);

void check_xpl_interface(void);


static uint32_t _fb_mem = 0x40000000;


void initialize(uint32_t *fb_mem)
{
    xpl_open();
    check_xpl_interface();
    activate_vdma_0(0, 1920, 1080, fb_mem);
	activate_vdma_1(0, 1920, 1080, fb_mem);
	// activate_vdma_2(0, 1920, 1080, fb_mem);
	activate_vdma_3(0, 1920, 1080, fb_mem);
	activate_vdma_4(0, 1920, 1080, fb_mem);
}

void finalize(void)
{
    usleep(1000);
    xpl_close();
}

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

int main(int argc, char **argv)
{
    uint32_t *fb_mem = &_fb_mem;
    uint32_t addr;

	printf("VDMA & FB Test Program - Optilogic 2019. 3. 1 a\n");
    if (argc >= 2) {
        addr = strtoul(argv[1], NULL, 0);
        fb_mem = &addr;
    }

    initialize(fb_mem);

    printf("Set Memory Address of Frame Buffer: 0x%08X\n", *fb_mem);

    finalize();
    return 0;
}
