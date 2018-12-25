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
#include "xplioctl.h"
#include "debug.h"

extern int activate_vdma_0(int base, int hsize, int vsize, uint32_t *fb_base);
static uint32_t fb_mem = 0x60000000;


void initialize(void)
{
    xpl_open();
    activate_vdma_0(0, 1920, 1080, &fb_mem);
}

void finalize(void)
{
    sleep(1);
    xpl_close();
}

void check_xpl_interface(void)
{
    uint32_t version;

    reg_read32(0x2C, &version);
    printf("VDMA Version %08X\n", version);
}

int main(int argc, char **argv)
{
    initialize();

    printf("Hello World!\n");
    DBG("Debug\n");
    ERR("Error\n");

    check_xpl_interface();

    finalize();
    return 0;
}
