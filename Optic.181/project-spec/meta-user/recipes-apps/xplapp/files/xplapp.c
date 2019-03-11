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
    int init;
    int reset;
    uint32_t vout_fd;
    uint32_t fb_mem;
};
typedef struct Config Config;

extern int activate_vdma_0(int base, int hsize, int vsize, uint32_t *fb_base);
extern int activate_vdma_1(int base, int hsize, int vsize, uint32_t *vdma1_base);
extern int activate_vdma_2(int base, int hsize, int vsize, uint32_t *vdma2_base);
extern int activate_vdma_3(int base, int hsize, int vsize, uint32_t *fb_base);
extern int activate_vdma_4(int base, int hsize, int vsize, uint32_t *vdma4_base, int Mode);

void check_xpl_interface(void);


void reset_fpga_core(void)
{
    int fd;
    size_t vlength = 0x3000;
    uint32_t *vaddr;

    fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        fprintf(stderr, "%s", strerror(errno));
        exit(EXIT_FAILURE);
    }
    vaddr = mmap(NULL, vlength, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x80000000);

    printf("Reset FPGA CORE, Virtual Address = %p.\n", vaddr);
    *(vaddr + 0x1000/4) = 0x00000001;
    *(vaddr + 0x2000/4) = 0x00000001;
    *(vaddr + 0x1000/4) = *(vaddr + 0x1000/4) & 0xFFFFFFFE;
    *(vaddr + 0x2000/4) = *(vaddr + 0x2000/4) & 0xFFFFFFFE;
    
    munmap(vaddr, vlength);
}

void initialize(Config *cfg)
{
    xpl_open();
    printf("xpl_open OK\n");
    check_xpl_interface();
    printf("FB memory address: 0x%08X\n", cfg->fb_mem);

    if (cfg->reset) {
        reset_fpga_core();
    }
    if (cfg->init) {
        // activate_vdma_0(0, 1920, 1080, fb_mem);
    	activate_vdma_1(0, 1920, 1080, &cfg->fb_mem);
    	// activate_vdma_2(0, 1920, 1080, fb_mem);
    	activate_vdma_3(0, 1920, 1080, &cfg->fb_mem);
    	activate_vdma_4(0, 1920, 1080, &cfg->fb_mem, 0);
    }
    else {
    	activate_vdma_3(0, 1920, 1080, &cfg->fb_mem);
    }
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

int options(Config *cfg, int argc, char **argv)
{
    int opt;
    uint32_t _fb_mem[8] = {\
        0x40000000, 0x40800000,
        0x41000000, 0x41800000,
        0x42000000, 0x42800000,
        0x43000000, 0x43800000,
    };
    

    while ((opt = getopt(argc, argv, "hid:")) != -1) {
        switch (opt) {
            case 'h':
                usage();
                break;
            case 'i':
                cfg->reset = 1;
                cfg->init = 1;
                break;
            case 'd':
                cfg->vout_fd = (uint32_t) atoi(optarg);
                break;
            case 'r':
                cfg->reset = 1;
                break;
            default: /* '?' */
                usage();
        }
    }
    if (cfg->vout_fd >= 8) {
        fprintf(stderr, "FB port is ranged from 0 to 7.\n");
        usage();
    }
    cfg->fb_mem = _fb_mem[cfg->vout_fd];

    fprintf(stderr, "Options, reset:%d init:%d video_out_fd:%d\n", 
            cfg->reset, cfg->init, cfg->vout_fd);

    return 0;
}

int main(int argc, char **argv)
{
    Config cfg = { 0, };
    
    options(&cfg, argc, argv);

	printf("VDMA & FB Test Program - Optilogic a %s %s\n", __DATE__, __TIME__);

    initialize(&cfg);

    finalize();
    return 0;
}
