#include <inttypes.h>
#include <stdio.h>  
#include <stdlib.h>  
#include <string.h>  

#include <fcntl.h>  
#include <unistd.h>  
#include <linux/fb.h>  
#include <sys/mman.h>  
#include <sys/ioctl.h>

typedef uint8_t Color;

struct Config {
    char *fb_device;
    int fd;
    Color order[3][3];
    int sleep_second;
};

void* get_color_order(char* mode) 
{
    void* order = NULL;
    if ( ! strcasecmp(mode, "RGB")) {
        static Color c[3][3] = {
            {0, 0, 255},
            {0, 255, 0},
            {255, 0, 0},
        };
        order = c;
    } else if ( ! strcasecmp(mode, "RBG")) {
        static Color c[3][3] = {
            {0, 0, 255},
            {255, 0, 0},
            {0, 255, 0},
        };
        order = c;
    } else if ( ! strcasecmp(mode, "GRB")) {
        static Color c[3][3] = {
            {0, 255, 0},
            {0, 0, 255},
            {255, 0, 0},
        };
        order = c;
    } else if ( ! strcasecmp(mode, "GBR")) {
        static Color c[3][3] = {
            {0, 255, 0},
            {255, 0, 0},
            {0, 0, 255},
        };
        order = c;
    } else if ( ! strcasecmp(mode, "BRG")) {
        static Color c[3][3] = {
            {255, 0, 0},
            {0, 0, 255},
            {0, 255, 0},
        };
        order = c;
    } else {
        static Color c[3][3] = {
            {255, 0, 0},
            {0, 255, 0},
            {0, 0, 255},
        };
        order = c;
    }
    return order;
}

void init_config(struct Config *cfg, int argc, char *argv[])
{
    int opt;
    int fb_num = 0;
    char fb_device[32];
    Color order[3][3];

    memcpy(order, get_color_order("RGB"), sizeof(order));

    while ((opt = getopt(argc, argv, "hf:c:s:")) != -1) {
        switch (opt) {
            case 'f':
                fb_num = strtoul(optarg, NULL, 0);
                break;
            case 'c':
                memcpy(order, get_color_order(optarg), sizeof(order));
                break;
            case 's':
                cfg->sleep_second = strtoul(optarg, NULL, 0);
                break;
            case 'h':
            default: /* '?' */
                fprintf(stderr, 
                        "  Usage: %s [-f <num>] [-c <order>] [-s <second>]\n"
                        "    -f <num>       Set a Number of a frame buffer.\n"
                        "    -c <order>     Set a color order - BGR, BRG, RGB, RBG, GBR or GRB\n" 
                        "    -s <second>    Set the duration of color bar display.\n"
                        "\n",
                        argv[0]);
                exit(EXIT_FAILURE);
        }
    }

    snprintf(fb_device, sizeof(fb_device), "/dev/fb%d", fb_num);
    cfg->fb_device = strdup(fb_device);
    memcpy(cfg->order, order, sizeof(order));
}

int main( int argc, char* argv[] )  
{  
    struct Config cfg;
    struct fb_var_screeninfo framebuffer_variable_screeninfo;  
    struct fb_fix_screeninfo framebuffer_fixed_screeninfo;  

    memset(&cfg, 0, sizeof(cfg));
    init_config(&cfg, argc, argv);

    printf("Frame Buffer: %s\n", cfg.fb_device);
    cfg.fd = open( cfg.fb_device, O_RDWR );  
    if ( cfg.fd <  0 ){  
        perror( "Error: cannot open framebuffer device\n" );  
        exit(1);  
    }  

    if ( ioctl(cfg.fd, FBIOGET_VSCREENINFO,   
                &framebuffer_variable_screeninfo) )  
    {  
        perror( "Error: reading variable screen infomation\n" );  
        exit(1);  
    }  
    framebuffer_variable_screeninfo.bits_per_pixel=32;  

    if ( ioctl(cfg.fd, FBIOPUT_VSCREENINFO,   
                &framebuffer_variable_screeninfo) )  
    {  
        perror( "Error: reading variable screen infomation\n" );  
        exit(1);  
    }  



    if ( ioctl(cfg.fd, FBIOGET_FSCREENINFO,   
                &framebuffer_fixed_screeninfo) )  
    {  
        perror( "Error: reading fixed screen infomation\n" );  
        exit(1);  
    }  

    printf( "framebuffer Display information\n" );  
    printf( " %d x %d  %d bpp(%d : %d)\n", framebuffer_variable_screeninfo.xres,  
            framebuffer_variable_screeninfo.yres,   
            framebuffer_variable_screeninfo.bits_per_pixel,
			framebuffer_variable_screeninfo.xoffset,
		    framebuffer_variable_screeninfo.yoffset );  


    int width  = framebuffer_variable_screeninfo.xres;  
    int height = framebuffer_variable_screeninfo.yres;  
    int bpp = framebuffer_variable_screeninfo.bits_per_pixel/8;  
    int xoffset = framebuffer_variable_screeninfo.xoffset;  
    int yoffset = framebuffer_variable_screeninfo.yoffset;  


    long int screensize = width*height*bpp;  


    char *framebuffer_pointer = (char*)mmap( 0, screensize,  
            PROT_READ|PROT_WRITE,  
            MAP_SHARED,  
            cfg.fd, 0 );  

    if ( framebuffer_pointer == MAP_FAILED )  
    {  
        perror( "Error : mmap\n" );  
        exit(1);  
    }  
    else  
    {  

		// clear Overlay Buffer - FB0
        int x,y;  
        int y_limit = height;
        for ( y=0; y<y_limit; y++)  
		{
            if ((y%100) != 0)
                continue;

            for ( x=0; x<width; x++)  
            {  
                // unsigned int pixel_offset = (y+yoffset)*framebuffer_fixed_screeninfo.line_length*2 +(x+xoffset)*bpp; 
				unsigned int pixel_offset = ((width * bpp) * (y + yoffset)) + ((x + xoffset) * bpp);
				if(bpp == 4)
				{		
					if(x < width)
					{		
						framebuffer_pointer[pixel_offset]	= 0;//B  
                		framebuffer_pointer[pixel_offset+1]	= 0;//G  
                		framebuffer_pointer[pixel_offset+2]	= 0xFF;//R  
                		framebuffer_pointer[pixel_offset+3]	= 128;//A
					}else
					{
						
					}
				}  
            }
		}  
        for ( y=0; y<y_limit; y++)  
		{
            for ( x=0; x<width; x++)  
            {  
                // unsigned int pixel_offset = (y+yoffset)*framebuffer_fixed_screeninfo.line_length*2 +(x+xoffset)*bpp; 
				unsigned int pixel_offset = ((width * bpp) * (y + yoffset)) + ((x + xoffset) * bpp);
				if(bpp == 4)
				{		
					if((x%100) == 0)
					{		
						framebuffer_pointer[pixel_offset]	= 0;//B  
                		framebuffer_pointer[pixel_offset+1]	= 0;//G  
                		framebuffer_pointer[pixel_offset+2]	= 0xFF;//R  
                		framebuffer_pointer[pixel_offset+3]	= 128;//A
					}else
					{
						
					}
				}  
            }
		}  

    }   

    if (cfg.sleep_second > 0) {
        int x;
        for (x=0; x<cfg.sleep_second; x++) {
            sleep(1);
        }
    }

    munmap( framebuffer_pointer, screensize );   
    close( cfg.fd );  

    return 0;
}  

