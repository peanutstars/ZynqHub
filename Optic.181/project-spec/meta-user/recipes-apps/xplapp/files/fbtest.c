#include <stdio.h>  
#include <stdlib.h>  
#include <string.h>  

#include <fcntl.h>  
#include <unistd.h>  
#include <linux/fb.h>  
#include <sys/mman.h>  
#include <sys/ioctl.h>


int main( int argc, char* argv[] )  
{  
    int framebuffer_fd = 0;  
    struct fb_var_screeninfo framebuffer_variable_screeninfo;  
    struct fb_fix_screeninfo framebuffer_fixed_screeninfo;  

    framebuffer_fd = open( "/dev/fb0", O_RDWR );  
    if ( framebuffer_fd <  0 ){  
        perror( "Error: cannot open framebuffer device\n" );  
        exit(1);  
    }  


    if ( ioctl(framebuffer_fd, FBIOGET_VSCREENINFO,   
                &framebuffer_variable_screeninfo) )  
    {  
        perror( "Error: reading variable screen infomation\n" );  
        exit(1);  
    }  
    framebuffer_variable_screeninfo.bits_per_pixel=32;  

    if ( ioctl(framebuffer_fd, FBIOPUT_VSCREENINFO,   
                &framebuffer_variable_screeninfo) )  
    {  
        perror( "Error: reading variable screen infomation\n" );  
        exit(1);  
    }  



    if ( ioctl(framebuffer_fd, FBIOGET_FSCREENINFO,   
                &framebuffer_fixed_screeninfo) )  
    {  
        perror( "Error: reading fixed screen infomation\n" );  
        exit(1);  
    }  

    printf( "framebuffer Display information\n" );  
    printf( " %d x %d  %d bpp\n", framebuffer_variable_screeninfo.xres,  
            framebuffer_variable_screeninfo.yres,   
            framebuffer_variable_screeninfo.bits_per_pixel );  


    int width  = framebuffer_variable_screeninfo.xres;  
    int height = framebuffer_variable_screeninfo.yres;  
    int bpp = framebuffer_variable_screeninfo.bits_per_pixel/8;  
    int xoffset = framebuffer_variable_screeninfo.xoffset;  
    int yoffset = framebuffer_variable_screeninfo.yoffset;  


    long int screensize = width*height*bpp;  


    char *framebuffer_pointer = (char*)mmap( 0, screensize,  
            PROT_READ|PROT_WRITE,  
            MAP_SHARED,  
            framebuffer_fd, 0 );  

    if ( framebuffer_pointer == MAP_FAILED )  
    {  
        perror( "Error : mmap\n" );  
        exit(1);  
    }  
    else  
    {  
        int x,y;  
        for ( y=0; y<height; y++)  
            for ( x=0; x<width; x++)  
            {  
                unsigned int pixel_offset = (y+yoffset)*framebuffer_fixed_screeninfo.line_length*2 +(x+xoffset)*bpp;  


                if (bpp==4){  
                    if ( x<=width*1/3){    
                        framebuffer_pointer[pixel_offset]=255;//B  
                        framebuffer_pointer[pixel_offset+1]=0;//G  
                        framebuffer_pointer[pixel_offset+2]=0;//R  
                        framebuffer_pointer[pixel_offset+3]=0;//A  
                    }  
                    if ( x>width*1/3 && x<=width*2/3){      
                        framebuffer_pointer[pixel_offset]=0;//B  
                        framebuffer_pointer[pixel_offset+1]=255;//G  
                        framebuffer_pointer[pixel_offset+2]=0;//R  
                        framebuffer_pointer[pixel_offset+3]=0;//A  
                    }  
                    if ( x>width*2/3){     
                        framebuffer_pointer[pixel_offset]=0;//B  
                        framebuffer_pointer[pixel_offset+1]=0;//G  
                        framebuffer_pointer[pixel_offset+2]=255;//R  
                        framebuffer_pointer[pixel_offset+3]=0;//A  
                    }  
                }  

            }  

    }   

    munmap( framebuffer_pointer, screensize );   
    close( framebuffer_fd );  

    return 0;
}  

