# Zynq with PetaLinux on MicroZED Board.


## Prepare
+ Install petalinux-v2018.2-final-installer.run to /opt/pkg which it linked as petalinux-v2018.2-final.  
  + /opt/pkg -> petalinux-v2018.2-final
+ BSP file could be getting from http://zedboard.org/support/design/1519/10 for MicroZED Board.  
  + /opt/pkg/mz7010_fmccc_2017_4.bsp


## Quick Start

### help

    source Optic.181/source.env  
    zynq

#### Create with the based HDF

    source Optic.181/source.env  
    zynq create \`pwd\`/optic.hdf  
    cd Optic.181
    zynq build  
    zynq build  
    zynq image

## SD-card Image

#### Generate Image

Generate SD-card image file which this file has two partitions.  First partition is FAT32 and contains a kernel image file.  Second partition is EXT4 and this is root file-system.

    zynq sd-img

#### Create SD-card

    zynq sd-disk <dev-file> <sd-img-file>

#### Root File-System

All files in get_root/rootfs apply to the final generated rootfs image.  
If you make a deploy image, please remove follow files:
  + git_root/etc/dropbear/dropbear_rsa_host_key

