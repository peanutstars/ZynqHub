# Zynq with PetaLinux on MicroZED Board.


## Prepare
+ Install petalinux-v2017.4-final-installer.run to /opt/pkg which linked as petalinux-v2017.4-final.  
  + /opt/pkg -> petalinux-v2017.4-final
+ BSP file could be refer from http://zedboard.org/support/design/1519/10 for MicroZED Board.  
  + /opt/pkg/mz7010_fmccc_2017_4.bsp


## Quick Start

### help

zynq

#### The based BSP

source MicroZED.bsp/source.env  
zynq create BSP_FILE  
cd MicroZED.bsp  
zynq build  
zynq build  
zynq image

#### The based HDF

source MicroZED.hdf/source.env  
zynq create HDF_DIR_PATH  
zynq build  
zynq build  
zynq image

#### Booting with SD-Card

zynq copy DEST_PART_1


## Booting Time

Displayed a message to request DHCP after pressed a reset button.

QSPI Flash + /dev/mmcblk0p2: About 8 seconds  
SD-Card + /dev/mmcblk0p2: About 11.5 seconds  
