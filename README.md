# Zynq with PetaLinux on MicroZED Board.


## Prepare
+ Install petalinux-v2018.2-final-installer.run to /opt/pkg which it linked as petalinux-v2018.2-final.  
  + /opt/pkg -> petalinux-v2018.2-final
+ BSP file could be getting from http://zedboard.org/support/design/1519/10 for MicroZED Board.  
  + /opt/pkg/mz7010_fmccc_2017_4.bsp


## Quick Start

### help

source MicroZED.182/source.env  
zynq

#### Create with the based HDF

source MicroZED.182/source.env  
zynq create \`pwd\`/hdf.182  
cd MicroZED.182
zynq build  
zynq build  
zynq image

#### Booting with SD-Card

## Copy BOOT.BIN and image.ub to partition 1 of SD-Card  

zynq copy [FOLDER_OF_SD_PARTITION_1]

## Make RootFS to Partition 2 of SD-Card

 

## Booting Time

Displayed a message to request DHCP after pressed a reset button.

QSPI Flash + /dev/mmcblk0p2: About 8 seconds  
SD-Card + /dev/mmcblk0p2: About 11.5 seconds  
