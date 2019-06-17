#
# This file is the xplapp recipe.
#

SUMMARY = "Simple xplinit application"
SECTION = "PETALINUX/apps"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = "file://xplapp.c \
	   file://debug.h \
	   file://xpldrv.h \
	   file://xplioctl.c \
	   file://xplioctl.h \
	   file://vdma.c \
	   file://xilinx/xdebug.h \
	   file://xilinx/xaxivdma_hw.h \
	   file://xilinx/xiltypes.h \
	   file://fbtest.c \
	   file://Makefile \
		  "

S = "${WORKDIR}"

do_compile() {
	     oe_runmake
}

do_install() {
	     install -d ${D}${bindir}
	     install -m 0755 xplinit ${D}${bindir}
	     install -m 0755 fbtest ${D}${bindir}
}
