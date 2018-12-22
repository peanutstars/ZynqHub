#
# This file is the xplapp recipe.
#

SUMMARY = "Simple xplapp application"
SECTION = "PETALINUX/apps"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = "file://xplapp.c \
	   file://debug.h \
	   file://xpldrv.h \
	   file://xplioctl.c \
	   file://xplioctl.h \
	   file://Makefile \
		  "

S = "${WORKDIR}"

do_compile() {
	     oe_runmake
}

do_install() {
	     install -d ${D}${bindir}
	     install -m 0755 xplapp ${D}${bindir}
}
