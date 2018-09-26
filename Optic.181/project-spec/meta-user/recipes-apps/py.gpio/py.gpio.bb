#
# This file is the py.gpio recipe.
#

SUMMARY = "Simple py.gpio application"
SECTION = "PETALINUX/apps"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"
RDEPENDS_${PN} += "python3"

SRC_URI = "file://gpio.py"

S = "${WORKDIR}"

do_install() {
         echo "#### PN : ${PN}"
	     install -d ${D}/${bindir}
	     install -m 0755 ${S}/gpio.py ${D}/${bindir}
}
