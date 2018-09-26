SRC_URI += "file://hwclock.sh"

FILESEXTRAPATHS_prepend := "${THISDIR}/files:"

do_install_append() {
    rm -rf ${D}${sysconfdir}/init.d/hwclock.sh
    install -m 0644 ${WORKDIR}/hwclock.sh ${D}${sysconfdir}/init.d/hwclock.sh
}
