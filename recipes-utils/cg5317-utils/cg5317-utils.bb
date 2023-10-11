DESCRIPTION = "Layer includes the services to initialize CG5317 in host loading mode"

LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://../COPYING.MIT;md5=3da9cfbcb788c80a0384361b4de20420"

SRC_URI = "file://cg5317-bringup.service \
           file://cg5317-bringup.sh \
           file://cg5317-host.service \
           file://resetmodem.service \
           file://resetmodem.sh \
           file://COPYING.MIT \
           "

do_install() {
	install -d ${D}${sysconfdir}/systemd/system/
	install -m 0644 ${WORKDIR}/cg5317-bringup.service ${D}${sysconfdir}/systemd/system
	install -m 0644 ${WORKDIR}/cg5317-host.service ${D}${sysconfdir}/systemd/system
	install -m 0644 ${WORKDIR}/resetmodem.service ${D}${sysconfdir}/systemd/system

	install -d ${D}${sysconfdir}/systemd/system/multi-user.target.wants
	ln -sf ../cg5317-bringup.service ${D}${sysconfdir}/systemd/system/multi-user.target.wants/
	ln -sf ../cg5317-host.service ${D}${sysconfdir}/systemd/system/multi-user.target.wants/

	install -d ${D}${sysconfdir}/systemd/system/default.target.wants
	ln -sf ../resetmodem.service ${D}${sysconfdir}/systemd/system/default.target.wants/

	install -d ${D}${bindir}
	install -m 0755 ${WORKDIR}/resetmodem.sh ${D}${bindir}
	install -m 0755 ${WORKDIR}/cg5317-bringup.sh ${D}${bindir}
}

FILES:${PN} += "${base_libdir}/systemd/system"
