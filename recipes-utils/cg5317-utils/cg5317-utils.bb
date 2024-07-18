DESCRIPTION = "Layer includes the services to initialize CG5317 in host loading mode"

LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://../COPYING.MIT;md5=3da9cfbcb788c80a0384361b4de20420"

ROOT_HOME = "/root"

SRC_URI = " \
	file://cg5317-bringup.service \
	file://cg5317-host.service \
	file://config.bin \
	file://COPYING.MIT \
	file://evse.ini \
	file://examples \
	file://FW.bin \
	file://pev.ini \
	file://spi_sta_config.bin \
	file://spi_cco_config.bin \
"

# libgpiod_2.x.x deploys libgpiod.so.3
RDEPENDS:${PN}:append = "libgpiod (<= 2.0.0)"

do_install() {
	install -d ${D}${sysconfdir}/systemd/system/
	install -m 0644 ${WORKDIR}/cg5317-bringup.service ${D}${sysconfdir}/systemd/system
	install -m 0644 ${WORKDIR}/cg5317-host.service ${D}${sysconfdir}/systemd/system

	install -m 0644 ${WORKDIR}/evse.ini ${D}${sysconfdir}/
	install -m 0644 ${WORKDIR}/pev.ini ${D}${sysconfdir}/

	install -d ${D}${sysconfdir}/systemd/system/multi-user.target.wants
	ln -sf ${D}${sysconfdir}/systemd/system/cg5317-bringup.service \
		${D}${sysconfdir}/systemd/system/multi-user.target.wants/
	ln -sf ${D}${sysconfdir}/systemd/system/cg5317-host.service \
		${D}${sysconfdir}/systemd/system/multi-user.target.wants/

	install -d ${D}${sysconfdir}/systemd/system/default.target.wants

	install -d ${D}${ROOT_HOME}
	cp -r ${WORKDIR}/examples ${D}${ROOT_HOME}
	find ${D}${ROOT_HOME}/examples -type f -exec chmod +x {} +

	install -d ${D}/usr/lib/firmware
	install -m 0644 ${WORKDIR}/config.bin ${D}/usr/lib/firmware
	install -m 0644 ${WORKDIR}/spi_sta_config.bin ${D}/usr/lib/firmware
	install -m 0644 ${WORKDIR}/spi_cco_config.bin ${D}/usr/lib/firmware
	install -m 0644 ${WORKDIR}/FW.bin ${D}/usr/lib/firmware
}

FILES:${PN} += "${base_libdir}/systemd/system"
FILES:${PN} += "/usr/lib/firmware/*"
FILES:${PN} += "${ROOT_HOME}/**"
