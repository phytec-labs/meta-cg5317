DESCRIPTION = "Open PLC utils"
HOMEPAGE = "https://github.com/qca/open-plc-utils"
LICENSE = "BSD-3-Clause-Clear"
LIC_FILES_CHKSUM = "file://LICENSE;md5=7d83a9e9a9788beb9357262af385f6c7"
PACKAGE_STRIP = "no"

RM_WORK_EXCLUDE += "${PN}"

S = "${WORKDIR}/git"

SRC_URI = "git://github.com/qca/open-plc-utils.git;protocol=https;branch=master \
	   file://0001-slac-fix-cm_mnbc_sound_indicate-not-according-to-spe.patch \
	   file://0002-slac-fix-RND-field-incorrect-size-inside-session-str.patch \
	   file://0003-slac-send-CM_START_ATTEN.IND-three-times.patch \
	   file://0004-rework-slac-code-to-support-multiple-device-situatio.patch \
	   file://0005-fix-evse-not-accepting-EVs-SLAC-PARAMS-with-RunId-0x.patch \
	   file://pev.ini \
	   file://evse.ini \
	   "
SRCREV = "358dfcf78bdaf7b0b13dcdf91cb1aae1789f2770"

TARGET_CC_ARCH += "${LDFLAGS}"

do_compile() {
	make
}

do_install() {
	install -d ${D}${bindir}
	install -m 0755 slac/evse ${D}${bindir}
	install -m 0755 slac/pev ${D}${bindir}

	install -d ${D}${sysconfdir}
	install -m 0644 ${WORKDIR}/pev.ini ${D}${sysconfdir}/pev.ini
	install -m 0644 ${WORKDIR}/evse.ini ${D}${sysconfdir}/evse.ini
}

inherit pkgconfig

DEPENDS = "ncurses gcc libnetfilter-conntrack libpcap libnl flex bison bison-native zlib libsodium liburcu libnet"
