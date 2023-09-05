SUMMARY = "Lumissil eth2spi driver"
DESCRIPTION = "SPI driver for lumissil chips"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://COPYING;md5=188737c400ccb238ad6a192ffaf4a7b9"
MODULE_NAME = "lms_eth2spi"

inherit module

SRC_URI = "file://Makefile \
           file://lms_eth2spi.c \
           file://COPYING \
          "

RM_WORK_EXCLUDE += "${PN}"
S = "${WORKDIR}"

KERNEL_MODULE_AUTOLOAD += "${MODULE_NAME}"

FILES_${PN} = "/lib/modules/${KERNEL_VERSION}/kernel/${MODULE_NAME} \
    ${sysconfdir}/modules-load.d/${MODULE_NAME}.conf"

# The inherit of module.bbclass will automatically name module packages with
# "kernel-module-" prefix as required by the oe-core build environment.

