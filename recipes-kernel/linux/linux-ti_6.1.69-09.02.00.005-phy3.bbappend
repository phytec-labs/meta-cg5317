FILESEXTRAPATHS:prepend := "${THISDIR}/files:"

SRC_URI:append = " \
	file://0001-arm64-dts-ti-k3-am62-phyboard-lyra-cg5317-overlay.patch \
	file://0002-arm64-dts-ti-k3-am64-phyboard-electra-cg5317-overlay.patch \
"
