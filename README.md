# Table of Contents
	1. Introduction
	2. Adding the meta-cg5317 Layer to Your Build
	3. Enabling and Testing Overlay

# 1 - Introduction
This repo holds the software necessary to enable SPI communication between PHYTEC's supported dev kits (AM62-Lyra / AM64-Electra) and the Lumissil IS31CG5317/IS32CG5327 device.

There are two items included in this meta-layer.

- **Device Tree Overlay** - contains the device tree changes that are needed to MUX the SPI pins on the dev kit.

- **Lumissil lms-eth2spi SPI Driver** - spi driver that will be built into the BSP.

# 2 - Adding the meta-cg5317 Layer to Your Build

Enter the `/sources` directory of your yocto build and clone the meta-layer

```bash
# Clone the meta-layer
git clone https://github.com/phytec-labs/meta-cg5317

# Add the layer to Yocto
bitbake-layers add-layer meta-cg5317
```

You can make sure the build was successful by checking to see if `*-cg5317.dtbo` is present on the SD cards `/boot ` partition.

# 3 - Enabling and Testing the Overlay

1. Stop the boot sequence in u-boot and run the following commands to enable the device tree overlay.

AM62
```bash
=> setenv overlays k3-am62-phyboard-lyra-cg5317.dtbo
=> saveenv
=> boot
```

AM64
```bash
=> setenv overlays k3-am64-phyboard-electra-cg5317.dtbo
=> saveenv
=> boot
```

2. Check to see if PLC device connected successfully.
``` bash 
dmesg | grep spi
```
AM62 Example Output
``` bash
root@phyboard-lyra-am62xx-2:~# dmesg | grep spi
[    1.180349] spi-nor spi0.0: mt35xu512aba (65536 Kbytes)
[    4.544930] lms_eth2spi: loading out-of-tree module taints kernel.
[    4.558884] Initialising lms_eth2spi version 0.0.7
[    4.569010] lms_eth2spi spi3.0: SPI controller min possibled speed  : 1464Hz
[    4.581273] lms_eth2spi spi3.0: SPI controller max possible speed   : 48000000Hz
[    4.588747] lms_eth2spi spi3.0: SPI lms driver configured max speed : 1000000Hz
[    4.596193] lms_eth2spi spi3.0: Translation to 'LSB first' happens in software
[    4.609659] lms_eth2spi spi3.0: Status 0 sanity check passed
[    4.620565] lms_eth2spi spi3.0: Using random MAC address: 00:16:e8:56:a0:13
```
3. At this point the hardware is setup and the PLC device is connected and waiting for application software to write/read from the device. Refer to and contact Lumissil for specific device documentation:
https://www.lumissil.com/applications/communication/electric-vehicles-charging/vehicle-charging/is32cg5317