# Table of Contents
	1. Introduction
	2. Using the manifest.xml
	3. Enabling and Testing Overlay

# 1 - Introduction
This repo holds the software necessary to enable SPI communication between PHYTEC's supported dev kits (AM62-Lyra / AM64-Electra) and the Lumissil IS31CG5317/IS32CG5327 device. This package is still a work in progress.

There are some key items included in this meta-layer.

- **Device Tree Overlay** - contains the device tree changes that are needed to MUX the SPI pins on the dev kit.

- **Lumissil lms-eth2spi SPI Driver** - spi driver that will be built into the BSP.

- **Host Load Services** - firmware loading services that are used on bootup when the lms is in host-load moad.

- **Manifest.xml** - a manifest that allows you to download all required software to build a BSP.

# 2 - Using the Manifest

The manifest is used to download all software required to build the Yocto BSP. This manifest uses the phyLinux software. More information can be found here:
[phyLinux Installation](https://docs.phytec.com/latest/phycore-am62x/developingwithyocto/buildBSP.html#download-the-bsp-meta-layers)

Begin by creating a directory for your BSP

```bash
# Make directory
mkdir ~/cg5317-bsp
```

Download the manifest.xml file and place it into your BSP folder. Use the following to checkout this software into a BSP.
```bash
# Downloading Yocto BSP
phyLinux init -x manifest.xml
```

You can then source and build your BSP.

To make sure the build was successful by checking to see if `*-cg5317.dtbo` is present on the SD cards `/boot ` partition.

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

# 3 - Host Software
1. The host software example binaries were cross-compiled and included in this layer.
### Communicating with the Device
Device information can be found by running the management tool and passing it the correct mac address of the hardware. Our kit gets a response using the following:
``` bash
root@phyboard-lyra-am62xx-2:~# ./examples/management_tool/management_tool -d 00:16:E8:00:00:01 -a`cat /sys/class/net/seth0/address` -c "device_info 3"
```
### Good Connection
``` bash
root@phyboard-lyra-am62xx-2:~# ./examples/management_tool/management_tool -d 00:16:E8:00:00:01 -a`cat /sys/class/net/seth0/address` -c "device_info 3"
device_info:
        cco mode                      = 2 (always)
        host iface                    = 0 (SPI)
        Terminal Equipment Identifier = 1
        MAC address                   = 00:16:E8:00:00:01
        Manufacturer HFID             = Lumissil
        User HFID                     = userHFID
        AVLN HFID                     = AVLNHFID
        Network Membership Key        = 49CC4981698D5F709BBE660C50306C94
        Network Identifier            = 2FECB0D5327903
        security level                = 0 (Simple connect)
        SNID                          = 0x0C
        Max Receiver sensitivity      = 43 dB
        PLC Frequency selection       = 0 (50 Hz)

command: 'device_info 3' finished successfully
```

The examples were cross-compiled using the latest [Phytec SDK](https://download.phytec.de/Software/Linux/BSP-Yocto-AM62x/BSP-Yocto-Ampliphy-AM62x-PD23.2.0/sdk/ampliphy-xwayland/)
