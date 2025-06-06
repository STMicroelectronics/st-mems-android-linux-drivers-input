Important Notice
================

The current repository contains a collection of Linux kernel device drivers for our MEMS sensors using input API, which has become obsolete.
The kernel support for sensors has evolved and an improved solution for kernel integration of MEMS sensors is provided by IIO API.
Due to intrinsic limitations of the input framework, the drivers here contained are therefore limited in terms of performance and only the basic hardware embedded features of the MEMS sensors are supported.
STMicroelectronics provides full support for our MEMS sensors through a complete suite of Linux Device Drivers (LDD) leveraging on the IIO API/framework made available on GitHub:

https://github.com/STMicroelectronics/st-mems-android-linux-drivers-iio/

__We strongly encourage and recommend that users upgrade to the suite of IIO drivers when integrating our sensors in Linux kernel. These upgrades are especially important for our most recent devices and for targets expected to run Android distributions.__

The current repository supports the simpler and/or old designs for pure Linux. The designs for the latest Android versions are supposed to fail their Android CTS tests for sensors due to tightening requirements impacting high-speed data read and use of buffers/FIFO.

Index
=======
	* Introduction
	* Integration details
	* More information
	* Copyright


Introduction
==============
This repository contains STMicroelectronics MEMS sensors for Linux/Android kernels leveraging on **Input framework**.

The repository currently contains drivers supporting the following Longterm Support (LTS) Kernel releases defined as for "The Linux Kernel Archives", see [Longterm Kernel releases][1]:

>    * 4.4
>    * 4.9
>    * 4.14
>    * 4.19
>    * 5.4
>    * 5.10
>    * 5.15
>    * 6.1
>    * 6.6
>    * 6.12

STM sensor drivers are located under the directory [drivers/input/misc/st](https://github.com/STMicroelectronics/st-mems-android-linux-drivers-input/tree/master/drivers/input/misc/st) organized in folders by sensor type:

### Inertial Module Unit (IMU):

> LSM330, LSM330DLC, LSM6DS0,
> LSM6DS3, LSM6DSL, LSM6DSM, LSM9DS0, LSM9DS1, ISM330DLC,
> ASM330LHH, LSM6DSO, LSM6DSO32, LSM6DSOX, LSM6DSO32X, LSM6DSR,
> ISM330DHCX

### eCompass:

> LSM303AGR, LSM303AH, LSM303C, LSM303D, LSM303DLHC, ISM303DAC

### Accelerometer:

> AIS2DW12, AIS2IH, AIS328DQ, AIS3624DQ, H3LIS100DL, H3LIS331DL, LIS2DE,
> LIS2DE12, LIS2DH, LIS2DH12, LIS2DS12, LIS2HH12, LIS331DLH, LIS331HH,
> LIS3DE, LIS3DH, LIS3DSH, LIS2DW12, LIS3DHH, IIS3DHHC, IIS2DH

### Gyroscope:

> A3G4250D, L3G4200D, L3GD20, L3GD20H

### Magnetometer:

> LIS3MDL, LIS2MDL, IIS2MDC

### Humidity:

> HTS221

### Pressure:

> LPS22CH, LPS22HB, LPS22HD, LPS22HH, LPS22DF, LPS25H, LPS25HB,
> LPS27HHW, LPS27HHTW, LPS33HW, LPS35HW

### Ultraviolet:

> UVIS25

### Temperature:

> STTS751, STTS22H


Data collected by STM sensors are pushed from kernel-space to user-space through the Linux kernel Input framework using *EV_MSC* events. User space applications can get sensor events by reading the related input device created in the /dev directory. Please see [Input][2] for more information.

All STM MEMS sensors support *I2C/SPI* digital interface. Please refer to [I2C][3] and [SPI][4] for detailed documentation.


Integration details
=====================

## Source code integration
From your kernel source code root directory add the git remote (i.e. stmems_input_github) for this repository:
```bash
git remote add stmems_input_github \
               https://github.com/STMicroelectronics/st-mems-android-linux-drivers-input.git
```

Fetch the just added remote:
```bash
git fetch stmems_input_github
```

There are now two ways you may choose for integrating the drivers code into your kernel target branch:
* merge (**suggested strategy**)
* rebase

### merge
Merge the remote branch stmems_input_github/master with your target kernel source branch (i.e branch linux-4.19.y):

```bash
git merge --allow-unrelated-histories \
          linux-4.19.y \
          stmems_input_github/master
```

### rebase
Rebase the remote branch stmems_input_github/master on top of your target kernel source branch (i.e branch linux-4.19.y):

```bash
git rebase --no-fork-point \
           linux-4.19.y \
           stmems_input_github/master
```

Note: older git versions (i.e.: 2.7.4) would require to use sligthly different options:

```bash
git merge --no-fork-point \
          linux-4.19.y \
          stmems_input_github/master
```

```bash
git rebase -Xno-renames \
           --no-fork-point \
           linux-4.19.y \
           stmems_input_github/master
```

## Apply patches
Now that drivers code has been added to the target kernel branch, few patches needs to be added in order to add STM drivers into Kconfig & Makefile systems

Apply the patches available in the just added repository selecting the proper kernel release directory (i.e for branch linux-4.19.y):

```bash
git am stm_input_patches/4.19.y/*-stm-*.patch
```

### Notes about Supported and WIP (Work In Progress) LTS kernel versions
Supported LTS kernel versions have patches for enabling their build system in directory:
```
stm_input_patches/<K.[X]X>.y/
```
where K stands for VERSION and [X]X for PATCHLEVEL according to kernel Makefile.

On the opposite, directories identified with suffix .WIP:
```
stm_input_patches/<K.[X]X>.y.WIP/
```
are for LTS kernel version whose support is WIP, hence using the above described procedure for enabling their build system and building them is premature and may result in errors.

## Configuration
A folder named ``stm_input_configs`` is provided containing the default configs for the supported drivers. Following are two proposed alternative ways (beside the traditional ones) for enabling configurations.
``common_defconfig`` is mandatory for enabling STM MEMS Sensors drivers' configuration.

### Modify target defconfig
Sensors defconfig can be appended to the board defconfig (i.e. if your current configuration file is arch/arm/configs/stm32_defconfig):

```bash
cat stm_input_configs/common_defconfig >> all_sensors_defconfig
cat stm_input_configs/accel_defconfig >> all_sensors_defconfig
cat all_sensors_defconfig >> arch/arm/configs/stm32_defconfig
```

Alternatively, it can be done at build time without altering the board config file, as follow.

### Merge configuration
Driver config can be merged into current target pre-configured kernel using a script available in the kernel itself:

```bash
export ARCH=arm
export CROSS_COMPILE=arm-linux-gnu-
cat stm_input_configs/common_defconfig >> ./all_sensors_defconfig
cat stm_input_configs/imus_defconfig >> ./all_sensors_defconfig
scripts/kconfig/merge_config.sh -n .config ./all_sensors_defconfig
```

### Traditional Kernel configuration

Configure kernel with *make menuconfig* (alternatively use *make xconfig* or *make qconfig*).

In order to explain how to integrate STM sensors in a different kernel, please consider the following *LSM6DSM* IMU example

>		Device Drivers  --->
>			Input device support  --->
>			[*]   Miscellaneous devices  --->
>				<*>   STM MEMs Device Drivers  --->
>					<M>   Inertial motion unit  --->
>						<M>   STMicroelectronics LSM6DSM sensor
>


### Device Tree configuration

> Documentation Device Tree Bindings are published under: Documentation/devicetree/bindings/input

> To enable driver probing, add the lsm6dsm node to the platform device tree as described below.

> **Required properties:**

> *- compatible*: "st,lsm6dsm"

> *- reg*: the I2C address or SPI chip select the device will respond to

> *- interrupt-parent*: phandle to the parent interrupt controller as documented in [interrupts][5]

> *- interrupts*: interrupt mapping for IRQ as documented in [interrupts][5]

>
>**Recommended properties for SPI bus usage:**

> *- spi-max-frequency*: maximum SPI bus frequency as documented in [SPI][4]
>
> **Optional properties:**

> *- st,drdy-int-pin*: MEMS sensor interrupt line to use (default 1)

> I2C example (based on Raspberry PI 3):

>		&i2c0 {
>			status = "ok";
>			#address-cells = <0x1>;
>			#size-cells = <0x0>;
>			lsm6dsm@6b {
>				compatible = "st,lsm6dsm";
>				reg = <0x6b>;
>				interrupt-parent = <&gpio>;
>				interrupts = <26 IRQ_TYPE_EDGE_RISING>;
>		};

> SPI example (based on Raspberry PI 3):

>		&spi0 {
>			status = "ok";
>			#address-cells = <0x1>;
>			#size-cells = <0x0>;
>			lsm6dsm@0 {
>				spi-max-frequency = <500000>;
>				compatible = "st,lsm6dsm";
>				reg = <0>;
>				interrupt-parent = <&gpio>;
>				interrupts = <26 IRQ_TYPE_EDGE_RISING>;
>			};

More Information
=================
[http://st.com](http://st.com)

[https://git.kernel.org/cgit/linux/kernel/git/torvalds/linux.git/tree/Documentation/input](https://git.kernel.org/cgit/linux/kernel/git/torvalds/linux.git/tree/Documentation/input)

[https://git.kernel.org/cgit/linux/kernel/git/torvalds/linux.git/tree/Documentation/i2c](https://git.kernel.org/cgit/linux/kernel/git/torvalds/linux.git/tree/Documentation/i2c)

[https://git.kernel.org/cgit/linux/kernel/git/torvalds/linux.git/tree/Documentation/spi](https://git.kernel.org/cgit/linux/kernel/git/torvalds/linux.git/tree/Documentation/spi)

[https://git.kernel.org/cgit/linux/kernel/git/torvalds/linux.git/tree/Documentation/devicetree/bindings/interrupt-controller/interrupts.txt](https://git.kernel.org/cgit/linux/kernel/git/torvalds/linux.git/tree/Documentation/devicetree/bindings/interrupt-controller/interrupts.txt)

[https://git.kernel.org/cgit/linux/kernel/git/torvalds/linux.git/tree/Documentation/devicetree/bindings/gpio/gpio.txt](https://git.kernel.org/cgit/linux/kernel/git/torvalds/linux.git/tree/Documentation/devicetree/bindings/gpio/gpio.txt)


Copyright
===========
Copyright (C) 2022, 2025 STMicroelectronics

This software is distributed under the GNU General Public License - see the accompanying COPYING file for more details.

[1]: https://www.kernel.org/category/releases.html "Longterm Kernel releases"
[2]: https://git.kernel.org/cgit/linux/kernel/git/torvalds/linux.git/tree/Documentation/input "Input"
[3]: https://git.kernel.org/cgit/linux/kernel/git/torvalds/linux.git/tree/Documentation/i2c "I2C"
[4]: https://git.kernel.org/cgit/linux/kernel/git/torvalds/linux.git/tree/Documentation/spi "SPI"
[5]: https://git.kernel.org/cgit/linux/kernel/git/torvalds/linux.git/tree/Documentation/devicetree/bindings/interrupt-controller/interrupts.txt "interrupts"
[6]: https://git.kernel.org/cgit/linux/kernel/git/torvalds/linux.git/tree/Documentation/devicetree/bindings/gpio/gpio.txt "gpio"
