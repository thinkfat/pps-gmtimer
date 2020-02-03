Beaglebone Black Hardware Counter Capture Driver
================================================

Changes to ddrown version
-------------------------

I adapted the pps-gmtimer from Dan Drown. He used functions that are not longer exported by linux-kernel-driver (dm-timer) (more info: https://patchwork.kernel.org/patch/10106287/). The adapted Module access the necessary functions through the omap_dm_timer_ops struct (dmtimer-omap.h). 
So the driver can compiled without changing the linux kernel (4.14.108).
I did NOT tested the tclkin feature. 


Building the module on BBB
--------------------------

  * sudo apt-get update && sudo apt-get upgrade
  * sudo apt-get install linux-headers-$(uname -r)
  * git clone https://github.com/kugelbit/pps-gmtimer
  * cd pps-gmtimer
  * make

The Output after make should be something like this:

```
make -C /lib/modules/4.14.108-ti-r104/build M=$PWD ARCH=arm
make[1]: Entering directory '/usr/src/linux-headers-4.14.108-ti-r104'
  AR      /home/ubuntu/pps-gmtimer/built-in.o
  CC [M]  /home/ubuntu/pps-gmtimer/pps-gmtimer.o
  Building modules, stage 2.
  MODPOST 1 modules
  CC      /home/ubuntu/pps-gmtimer/pps-gmtimer.mod.o
  LD [M]  /home/ubuntu/pps-gmtimer/pps-gmtimer.ko
make[1]: Leaving directory '/usr/src/linux-headers-4.14.108-ti-r104'
```


Installing the Device Tree Overlay on BBB
-----------------------------------------

The device-tree-overlay-file (DD-GPS-00A0.dtbo) defines to use UART2 (Pins: P9.21, P9.22) as UART for your GPS-Board and to use Timer4 (P8.7) as PPS-Device Pin

 * make DD-GPS-00A0.dtbo
 * cp DD-GPS-00A0.dtbo /lib/firmware/
 * add the following lines to /boot/uEnv.txt:
 * * under line ```###Custom Cape```
 * * dtb_overlay=/lib/firmware/DD-GPS-00A0.dtbo
 * * under line ```###Cape Universal Enable```
 * * enable_uboot_cape_universal=1
 * * cape_enable=bone_capemgr.enable_partno=DD-GPS
 * * under line ```###Disable auto loading of virtual capes (emmc/video/wireless/adc)```
 * * disable_uboot_overlay_video=1
 * * disable_uboot_overlay_audio=1
 * sudo reboot
 
 load pps-gmtimer kernel modul on BBB
-------------------------------------

  * go to build directory
  * ```sudo insmod pps-gmtimer.ko```
  * check log messages with ```dmesg```

In Case of Success the Output for dmesg should be something like this:

```
[  175.901727] pps_gmtimer: loading out-of-tree module taints kernel.
[  175.908995] pps-gmtimer: timer name=timer4 rate=24000000Hz
[  175.909064] pps-gmtimer: using system clock
[  175.913184] pps pps0: new PPS source timer4
[  175.913217] clocksource: timer4: mask: 0xffffffff max_cycles: 0xffffffff, max_idle_ns: 79635851949 ns
```

Using an external oscillator (TCLKIN) - NOT TESTED
--------------------------------------------------

To use an external clock source on pin P9.41 (TCLKIN).  It accepts up to a 24MHz clock.

 * apply the patch kernel-tclkin.patch to your kernel
 * If you're not using a 24MHz clock, update the DEFINE\_CLK\_FIXED\_RATE tclkin\_ck definition in arch/arm/mach-omap2/cclock33xx\_data.c 
 * rebuild your kernel
 * Use the device tree overlay file DD-GPS-TCLKIN-00A0.dtbo, which has the pinctl changes needed

To use this clock as your system time source:

> echo timer4 > /sys/devices/system/clocksource/clocksource0/current\_clocksource

If you're not using the timer4 hardware, use the other timer's name in place.

To switch back to the default time source:

> echo gp\_timer > /sys/devices/system/clocksource/clocksource0/current\_clocksource


Monitoring operation
--------------------

  * cat /sys/class/pps/pps0/assert

The sysfs files in /sys/devices/ocp.3/pps\_gmtimer.\* contain the counter's current state:

 * capture - the counter's raw value at PPS capture
 * count\_at\_interrupt - the counter's value at interrupt time
 * interrupt\_delta - the value used for the interrupt latency offset
 * pps\_ts - the final timestamp value sent to the pps system
 * timer\_counter - the raw counter value
 * stats - the number of captures and timer overflows
 * timer\_name - the name of time timer hardware
 * ctrlstatus - the state of the TCLR register (see the AM335x Technical Reference Manual for bit meanings)

The program "watch-pps" will watch these files and produce an output that looks like:

 > 1423775690.000 24000010 169 0.000007041 0 3988681035 -0.000001434

The columns are: pps timestamp, capture difference, cycles between capture and interrupt, interrupt\_delta, raw capture value, offset
