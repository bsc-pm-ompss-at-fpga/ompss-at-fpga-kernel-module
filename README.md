# OmpSs@FPGA Linux Driver

This repository contains the Linux device driver for the OmpSs@FPGA toolchain.
It provides access to different information/capabilities of the fpga device.
The main devices created by the driver are:

 - `/dev/hwruntime/` contains a set of device files allowing to map the HW runtime queues into userspace memory and interact with them.
 - `/dev/xdma` provides the capabilities to allocate memory accessible by the fpga device and communicate with DMA engines.
 - `/dev/hwcounter` provides access to the HW instrumentation counter used to trace the accelerators execution.
 - `/dev/bitinfo/` contains a set of device files with the information about the bitstream.
    How it was generated, the accelerators that it contains, the features available in the design, etc.

### Build the driver

To build the driver, you need the kernel headers or sources of your revision.
In a Debian based system, you can install the kernel header files for the currently running kernel by running the following in a terminal:
```
sudo apt-get install linux-headers-$(uname -r)
```

Once you have the kernel headers available, you can proceed with:

  1. Clone the repository or download the latest stable version.
     ```
     git clone https://gitlab.bsc.es/ompss-at-fpga/ompss-at-fpga-kernel-module.git
     cd ompss-at-fpga-kernel-module
     ```

  2. (Optional) If you are cross-compiling, set `KDIR`, `CROSS_COMPILE` and `ARCH` environment variables.
     - `KDIR` should point to the folder containing the kernel headers.
       Otherwise, they are expected to be in `/lib/modules/$(shell uname -r)/build` folder.   
       ```
       export KDIR=/home/my_user/kernel-headers
       ```

     - `ARCH` should be set to the target architecture you're compiling to.
       ```
       export ARCH=arm64
       ```

     - `CROSS_COMPILE` must contain the build triplet for your target system.
        ```
        export CROSS_COMPILE=aarch64-linux-gnu-
        ```

  3. Build the kernel module.
     ```
     make
     ```

  4. (Optional) Install the kernel module and udev rules.
     ```
     make install
     ```
