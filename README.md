# LVGL ported to STM32U5G9J-DK2

## Overview

This STM32U5 series devboard is part of the family of ultra low power ST MCUs.
It has a large amount of internal memory and the embedded Neo-Chrom GPU
which accelerates LVGL rendering.
With such a large internal SRAM, direct double buffering is possible
at the great speed of internal SRAM.

## Buy

You can purchase a STM32U5G9J-DK2 from https://estore.st.com/en/stm32u5g9j-dk2-cpn.html

## Benchmark

Here is a video and measurement results of the benchmark demo running
in double-buffered direct mode with NemaGFX+NemaVG, LTDC driver with
VSYNC, and no OS enabled.

<!-- <a href="https://www.youtube.com/watch?v=XXXXXXXXXXXXXXXXXXXXXXX">
    <img src="https://github.com/user-attachments/assets/87c1f2e5-0260-4772-b711-13fdab467474" width="75%">
</a> -->

| Name                      | Avg. CPU | Avg. FPS | Avg. time | render time | flush time |
| :------------------------ | -------: | -------: | --------: | ----------: | ---------: |
| Empty screen              | 27%      | 29       | 6         | 6           | 0          |
| Moving wallpaper          | 28%      | 29       | 10        | 10          | 0          |
| Single rectangle          | 5%       | 29       | 0         | 0           | 0          |
| Multiple rectangles       | 10%      | 29       | 3         | 3           | 0          |
| Multiple RGB images       | 12%      | 30       | 3         | 3           | 0          |
| Multiple ARGB images      | 11%      | 30       | 3         | 3           | 0          |
| Rotated ARGB images       | 8%       | 29       | 2         | 2           | 0          |
| Multiple labels           | 61%      | 29       | 20        | 17          | 3          |
| Screen sized text         | 94%      | 20       | 46        | 40          | 6          |
| Multiple arcs             | 21%      | 29       | 3         | 3           | 0          |
| Containers                | 23%      | 29       | 5         | 5           | 0          |
| Containers with overlay   | 66%      | 29       | 21        | 20          | 1          |
| Containers with opa       | 29%      | 29       | 7         | 7           | 0          |
| Containers with opa_layer | 48%      | 29       | 14        | 14          | 0          |
| Containers with scrolling | 47%      | 29       | 15        | 15          | 0          |
| Widgets demo              | 91%      | 25       | 20        | 20          | 0          |
| All scenes avg.           | 36%      | 28       | 10        | 10          | 0          |

## Specification

### CPU and Memory
- **MCU:** Arm Cortex-M33 @160MHz
- **RAM:** 3MB internal
- **Flash:** 4MB internal, 128MB External
- **GPU:** Neo-Chrom (GPU2D), Chrom-Art (DMA2D)

### Display and Touch
- **Resolution:** 800x480
- **Display Size:** 5"
- **Interface:** Parallel RGB
- **Color Depth:** 24-bit (this project uses 16 by default)
- **Technology:** LCD
- **DPI:** 187 px/inch
- **Touch Pad:** Capacitive

### Connectivity
- Arduino-compatible pin headers
- Application USB-C
- GPIO header

## Getting started

### Hardware setup
- Connect a USB-C cable to the USB-C port labeled
  STLINK USB CN8 and your PC.

### Software setup
- Install [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html).

### Run the project
- Clone this repository repository:
  ```shell
  git clone --recurse-submodules https://github.com/lvgl/lv_port_stm32u5g9j-dk2.git 
  ```
- In STM32CubeIDE, click **File > Open Projects from File System**, click **Directory**,
  choose the cloned repo, and then click **Finish**.
- Click the hammer icon to build the project.
  The "Debug" build mode is used, even though full optimization is enabled.
- Click the play icon to upload it to the board.

### Debugging
- Click the bug icon to start the debugger.
- Interactive debugging is available over USB-C
- For the best debugging experience, change the optimization level to `-O0` instead
  of `-O3`. The program execution will be more similar to the source code.
  Right click the project in the **Project Explorer** sidebar, click **Properties**.
  Go to **C/C++ Build > Settings > MCU/MPU GCC Compiler > Optimization**.
  Choose **None (-O0)** Under **Optimization level**.

## Notes

Some things were changed from the default code generation.

The two linker scripts were modified
to reserve 750kb of RAM at the start of the region for a framebuffer.

```
/* Memories definition */
MEMORY
{
  FLASH	(rx)	: ORIGIN = 0x08000000, LENGTH = 4096K
  RAM2	(xrw)	: ORIGIN = 0x20000000, LENGTH = 750K
  RAM	(xrw)	: ORIGIN = 0x200bb800, LENGTH = 1746K
}
```

`RAM2` is not referenced anywhere. Instead, `Core/Src/lvgl_port.c` hardcodes the memory address `0x20000000`
for the LTDC driver. Also, in the .ioc config, LTDC has the framebuffer address set to `0x20000000`.
The 750kb size is 800x480x2 bytes.

The second framebuffer is declared in `Core/Src/lvgl_port.c` and is the same size.

The display is capable of 24 bit color depth but LTDC is configured as 16 bits for
reduced memory usage.

`USE_HAL_LTDC_REGISTER_CALLBACKS` has been enabled in `Core/Inc/stm32u5xx_hal_conf.h` to support the
LVGL LTDC driver.

## Contribution and Support

If you find any issues with the development board feel free to open an Issue in this repository. For LVGL related issues (features, bugs, etc) please use the main [lvgl repository](https://github.com/lvgl/lvgl).

If you found a bug and found a solution too please send a Pull request. If you are new to Pull requests refer to [Our Guide](https://docs.lvgl.io/master/CONTRIBUTING.html#pull-request) to learn the basics.

