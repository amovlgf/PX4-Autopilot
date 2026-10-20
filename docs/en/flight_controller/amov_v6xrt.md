# AMOV-V6XRT Bring-Up

:::warning
This is an experimental, internal bench target, not a supported flight controller.
The first milestone is NOR boot and USB communication with QGroundControl.
There are no sensor drivers, actuator outputs or flight control tasks in this image.
Successful compilation does not establish that the hardware acceptance tests below have passed.
:::

## Hardware And References

This port targets the `RT1176_CORE_V1_1` core board with the `RT1176_BASEBOARD_V1_0` baseboard described in the supplied hardware package.
Its development name is `amovlab/amov-v6xrt`.
Manufacturer support and a public purchase page have not been established; this is not an upstream board submission.

The reference combination is:

| Reference | Purpose | Boundary |
| --- | --- | --- |
| `boards/px4/fmu-v6xrt` | PX4/NuttX integration, RT1176 clocks, bootloader, USB and linker structure | No octal flash transition, DQS training, FRAM, sensor rails or flight-controller pin assignments |
| NuttX `boards/arm/imxrt/imxrt1170-evk` | Serial NOR FCB and ROM command sequence numbering | Convert to PX4's configuration structure; do not copy quad frequency or DQS sampling |
| Core/baseboard schematics, pin spreadsheet and component datasheets | Actual wiring, voltage, memory geometry and flash commands | Authoritative when demo configuration conflicts with schematics |
| ARK/AGAM FMUv6X-RT | Cross-check RT1176 integration | No additional copied BSP |
| NXP MR-Tropic/Tropic Community | Future NOR storage/parameter persistence | Deferred; RT106x addresses, clocks and ROM API calls are not portable verbatim |

Local source documents are under `/home/amovlgf/RT1176`:

- `RT1176核心板硬件设计说明/核心板硬件设计说明.pdf`
- `RT1176_BASEBOARD_V1_0.pdf`
- `1176核心板连接器接口引脚定义.xlsx`
- `1176核心板芯片资料/MT25QU128ABA1EW9-0SIT.PDF`
- `IMXRT1170RM.pdf` and `IMXRT1170IEC.pdf`

The vendor USB MSC/SDRAM examples are cross-checks only.
In particular, their UART12 console setting does not match the baseboard DEBUG connector, which is wired to UART1.

## Enabled Hardware

| Function | Configuration |
| --- | --- |
| CPU | M7, 24 MHz crystal, 996 MHz ARM PLL; M4 is not started |
| Main RAM | Dedicated OCRAM1/2: `0x20240000–0x2033ffff`, 1 MiB |
| FlexRAM | Application: 256 KiB ITCM and 256 KiB DTCM; DTCM is an additional heap region |
| HRT | GPT5, 24 MHz reference |
| DMA | NuttX eDMA and PX4 DMA allocator |
| Flash | MT25QU128ABA, 16 MiB, 1.8 V, FlexSPI1 |
| Console | UART1, 115200 baud, 8N1, no hardware flow control |
| USB | USB OTG1 device, CDC ACM, `/dev/ttyACM0` |
| LED | R13 blue LED, `GPIO_AD_02 / GPIO3_IO01`, active low |

SDRAM, eMMC, SD card, M4, sensor buses, ADC, CAN, Ethernet, PWM and PX4IO are not initialised.
The ordinary RAM region deliberately excludes the FlexRAM window and ECC areas used differently by other RT1176 boards.

### Pins And Connections

| Signal | MCU Pad | Core Connector / Baseboard |
| --- | --- | --- |
| NOR CS | `GPIO_SD_B2_06` | On core board |
| NOR CLK | `GPIO_SD_B2_07` | On core board |
| NOR IO0/IO1/IO2/IO3 | `GPIO_SD_B2_08/09/10/11` | On core board |
| UART1 TX | `GPIO_DISP_B2_08` | B5; DEBUG pin 3 → USB-UART RX |
| UART1 RX | `GPIO_DISP_B2_09` | D8; DEBUG pin 2 ← USB-UART TX |
| UART ground | GND | DEBUG pin 1 |
| USB1 D+/D− | USB OTG1 PHY | C10/C12; baseboard USB1 Type-C |
| SWD clock | SWCLK | C19; J4 pin 2 |
| SWD data | SWDIO | C21; J4 pin 3 |
| Debug reference | 3.3 V | J4 pin 1 → probe VTref |
| Debug ground | GND | J4 pin 4 |

Use a 3.3 V TTL UART adapter, not RS-232 levels.
VTref senses target voltage; power the board through its normal supply connection.
Check pin numbering against the connector marking before connecting a probe.

Start USB testing with a USB-A to Type-C data cable.
The supplied schematic does not show Type-C CC pull-down resistors, so C-to-C operation is not an acceptance requirement.
USB VBUS detection is connected to the shared 5 V supply: when externally powered, it may remain asserted after cable removal.
USB reconnection therefore requires hardware testing; the current port retains the upstream CDC/MAVLink behaviour.

## Flash And Image Layout

Both images contain the same 512-byte FCB, using single-pad SDR at 30 MHz with internal sampling loopback.
The commands are READ `03h`, READ STATUS `05h`, WRITE ENABLE `06h`, PAGE PROGRAM `02h`, 4 KiB ERASE `20h` and 64 KiB ERASE `D8h`.
All addresses are three bytes; page size is 256 bytes.
There is no mode-switch command, external DQS or chip-erase LUT sequence.
The flash must power up accepting standard SPI commands with three-byte addressing.
Check its non-volatile configuration if previous firmware changed it.

| Region | Start | End (Exclusive) | Maximum Size |
| --- | --- | --- | --- |
| Bootloader | `0x30000000` | `0x30020000` | 128 KiB |
| Application | `0x30020000` | `0x30400000` | 3968 KiB |
| Reserved / Untouched | `0x30400000` | `0x31000000` | 12 MiB |

FCB, ROM IVT, boot data and vectors are at offsets `0x400`, `0x1000`, `0x1020` and `0x2000` respectively in each image.
Application vectors are therefore at `0x30022000`.
ROM boot-data addresses and image lengths come from the corresponding linker script.
The common bootloader uses 4 KiB sector indices 32–1023 for application erasure, with a maximum application length of 4,063,232 bytes.
The upper bound is an inclusive sector index, not a sector count.
The bootloader's ROM flash operations execute from RAM.
The application preserves the ROM/bootloader XIP clock and does not reconfigure the flash.

### Development Identifiers

The bootloader and `.px4` package use local board ID `65001`, checked against the upstream prototypes at implementation time.
It is not an official allocation.
USB uses `1209:0001` in both images, with distinct product strings.
This is the [pid.codes test PID](https://pid.codes/1209/0001/), for internal experimentation only.
Obtain assigned USB and PX4 identifiers before distributing hardware/firmware.
QGC automatic board recognition and online firmware distribution are outside this milestone.

## Build And Inspect

From `/home/amovlgf/PX4-Autopilot`, with the GNU Arm Embedded 10.2.1 toolchain on `PATH`:

```sh
export PATH=/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH
export CCACHE_DIR=/tmp/rt1176-ccache
make amovlab_amov-v6xrt_bootloader
make amovlab_amov-v6xrt_default
python3 boards/amovlab/amov-v6xrt/tools/check_images.py
make check_format
```

After modifying a NuttX defconfig or changing toolchains, remove that target's `build/<target>` directory before rebuilding.
Build the two targets sequentially because they share the checkout's NuttX source/configuration.

Artifacts are:

- `build/amovlab_amov-v6xrt_bootloader/amovlab_amov-v6xrt_bootloader.elf`
- `build/amovlab_amov-v6xrt_bootloader/amovlab_amov-v6xrt_bootloader.bin`
- `build/amovlab_amov-v6xrt_default/amovlab_amov-v6xrt_default.elf`
- `build/amovlab_amov-v6xrt_default/amovlab_amov-v6xrt_default.bin`
- `build/amovlab_amov-v6xrt_default/amovlab_amov-v6xrt_default.px4`

The inspection tool checks FCB fields and LUTs, ROM headers, vector addresses, initial stack/reset vectors, loadable ELF segments, image bounds and `.px4` payload/board ID.
It also checks that the bootloader's flash routines are in RAM and that both FCBs match.
It cannot test electrical operation or a ROM API call on the target.

## Backup And Program With J-Link

Install SEGGER's J-Link software and pass the probe through to this Linux VM if applicable.
Use Cortex-M7 via SWD, initially at 1000 kHz.
The [SEGGER RT1170 support page](https://kb.segger.com/NXP_i.MX_RT1170) lists the required FlexSPI1 pin combination.
Select it explicitly with the [multiple flashloader syntax](https://kb.segger.com/J-Link_Multiple_Flashloader):

```sh
JLinkExe -device 'MIMXRT1176xxxA_M7?BankAddr=0x30000000&Loader=nCS@SDB206_CLK@SDB207_D0@SDB208_D1@SDB209_D2@SDB210_D3@SDB211' -if SWD -speed 1000 -autoconnect 1
```

Confirm that the device name is available in the installed J-Link version and matches the actual silicon.
Pin compatibility alone does not prove that the algorithm supports this particular 1.8 V Micron part.
Before any write, confirm JEDEC manufacturer/type/density `20 BB 18`, 16 MiB capacity and successful identification by the selected algorithm.
Record the silicon revision, J-Link version, flash information and boot straps.
Do not proceed on a loader/flash mismatch.

1. With the original firmware and memory mapping still readable, halt and run `tools/backup.jlink` using the selected device/loader arguments above plus `-CommanderScript boards/amovlab/amov-v6xrt/tools/backup.jlink`.
   This saves the full 16 MiB twice to `build/amov-v6xrt-original-1.bin` and `build/amov-v6xrt-original-2.bin`.
   Confirm both files are exactly 16,777,216 bytes, compare them with `cmp`, and retain their SHA-256 hashes and a separate backup copy.
   Check that the contents are plausible, not a failed read or all-ones mapping.
   If the existing mapping is unreadable, stop and establish a verified read-only Flash access method before erasing anything.
2. Run `tools/check_images.py` and retain its output.
3. Run `tools/flash_bootloader.jlink` with the same probe arguments.
   It programs the BIN at `0x30000000` and uses `verifybin` for readback verification.
   Check the entire log for success before proceeding.
4. Power-cycle and verify the bootloader stage, including USB enumeration.
   An existing application may still occupy the application partition; do not use its behaviour as validation of this new application.
5. Run `tools/flash_application.jlink` with the same probe arguments.
   It programs the application BIN at `0x30020000` and verifies it.
   Check the full log, then disconnect the debugger and perform a real power cycle.

All command files assume the current directory is the repository root.
The write scripts are deliberately separate from the backup script and are not run by the build.
They are templates pending validation on this exact board/flash combination.
`loadbin` uses the selected algorithm's sector programming; do not issue a whole-chip erase.
Never pass the compressed JSON `.px4` file to `loadbin`.
The bootloader has USB upload support, but the initial delivery/acceptance path is J-Link and readback verification.

## Boot And Recovery

Normal internal boot uses logical `BOOT_MODE[1:0]=10`; serial downloader uses `01`.
Confirm switch polarity and the FlexSPI1 boot-source straps against this baseboard, not an EVK switch-number table.
These mode encodings are documented in NXP's [RT1170 manufacturing guide](https://mcuxpresso.nxp.com/mcuxsdk/25.09.00/html/middleware/mcu_bootloader/docs/iMXRT1170_Manufacturing_User_Guide/topics/appendix.html).
Do not program OTP fuses for this bring-up.

The application uses UART1 for NSH.
The bootloader uses USB only; with VBUS present its normal wait is configured as five seconds.
An invalid application or explicit bootloader request can leave it waiting for upload.

Recovery options:

- J-Link SWD: restore the backed-up image using the verified loader and address, then read it back and compare.
- MCU-Link/LinkServer: use an RT1176 M7 target and a Flash algorithm verified for the same FlexSPI1 SD_B2 pins, 1.8 V MT25QU128 and 16 MiB capacity.
  Do not assume an arbitrary EVK board preset has the correct wiring or memory geometry.
- ROM USB downloader: set `BOOT_MODE[1:0]=01`, reset, and use NXP's RT1170 manufacturing tools to load a compatible RAM flashloader.
  Configure that flashloader for this specific NOR part before programming the two BINs at the addresses above.
  The ROM downloader is a separate USB protocol from PX4 CDC ACM.

Return the boot straps to normal internal boot and remove the debugger for cold-start tests.
A debugger-assisted reset is insufficient proof: SEGGER documents RT1170 reset handling that can manually set SP/PC after ROM enters its debug loop.

## Startup And QGroundControl

The dedicated `ROMFS/amov-v6xrt/init.d/rcS` mounts tmpfs at `/fs/ram` and selects `/fs/ram/parameters.bson`.
It initialises empty parameter storage without an SD-card or MTD recovery loop.
`SYS_USB_AUTO=2` starts MAVLink when USB CDC becomes available.
uORB and the parameter/work-queue infrastructure are initialised by the board; the startup script starts Commander, load monitoring, events and USB autostart.
No navigation, sensor or control tasks are started.
Some upstream sensor/geofence parameter definitions are included because Commander consumes them even when those tasks are absent.
They remain owned by their original modules.

Normal health/arming checks remain active.
Missing IMU, barometer, estimator, power and other flight inputs are expected to prevent readiness.
The board has `CONFIG_BOARD_NO_SDCARD=y`; this disables the check for an intentionally absent SD card without bypassing sensor checks.
Do not disable checks to obtain a green vehicle status.

Connect USB1 using an A-to-C data cable and identify the new `/dev/ttyACM*` on the host.
If QGC does not connect automatically, add a serial communication link for that device in QGC.
A serial baud setting of 115200 is sufficient for the UI; USB CDC does not use the UART1 baud rate.
The USB device name may change between bootloader and application, so select the application after it has started.
Use MAVLink Inspector to check `HEARTBEAT` and request/version-check `AUTOPILOT_VERSION`.
Open MAVLink Console and run:

```sh
ver all
top
mavlink status
cdcacm_autostart status
param show SYS_USB_AUTO
param show MAV_SYS_ID
```

For a benign write/readback test, change `MAV_SYS_ID` from its default `1` to `2`, save it and read it back in the same boot.
QGC may reconnect under the new system ID.
Verify that `param save` reports success on RAM storage.
After `reboot` and after a complete power cycle, confirm `MAV_SYS_ID` is `1` again and `SYS_USB_AUTO` is `2`.
There is intentionally no persistent parameter backend in this phase.

## Hardware Acceptance Record

Record the artifact SHA-256 values and test results; leave a row pending until measured on hardware.

| Test | Required Result |
| --- | --- |
| Both builds and image inspection | Success; FCB, IVT, entry, load segments and partition checks pass |
| Bootloader programming | Readback matches; USB enumeration observed |
| Application programming | Readback matches at `0x30020000` |
| Ten independent cold boots, debugger disconnected | All reach UART NSH without abnormal resets |
| UART commands | `ver all`, `top`, `mavlink status` work at 115200 8N1 |
| QGC for at least ten minutes | Continuous heartbeat/link; all advertised parameters received |
| MAVLink functionality | Parameter write/readback, version query and interactive console work |
| USB reconnect on USB power | Application enumerates and QGC reconnects after reconnect/power-up |
| USB reconnect on external power | Cable removal/reinsertion recovers even with shared 5 V keeping VBUS sense high |
| Parameter lifetime | Same-boot save/read works; reboot and power-off restore defaults |
| Missing storage | No SD/MTD-driven startup loop or repeated automatic reboot |

If externally powered hotplug fails, capture UART output, `mavlink status`, `cdcacm_autostart status` and host USB logs before changing the shared USB logic.
The shared 5 V VBUS wiring is a known acceptance risk, not a tested success.

After this milestone passes, evaluate NOR partitioning, MTD/LittleFS and persistent parameters using Tropic as a design reference.
Then validate external storage, each sensor, communication port and output separately.
