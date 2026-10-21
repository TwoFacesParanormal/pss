# Paranormal Sensor Station

Paranormal Sensor Station by Two Faces records sensor changes, displays their history, and sends alerts to the [Bluetooth web app](https://twofacesparanormal.com/pssbt/index.html).

## Choose your hardware and firmware

**There are two hardware generations and two separate versions of this code. Choose the sketch for the board you own.**

| Hardware | PSS firmware | Arduino sketch | Status as of October 21, 2026 |
| --- | --- | --- | --- |
| **M5Stack StickS3 / M5StickS3** (ESP32-S3, K150) | **3.1** | [`pss2_S3/pss2_S3.ino`](pss2_S3/pss2_S3.ino) | Current hardware and current PSS firmware |
| **M5Stack StickC Plus2 / M5StickC PLUS2** (ESP32, K016-P2) | **2.1** | [`pss2_1/pss2_1.ino`](pss2_1/pss2_1.ino) | Legacy hardware and legacy PSS firmware |

M5Stack marks the [StickC Plus2 as EOL (End of Life)](https://shop.m5stack.com/products/m5stickc-plus2-esp32-mini-iot-development-kit) and recommends the [StickS3](https://docs.m5stack.com/en/core/StickS3) as its replacement. The Plus2 code remains here for existing stations. Firmware 3.1 targets the StickS3; it is not a Plus2 update. Use the matching firmware for Wi-Fi updates, too.

The folder names retain their earlier names so existing Arduino sketch paths keep working. The `pss2_S3` folder contains **version 3.1**, despite the `2` in its name. These are two sketch folders in one repository, not two Git branches. The former root-level `pss.ino` is now in `pss2_1/pss2_1.ino`.

## Current version: StickS3, firmware 3.1

- [StickS3 build instructions, dependencies, and controls](pss2_S3/README.md)
- [Precompiled StickS3 3.1 application firmware](firmware/StickS3/3.1/pss-sticks3-v3.1.bin), for the existing PSS Wi-Fi updater
- [Firmware checksums and build information](firmware/StickS3/3.1/README.md)
- [Changes in 3.1](CHANGELOG.md)

The S3 version uses M5Unified for the display, speaker, IMU, buttons, and power management. It supports the ENV III HAT on the original single-row HAT position of the StickS3's two-row Hat2-Bus. That connector uses **SDA GPIO 8 and SCL GPIO 0**. The Grove connector uses **SDA GPIO 9 and SCL GPIO 10**. The extra HAT row is not required for this ENV III HAT setup.

ENV III uses an SHT30 for temperature/humidity and a QMP6988 for pressure. The HAT's pressure address is `0x56`; the Grove ENV III Unit commonly uses `0x70`. Version 3.1 checks both addresses, runs the bus at 100 kHz, validates temperature/humidity data, and retries a detected sensor after repeated read failures. Connect the sensor before startup; use Reset after changing the peripheral setup.

The Plus2 and S3 have different processors, pin assignments, power management, audio, and buttons. The S3 has a speaker and no programmable user LED, so references to the older station's flashing red indicator do not apply to the S3.

## Legacy version: StickC Plus2, firmware 2.1

Use [`pss2_1/pss2_1.ino`](pss2_1/pss2_1.ino) for an existing StickC Plus2 station. Its original ENV HAT pins are **SDA GPIO 0 and SCL GPIO 26**; its Grove pins are **SDA GPIO 32 and SCL GPIO 33**.

The legacy version has its own dependencies and controls. See the [Plus2 notes](pss2_1/README.md). The S3 sensor fixes and screen changes have not been backported to it. Older StickC and StickC Plus boards are not additional verified targets of this repository.

## Project location and the old Arduino copy

Development moved out of the Arduino sketchbook into:

```text
~/Projects/Paranormal Sensor Station/
```

That Projects folder is the working repository. An older copy still exists at `~/Documents/Arduino/pss2_1/`. It is a pre-migration copy, not the current workspace. The two local Plus2 sketches matched when checked on October 21, 2026. Open sketches from Projects to avoid editing or uploading an old Arduino copy by mistake. Arduino libraries can still live in `~/Documents/Arduino/libraries/`; that is separate from the sketch location.

For a fresh clone, keep each `.ino` inside its matching folder with `logo.h` alongside it. In Arduino IDE, open the specific sketch listed in the hardware table, then select the matching board.

## Build outputs and upload files

The current local S3 build is exported to:

```text
pss2_S3/build/m5stack.esp32.m5stack_sticks3/
```

From that folder, `pss2_S3.ino.bin` is the **application image** used by the PSS Wi-Fi updater. The published copy is named `pss-sticks3-v3.1.bin` to make its hardware and version explicit. Bootloader, partition, and merged images are different files; do not select them in the Wi-Fi updater. For a first installation over USB, use Arduino IDE or Arduino CLI with the correct board and sketch so the required images are flashed at the correct offsets.

Build directories and historical local build files are excluded from Git. The deliberately published S3 application image and its checksums are under `firmware/StickS3/3.1/`. No new Plus2 binary is included in this update.

## Validation

StickS3 firmware 3.1 compiled successfully with M5Stack ESP32 core 3.3.10. Simulated sensor checks covered valid data, CRC errors, short reads, brief dropouts, repeated failures, pressure-read failures, and recovery on both HAT and Grove pins. These checks do not establish physical-device or accessory compatibility. The Plus2 source is retained as legacy code and was not rebuilt for this update.

## Background and license

The [original project overview](docs/legacy-project-overview.md) is preserved as historical material about the earlier station. Its hardware descriptions, controls, pricing, availability, and accessory claims should be read in that context. Use this README for the current hardware and firmware selection.

Code is provided under [CC BY-NC-SA 4.0, Attribution-NonCommercial-ShareAlike](https://creativecommons.org/licenses/by-nc-sa/4.0/). The existing Two Faces requirement for written permission before distributing this code in a commercial product remains in place. See the original overview for the existing project notices.
