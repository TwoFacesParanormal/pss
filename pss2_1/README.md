# Legacy StickC Plus2 firmware 2.1

This sketch is for the **M5Stack StickC Plus2 / M5StickC PLUS2 (K016-P2, ESP32)**. As of October 21, 2026, M5Stack lists this board as [EOL (End of Life)](https://shop.m5stack.com/products/m5stickc-plus2-esp32-mini-iot-development-kit) and recommends the StickS3 as its replacement. Existing Plus2 owners can continue using the code here. New StickS3 stations use [firmware 3.1](../pss2_S3/README.md).

The former root-level `pss.ino` is now named `pss2_1.ino` inside this matching Arduino sketch folder, with its own `logo.h`. This import uses the existing local Plus2 copy; compared with the former GitHub sketch, its default station ID is `49224` instead of `10000`. Its firmware version remains **2.1**.

## Build and hardware

Open `pss2_1.ino` and select **M5StickCPlus2**, not M5StickS3. The sketch uses M5StickCPlus2, M5Unified, M5Unit-ENV, ClosedCube SHT31D, HX711, ESP Async WebServer, and AsyncTCP, plus the ESP32 board package's bundled libraries. Its historical toolchain was not rebuilt or validated in this update; the S3 build instructions are not a tested build recipe for this legacy sketch.

| Connection | SDA | SCL |
| --- | --- | --- |
| ENV HAT | GPIO 0 | GPIO 26 |
| Grove / Port A | GPIO 32 | GPIO 33 |

The original Plus2 power-button behavior, buzzer, and user LED differ from those on the S3. Firmware 3.1's sensor-read, peripheral-detection, and title-screen changes apply to the S3 sketch only. Use Plus2 firmware for an existing Plus2 station's Wi-Fi update. No newly compiled Plus2 image is provided in this update.

## Project migration

This folder was moved from the Arduino sketchbook into `~/Projects/Paranormal Sensor Station/`. The older `~/Documents/Arduino/pss2_1/` copy still existed and matched this local sketch when checked on October 21, 2026. Use the Projects copy for ongoing work. Historical local `.bin`, `.elf`, `.map`, and build-directory files are excluded from Git; their names are not a reliable indication of the current source version.
