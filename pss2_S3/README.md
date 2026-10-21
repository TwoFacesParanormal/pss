# StickS3 firmware 3.1

This sketch is for the **M5Stack StickS3 / M5StickS3 (K150, ESP32-S3)**, the current PSS hardware as of October 21, 2026. For the older StickC Plus2, use [firmware 2.1](../pss2_1/README.md).

Open `pss2_S3.ino` from this folder. Keep `logo.h` beside it. `FIRMWARE_VERSION` in the sketch is the shared version string used by the title screen and Wi-Fi firmware-update page.

## Build

Select **M5StickS3** from the M5Stack ESP32 board package. The verified board identifier is `m5stack:esp32:m5stack_sticks3`, using package version **3.3.10**.

These external library versions were installed for the successful 3.1 build:

| Library | Version |
| --- | --- |
| M5Unified | 0.2.22 |
| M5GFX | 0.2.29 |
| M5Unit-ENV | 1.0.1 |
| HX711 Arduino Library | 0.7.5 |
| ESP Async WebServer | 2.9.4 |
| AsyncTCP | 1.1.4 |

Wire, EEPROM, WiFi, Update, and BLE are supplied by the board package. The S3 sketch does not require ClosedCube SHT31D or M5StickCPlus2. These are the versions used for this build; later library versions have not been validated here.

From the repository root:

```sh
arduino-cli compile \
  --fqbn m5stack:esp32:m5stack_sticks3 \
  --output-dir "pss2_S3/build/m5stack.esp32.m5stack_sticks3" \
  "pss2_S3"
```

For a first USB installation, select the connected StickS3 port in Arduino IDE and upload this sketch. For an existing PSS installation using its Wi-Fi updater, select the compiled **`pss2_S3.ino.bin`**, or the [published 3.1 application image](../firmware/StickS3/3.1/pss-sticks3-v3.1.bin). Use S3 firmware on S3 hardware.

## ENV III connections

| Connection | SDA | SCL |
| --- | --- | --- |
| ENV III HAT, original row of the Hat2-Bus | GPIO 8 | GPIO 0 |
| Grove / Port A | GPIO 9 | GPIO 10 |

The extra HAT row is not used for this ENV III HAT connection. SHT30 is at `0x44`. The firmware prefers pressure address `0x56` on the HAT and `0x70` on Grove, with the other address as a fallback. Both use a 100 kHz bus. See the [M5Stack HAT documentation](https://docs.m5stack.com/en/hat/Hat_ENV-III) and [StickS3 pin map](https://docs.m5stack.com/en/core/StickS3).

Connect the peripheral before startup. A detected ENV sensor keeps its display through brief failed reads; after five consecutive failures, the firmware retries the same connector every two seconds. Use Reset to rerun full peripheral detection after changing the setup. Sensor Bar and REM Duo paths are carried over in the source and require physical validation on the S3.

## Opening-screen controls

```text
Left Btn: Reset
          2x Off
Right Btn: Beep/Menu
Front Btn: SensThresh
```

The left control is the power/reset button. A single press resets the device; a double press powers it off. The S3 speaker supplies the alert tones. There is no programmable user LED on this board.

## Working copy

The maintained local sketch is `~/Projects/Paranormal Sensor Station/pss2_S3/pss2_S3.ino`. The older `~/Documents/Arduino/pss2_1/` copy belongs to the Plus2 generation. The folder name `pss2_S3` is retained for Arduino compatibility; the firmware version is **3.1**.
