# Changelog

## 3.1 — 2026-10-21 — StickS3

- Add a separate firmware sketch for the current M5Stack StickS3 (ESP32-S3) while retaining Plus2 firmware 2.1 for legacy hardware.
- Use M5Unified for the S3 display, IMU, speaker, buttons, and power management, including enabling external power.
- Map the original HAT row to SDA GPIO 8 / SCL GPIO 0 and Grove to SDA GPIO 9 / SCL GPIO 10.
- Detect the ENV III HAT before testing for other peripherals. Bound the initial load-cell readiness check.
- Use a 100 kHz sensor bus and probe the HAT and Grove pressure addresses (`0x56` / `0x70`).
- Replace the S3 sketch's ClosedCube temperature read with a complete, CRC-validated SHT30 read. Retain good data across brief failures and retry a detected sensor on the same connector after repeated failures.
- Seed sensor baselines on detection and recovery to avoid false startup/recovery graph spikes. Reject failed pressure values.
- Move the opening-screen instructions up one small-font row, put `2x Off` on an indented line beneath `Left Btn: Reset`, and remove trailing punctuation from the control labels.
- Use one `FIRMWARE_VERSION` value, `3.1`, for the title screen and firmware-update page.
- Document the two hardware generations, the Plus2 EOL notice, and the move from Documents/Arduino into Projects.

Validation: successful StickS3 compilation with M5Stack ESP32 core 3.3.10; simulated checks of valid/invalid samples, dropouts, pressure failures, and HAT/Grove recovery. Physical-device and accessory validation remains separate.

## 2.1 — Legacy StickC Plus2

The pre-existing Plus2 sketch remains firmware 2.1. It has been placed in `pss2_1/` with its logo and hardware notes. The imported local copy uses default station ID `49224`; the former root-level GitHub sketch used `10000`. No S3 changes were backported and no new Plus2 binary was built.
