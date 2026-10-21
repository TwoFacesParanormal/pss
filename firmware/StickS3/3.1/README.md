# StickS3 3.1 application firmware

**For the PSS Wi-Fi / ESP user-firmware update screen, choose [`pss-sticks3-v3.1.bin`](pss-sticks3-v3.1.bin).**

This is the application image for the **M5Stack StickS3 (ESP32-S3, K150)**. It is byte-for-byte the same file as the locally compiled `pss2_S3.ino.bin`, renamed to identify the board and version. It is not firmware for the legacy StickC Plus2.

On the development computer, the original upload file is:

```text
~/Projects/Paranormal Sensor Station/pss2_S3/build/m5stack.esp32.m5stack_sticks3/pss2_S3.ino.bin
```

Upload this plain application `.bin` through the existing PSS firmware-update page at `192.168.4.1` after joining the station's `PSS_...` Wi-Fi network. Bootloader, partition, and merged images are not the application file expected by that updater. For a first USB installation, use Arduino IDE or Arduino CLI with the [StickS3 sketch and board settings](../../../pss2_S3/README.md).

Compiled October 21, 2026 with M5Stack ESP32 core 3.3.10. Exact dependency versions, source hashes, image hash, and validation status are recorded in [`build-info.json`](build-info.json). [`SHA256SUMS`](SHA256SUMS) verifies the application image:

```sh
cd firmware/StickS3/3.1
shasum -a 256 -c SHA256SUMS
```

Compilation and simulated sensor checks passed. This update does not claim verification on the physical device. The legacy Plus2 source remains separately available as firmware 2.1.
