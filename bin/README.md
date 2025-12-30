# How to Burn Binaries

- these binaries are for ESP32 and ESP32E (not ESP32-S3, etc.)
- install Arduino 1.8.19 for Windows and Espressif 2.0.18 toolchain
- modify path in the command below to match your PC setup
  - run the command
- upload sketch data

```
C:\Users\ASUS\AppData\Local\Arduino15\packages\esp32\tools\esptool_py\4.5.1/esptool.exe --chip esp32 --port COM3 --baud 921600 --before default_reset --after hard_reset write_flash -e -z --flash_mode dio --flash_freq 80m --flash_size 4MB 0x1000 C:\Users\ASUS\OneDrive\Documents\Arduino\kaiaai-esp32\bin\kaiaai-esp32.ino.bootloader.bin 0x8000 C:\Users\ASUS\OneDrive\Documents\Arduino\kaiaai-esp32\bin\kaiaai-esp32.ino.partitions.bin 0xe000 C:\Users\ASUS\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.17/tools/partitions/boot_app0.bin 0x10000 C:\Users\ASUS\OneDrive\Documents\Arduino\kaiaai-esp32\bin\kaiaai-esp32.ino.bin
```
