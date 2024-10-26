# Tiny DFU Bootloader for AVR DU
- Directly read/write DU's flash, EEPROM and user row from PC using USB
- 1.5 KiB (3 pages) flash footprint, with read-only data stored in boot row

## Usage
### Flashing bootloader

`avrdude -c <programmer> -p avr<flash>du<n_pin> -Ubootsize:w:0x03:m -Ubootrow:w:build/program.btr -Uflash:w:build/program.hex`

A UPDI programmer is required to flash the bootloader itself.
Following memories have to be programmed:
- `FUSE.BOOTSIZE` (bootloader size in 512-byte blocks of flash)
- Boot Row (used to store constants including descriptors)
- Flash (bootloader)

### Downloading application
#### Build
- As the bootloader occupies the first three pages of flash, the application must have its start offset by `0x600` bytes. This can be achieved by adding `-Wl,--section-start=.text=0x600` to the linker options. By default, the interrupt vector table is placed at the start of the application region, so offsetting the whole `.text` section works as long as `FUSE.BOOTSIZE` is properly configured.
- The images has to be in binary format. This can be done with `avr-objcopy -I ihex -O binary <input_hex> <output_binary>`.

#### Download
- Bootloader can be entered by pulling `PC3` low on reset. The pin is internally pulled up during sampling, and while the bootloader is running.
- DFU interface has three alternate settings each corresponding to flash (0), EEPROM (1) and user row (2). The target partition can be specified using `-a <alt_num>` option if you are using `dfu-util`.
- All of these partitions support both writing/downloading `-D` and reading/uploading `-U`.
- First 1.5 KiB of flash, occupied by bootloader, does not appear in either downloaded or uploaded image. The binary file starts at `0x600`. For the other two memories, the image starts at `0x0` and the entire region is accessible.
- Full command would look like `dfu-util -a 0 -D firmware.bin` for downloading firmware, or `dfu-util -a 1 -U eeprom.bin` for uploading EEPROM content.

## Notes
- Various checks are skipped assuming the host will behave politely. Hardly compliant to USB/DFU specs. Don't use for anything remotely serious.
- That said, the bootloader was tested on AVR32DU32 and AVR16DU20 with `dfu-util` and seemed to be okay. Should work on 64 KiB parts too, though untested since I don't have one. As it involves dynamic `FLMAP` reconfiguration, it has more chance of something going wrong.