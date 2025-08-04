# VLF5 Bootloader

- Update firmware from CAN Bus
- Automatically reset to DFU mode if the new firmware halted
- Cryptographic signature verification of the new firmware

## Memory Layout

The memory layout is defined in `memory.x`.

- Flash: total 2048KiB
  - 64KiB for bootloader
  - 1984KiB for application
- RAM: total 512KiB
  - (512KiB - 8 bytes) for bootloader or application
  - 8 bytes for bootloader state across reset

## Production Build

The production build does not include logging and is able to fit within 64KiB flash.

- Flash bootloader: `cargo run --release --no-default-features`
- Check bootloader size: `python size.py`

## Debug Build

Debug build also needs 64KiB of flash so no modification to `memory.x` are needed.

- Flash bootloader: `cargo run` or the run button on top of the main function

## Debug Notes

Test bootloader behaviour without connecting to ST-Link, because ST-Link may initiate additional resets not included in the code.
