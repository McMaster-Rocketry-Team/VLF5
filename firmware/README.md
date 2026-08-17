# VLF5 Firmware

This directory contains the avionics firmware and the GCM (ground control module) firmware for VLF5. Depending on the firmware running, VLF5 can be used as an avionics or GCM.

# Development Notes

- Install workspace recommended extensions by opening the extension sidebar and searching "@recommended".
- Four buttons should show up in the bottom status bar:
  1. Debug Avionics: Run the avionics firmware and attach a debugger to it.
  2. Debug GCM: Run the GCM firmware and attach a debugger to it.
  3. Release Avionics: Run the avionics firmware in release mode.
  4. Release GCM: Run the GCM firmware in release mode.

# Debugging Notes

- the debug buttons will open up the vscode debugger which allows you to add break points and inspect variables
- Unfortunately if a variable is used across `.await` points its not going to show up in the debug panel.
- The debug modes will disable the watchdog, so run release mode once before installing VLF5 to the rocket.

# Operator guide

Pad ops (beeps + rocket-cli): [OPERATOR.md](OPERATOR.md).

# Hardware-in-the-Loop (HIL)

The HIL model itself lives in `src/hil/mod.rs`. Bench identity, measured hardware properties,
verified runs and known gaps: [HIL2.md](HIL2.md).
