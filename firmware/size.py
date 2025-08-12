#!/usr/bin/env python3

import subprocess
import re
import math
import os

FLASH_SIZE_KIB = 1984
RAM_SIZE_KIB = 512

build_env = os.environ.copy()

subprocess.run('cargo build'.split(' '))
result = subprocess.run(
    'cargo bloat --bin main --crates --split-std --no-relative-size -n 20'.split(' '), capture_output=True)
if result.stderr.decode("utf-8").find("error: could not compile") >= 0:
    exit(1)
print()
print(result.stdout.decode("utf-8")[:-93])

result = subprocess.run(
    'cargo bloat --bin main --split-std --no-relative-size -n 20'.split(' '), capture_output=True)
print(result.stdout.decode("utf-8")[:-93])

result = subprocess.run(
    'cargo size --bin main -- -A'.split(' '), capture_output=True)
lines = result.stdout.decode("utf-8").split('\n')

global_bytes = 0
rom_bytes = 0

for line in lines:
    match_result = re.match(
        r'^\.([a-zA-Z\._]*)\s*(\d*)\s*0x[1-9][0-9a-f]*$', line)
    if match_result:
        name, size = match_result.groups()
        if name in ["bss", "data", "uninit"]:
            global_bytes += int(size)
        elif name in ["vector_table", "text", "rodata", "data"]:
            rom_bytes += int(size)

print()
print(f"Total size:")
print(f"Flash: {math.ceil(rom_bytes / 1024)}KiB / {FLASH_SIZE_KIB}KiB ({round(rom_bytes / (FLASH_SIZE_KIB * 1024) * 100)}%)")
print(
    f"Global variables: {math.ceil(global_bytes / 1024)}KiB / {RAM_SIZE_KIB}KiB ({round(global_bytes / (RAM_SIZE_KIB * 1024) * 100)}%)")
print(f"Avaliable RAM: {math.floor(RAM_SIZE_KIB - global_bytes / 1024)}KiB")