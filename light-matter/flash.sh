#!/bin/bash
set -euo pipefail
ROOT="/Users/nana/codes/misk/light/light-matter"
source "$ROOT/esp-idf/export.sh"
export ESP_MATTER_PATH="$ROOT/esp-matter"
export PATH="$ROOT/bin:$PATH"
cd "$ROOT/firmware"
idf.py -p /dev/cu.usbmodem1101 flash
echo "== FLASH DONE =="
