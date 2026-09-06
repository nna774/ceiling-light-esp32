#!/bin/bash
set -euo pipefail
ROOT="/Users/nana/codes/misk/light/light-matter"
source "$ROOT/esp-idf/export.sh"
esptool.py -p /dev/cu.usbmodem1101 erase_region 0x10000 0xC000
echo "== ERASE NVS DONE =="
