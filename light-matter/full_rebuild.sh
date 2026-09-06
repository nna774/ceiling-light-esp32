#!/bin/bash
set -euo pipefail
ROOT="/Users/nana/codes/misk/light/light-matter"
source "$ROOT/esp-idf/export.sh"
export ESP_MATTER_PATH="$ROOT/esp-matter"
source "$ROOT/esp-matter/export.sh"
export IDF_TARGET=esp32c3
export PATH="$ROOT/bin:$PATH"
cd "$ROOT/firmware"
idf.py set-target esp32c3
ninja -C build
echo "== FULL REBUILD DONE =="
