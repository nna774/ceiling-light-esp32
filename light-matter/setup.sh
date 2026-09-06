#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"
ROOT="$(pwd)"

echo "== [1/6] cloning esp-idf v6.0.2 (shallow) =="
if [ ! -d esp-idf ]; then
  git clone --branch v6.0.2 --depth 1 --shallow-submodules --recurse-submodules https://github.com/espressif/esp-idf.git
else
  echo "esp-idf already exists, skipping clone"
fi

echo "== [2/6] installing esp-idf toolchain (esp32c3 only) =="
cd "$ROOT/esp-idf"
./install.sh esp32c3
cd "$ROOT"

echo "== [3/6] cloning esp-matter (shallow) =="
if [ ! -d esp-matter ]; then
  git clone --depth 1 https://github.com/espressif/esp-matter.git
else
  echo "esp-matter already exists, skipping clone"
fi
cd "$ROOT/esp-matter"
git submodule update --init --depth 1
cd connectedhomeip/connectedhomeip
python3 scripts/checkout_submodules.py --platform esp32 linux --shallow
cd "$ROOT/esp-matter"

echo "== [4/6] sourcing esp-idf environment =="
source "$ROOT/esp-idf/export.sh"

echo "== [5/6] installing esp-matter (no host tools, saves a lot of time) =="
cd "$ROOT/esp-matter"
export PIP_INDEX_URL=https://pypi.org/simple
./install.sh --no-host-tool
export ESP_MATTER_PATH="$ROOT/esp-matter"
source "$ROOT/esp-matter/export.sh"
cd "$ROOT"

echo "== [6/6] copying examples/light as project base and test build =="
cp -r "$ROOT/esp-matter/examples/light" "$ROOT/firmware"
cd "$ROOT/firmware"
idf.py set-target esp32c3
idf.py build

echo "== DONE =="
