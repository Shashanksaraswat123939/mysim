#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

BUILD=build
mkdir -p "$BUILD/hil"

CORE=(
  firmware/LineFollower/track.cpp
  firmware/LineFollower/maps.cpp
  firmware/LineFollower/planner.cpp
  firmware/LineFollower/line_controller.cpp
)
HOST_FLAGS=(-std=c++11 -O2 -Wall -Wextra -Werror -Ifirmware/LineFollower -Isim)
WASM_FLAGS=(-O2 -std=c++11 -ffreestanding -fno-exceptions -fno-rtti -fno-threadsafe-statics
  -fno-asynchronous-unwind-tables -nostdlib -Wl,--no-entry -Wl,--strip-debug -Wall -Wextra
  -Ifirmware/LineFollower -Isim -fno-builtin-memcpy)

if command -v clang++ > /dev/null && command -v wasm-ld > /dev/null; then
  WASM_CXX=(clang++ --target=wasm32-unknown-unknown)
else
  WASM_CXX=(python3 -m ziglang c++ -target wasm32-freestanding)
fi

echo "== 0. Essentials folder is in step with the sources"
scripts/sync_essentials.sh --check

echo "== 1. Host tests"
g++ "${HOST_FLAGS[@]}" tests/test_core.cpp "${CORE[@]}" sim/world.cpp -o "$BUILD/test_core"
"$BUILD/test_core"

echo "== 2. Arduino sketch, hardware-in-the-loop"
cp firmware/LineFollower/LineFollower.ino "$BUILD/hil/LineFollower.cpp"
g++ "${HOST_FLAGS[@]}" -Itests/hil tests/hil/hil_main.cpp "$BUILD/hil/LineFollower.cpp" \
  firmware/LineFollower/hardware.cpp "${CORE[@]}" sim/world.cpp -o "$BUILD/hil/hil"
"$BUILD/hil/hil" > "$BUILD/hil/output.txt"
grep "^HIL result" "$BUILD/hil/output.txt"
grep -q "outcome=1" "$BUILD/hil/output.txt" || { echo "sketch did not finish the map"; exit 1; }

echo "== 3. Browser simulator (WebAssembly)"
"${WASM_CXX[@]}" "${WASM_FLAGS[@]}" "${CORE[@]}" sim/world.cpp sim/wasm_exports.cpp sim/wasm_runtime.cpp -o "$BUILD/sim.wasm"
g++ "${HOST_FLAGS[@]}" tools/export_results.cpp "${CORE[@]}" sim/world.cpp -o "$BUILD/export_results"
"$BUILD/export_results" > "$BUILD/results.json"
python3 scripts/embed_wasm.py web/template.html "$BUILD/sim.wasm" "$BUILD/results.json" web/line_follower_sim.html
if command -v node > /dev/null; then
  node tests/wasm_check.js "$BUILD/sim.wasm" "$BUILD/results.json"
fi
mkdir -p "$BUILD/site"
cp web/line_follower_sim.html "$BUILD/site/index.html"
cp "$BUILD/results.json" "$BUILD/site/results.json"
touch "$BUILD/site/.nojekyll"
echo "results: $(python3 -c "import json;r=json.load(open('$BUILD/results.json'));print(len(r['maps']),'maps x',len(r['maps'][0]['modes']),'modes, seeds',r['seeds'])")"

echo "== Done"
