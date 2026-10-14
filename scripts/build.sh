#!/usr/bin/env bash
# Builds and checks the whole project from the C++ sources.
#
#   1. Host unit tests for the firmware core and the simulator.
#   2. Hardware-in-the-loop run: the Arduino sketch's own setup() and loop(), compiled
#      against a mock Arduino API and wired to the simulated robot.
#   3. The browser simulator: the same C++ compiled to WebAssembly and embedded in
#      web/line_follower_sim.html, together with results the C++ computes for every map
#      and mode (tools/export_results.cpp). scripts/publish_pages.sh pushes the site.
#
# Requirements: g++ (C++11), clang++ with the wasm32 target and wasm-ld, python3.
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
clang++ --target=wasm32-unknown-unknown -O2 -std=c++11 -ffreestanding -fno-exceptions -fno-rtti \
  -fno-threadsafe-statics -fno-asynchronous-unwind-tables -nostdlib -Wl,--no-entry -Wl,--strip-debug \
  -Wall -Wextra -Ifirmware/LineFollower -Isim \
  "${CORE[@]}" sim/world.cpp sim/wasm_exports.cpp sim/wasm_runtime.cpp \
  -fno-builtin-memcpy -o "$BUILD/sim.wasm"
g++ "${HOST_FLAGS[@]}" tools/export_results.cpp "${CORE[@]}" sim/world.cpp -o "$BUILD/export_results"
"$BUILD/export_results" > "$BUILD/results.json"
python3 scripts/embed_wasm.py web/template.html "$BUILD/sim.wasm" "$BUILD/results.json" web/line_follower_sim.html
# The site folder that scripts/publish_pages.sh pushes to GitHub Pages.
mkdir -p "$BUILD/site"
cp web/line_follower_sim.html "$BUILD/site/index.html"
cp "$BUILD/results.json" "$BUILD/site/results.json"
touch "$BUILD/site/.nojekyll"
echo "results: $(python3 -c "import json;r=json.load(open('$BUILD/results.json'));print(len(r['maps']),'maps x',len(r['maps'][0]['modes']),'modes, seeds',r['seeds'])")"

echo "== Done"
