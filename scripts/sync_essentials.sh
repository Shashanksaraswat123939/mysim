#!/usr/bin/env bash
# Copies the C++ and Arduino files that run the robot into essentials/, a flat folder for reading.
#   scripts/sync_essentials.sh          regenerate essentials/
#   scripts/sync_essentials.sh --check  fail if essentials/ is out of date (used by scripts/build.sh)
# Edit the sources, never the copies.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

FILES=(
  firmware/LineFollower/LineFollower.ino   # Arduino setup() and loop()
  firmware/LineFollower/config.h           # geometry, limits, tuning
  firmware/LineFollower/mathlite.h         # sin, cos, sqrt without a C library
  firmware/LineFollower/track.h            # a map: straights, arcs, gaps
  firmware/LineFollower/track.cpp
  firmware/LineFollower/maps.h             # the map catalogue
  firmware/LineFollower/maps.cpp
  firmware/LineFollower/planner.h          # racing speed profile
  firmware/LineFollower/planner.cpp
  firmware/LineFollower/line_controller.h  # the three control modes
  firmware/LineFollower/line_controller.cpp
  firmware/LineFollower/hardware.h         # Arduino pins, sensors, motors
  firmware/LineFollower/hardware.cpp
  sim/world.h                              # the simulated robot
  sim/world.cpp
  sim/wasm_exports.cpp                     # the functions the web page calls
)

if [ "${1:-}" = "--check" ]; then
  TMP="$(mktemp -d)"
  for f in "${FILES[@]}"; do cp "$f" "$TMP/$(basename "$f")"; done
  cp scripts/essentials_README.md "$TMP/README.md"
  diff -r -q "$TMP" essentials > /dev/null 2>&1 || {
    echo "essentials/ is out of date: run scripts/sync_essentials.sh"; exit 1; }
  rm -rf "$TMP"
  echo "essentials/ is up to date"
  exit 0
fi

rm -rf essentials
mkdir -p essentials
for f in "${FILES[@]}"; do cp "$f" "essentials/$(basename "$f")"; done
cp scripts/essentials_README.md essentials/README.md
echo "copied ${#FILES[@]} files to essentials/"
