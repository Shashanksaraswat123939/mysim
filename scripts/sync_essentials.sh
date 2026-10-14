#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

FILES=(
  firmware/LineFollower/LineFollower.ino
  firmware/LineFollower/config.h
  firmware/LineFollower/body.h
  firmware/LineFollower/mathlite.h
  firmware/LineFollower/track.h
  firmware/LineFollower/track.cpp
  firmware/LineFollower/maps.h
  firmware/LineFollower/maps.cpp
  firmware/LineFollower/planner.h
  firmware/LineFollower/planner.cpp
  firmware/LineFollower/line_controller.h
  firmware/LineFollower/line_controller.cpp
  firmware/LineFollower/hardware.h
  firmware/LineFollower/hardware.cpp
  sim/world.h
  sim/world.cpp
  sim/wasm_exports.cpp
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
