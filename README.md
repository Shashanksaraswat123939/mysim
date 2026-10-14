# Line Follower Simulator

A line-following robot written in C++ for an Arduino Mega 2560, plus a browser simulator
that runs **the same C++ files** through WebAssembly. Nothing on the simulator page is
hand-written: the maps, racing speed profile, sensor readings, robot physics and every
number in the results table come from `firmware/LineFollower/` and `sim/`.

Open `web/line_follower_sim.html` in a browser. No server is needed.

## Four ways to drive a map

| Mode | What the robot knows | How it steers |
|---|---|---|
| **Normal PID** | Only what its IR sensors see | PID on the measured line offset. Slows down only when it has to. |
| **Full map (blind)** | The whole map, and the racing speed profile computed from it | Replays the planned speed and curvature open loop. It dead-reckons its position and never looks at the line. |
| **Full map + IR correction** | The whole map and the racing profile | Same feed-forward as blind, plus PID on the IR sensors that corrects drift, inaccuracies and gaps. |
| **Double IR** | No map. Two sensor rows (ahead of and behind the axle) and wheel encoders | Measures the curvature of the bend it is in, and steers by PID with curvature feed-forward. Speed follows the grip limit for that radius, capped at 0.40 m/s. Needs the extra hardware (see below). |

Double IR needs hardware the current robot does not have: a second sensor row behind the axle, and
wheel encoders. Two rows give the line's direction relative to the robot. Converting that to a radius needs
the robot's own turn rate, which only the encoders can give. The sketch refuses to build this mode until
the hardware layer reads both (`static_assert` in `LineFollower.ino`).

The "racing line" is the speed profile. A line follower cannot cut corners, so the planner
works out the fastest speed at every point of the route, limited by grip
(`v² · |curvature| ≤ aLat`) and by acceleration and braking, using forward and backward passes.

## Results

Lap times in seconds from the start to the finish. `plan` is the ideal lap for the racing profile.
"derailed" means the robot drifted out of sensor reach. These numbers come from
`build/test_core` and `scripts/build.sh`, with the default seed and the simulated real robot. The
simulated robot is a physics model (mass, friction, rolling loss, motor stall force, tyre slide); see
`docs/PROJECT.md` section 6.1.

| Map | plan | Normal PID (0.25 m/s) | Full map (blind) | Full map + IR | Double IR |
|---|---|---|---|---|---|
| Warm-Up Oval | 6.06 | 23.30 | derailed | 7.20 | 12.05 |
| Chicane | 3.74 | 12.08 | derailed | 4.28 | 6.08 |
| Hairpin Alley | 5.18 | 16.27 | derailed | 5.46 | 7.50 |
| Dashed Line | 3.57 | 13.32 | derailed | 4.30 | 6.90 |
| Zig-Zag | 3.99 | 10.94 | derailed | 4.17 | 5.03 |
| Sweepers | 5.58 | 24.08 | derailed | 6.68 | 13.15 |

Robustness, from the tests (`tests/test_core.cpp`):

- Full map + IR finishes every map on the default run, and 239 of 240 seeded runs (40 noise seeds per map).
  Its gains were retuned when the physics model changed, so the earlier figure of 0 failures no longer applies.
- Normal PID completes every map on the default run, and 240 of 240 seeded runs (40 per map) at the default
  speed. Its steering gains scale up with its base speed, and it slows down as its steering effort and line
  offset grow (`kPidGain*`, `kPidSlowdown*` and `kPidOffsetSlowdownHigh` in `config.h`). Re-checked on the
  physics model: 240 of 240 seeded runs at every slider speed from 0.20 to 0.90 m/s.
- Double IR finishes every map on the default run and 240 of 240 seeded runs (40 per map). Its curvature
  estimate is within a median of 6% to 16% on the curved parts of each map. Its speed is capped at 0.40 m/s
  because the front row sees a bend only 4 cm ahead. Above about 0.5 m/s, it fails. The radius term only
  changes the speed below a radius of about 10 cm; `docs/PROJECT.md` section 5.4 has the data.
- Full map (blind) finishes 0 of 240 seeded runs and none on the default run (`build/site/results.json`,
  from the C++ build). It fails because of the robot's imperfections, not its logic: with a perfect plant
  (`kIdealRobot` in `sim/world.h`) it completes all six maps. The real robot's motors have no speed
  feedback, so they deliver a few percent less speed than commanded (about 6% at 0.25 m/s), and its
  dead-reckoned position falls behind. Blind has no way to correct any of that. The page shows these
  values in the "Robot physics" panel.

## Documentation

`essentials/` holds only the C++ and Arduino files that run the robot, in reading order; its
`README.md` says what to read first. `docs/PROJECT.md` explains each control idea, the code behind it, the C++ concepts, how the
website gets its results, and every value that is approximated or invented.

## Layout

```
firmware/LineFollower/   Arduino sketch + the control code it shares with the simulator
  LineFollower.ino       setup()/loop(): choose map and mode, run at 500 Hz, telemetry
  config.h               geometry, limits and tuning constants
  hardware.h/.cpp        Arduino-only: sensor calibration, TB6612FNG motor driver, button
  track.h/.cpp           map geometry: straights, arcs and gaps
  maps.h/.cpp            the map catalogue (edit this to add routes)
  planner.h/.cpp         racing-line speed profile (forward/backward pass)
  line_controller.h/.cpp the four control modes, line estimate from the IR sensors
  mathlite.h             small sin/cos/sqrt so the code builds for AVR, host and wasm
sim/
  world.h/.cpp           simulated robot: motors, sensors, ground truth, outcomes
  wasm_exports.cpp       C ABI used by the page
  wasm_runtime.cpp       memcpy/memset for freestanding WebAssembly
web/
  template.html          page source; the WASM is embedded at build time
  line_follower_sim.html built page (committed, so it opens without a build)
tests/
  test_core.cpp          host unit and simulation tests
  hil/                   hardware-in-the-loop harness: runs the real sketch against the simulated robot
scripts/
  build.sh               tests, hardware-in-the-loop run, WebAssembly build and page build
  embed_wasm.py          embeds the WASM into the page
```

## Results on the website

`scripts/build.sh` runs `tools/export_results.cpp`, which simulates every map in every mode
(the default run, plus 40 noise seeds per map) and writes `results.json`. That JSON is embedded in
the page, so the comparison table is correct on first load, and it is also in the published site.
`scripts/publish_pages.sh` pushes `build/site` to the `gh-pages` branch. Enable GitHub Pages once
in Settings → Pages → Deploy from a branch → `gh-pages`, `/ (root)`.

## Build and test

Requires `g++` (C++11), `clang++` with the `wasm32` target and `wasm-ld`, and `python3`.

```sh
scripts/build.sh
```

This runs the host tests, runs the Arduino sketch's own `setup()` and `loop()` against the
simulated robot (a hardware-in-the-loop check), and then rebuilds `web/line_follower_sim.html`.

## Flash the robot

1. Hardware: Arduino Mega 2560, five reflectance sensors on A0–A4 (black reads low),
   a TB6612FNG motor driver (see the pin table in `hardware.h`), and a button on pin 2 to GND.
2. In `LineFollower.ino`, pick `kSelectedMap` and `kSelectedMode`.
3. Open the `firmware/LineFollower` folder in the Arduino IDE (or run `arduino-cli compile`)
   and upload to a Mega. Do not use an Uno: the racing-plan buffer needs about 1.6 KB of RAM.
4. Power on with the sensor bar across the line. The robot spins for 2 s to calibrate, then
   says "Ready". Press the button, and it runs the map with telemetry on Serial at 115200.

## Limits and assumptions

- The page's simulator is a physics model, not a measurement of a real robot. Its values (mass, friction,
  rolling loss, motor stall force, tyre stiffness, sensor noise) are chosen to be plausible, and the results
  depend on those choices. They are in `PlantParams` in `sim/world.h`. The controllers are never told them.
- I could not compile the sketch with the real AVR toolchain here (no `avr-gcc` or
  `arduino-cli`). It is checked by compiling against a mock Arduino API and running it in the
  hardware-in-the-loop harness. A bench test on a real robot is still needed.
- The planner assumes the robot tracks the planned speed. The mixed mode corrects steering
  from the IR sensors, but not along-track progress, so a large speed error would show up as a
  wrong feed-forward at corners.
- Maps must be at most 8 m long (the planner's buffer).
