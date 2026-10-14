# Line Follower Simulator

A line-following robot written in C++ for an Arduino Mega 2560, plus a browser simulator
that runs **the same C++ files** through WebAssembly. Nothing on the simulator page is
hand-written: the maps, racing speed profile, sensor readings, robot physics and every
number in the results table come from `firmware/LineFollower/` and `sim/`.

Open `web/line_follower_sim.html` in a browser. No server is needed.

## Three ways to drive a map

| Mode | What the robot knows | How it steers |
|---|---|---|
| **Normal PID** | Only what its IR sensors see | PID on the measured line offset. Slows down only when it has to. |
| **Full map (blind)** | The whole map, and the racing speed profile computed from it | Replays the planned speed and curvature open loop. It dead-reckons its position and never looks at the line. |
| **Full map + IR correction** | The whole map and the racing profile | Same feed-forward as blind, plus PID on the IR sensors that corrects drift, inaccuracies and gaps. |

The "racing line" is the speed profile. A line follower cannot cut corners, so the planner
works out the fastest speed at every point of the route, limited by grip
(`v² · |curvature| ≤ aLat`) and by acceleration and braking, using forward and backward passes.

## Results

Lap times in seconds from the start to the finish. `plan` is the ideal lap for the racing profile.
"derailed" means the robot drifted out of sensor reach. These numbers come from
`build/test_core` and `scripts/build.sh`, with the default seed and the simulated real robot.

| Map | plan | Normal PID (0.25 m/s) | Full map (blind) | Full map + IR |
|---|---|---|---|---|
| Warm-Up Oval | 6.06 | 21.78 | derailed | 6.04 |
| Chicane | 3.74 | 11.25 | derailed | 3.71 |
| Hairpin Alley | 5.18 | 15.29 | derailed | 5.06 |
| Dashed Line | 3.57 | 12.41 | derailed | 3.59 |
| Zig-Zag | 3.99 | 10.21 | derailed | 3.90 |
| Sweepers | 5.58 | 22.49 | derailed | 5.59 |

Robustness, from the tests (`tests/test_core.cpp`):

- Full map + IR finishes every map on the default run and on 8 other sensor-noise seeds,
  and across a 40-seed sweep of each map (0 failures in 240 runs).
- Normal PID completes every map on the default run and at every slider speed from 0.20 to 0.90 m/s.
  Across 40 seeded noise runs per map (240 per speed), it finishes 238–240 at each speed. It needs
  two things Normal PID is allowed to have: its steering gains scale up with its base speed, and it
  slows down as its steering effort and line offset grow (`kPidGain*`, `kPidSlowdown*` and
  `kPidOffsetSlowdownHigh` in `config.h`). Below 0.20 m/s it drops off on some maps.
- Full map (blind) fails because of the robot's imperfections, not because of its logic: with a
  perfect plant (`kIdealRobot` in `sim/world.h`) it completes all six maps. The simulated real
  robot has a 0.6% weaker right motor, wheel slip, sensor noise and a start placed 3 cm off the line.
  Blind has no way to correct any of that. The page shows these values in the "Robot physics" panel.

## Layout

```
firmware/LineFollower/   Arduino sketch + the control code it shares with the simulator
  LineFollower.ino       setup()/loop(): choose map and mode, run at 500 Hz, telemetry
  config.h               geometry, limits and tuning constants
  hardware.h/.cpp        Arduino-only: sensor calibration, TB6612FNG motor driver, button
  track.h/.cpp           map geometry: straights, arcs and gaps
  maps.h/.cpp            the map catalogue (edit this to add routes)
  planner.h/.cpp         racing-line speed profile (forward/backward pass)
  line_controller.h/.cpp the three control modes, line estimate from the IR sensors
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

- The page's simulator is a model, not a measurement of a real robot. Its physics
  (motor lag, drift, slip, noise, sensor reach) are chosen to be plausible, and the results
  depend on those choices. They are in `PlantParams` in `sim/world.h`. The controllers are never told them.
- I could not compile the sketch with the real AVR toolchain here (no `avr-gcc` or
  `arduino-cli`). It is checked by compiling against a mock Arduino API and running it in the
  hardware-in-the-loop harness. A bench test on a real robot is still needed.
- The planner assumes the robot tracks the planned speed. The mixed mode corrects steering
  from the IR sensors, but not along-track progress, so a large speed error would show up as a
  wrong feed-forward at corners.
- Maps must be at most 8 m long (the planner's buffer).
