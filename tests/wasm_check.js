const fs = require("fs");

const wasmPath = process.argv[2];
const results = JSON.parse(fs.readFileSync(process.argv[3], "utf8"));

WebAssembly.instantiate(fs.readFileSync(wasmPath), {}).then(({ instance }) => {
  const sim = instance.exports;
  let checked = 0;
  let mismatches = 0;
  for (let m = 0; m < results.maps.length; m++) {
    results.maps[m].modes.forEach((mode, md) => {
      const want = mode.defaultLap;
      const got = sim.headless_lap(m, md, results.pidSpeed);
      checked++;
      const same = want < 0 ? got < 0 : Math.abs(got - want) <= 0.006;
      if (!same) {
        mismatches++;
        console.log(`MISMATCH ${results.maps[m].name} ${mode.mode}: wasm ${got} C++ ${want}`);
      }
    });
  }
  console.log(`wasm vs C++ default laps: ${checked} checked, ${mismatches} mismatches`);
  if (mismatches) process.exit(1);
});
