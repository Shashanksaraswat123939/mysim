// Builds the blueprint presentation. Run from the repo root:  node deck/build_deck.js
// Slide text avoids hyphens and heavy punctuation on purpose. Drawings come from cad/make_cad.py.
const path = require("path");
const fs = require("fs");
const pptxgen = require("pptxgenjs");
const SKILL = process.env.PPTX_SKILL_DIR;
const { applyTheme } = require(path.join(SKILL, "scripts/apply_theme.js"));

const ROOT = path.resolve(__dirname, "..");
const D = (f) => path.join(ROOT, "docs/drawings", f);
const S = (f) => path.join(ROOT, "docs/screens", f);
const results = JSON.parse(fs.readFileSync(path.join(ROOT, "build/site/results.json"), "utf8"));

const THEME = {
  name: "Mono Blueprint",
  headFontFace: "Arial",
  bodyFontFace: "Calibri",
  colors: { dk1: "111111", lt1: "FFFFFF", dk2: "444444", lt2: "F3F3F3", accent1: "111111", accent2: "6B6B6B", accent3: "9A9A9A", accent4: "BDBDBD", accent5: "D9D9D9", accent6: "444444", hlink: "111111", folHlink: "6B6B6B" },
};

const pres = new pptxgen();
pres.layout = "LAYOUT_WIDE"; // 13.33 x 7.5 inches
pres.title = "Line follower robot: the blueprint";
pres.theme = { headFontFace: THEME.headFontFace, bodyFontFace: THEME.bodyFontFace };
const C = pres.SchemeColor;

pres.defineSlideMaster({
  title: "COVER",
  background: { color: "111111" },
  objects: [],
  slideNumber: undefined,
});
pres.defineSlideMaster({
  title: "CONTENT",
  background: { color: "FFFFFF" },
  objects: [
    { placeholder: { options: { name: "title", type: "title", x: 0.6, y: 0.35, w: 12.1, h: 0.7, fontSize: 30, bold: true, color: C.text1, fontFace: "Arial", margin: 0, valign: "top", align: "left" }, text: "" } },
    { placeholder: { options: { name: "sub", type: "body", x: 0.6, y: 1.05, w: 12.1, h: 0.4, fontSize: 16, color: C.accent2, margin: 0, valign: "top", align: "left" }, text: "" } },
  ],
  slideNumber: { x: 12.3, y: 7.05, w: 0.5, h: 0.3, fontSize: 10, color: C.accent2, align: "right" },
});

const notes = [];
function content(title, sub, section, note) {
  const s = pres.addSlide({ masterName: "CONTENT", sectionTitle: section });
  s.addText(title, { placeholder: "title" });
  if (sub) s.addText(sub, { placeholder: "sub" });
  if (note) s.addNotes(note);
  return s;
}
function drawing(file, title, sub, section, note) {
  const s = content(title, sub, section, note);
  const w = 11.4, h = w * 6.2 / 13.0;
  s.addImage({ path: D(file), x: (13.33 - w) / 2, y: 1.6, w, h, altText: title });
  return s;
}
const card = (s, x, y, w, h, head, body, name, size = 14) => {
  s.addShape(pres.ShapeType.rect, { x, y, w, h, fill: { color: C.background2 }, line: { color: C.accent5, width: 0.75 }, objectName: name + " card" });
  s.addText(head, { x: x + 0.25, y: y + 0.2, w: w - 0.5, h: 0.45, fontSize: 18, bold: true, color: C.text1, fontFace: "Arial", margin: 0, isTextBox: true, objectName: name + " heading" });
  s.addText(body, { x: x + 0.25, y: y + 0.75, w: w - 0.5, h: h - 0.95, fontSize: size, color: C.text2, margin: 0, valign: "top", paraSpaceAfter: size > 14 ? 8 : 0, isTextBox: true, objectName: name + " text" });
};

// 1 Cover
pres.addSection({ title: "Start" });
{
  const s = pres.addSlide({ masterName: "COVER", sectionTitle: "Start" });
  s.addText("Line follower robot", { x: 0.8, y: 2.2, w: 6.6, h: 1.0, fontSize: 48, bold: true, color: "FFFFFF", fontFace: "Arial", margin: 0, isTextBox: true });
  s.addText("The blueprint for our idea", { x: 0.8, y: 3.3, w: 6.6, h: 0.6, fontSize: 24, color: "D9D9D9", margin: 0, isTextBox: true });
  s.addText("Firmware, simulator, 3D model and the maths behind them", { x: 0.8, y: 4.2, w: 6.2, h: 0.8, fontSize: 16, color: "9A9A9A", margin: 0, valign: "top", isTextBox: true });
  s.addImage({ path: S("robot_crop.png"), x: 7.6, y: 1.9, w: 5.0, h: 5.0 * 690 / 1050 * 1.0, altText: "3D model of the practice robot" });
  s.addText("The board on top of the model is a dummy line follower card for practice", { x: 7.6, y: 5.5, w: 5.0, h: 0.6, fontSize: 12, color: "9A9A9A", margin: 0, valign: "top", isTextBox: true });
  s.addNotes("This deck explains the whole plan for our line follower robot in simple words. The robot model you see is a practice model. The card on top is a dummy board so we can generate drawings and test the pipeline before the real hardware exists.");
}

// 2 What we are building
pres.addSection({ title: "Idea" });
{
  const s = content("What we are building", "A robot that follows a painted line, and a simulator that shows exactly how it will behave", "Idea",
    "We write the robot brain once in C plus plus. The same brain runs on the Arduino, in our tests and inside the web simulator. Because it is one program there is no gap between what we see and what the robot does.");
  const stats = [["1", "C++ program shared by robot, tests and browser"], ["6", "maps from easy ovals to tight hairpins"], ["4", "ways to drive each map"], ["2436", "automatic checks and all of them pass"]];
  stats.forEach(([n, l], i) => {
    const x = 0.6 + i * 3.08;
    s.addShape(pres.ShapeType.rect, { x, y: 2.1, w: 2.85, h: 3.2, fill: { color: C.background2 }, line: { color: C.accent5, width: 0.75 }, objectName: "stat card " + (i + 1) });
    s.addText(n, { x: x + 0.25, y: 2.4, w: 2.35, h: 1.3, fontSize: 60, bold: true, color: C.text1, fontFace: "Arial", margin: 0, isTextBox: true, objectName: "stat number " + (i + 1) });
    s.addText(l, { x: x + 0.25, y: 3.85, w: 2.35, h: 1.2, fontSize: 16, color: C.text2, margin: 0, valign: "top", isTextBox: true, objectName: "stat label " + (i + 1) });
  });
  s.addText("The simulator adds real physics: weight, grip, wheel slip, sideways slide and noisy sensors", { x: 0.6, y: 5.9, w: 12.1, h: 0.6, fontSize: 18, color: C.text1, margin: 0, isTextBox: true });
}

// 3 One program everywhere
{
  const s = content("One program runs everywhere", "Change a number once and the robot, the tests and the website all follow", "Idea",
    "On the left is the single source of truth. The config file holds every size and tuning number. The control code reads it. From there the same code is built three ways. For the Arduino for the laptop tests and for the browser through WebAssembly.");
  s.addShape(pres.ShapeType.rect, { x: 0.6, y: 2.3, w: 3.6, h: 3.4, fill: { color: "111111" }, line: { color: "111111" }, objectName: "source box" });
  s.addText([
    { text: "Shared C++ code", options: { bold: true, fontSize: 20, color: "FFFFFF", breakLine: true } },
    { text: " ", options: { fontSize: 8, breakLine: true } },
    { text: "config.h sizes and tuning", options: { fontSize: 15, color: "D9D9D9", breakLine: true } },
    { text: "line controller", options: { fontSize: 15, color: "D9D9D9", breakLine: true } },
    { text: "racing planner", options: { fontSize: 15, color: "D9D9D9", breakLine: true } },
    { text: "maps and track", options: { fontSize: 15, color: "D9D9D9" } },
  ], { x: 0.9, y: 2.55, w: 3.0, h: 2.9, margin: 0, valign: "top", isTextBox: true });
  const outs = [["Arduino board", "Compiled for the real robot"], ["Laptop tests", "2436 checks plus a hardware loop test"], ["Browser simulator", "Compiled to WebAssembly with physics"]];
  outs.forEach(([h, b], i) => {
    const y = 1.95 + i * 1.5;
    s.addShape(pres.ShapeType.line, { x: 4.2, y: 4.0, w: 2.0, h: (y + 0.55) - 4.0, line: { color: "6B6B6B", width: 1.25, endArrowType: "triangle" }, objectName: "arrow " + (i + 1) });
    card(s, 6.2, y, 6.5, 1.25, h, b, "output " + (i + 1));
  });
}

// 4 The robot
pres.addSection({ title: "Robot" });
{
  const s = content("The robot", "A small two wheel differential drive car with two rows of line sensors", "Robot",
    "Two drive wheels and two small casters. Five light sensors sit in a row in front of the axle. A second identical row sits behind it for the Double IR mode. The board on top is a dummy line follower card used for practice.");
  s.addImage({ path: S("robot_crop.png"), x: 0.6, y: 1.8, w: 6.2, h: 6.2 * 690 / 1050, altText: "3D model of the robot" });
  const rows = [["Wheelbase", "90 mm"], ["Wheel diameter", "40 mm"], ["Body length", "64 mm"], ["Front sensor row", "40 mm ahead of axle"], ["Rear sensor row", "40 mm behind axle"], ["Sensors per row", "5 at 12 mm spacing"], ["Mass", "300 g"]];
  rows.forEach(([a, b], i) => {
    s.addText(a, { x: 7.4, y: 1.85 + i * 0.55, w: 2.4, h: 0.4, fontSize: 16, color: C.accent2, margin: 0, isTextBox: true });
    s.addText(b, { x: 9.9, y: 1.85 + i * 0.55, w: 2.9, h: 0.4, fontSize: 16, bold: true, color: C.text1, margin: 0, isTextBox: true });
  });
  s.addText("Dummy line follower card on top, for practice only", { x: 7.4, y: 5.85, w: 5.3, h: 0.7, fontSize: 14, color: C.text2, margin: 0, valign: "top", isTextBox: true });
}
drawing("01_orthographic_views.png", "Engineering drawing of the robot", "Top, front and side views with the sizes the firmware uses", "Robot",
  "This is a standard three view drawing. Top view shows the sensor rows and the wheelbase. Front and side views show the height and the wheel size. Every size comes from the config file so the drawing can never disagree with the code.");

// 6 Four ways to drive
pres.addSection({ title: "Driving" });
{
  const s = content("Four ways to drive a map", "From simple to smart", "Driving",
    "Normal PID only looks at the sensors. Full map blind trusts the map completely and drifts. Full map plus IR uses the map and the sensors together and is the fastest. Double IR needs no map. It measures how tight the curve is and slows down or speeds up.");
  const m = [
    ["Normal PID", "Steers only from the five IR sensors. No map. Logic: the sensors give how far the line is from the centre. A PID steering loop turns that error into a turn rate, and the robot slows when steering effort or error grows, so it eases off before it loses the line."],
    ["Full map blind", "Replays the racing plan open loop. Logic: the planner sets a speed for every point and the map gives the curvature, so the turn rate is speed times curvature. The robot tracks its own distance along the route. The sensors are ignored, so any drift grows until it derails."],
    ["Full map plus IR", "Same plan as blind, plus a correction. Logic: the same feed forward, with a PID on the sensed line offset that steers the robot back. The sensors also bridge short gaps and show the finish, where the tape ends."],
    ["Double IR", "No map needed. Logic: the front and rear rows give the line direction, and the encoders give the robot turn rate. Together they give the curvature of the bend. Speed is set to the square root of grip over curvature, capped at 0.4 m/s and rate limited like the plan."],
  ];
  m.forEach(([h, b], i) => card(s, 0.6 + (i % 2) * 6.15, 1.7 + Math.floor(i / 2) * 2.75, 5.95, 2.6, h, b, "mode " + (i + 1), 14));
}

// 7 The racing plan (chart)
{
  const s = content("How fast is safe in a corner", "Tighter corners need lower speed because grip is limited", "Driving",
    "The planner uses one rule. Speed squared times curvature must stay below the sideways grip. So speed equals the square root of grip times radius. A tight corner of five centimetres allows a quarter of a metre per second. A wide curve allows a full metre per second. Double IR uses a lower grip number and a lower speed cap because it cannot see the corner coming.");
  const radii = [5, 10, 15, 25, 40, 60, 100];
  const plan = radii.map((r) => +Math.min(1.0, Math.sqrt(1.5 * r / 100)).toFixed(2));
  const dbl = radii.map((r) => +Math.min(0.4, Math.sqrt(1.6 * r / 100)).toFixed(2));
  s.addChart(pres.charts.LINE, [
    { name: "Racing plan with the map", labels: radii.map((r) => r + " cm"), values: plan },
    { name: "Double IR without a map", labels: radii.map((r) => r + " cm"), values: dbl },
  ], {
    x: 0.6, y: 1.7, w: 8.2, h: 5.1, chartColors: ["111111", "9A9A9A"], lineSize: 3, lineDataSymbolSize: 8,
    showLegend: true, legendPos: "b", legendFontSize: 12, legendColor: "444444", legendFontFace: "+mn-lt",
    showValAxisTitle: true, valAxisTitle: "Speed in metres per second", valAxisTitleFontSize: 12, valAxisTitleColor: "444444",
    showCatAxisTitle: true, catAxisTitle: "Corner radius", catAxisTitleFontSize: 12, catAxisTitleColor: "444444",
    catAxisLabelColor: "444444", valAxisLabelColor: "444444", catAxisLabelFontSize: 12, valAxisLabelFontSize: 12,
    catAxisLabelFontFace: "+mn-lt", valAxisLabelFontFace: "+mn-lt",
    valGridLine: { color: "D9D9D9", size: 0.5 }, catGridLine: { style: "none" }, valAxisMinVal: 0, valAxisMaxVal: 1.1,
  });
  s.addText("Speed equals the square root of grip times radius", { x: 9.2, y: 2.0, w: 3.5, h: 1.0, fontSize: 20, bold: true, color: C.text1, fontFace: "Arial", margin: 0, valign: "top", isTextBox: true });
  s.addText("The racing plan allows up to 1.5 metres per second squared sideways and tops out at 1 metre per second", { x: 9.2, y: 3.3, w: 3.5, h: 1.4, fontSize: 15, color: C.text2, margin: 0, valign: "top", isTextBox: true });
  s.addText("Double IR uses 1.6 and never goes above 0.4 metres per second", { x: 9.2, y: 4.9, w: 3.5, h: 1.2, fontSize: 15, color: C.text2, margin: 0, valign: "top", isTextBox: true });
}

// 8 to 12 drawings
pres.addSection({ title: "Calculations" });
drawing("02_inertia.png", "Inertia: how hard the robot is to turn", "Add up every part as a small block and spin it about the centre of mass", "Calculations",
  "Inertia tells us how much the robot resists turning. Heavy parts far from the centre count the most. We list each part with its mass and position. Our assumed parts give about 2.8 times ten to the minus four. The simulator uses 5 times ten to the minus four. So the simulated robot is a bit harder to turn than this build. We assumed the masses and will weigh the real parts later.");
drawing("03_slip.png", "Grip and wheel slip", "A tyre pushes harder the more it slips until it reaches the grip limit", "Calculations",
  "Each wheel carries half the weight. Friction times that load is the most force the tyre can give. Slip is the difference between wheel speed and ground speed. At a slip of 0.2 the tyre already gives 76 percent of its limit. Drive force and sideways force share one budget which is the friction circle.");
drawing("04_encoders.png", "Wheel encoders and turn rate", "Counting wheel turns tells us speed and how sharply the robot is turning", "Calculations",
  "An encoder counts slots as the wheel turns. Each count is a known distance. Two wheels at different speeds make the robot turn. The turn rate is the speed difference divided by the wheelbase. Double IR combines this with the two sensor rows to find how tight the curve is. The 20 counts per turn is an assumption until we pick the real encoder.");
drawing("05_gaps.png", "Dashed lines and gaps", "In a gap the sensors see nothing so the robot keeps going on what it knew", "Calculations",
  "When the tape stops the sensors see white. Normal PID holds the last offset. The map modes keep following the plan. The robot gives up only after 0.6 seconds without tape. The widest gap in our maps is 0.1 metres which takes at most 400 milliseconds at the slowest speed so there is margin.");
drawing("06_timing.png", "Timestamps", "Every time value the robot and the simulator run on", "Calculations",
  "The control loop runs every 2 milliseconds. The simulator steps physics at the same rate. An encoder count at 0.4 metres per second arrives every 15.7 milliseconds so speed has to be counted over several loops. The robot gives up after 0.6 seconds without tape and the simulator calls a derail after a quarter second more than 35 millimetres off the line.");

// 13 Double IR flow
pres.addSection({ title: "Double IR" });
{
  const s = content("Double IR in four steps", "How the robot works out the curve it is in", "Double IR",
    "Step one the two rows see where the line is ahead and behind the axle. Their difference gives the line direction without needing to know exactly where the robot sits. Step two encoders give the robot turn rate. Step three adds them to get the curvature. Step four turns curvature into a safe speed.");
  const steps = [["1", "Two rows", "Front and rear sensors give the direction of the line"], ["2", "Encoders", "Wheel speeds give the robot turn rate"], ["3", "Curve", "Line turning plus robot turning gives how tight the bend is"], ["4", "Speed", "Safe speed is the square root of grip over curvature"]];
  steps.forEach(([n, h, b], i) => {
    const x = 0.6 + i * 3.08;
    s.addShape(pres.ShapeType.rect, { x, y: 2.2, w: 2.85, h: 3.6, fill: { color: C.background2 }, line: { color: C.accent5, width: 0.75 }, objectName: "step card " + n });
    s.addShape(pres.ShapeType.ellipse, { x: x + 0.25, y: 2.45, w: 0.7, h: 0.7, fill: { color: "111111" }, line: { color: "111111" }, objectName: "step badge " + n });
    s.addText(n, { x: x + 0.25, y: 2.45, w: 0.7, h: 0.7, fontSize: 20, bold: true, color: "FFFFFF", align: "center", valign: "middle", margin: 0, isTextBox: true, objectName: "step number " + n });
    s.addText(h, { x: x + 0.25, y: 3.4, w: 2.35, h: 0.5, fontSize: 20, bold: true, color: C.text1, fontFace: "Arial", margin: 0, isTextBox: true, objectName: "step heading " + n });
    s.addText(b, { x: x + 0.25, y: 4.05, w: 2.35, h: 1.5, fontSize: 15, color: C.text2, margin: 0, valign: "top", isTextBox: true, objectName: "step text " + n });
  });
  s.addText("In tests it finishes all six maps in 240 of 240 seeded runs", { x: 0.6, y: 6.2, w: 12.1, h: 0.5, fontSize: 18, bold: true, color: C.text1, margin: 0, isTextBox: true });
}

// 14 Results
pres.addSection({ title: "Results" });
{
  const s = content("Lap times from the C++ simulator", "Seconds on each map with the default start and noise", "Results",
    "These numbers come from the same C plus plus code that would run on the robot. Full map blind drifts off every map because small errors are never corrected. Full map plus IR is the fastest. Double IR is slower on purpose because it cannot see the corner coming. Normal PID is the slowest because it never speeds up on straights.");
  const nm = (x) => x.replace(/-/g, " ");
  const head = ["Map", "Planned", "Normal PID", "Full map blind", "Full map plus IR", "Double IR"].map((t, i) => ({ text: t, options: { bold: true, color: "FFFFFF", fill: { color: "111111" }, align: i === 0 ? "left" : "right", fontSize: 14 } }));
  const rows = results.maps.map((m) => [
    { text: nm(m.name), options: { align: "left", fontSize: 14 } },
    { text: m.planLap.toFixed(2), options: { align: "right", fontSize: 14 } },
    ...m.modes.map((md) => ({ text: md.defaultLap > 0 ? md.defaultLap.toFixed(2) : "Derailed", options: { align: "right", fontSize: 14, color: md.defaultLap > 0 ? "111111" : "6B6B6B" } })),
  ]);
  s.addTable([head, ...rows], { x: 0.6, y: 1.8, w: 12.1, colW: [2.6, 1.5, 1.9, 2.0, 2.2, 1.9], rowH: 0.52, border: { type: "solid", color: "D9D9D9", pt: 0.75 }, fontFace: "Calibri", color: "111111", valign: "middle", margin: [0, 0.12, 0, 0.12] });
  s.addText("Blind mode derails on every map. Over 40 noise seeds per map the other three modes finish nearly every run", { x: 0.6, y: 5.9, w: 12.1, h: 0.7, fontSize: 16, color: C.text2, margin: 0, valign: "top", isTextBox: true });
}

// 15 3D simulator
pres.addSection({ title: "Simulator" });
{
  const s = content("The 3D simulator", "Orbit, look from above or follow the robot", "Simulator",
    "The page draws a three dimensional view with plain flat polygons and no shaders. There are three cameras. A free orbit view. A top view that is an orthographic projection. And a follow camera that sits behind the robot. The wheels turn at the simulated wheel speed and the dots on the floor show what each sensor sees.");
  s.addImage({ path: S("orbit_view.png"), x: 0.6, y: 1.65, w: 7.6, h: 7.6 * 712 / 916, altText: "3D simulator view of the Hairpin map" });
  const b = [["3D", "Free orbit with drag and scroll"], ["Top", "Orthographic view for measuring"], ["Follow", "Camera behind the robot"], ["Floor dots", "Show what every sensor reads"]];
  b.forEach(([h, t], i) => {
    s.addText(h, { x: 8.7, y: 1.8 + i * 1.2, w: 4.0, h: 0.4, fontSize: 18, bold: true, color: C.text1, fontFace: "Arial", margin: 0, isTextBox: true });
    s.addText(t, { x: 8.7, y: 2.25 + i * 1.2, w: 4.0, h: 0.5, fontSize: 15, color: C.text2, margin: 0, valign: "top", isTextBox: true });
  });
}

// 16 Testing
pres.addSection({ title: "Proof" });
{
  const s = content("How we know it works", "Three layers of testing run on every build", "Proof",
    "First the laptop tests check the maths and every map in every mode. Second the hardware in the loop test runs the real Arduino sketch against the simulated robot using a pretend Arduino library. Third we repeat every map with forty different noise seeds so we are not just lucky once.");
  const t = [["2436", "checks on the laptop", "Maths, planner, grip, every map and mode"], ["1", "hardware loop test", "The real Arduino sketch drives the simulated robot"], ["40", "noise seeds per map", "Same robot with different random sensor noise"]];
  t.forEach(([n, h, b], i) => {
    const x = 0.6 + i * 4.15;
    s.addShape(pres.ShapeType.rect, { x, y: 2.0, w: 3.9, h: 3.8, fill: { color: C.background2 }, line: { color: C.accent5, width: 0.75 }, objectName: "proof card " + (i + 1) });
    s.addText(n, { x: x + 0.3, y: 2.3, w: 3.3, h: 1.2, fontSize: 60, bold: true, color: C.text1, fontFace: "Arial", margin: 0, isTextBox: true, objectName: "proof number " + (i + 1) });
    s.addText(h, { x: x + 0.3, y: 3.6, w: 3.3, h: 0.5, fontSize: 20, bold: true, color: C.text1, fontFace: "Arial", margin: 0, isTextBox: true, objectName: "proof heading " + (i + 1) });
    s.addText(b, { x: x + 0.3, y: 4.2, w: 3.3, h: 1.4, fontSize: 15, color: C.text2, margin: 0, valign: "top", isTextBox: true, objectName: "proof text " + (i + 1) });
  });
  s.addText("All checks pass today", { x: 0.6, y: 6.2, w: 12.1, h: 0.5, fontSize: 18, bold: true, color: C.text1, margin: 0, isTextBox: true });
}

// 17 Honest limits and next steps
{
  const s = content("What is estimated and what comes next", "The simulator is only as good as the numbers we feed it", "Proof",
    "Be clear about what is a guess. Friction motor strength and the grip limit are chosen to look realistic. The part masses and the encoder resolution are assumptions. Double IR needs the rear sensor row and encoders which are not wired on the Arduino yet. The next job is to measure the real robot and replace each guess with a number.");
  card(s, 0.6, 1.8, 5.95, 3.6, "Still estimated", "Tyre friction and motor strength\nGrip limit of 1.6 and speed cap of 0.4\nPart masses and the yaw inertia gap\nEncoder resolution of 20 counts\nThe line follower card is a dummy", "limits", 18);
  card(s, 6.75, 1.8, 5.95, 3.6, "Next steps", "Weigh the real parts and update the inertia\nChoose and wire the encoders\nAdd the rear sensor row to the hardware layer\nDrive the real robot and fit the physics to it\nRetune the controllers from the real data", "next", 18);
}

(async () => {
  const out = path.join(ROOT, "deck/line_follower_blueprint.pptx");
  await pres.writeFile({ fileName: out });
  await applyTheme(out, THEME);
  console.log("wrote", out);
})();
