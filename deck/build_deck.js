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
  name: "Mono Pitch",
  headFontFace: "Arial",
  bodyFontFace: "Calibri",
  colors: { dk1: "111111", lt1: "FFFFFF", dk2: "444444", lt2: "F3F3F3", accent1: "111111", accent2: "6B6B6B", accent3: "9A9A9A", accent4: "BDBDBD", accent5: "D9D9D9", accent6: "444444", hlink: "111111", folHlink: "6B6B6B" },
};

const pres = new pptxgen();
pres.layout = "LAYOUT_WIDE";
pres.title = "Line follower robot: teacher pitch";
pres.theme = { headFontFace: THEME.headFontFace, bodyFontFace: THEME.bodyFontFace };
const C = pres.SchemeColor;

pres.defineSlideMaster({
  title: "COVER",
  background: { color: "111111" },
  objects: [],
});
pres.defineSlideMaster({
  title: "CONTENT",
  background: { color: "FFFFFF" },
  objects: [
    { placeholder: { options: { name: "title", type: "title", x: 0.6, y: 0.4, w: 12.1, h: 0.75, fontSize: 32, bold: true, color: C.text1, fontFace: "Arial", margin: 0, valign: "top", align: "left" }, text: "" } },
    { placeholder: { options: { name: "sub", type: "body", x: 0.6, y: 1.15, w: 12.1, h: 0.4, fontSize: 16, color: C.accent2, margin: 0, valign: "top", align: "left" }, text: "" } },
  ],
  slideNumber: { x: 12.3, y: 7.0, w: 0.5, h: 0.3, fontSize: 10, color: C.accent2, align: "right" },
});

function slide(title, sub, section, note) {
  const s = pres.addSlide({ masterName: "CONTENT", sectionTitle: section });
  s.addText(title, { placeholder: "title" });
  if (sub) s.addText(sub, { placeholder: "sub" });
  if (note) s.addNotes(note);
  return s;
}
function panel(s, x, y, w, h, name) {
  s.addShape(pres.ShapeType.rect, { x, y, w, h, fill: { color: C.background2 }, line: { color: C.accent5, width: 0.75 }, objectName: name });
}
function heading(s, text, x, y, w, name, size = 20) {
  s.addText(text, { x, y, w, h: 0.5, fontSize: size, bold: true, color: C.text1, fontFace: "Arial", margin: 0, isTextBox: true, objectName: name });
}
function body(s, text, x, y, w, h, name, size = 16) {
  s.addText(text, { x, y, w, h, fontSize: size, color: C.text2, margin: 0, valign: "top", isTextBox: true, objectName: name });
}
function bullets(s, items, x, y, w, h, name, size = 16) {
  s.addText(items.map((t, i) => ({ text: t, options: { bullet: true, breakLine: i < items.length - 1 } })), {
    x, y, w, h, fontSize: size, color: C.text2, margin: 0, valign: "top", paraSpaceAfter: 8, isTextBox: true, objectName: name,
  });
}
function picture(s, file, x, y, w, h, alt) {
  s.addImage({ path: file, x, y, w, h, altText: alt });
}
function stat(s, x, y, w, h, n, label, name) {
  panel(s, x, y, w, h, name + " card");
  s.addText(n, { x: x + 0.3, y: y + 0.25, w: w - 0.6, h: 1.0, fontSize: 48, bold: true, color: C.text1, fontFace: "Arial", margin: 0, isTextBox: true, objectName: name + " number" });
  body(s, label, x + 0.3, y + 1.35, w - 0.6, h - 1.5, name + " label", 15);
}

// Results from the C++ build (tools/export_results.cpp)
const runsPerMode = results.maps.length * results.seeds;
const finished = results.maps[0].modes.map((_, i) => results.maps.reduce((sum, m) => sum + m.modes[i].seedsFinished, 0));
const pct = (i) => Math.round((100 * finished[i]) / runsPerMode);

// 1 Cover
pres.addSection({ title: "Introduction" });
{
  const s = pres.addSlide({ masterName: "COVER", sectionTitle: "Introduction" });
  s.addText("Line follower robot", { x: 0.8, y: 2.1, w: 7.0, h: 1.1, fontSize: 52, bold: true, color: "FFFFFF", fontFace: "Arial", margin: 0, isTextBox: true });
  s.addText("A robot that reads a track, plans its speed, and learns from its sensors", { x: 0.8, y: 3.4, w: 7.0, h: 0.9, fontSize: 22, color: "D9D9D9", margin: 0, valign: "top", isTextBox: true });
  s.addText("A proposal for teachers: what we built, how it works, and why it matters", { x: 0.8, y: 4.5, w: 7.0, h: 0.8, fontSize: 16, color: "9A9A9A", margin: 0, valign: "top", isTextBox: true });
  s.addImage({ path: S("robot_crop.png"), x: 8.4, y: 2.0, w: 4.3, h: 4.3 * 690 / 1050, altText: "3D model of the robot" });
  s.addNotes("Hello. Our project is a line follower robot. It drives along a black line on the floor. We built the brain for it in C plus plus, a simulator that shows how it drives, and a set of engineering drawings. Today we will show you the idea, the four ways the robot can drive, the engineering behind it, and why we think it matters.");
}

// 2 The idea
pres.addSection({ title: "Our idea" });
{
  const s = slide("Our idea", "One robot, one program, and a simulator that shows what it will do before we build it", "Our idea",
    "The big idea is simple. A robot follows a line. We wanted to find out how much a robot can do with a map, and how much it can do with only its sensors. So we built four ways to drive, and we compare them fairly.");
  stat(s, 0.6, 1.9, 3.8, 2.9, "1", "brain program that runs on the robot, in the simulator and on the web page", "idea one");
  stat(s, 4.75, 1.9, 3.8, 2.9, "4", "ways to drive, from simple sensor steering to a smart curve reader", "idea two");
  stat(s, 8.9, 1.9, 3.8, 2.9, "6", "different tracks, from a gentle oval to tight hairpin bends", "idea three");
  body(s, "The question we are answering: how fast can a small robot go round a track safely, and what does it need to know to do it?", 0.6, 5.3, 12.1, 0.9, "idea question", 20);
}

// 3 What makes us different
{
  const s = slide("What makes us different", "Four things most student robots do not have", "Our idea",
    "Most student projects write code for the robot, then test it by driving the robot. We did two things differently. First, the simulator uses the same C plus plus as the robot, so there is no gap between what we see and what the robot does. Second, we measure the things a real robot would struggle with, like grip, slip and sensor noise.");
  const items = [
    ["One brain everywhere", "The same C plus plus code runs on the Arduino, on the computer and in the web browser. What you see is what the robot does."],
    ["Real physics", "The simulator models weight, grip, wheel slip, sideways slide and noisy sensors, so problems show up before we build."],
    ["A fair comparison", "All four modes drive the same robot, with the same noise, on the same six tracks. Any difference comes from the method."],
    ["Honest numbers", "Every estimate is written down. We say which numbers we measured and which we still have to check on a real robot."],
  ];
  items.forEach(([h, b], i) => {
    const x = 0.6 + (i % 2) * 6.15, y = 1.9 + Math.floor(i / 2) * 2.55;
    panel(s, x, y, 5.95, 2.3, "difference " + (i + 1));
    heading(s, h, x + 0.3, y + 0.3, 5.3, "difference heading " + (i + 1), 20);
    body(s, b, x + 0.3, y + 0.95, 5.35, 1.2, "difference text " + (i + 1), 16);
  });
}

// 4 How the parts connect
{
  const s = slide("How the robot works, at a glance", "Sensors see the line, the brain decides, the motors act", "Our idea",
    "Read this from left to right. The sensors see the black line. The brain turns that into a steering decision. The motors turn the wheels. The wheel counters tell the brain how fast each wheel is turning. A map, when we use one, tells the brain how fast to go at every point.");
  const boxes = [["Sensors", "Five sensors in a row, and a second row for one mode"], ["Brain", "Reads the sensors, the map and the wheel counters, then decides"], ["Motors", "Turn the two wheels at different speeds to steer"], ["Track", "The robot drives, and the sensors see the line again"]];
  boxes.forEach(([h, b], i) => {
    const x = 0.6 + i * 3.1;
    panel(s, x, 2.3, 2.75, 2.8, "flow box " + (i + 1));
    heading(s, h, x + 0.25, 2.6, 2.3, "flow heading " + (i + 1), 20);
    body(s, b, x + 0.25, 3.25, 2.3, 1.7, "flow text " + (i + 1), 15);
    if (i < 3) s.addShape(pres.ShapeType.line, { x: x + 2.8, y: 3.7, w: 0.25, h: 0, line: { color: "6B6B6B", width: 1.5, endArrowType: "triangle" }, objectName: "flow arrow " + (i + 1) });
  });
  body(s, "The brain does this about 500 times every second, so it can react quickly to what it sees.", 0.6, 5.7, 12.1, 0.7, "flow footer", 18);
}

// 5 The four boards
pres.addSection({ title: "The four ways to drive" });
{
  const s = slide("The four ways to drive", "Four boards, from simplest to smartest", "The four ways to drive",
    "Think of these as four boards that we can switch between. The first uses only sensors. The second uses only the map. The third uses the map and the sensors together. The fourth reads the shape of the curve as it drives. We will go through each one in detail.");
  const m = [["Normal PID", "Sensors only"], ["Full map (blind)", "Map only"], ["Full map plus IR", "Map and sensors"], ["Double IR", "Two sensor rows and wheel counters"]];
  m.forEach(([h, b], i) => {
    const x = 0.6 + (i % 2) * 6.15, y = 1.9 + Math.floor(i / 2) * 2.55;
    panel(s, x, y, 5.95, 2.3, "board " + (i + 1));
    s.addText(String(i + 1), { x: x + 0.3, y: y + 0.3, w: 0.8, h: 0.8, fontSize: 36, bold: true, color: C.text1, fontFace: "Arial", margin: 0, isTextBox: true, objectName: "board number " + (i + 1) });
    heading(s, h, x + 1.3, y + 0.4, 4.4, "board heading " + (i + 1), 22);
    body(s, b, x + 1.3, y + 1.0, 4.4, 1.0, "board text " + (i + 1), 16);
  });
}

// 6-9 Each board in detail
const boards = [
  ["Board 1: Normal PID", "Steering from the sensors only", "Uses five light sensors. It looks at how far the line is from the middle of the sensor bar, and turns the robot back toward it. It has no map, so it cannot know a corner is coming. To stay safe, it slows down when it turns hard or when the line is far away.",
    ["It only knows what it can see right now", "Slows when steering effort grows", "Gives up if it loses the line for 0.6 seconds"],
    "Normal PID finishes every map, but it is the slowest, because it cannot prepare for the next corner.",
    "This is the baseline. Every other board must beat it to be worth using."],
  ["Board 2: Full map (blind)", "Following a plan without looking", "Before the run, the brain works out the best speed for every point on the track. The fastest safe speed is slow in tight bends and fast on straights. During the run, the robot follows that plan and ignores its sensors.",
    ["It knows where it is on the track by counting distance", "It uses the plan for speed and turn", "Small errors grow, and it leaves the line"],
    "Blind mode derails on every map. A plan with no correction drifts off the line.",
    "This shows why sensors matter. A perfect plan is not enough, because the real robot never follows a perfect path."],
  ["Board 3: Full map plus IR", "The plan, with the sensors correcting it", "This board uses the same plan as blind mode, and adds a correction. The sensors measure how far the robot has drifted, and a steering loop nudges the wheels back on the line. The sensors also bridge short gaps, and spot the finish when the tape ends.",
    ["Same fast plan as blind mode", "Sensors correct drift in real time", "Bridges short gaps and finds the finish"],
    "Full map plus IR is our fastest and most reliable board across the six tracks.",
    "The best of both: the map gives the speed, and the sensors keep the robot honest."],
  ["Board 4: Double IR", "Reading the curve as it drives", "This board needs no map. It has two rows of sensors, one in front of the wheels and one behind. The two rows tell the robot which way the line is turning. Wheel counters tell it how fast it is turning itself. Together, these give the sharpness of the bend, and the robot sets its speed to match.",
    ["Two sensor rows give the direction of the line", "Wheel counters give the robot turn rate", "Speed is the highest safe speed for that bend, capped at 0.4 m/s"],
    "Double IR finishes every map and every seeded run. It is slower than the map board, but it needs no map at all.",
    "This is the most interesting board for the real world. A robot that can read a bend without a map can work in places nobody has mapped."],
];
boards.forEach(([title, sub, para, pts, result, note], i) => {
  const s = slide(title, sub, "The four ways to drive", note);
  body(s, para, 0.6, 1.9, 6.0, 3.4, "board detail text " + (i + 1), 16);
  panel(s, 7.1, 1.9, 5.6, 4.6, "board facts " + (i + 1));
  heading(s, "How it works", 7.4, 2.15, 5.0, "facts heading " + (i + 1), 18);
  bullets(s, pts, 7.4, 2.75, 5.0, 2.0, "facts list " + (i + 1), 15);
  body(s, result, 7.4, 4.9, 5.0, 1.5, "facts result " + (i + 1), 15);
});

// 10 Results
{
  const s = slide("How the four boards compare", "Lap times in seconds on each track, same robot, same start", "The four ways to drive",
    "Times are in seconds, from the start to the end of each track. A dash in a box means the robot derailed. The first column is the time the racing plan expects for a perfect run. The best time in each row is the fastest finished lap on that track.");
  const nm = (x) => x;
  const head = ["Track", "Plan", "Normal PID", "Blind", "Map plus IR", "Double IR"].map((t, i) => ({ text: t, options: { bold: true, color: "FFFFFF", fill: { color: "111111" }, align: i === 0 ? "left" : "right", fontSize: 14 } }));
  const rows = results.maps.map((m) => {
    const laps = m.modes.map((md) => md.defaultLap);
    const best = Math.min(...laps.filter((l) => l > 0));
    return [
      { text: nm(m.name), options: { align: "left", fontSize: 14 } },
      { text: m.planLap.toFixed(2), options: { align: "right", fontSize: 14, color: "6B6B6B" } },
      ...laps.map((l) => ({ text: l > 0 ? l.toFixed(2) : "derailed", options: { align: "right", fontSize: 14, bold: l === best, color: l > 0 ? "111111" : "6B6B6B" } })),
    ];
  });
  s.addTable([head, ...rows], { x: 0.6, y: 1.8, w: 12.1, colW: [2.9, 1.5, 2.0, 1.8, 2.0, 1.9], rowH: 0.5, border: { type: "solid", color: "D9D9D9", pt: 0.75 }, fontFace: "Calibri", color: "111111", valign: "middle", margin: [0, 0.12, 0, 0.12] });
  body(s, "Share of runs that finished, across all six tracks and 40 different noise settings: Normal PID " + pct(0) + " percent, Blind " + pct(1) + " percent, Map plus IR " + pct(2) + " percent, Double IR " + pct(3) + " percent.", 0.6, 5.85, 12.1, 0.8, "results summary", 16);
}

// Technical section: 10 slides
pres.addSection({ title: "The engineering, in plain words" });
{
  const s = slide("Technical 1: The robot body", "Simple numbers, shown as an engineering drawing", "The engineering, in plain words",
    "This drawing shows the robot from above, from the front, and from the side. The two wheels are 90 millimetres apart. The sensor bar sits 40 millimetres ahead of the wheels, so the robot sees the line before it reaches it. The rear sensor bar is for the Double IR board.");
  picture(s, D("01_orthographic_views.png"), 0.6, 1.75, 12.1, 12.1 * 6.2 / 13.0 * 0.92, "Engineering drawing of the robot body");
}
{
  const s = slide("Technical 2: How the robot finds the line", "Five sensors, and a weighted average", "The engineering, in plain words",
    "Each sensor reports light or dark. The robot takes all five readings and works out the centre of the dark part. If the centre is to the left of the middle, the line is to the left, and the robot steers left. This is the same idea as finding the centre of a balance.");
  panel(s, 0.6, 1.9, 5.8, 4.7, "sensor panel");
  bullets(s, ["Five sensors, 12 millimetres apart", "Each reads light or dark", "The line position is the average of the dark readings", "If the line is off centre, the robot steers back"], 0.9, 2.3, 5.2, 4.0, "sensor list", 17);
  panel(s, 6.9, 1.9, 5.8, 4.7, "sensor example panel");
  heading(s, "Example", 7.2, 2.2, 5.2, "sensor example heading", 20);
  body(s, "If only the second and third sensors see the line, the line is near the middle. If only the far left sensor sees it, the line is far to the left and the robot needs a hard turn.", 7.2, 2.85, 5.2, 2.6, "sensor example text", 17);
}
{
  const s = slide("Technical 3: The steering loop", "A simple rule, applied 500 times a second", "The engineering, in plain words",
    "The steering loop is called PID. It has three parts, and each one answers a simple question. How far off are we right now? How long have we been off? How fast are we getting more off? We add the three answers together to get a turn command. The gains, the three strengths, are chosen by testing.");
  const parts = [["P: how far off", "Turn harder when the line is further away"], ["I: how long off", "Correct a steady error that the P part cannot clear alone"], ["D: how fast it changes", "Slow down the turn when the error is closing quickly"]];
  parts.forEach(([h, b], i) => {
    const x = 0.6 + i * 4.1;
    panel(s, x, 1.9, 3.8, 3.1, "pid card " + (i + 1));
    heading(s, h, x + 0.3, 2.2, 3.2, "pid heading " + (i + 1), 19);
    body(s, b, x + 0.3, 2.9, 3.2, 1.9, "pid text " + (i + 1), 16);
  });
  body(s, "The loop runs every 2 milliseconds. That is fast enough to correct the robot long before it drifts far from the line.", 0.6, 5.4, 12.1, 0.8, "pid footer", 17);
}
{
  const s = slide("Technical 4: How fast is safe in a corner", "The tighter the corner, the slower the robot must go", "The engineering, in plain words",
    "There is a simple rule behind the racing plan. Grip gives a sideways limit. Speed squared divided by the radius must stay below that limit. So the safe speed is the square root of the grip times the radius. Wide bends let the robot go faster, tight bends make it slow down. This chart shows the rule for both the map board and the Double IR board.");
  const radii = [5, 10, 15, 25, 40, 60, 100];
  const plan = radii.map((r) => +Math.min(1.0, Math.sqrt(1.5 * r / 100)).toFixed(2));
  const dbl = radii.map((r) => +Math.min(0.4, Math.sqrt(1.6 * r / 100)).toFixed(2));
  s.addChart(pres.charts.LINE, [
    { name: "Map boards", labels: radii.map((r) => r + " cm"), values: plan },
    { name: "Double IR board", labels: radii.map((r) => r + " cm"), values: dbl },
  ], {
    x: 0.6, y: 1.75, w: 7.9, h: 5.0, chartColors: ["111111", "9A9A9A"], lineSize: 3, lineDataSymbolSize: 8,
    showLegend: true, legendPos: "b", legendFontSize: 13, legendColor: "444444", legendFontFace: "+mn-lt",
    showValAxisTitle: true, valAxisTitle: "Safe speed (m/s)", valAxisTitleFontSize: 13, valAxisTitleColor: "444444",
    showCatAxisTitle: true, catAxisTitle: "Radius of the bend", catAxisTitleFontSize: 13, catAxisTitleColor: "444444",
    catAxisLabelColor: "444444", valAxisLabelColor: "444444", catAxisLabelFontSize: 13, valAxisLabelFontSize: 13,
    catAxisLabelFontFace: "+mn-lt", valAxisLabelFontFace: "+mn-lt",
    valGridLine: { color: "D9D9D9", size: 0.5 }, catGridLine: { style: "none" }, valAxisMinVal: 0, valAxisMaxVal: 1.1,
  });
  panel(s, 8.9, 1.9, 3.9, 4.6, "corner panel");
  heading(s, "The rule", 9.2, 2.2, 3.4, "corner rule heading", 19);
  body(s, "Safe speed = square root of (grip times radius)", 9.2, 2.8, 3.4, 1.0, "corner rule", 16);
  body(s, "The map boards use a grip of 1.5 metres per second squared, with a top speed of 1 metre per second. Double IR uses a more careful grip of 1.6 and never goes above 0.4 metres per second.", 9.2, 3.9, 3.4, 2.4, "corner note", 15);
}
{
  const s = slide("Technical 5: Grip and wheel slip", "A tyre can only push so hard before it slips", "The engineering, in plain words",
    "A wheel can only push the ground so hard. The limit is friction times the weight on that wheel. If the wheel spins faster than the ground moves, it slips, and it can push a little harder for a moment. Past a certain slip, the grip is used up. This drawing shows the friction limit, and the circle of drive and sideways force that a wheel can use at the same time.");
  picture(s, D("03_slip.png"), 0.6, 1.75, 12.1, 12.1 * 6.2 / 13.0, "Grip and slip drawing");
}
{
  const s = slide("Technical 6: How hard the robot is to turn", "Inertia: the heavier and wider the parts, the harder it is to spin", "The engineering, in plain words",
    "Inertia tells us how much the robot resists turning. We add up each part's mass times how far it sits from the centre. Parts far from the middle count much more. The estimate from our assumed part weights is close to the value the simulator uses, and we will weigh the real parts to confirm it.");
  picture(s, D("02_inertia.png"), 0.6, 1.75, 12.1, 12.1 * 6.2 / 13.0, "Yaw inertia drawing");
}
{
  const s = slide("Technical 7: Counting wheel turns", "Wheel counters tell us speed, and how sharply the robot turns", "The engineering, in plain words",
    "An encoder counts small slots as a wheel turns. Each count is a known distance. Speed is distance per count times counts per second. If one wheel is faster than the other, the robot turns. Double IR uses this turn rate to know how sharp the bend is, even when the sensors cannot see far enough ahead.");
  picture(s, D("04_encoders.png"), 0.6, 1.75, 12.1, 12.1 * 6.2 / 13.0, "Wheel encoder drawing");
}
{
  const s = slide("Technical 8: Dashed lines and gaps", "When the line disappears, the robot keeps its nerve", "The engineering, in plain words",
    "Some tracks have gaps. In a gap the sensors see only the floor. The map boards keep going because they know the plan. The plain PID board holds its last correction. If the robot is blind for more than 0.6 seconds, it stops. The widest gap on our tracks is 0.1 metres, so the robot always gets through.");
  picture(s, D("05_gaps.png"), 0.6, 1.75, 12.1, 12.1 * 6.2 / 13.0, "Gaps and dashed lines drawing");
}
{
  const s = slide("Technical 9: Timing and timestamps", "Every number the robot runs on has a clock behind it", "The engineering, in plain words",
    "Timing matters. The control loop runs every 2 milliseconds. The simulator uses the same step. Each rule has its own time limit, like giving up after 0.6 seconds without the line, or calling a derail after a quarter second more than 35 millimetres off the line. Getting these right stops the robot reacting too slowly or too quickly.");
  picture(s, D("06_timing.png"), 0.6, 1.75, 12.1, 12.1 * 6.2 / 13.0, "Timing chart");
}
{
  const s = slide("Technical 10: The simulator", "We copy the real physics, then mark what is still a guess", "The engineering, in plain words",
    "The simulator models the robot's mass, motor strength, friction, wheel slip, sideways slide and sensor noise. Some numbers are measured, and some are estimates. We show the estimates openly, because they are the first things we will check when we have the real robot.");
  picture(s, S("orbit_view.png"), 0.6, 1.75, 6.9, 6.9 * 712 / 916, "3D simulator view");
  panel(s, 7.9, 1.75, 4.9, 4.9, "sim panel");
  heading(s, "What the simulator models", 8.2, 2.05, 4.3, "sim heading", 18);
  bullets(s, ["Motor pull for each wheel", "Grip and wheel slip", "Sideways slide in fast bends", "Noisy sensors", "Rolling resistance"], 8.2, 2.6, 4.3, 2.2, "sim list", 15);
  heading(s, "Still to check on hardware", 8.2, 4.85, 4.3, "sim todo heading", 18);
  body(s, "Friction, motor strength and the grip value are estimates until we measure the real robot.", 8.2, 5.35, 4.3, 1.2, "sim todo text", 15);
}

// Why it matters
pres.addSection({ title: "Why it matters" });
{
  const s = slide("Why this matters in the real world", "Line following is a small version of a very big industry", "Why it matters",
    "Line following is the same idea behind automated carts in warehouses, hospitals and factories. These machines carry goods and follow a fixed route. A robot that reads its own path, and knows when it is about to lose it, is the kind of machine those places need.");
  const items = [["Warehouses and factories", "Carts and trucks that follow a floor line, and need to slow for turns."], ["Hospitals and labs", "Delivery robots that carry samples or meals on fixed routes."], ["Safety", "Robots that know when to stop, instead of carrying on blind."], ["Learning", "Control, sensing, physics and testing, all in one project."]];
  items.forEach(([h, b], i) => {
    const x = 0.6 + (i % 2) * 6.15, y = 1.9 + Math.floor(i / 2) * 2.55;
    panel(s, x, y, 5.95, 2.3, "impact " + (i + 1));
    heading(s, h, x + 0.3, y + 0.3, 5.3, "impact heading " + (i + 1), 20);
    body(s, b, x + 0.3, y + 0.95, 5.35, 1.2, "impact text " + (i + 1), 16);
  });
}
{
  const s = slide("The serious engineering behind it", "What we did to make the work reliable, not just impressive", "Why it matters",
    "A robot that works once is a demo. A robot that works every time is engineering. We used the same code everywhere, so a fix in one place fixes all three. We ran each map forty times with different noise, to see how often each board works. And we wrote down every estimate, so that the next step is measurement, not guessing.");
  bullets(s, [
    "One source of truth: sizes and tuning live in one file, used by the robot and the simulator",
    "Repeated runs: every map was run 40 times with different sensor noise, so results are not luck",
    "Safety rules: a robot that loses the line for 0.6 seconds stops, instead of driving on blind",
    "Honest records: estimates are listed, with the next measurement to make for each one",
    "Documentation: drawings, a design write-up and a plain-English guide to the code",
  ], 0.6, 1.9, 12.1, 4.8, "engineering list", 18);
}
{
  const s = slide("What we need next", "A real robot, a small budget, and a teacher to check our work", "Why it matters",
    "Our next step is to build the real robot and check the estimates. We need a set of parts, a place to test on a real track, and a teacher to check our measurements and our method. We will report every number we measure, including the ones that disagree with the simulator.");
  const next = [["Build", "Assemble the real robot from the drawings and the parts list"], ["Measure", "Weigh the parts, time the motors, and check the grip on our floor"], ["Compare", "Run the same six tracks on the real robot and against the simulator"], ["Improve", "Update the simulator with the real numbers, then retune the controllers"]];
  next.forEach(([h, b], i) => {
    const x = 0.6 + i * 3.1;
    panel(s, x, 1.9, 2.8, 3.6, "next card " + (i + 1));
    s.addText(String(i + 1), { x: x + 0.25, y: 2.15, w: 0.8, h: 0.8, fontSize: 34, bold: true, color: C.text1, fontFace: "Arial", margin: 0, isTextBox: true, objectName: "next number " + (i + 1) });
    heading(s, h, x + 0.25, 3.05, 2.4, "next heading " + (i + 1), 20);
    body(s, b, x + 0.25, 3.6, 2.4, 1.8, "next text " + (i + 1), 15);
  });
  body(s, "We would welcome your guidance on the measurements, the safety rules, and the time we need in the workshop.", 0.6, 5.9, 12.1, 0.8, "next footer", 17);
}
{
  const s = slide("Thank you", "Questions and discussion", "Why it matters",
    "Thank you for your time. We are happy to answer questions, to show the simulator running, or to walk through any of the engineering drawings in more detail.");
  body(s, "Line follower robot: one brain, four ways to drive, and a simulator that is honest about what it knows.", 0.6, 2.4, 12.1, 1.0, "closing line", 22);
}

(async () => {
  const out = path.join(ROOT, "deck/line_follower_blueprint.pptx");
  await pres.writeFile({ fileName: out });
  await applyTheme(out, THEME);
  console.log("wrote", out);
})();
