// Roll stability sweep (README section 3.7): speeds x mass variants x disturbances.
// A case fails if the droid swings up or tips: roll servo load > 80% or |lean| > 25 deg at any time.
// Usage: node robustness.js   (without Node on macOS: ./check-jsc.sh robustness.js), takes about 25 s.
const {BB8Sim} = require('./sim.js');

// Drive at stick `vel` for 10 s, then stop. From t = 4 s: head and/or roll stick fully to one side for 3 s, then to the other side for 3 s.
function run(o, vel, headAmp, rollAmp) {
  const s = new BB8Sim(o); let maxLoad = 0, maxLean = 0;
  for (let i = 0; i < 1800; i++) { const t = i * 0.01; s.input.vel = t < 10 ? vel : 0;
    const side = (t >= 4 && t < 7) ? 1 : (t >= 7 && t < 10) ? -1 : 0;
    s.input.headR = side * headAmp; s.input.roll = side * rollAmp;
    s.advance(0.01); maxLean = Math.max(maxLean, Math.abs(s.leanDeg())); maxLoad = Math.max(maxLoad, s.rollLoad); }
  return {bad: maxLoad > 0.8 || maxLean > 25, maxLean, maxLoad: maxLoad * 100};
}
const VARIANTS = [{}]; for (const mp of [4, 5]) for (const l of [0.145, 0.165]) for (const mh of [0.7, 1.3]) VARIANTS.push({mp, l, mh});
// [head swing in servo degrees, roll stick swing (0.5 = +-10 deg lean)]
const SCENARIOS = [[20, 0], [10, 0], [5, 0], [0, 0.25], [0, 0.5], [20, 0.25]];
const SPEEDS = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]; // stick; 1.0 is about 2.5 m/s

function sweep(base) {
  const fails = []; let n = 0, wl = 0, wL = 0;
  for (const vel of SPEEDS) for (const v of VARIANTS) for (const [h, r] of SCENARIOS) {
    const q = run({...v, ...base}, vel, h, r); n++; wl = Math.max(wl, q.maxLoad); wL = Math.max(wL, q.maxLean);
    if (q.bad) fails.push(`stick ${vel} ${JSON.stringify(v)} head ${h} roll ${r}: lean ${q.maxLean.toFixed(0)} deg, load ${q.maxLoad.toFixed(0)}%`);
  }
  return `failed ${fails.length}/${n}, worst lean ${wL.toFixed(1)} deg, worst load ${wl.toFixed(0)}%` + (fails.length ? '\n    e.g. ' + fails.slice(0, 3).join('\n    e.g. ') : '');
}
for (const [name, o] of [['body_roll_accel 30, no kd scheduling (before)', {bodyRollAccel: 30, rollKdSpeed: 0}],
                         ['body_roll_accel 30 + roll_kd_speed 1000', {bodyRollAccel: 30}],
                         ['body_roll_accel 60, no kd scheduling', {rollKdSpeed: 0}],
                         ['new defaults (60 + roll_kd_speed 1000)', {}],
                         ['new defaults, drive_speed_max 3000', {driveSpeedMax: 3000}],
                         ['new defaults, 1 kg head estimate', {mh: 1.0, L: 0.32, faDomeRollToWeight: -0.46, faDomePitchToWeight: -0.85}]])
  console.log(`${name.padEnd(48)} ${sweep(o)}`);
