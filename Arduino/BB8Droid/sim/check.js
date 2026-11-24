// Reproduces the figures in README.md with the simulation core sim.js.
// Usage: node check.js   (without Node on macOS: ./check-jsc.sh)
const {BB8Sim, R2D} = require('./sim.js');

// Head/drive settings of the firmware before the simulation work. The simulation defaults are now the
// "ideal" head set (same as BB8Config.h), so comparisons against the old firmware spell these out.
const FW_OLD = {faSpeedSPToDome: -0.01, faDomeRollToWeight: 0, faDomePitchToWeight: 0, faAccelToDome: 0, domeAccelTau: 0.2,
                driveAccelLimit: 0, domeCompRemoteOnly: false, faPitchToDome: 0};

const ROLL_FW  = {rollKp: 0.3, rollKi: 0, rollKd: 0.1, rollIBleed: 0};
const ROLL_REC = {rollKp: 0.5, rollKi: 0.4, rollKd: 0.4, rollIBleed: 0};
const ROLL_ZVD = {rollKp: 0.85, rollKi: 1.3, rollKd: 0.3, rollShaper: 2, rollShaperT: 2.3, rollIBleed: 0};
const VARIANTS = [{}]; for (const mp of [4, 5]) for (const l of [0.145, 0.165]) for (const mh of [0.7, 1.3]) VARIANTS.push({mp, l, mh});

function rollStep(o, amp = 10, T = 10) {
  const s = new BB8Sim(o); s.advance(0.3); s.input.roll = amp / 20;
  let peak = -1e9, ts = 0, load = 0, t90 = null;
  while (s.t < T + 0.3) { s.advance(0.001); const t = s.t - 0.3, y = s.leanDeg();
    peak = Math.max(peak, y); if (Math.abs(y - amp) > 0.05 * amp) ts = t; if (t90 === null && y >= 0.9 * amp) t90 = t; load = Math.max(load, s.rollLoad); }
  return {end: s.leanDeg(), over: Math.max(0, (peak - amp) / amp * 100), ts, t90, load: load * 100};
}
function rollReturn(o, extra = {}) {
  const s = new BB8Sim({...o, ...extra}); s.input.vel = 0.25; s.advance(2);
  const step = tgt => { const d = tgt - s.input.roll; s.input.roll += Math.max(-0.02, Math.min(0.02, d)); s.advance(0.01); };
  for (let i = 0; i < 1000; i++) step(0.5);
  const t0 = s.t, psi0 = s.psi; let tUp = null, under = 0, load = 0;
  while (s.t < t0 + 12) { step(0); const y = s.leanDeg(); under = Math.min(under, y); load = Math.max(load, s.rollLoad);
    if (tUp === null && Math.abs(y) < 1) tUp = s.t - t0; else if (tUp !== null && Math.abs(y) > 1) tUp = null; }
  return {tUp, under, turn: (s.psi - psi0) * R2D, load: load * 100};
}
function headStand(o) {
  const s = new BB8Sim(o); s.advance(1); const x0 = s.pq[0]; s.input.headP = 30; s.advance(8);
  const r = new BB8Sim(o); r.advance(1); r.input.headR = 20; let ml = 0; for (let i = 0; i < 800; i++) { r.advance(0.01); ml = Math.max(ml, Math.abs(r.leanDeg())); }
  return {drift: s.pq[0] - x0, v: s.pqd[0] * 1000, maxLean: ml};
}
function accelHead(o) {
  const s = new BB8Sim(o); const r = {accFwd: 0, accBack: 0, cruise: 0, brkBack: 0, brkFwd: 0, pend: 0, stop: null};
  for (let i = 0; i < 1100; i++) { const t = i * 0.01; s.input.vel = (t > 0.5 && t < 5) ? 0.5 : 0; s.advance(0.01);
    const a = s.pq[2] * R2D; r.pend = Math.max(r.pend, Math.abs(s.pitchDeg()));
    if (t < 5) { r.accFwd = Math.max(r.accFwd, a); r.accBack = Math.max(r.accBack, -a); if (t > 4.5) r.cruise = a; }
    else { r.brkBack = Math.max(r.brkBack, -a); r.brkFwd = Math.max(r.brkFwd, a); if (r.stop === null && Math.abs(s.pqd[0]) < 0.05) r.stop = t - 5; } }
  return r;
}
const f = (x, d = 1) => (x === null || x === undefined) ? '-' : x.toFixed(d);

for (const [title, base] of [['new defaults (ideal head set)', {}], ['old firmware head settings', FW_OLD]]) {
  console.log(`=== Roll step 10 deg (1:1, pendulum 4.5 kg @ 155 mm, head 1 kg), ${title} ===`);
  for (const [n, o] of [['firmware default 0.3/0/0.1', ROLL_FW], ['recommended 0.5/0.4/0.4', ROLL_REC], ['ZVD 2.3s + 0.85/1.3/0.3', ROLL_ZVD]]) {
    const r = rollStep({...base, ...o}); let wts = 0, wov = 0, wl = 0;
    for (const v of VARIANTS) for (const amp of [10, 20]) { const q = rollStep({...base, ...o, ...v}, amp); wts = Math.max(wts, q.ts); wov = Math.max(wov, q.over); wl = Math.max(wl, q.load); }
    console.log(`${n.padEnd(28)} end ${f(r.end)} deg, t90 ${f(r.t90, 2)} s, t+-5% ${f(r.ts, 2)} s, overshoot ${f(r.over, 0)}%, load ${f(r.load, 0)}% | over variants: t+-5% max ${f(wts, 2)} s, overshoot max ${f(wov, 0)}%, load max ${f(wl, 0)}%`);
  }
  console.log('');
}
console.log('=== Steering back from a 10 deg curve (0.75 m/s setpoint), new defaults ===');
for (const [n, e] of [['no I bleed', {}], ['K_p 1.5 on the way back', {rollKpReturn: 1.5}], ['I bleed 0.4 s', {rollIBleed: 0.4}]]) {
  const r = rollReturn(ROLL_REC, e); console.log(`${n.padEnd(24)} upright after ${f(r.tUp, 2)} s, undershoot ${f(r.under)} deg, keeps turning ${f(r.turn, 0)} deg, load ${f(r.load, 0)}%`);
}
console.log('\n=== Head compensation at standstill (head 30 servo deg forward / 20 servo deg sideways) ===');
const ip = new BB8Sim().idealDomePitchToWeight(), ir = new BB8Sim().idealDomeRollToWeight();
console.log(`ideal values: fa_dome_pitch_to_weight ${ip.toFixed(2)}, fa_dome_roll_to_weight ${ir.toFixed(2)}, pendulum per m/s^2 ${new BB8Sim().pendulumPerAccel().toFixed(1)} deg`);
for (const [n, o] of [['without', FW_OLD], ['with (new defaults)', {}]]) {
  const r = headStand(o); console.log(`${n.padEnd(20)} drift ${f(r.drift, 2)} m, v ${f(r.v, 0)} mm/s, max sideways lean ${f(r.maxLean)} deg`);
}
console.log('\n=== Head when pulling away / braking (stick step 0.5) ===');
for (const [n, o] of [['old firmware', FW_OLD], ['IMU tracking k=1.3 (rejected)', {...FW_OLD, faSpeedSPToDome: 0, faPitchToDome: 1.3, faDomePitchToWeight: ip}], ['feedforward (new defaults)', {}]]) {
  const r = accelHead(o);
  console.log(`${n.padEnd(30)} pendulum ${f(r.pend, 0)} deg | pulling away ${f(r.accFwd, 0)} fwd / ${f(r.accBack, 0)} back | cruise ${f(r.cruise, 0)} | braking ${f(r.brkBack, 0)} back / ${f(r.brkFwd, 0)} fwd | stopped after ${f(r.stop)} s`);
}
console.log('\n=== Top speed (full throttle, drive_speed_max 3000, ramp 2000 mm/s^2) ===');
// Old firmware: errI <= 255, I share = controlGain*ki*errI <= 3.0*0.2*255 ~ 153 PWM
const OLD_IB = 255 * 0.2 * 4776.384 / (2 * Math.PI * 253);
for (const [hn, base] of [['old head settings', FW_OLD], ['new defaults', {}]])
  for (const [n, ib] of [[`old I bound (errI <= 255 -> I share <= ${OLD_IB.toFixed(0)} PWM)`, OLD_IB], ['new: I share <= 255 PWM', 255]]) { const s = new BB8Sim({...base, driveIBound: ib, driveAccelLimit: 2000}); s.input.vel = 1; let pm = 0; for (let i = 0; i < 1500; i++) { s.advance(0.01); pm = Math.max(pm, Math.abs(s.pitchDeg())); } console.log(`${hn.padEnd(18)} ${n.padEnd(46)} ${f(s.pqd[0] * 1000, 0)} mm/s, PWM ${f(s.pwm, 0)}, pendulum steady ${f(s.pitchDeg(), 0)} deg, max ${f(pm, 0)} deg, head ${f(s.pq[2] * R2D, 0)} deg`); }
console.log('\n=== Cornering (10 deg lean, 0.75 m/s setpoint), new defaults ===');
{ const s = new BB8Sim(ROLL_REC); s.input.vel = 0.25; s.advance(3); s.input.roll = 0.5; s.advance(8); console.log(`v ${f(s.pqd[0] * 1000, 0)} mm/s, lean ${f(s.leanDeg())} deg, radius ${f(s.pqd[0] / s.psiDot, 2)} m (R/sin rho = ${f(s.p.R / Math.sin(s.leanDeg() / R2D), 2)} m)`); }
console.log('\n=== Full speed, head fully right for 3 s, then fully left for 3 s, then stop (no roll input) ===');
function headSwingStop(o) {
  const s = new BB8Sim(o); let md = 0, ms = 0, ml = 0;
  for (let i = 0; i < 1800; i++) { const t = i * 0.01; s.input.vel = t < 10 ? 1 : 0;
    s.input.headR = (t >= 4 && t < 7) ? s.p.domeRollRange : (t >= 7 && t < 10) ? -s.p.domeRollRange : 0;
    s.advance(0.01); const l = Math.abs(s.leanDeg()); if (t < 10) md = Math.max(md, l); else ms = Math.max(ms, l); ml = Math.max(ml, s.rollLoad); }
  return {md, ms, ml: ml * 100};
}
for (const [n, o] of [['body_roll_accel 30, no kd scheduling (before)', {bodyRollAccel: 30, rollKdSpeed: 0}],
                      ['kd scheduling only', {bodyRollAccel: 30}], ['body_roll_accel 60 only', {rollKdSpeed: 0}], ['new defaults (60 + roll_kd_speed 1000)', {}]]) {
  const r = headSwingStop(o); console.log(`${n.padEnd(46)} max lean driving ${f(r.md)} deg, after stop ${f(r.ms)} deg, roll servo load ${f(r.ml, 0)}%`);
}
