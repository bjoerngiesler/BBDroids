// ============================================================================
// BB-8 "Cary Christie V1" drive — simulation core
//  Physics: same planar models as the BBDroids BB-8 sim (pitch plane x-z and
//  roll plane y-z, 3 DOF each: shell position, pendulum angle, head angle).
//  Firmware: BB8_Drive_System V1.4.9 (Stephane Beaulieu), replicated:
//   - 50 Hz loop (intervalUSB 20 ms), PID_v1 (Brett Beauregard, P_ON_E,
//     output/integral clamp ±255, derivative on measurement)
//   - Arduino integer map(), integer servo commands
//   - Drive: Sabertooth R/C via VarSpeedServo with ramp (RAMPINGSPEEDx)
//   - Body tilt and head: hobby servos on a Pololu Maestro with speed limit
// ============================================================================
(function (global) {
  'use strict';
  const D2R = Math.PI / 180, R2D = 180 / Math.PI, G = 9.81;

  // ---- PID_v1 (v1.2.x), DIRECT, P_ON_E, SampleTime 20 ms
  class PIDv1 {
    constructor(kp, ki, kd, ts = 0.02) { this.ts = ts; this.outMin = -255; this.outMax = 255; this.setTunings(kp, ki, kd); this.outputSum = 0; this.lastInput = 0; this.output = 0; }
    setTunings(kp, ki, kd) { this.kp = kp; this.ki = ki * this.ts; this.kd = kd / this.ts; }
    compute(input, setpoint) {
      const error = setpoint - input, dInput = input - this.lastInput;
      this.outputSum += this.ki * error;
      this.outputSum = Math.min(this.outMax, Math.max(this.outMin, this.outputSum));
      let out = this.kp * error + this.outputSum - this.kd * dInput;
      out = Math.min(this.outMax, Math.max(this.outMin, out));
      this.lastInput = input; this.output = out;
      return out;
    }
    reset(input = 0) { this.outputSum = 0; this.lastInput = input; this.output = 0; }
  }

  // Arduino map(): long arithmetic, integer division truncating toward zero
  const amap = (x, a, b, c, d) => Math.trunc(Math.trunc(x - a) * (d - c) / (b - a)) + c;
  const mapf = (x, a, b, c, d) => (x - a) * (d - c) / (b - a) + c;
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

  // Rate-limited command (Maestro speed / VarSpeedServo ramp). rate in units/s, 0 = unlimited.
  class RateLimiter {
    constructor(v0) { this.v = v0; }
    step(target, rate, dt) { if (!(rate > 0)) { this.v = target; return this.v; } const m = rate * dt; this.v += clamp(target - this.v, -m, m); return this.v; }
  }

  function defaultParams() {
    return {
      // Masses (Björn: head ~0.9 kg, counterweight incl. batteries 5.6 kg at 200 mm)
      mp: 5.6, l: 0.20, mh: 0.9,
      // Further mechanics — kept identical to the BBDroids BB-8 sim so only the control differs
      R: 0.254, ms: 3.0, L: 0.32, cRoll: 3.0, crr: 0, bBear: 0.05, inclineX: 0, inclineY: 0,   // crr/cRoll: ground, same model and presets as sim.js
      tauStall: 30, vNoLoad: 2600,     // drive motor at the shell, at full Sabertooth command
      // Hobby servos (assumptions): servo travel over 1000–2000 µs, gearing servo:joint
      tiltServoTravel: 90, tiltGear: 3, tiltServoStall: 3.4, tiltServoNoLoad: 67,
      headServoTravel: 90, headGear: 3, headServoStall: 1.0, headServoNoLoad: 60,
      servoF: 5.0, servoZeta: 0.8,
      // Spin on spot (mechanism unknown -> kinematic: yaw rate ∝ Sabertooth ch. 2, first-order lag)
      spinRateMax: 120, spinTau: 0.3,
      domeShellMass: 0.75, domeShellL: 0.307, domeHoldG: 1.2, // dome shell for the shear estimate (assumption for the Cary head; capacity as in sim.js)
      // ---- Firmware / EEPROM defaults (firstRunSetup)
      bodyTiltMax: 30, headPanMax: 50, headTiltMax: 50,
      driveSpeed1: 50, driveSpeed2: 70, driveSpeed3: 90, speedLevel: 1,
      ramping1: 10, ramping2: 10, ramping3: 10,
      headSpeed: 40, leanSpeed: 20, headAccel: 0,
      spinRamping: 10, spinSpeed1: 90, spinSpeed2: 40, spinLevel: 1,
      kp1: 1, ki1: 0, kd1: 0,   // PID1 body tilt
      kp2: 1, ki2: 0, kd2: 0,   // PID2 drive
      kp3: 2, ki3: 0, kd3: 0,   // PID3 head tilt compensation
      driveDeadzone: 7500, headDeadzone: 7500,
      bodyPitchOffset: 0, bodyRollOffset: 0,
      gyroEnabled: true,        // Back button
      headGyroEnabled: false,   // X + B
      driveEnabled: true,       // Start button (firmware starts disabled)
      pitchSign: 1, rollSign: 1, // IMU orientation (IsBodyPitch/RollReversed); +1 = the stabilising sense
      turning: true,
    };
  }

  // Ground rolling resistance, identical to sim.js (presets there in GROUNDS)
  function rollingResistance(p, v) { return p.crr * (p.ms + p.mp + p.mh) * G * Math.tanh(v / 0.02) + p.cRoll * v; }

  // ---- Planar dynamics (identical to BB-8 sim): q=[s,a1,a2]
  function planarAcc(p, q, qd, Q, beta) {
    const { mp, l, mh, L, ms, R } = p;
    const Is = 2 / 3 * ms * R * R, eps = 1e-4;
    const c1 = Math.cos(q[1]), s1 = Math.sin(q[1]), c2 = Math.cos(q[2]), s2 = Math.sin(q[2]);
    const M00 = ms + Is / (R * R) + mp + mh, M01 = mp * l * c1, M02 = mh * L * c2;
    const M11 = mp * l * l + eps, M22 = mh * L * L + eps;
    // Ground inclined by beta in this plane (rising towards +s). State is in the slope frame: s along the slope,
    // a1 (pendulum, hanging convention) and a2 (head, upright convention) measured from the slope normal.
    // World angle from vertical: pendulum a1 + beta, head a2 - beta. beta = 0 gives the flat-ground equations.
    const b = beta || 0;
    const r0 = Q[0] + mp * l * s1 * qd[1] * qd[1] + mh * L * s2 * qd[2] * qd[2] - (ms + mp + mh) * G * Math.sin(b);
    const r1 = Q[1] - mp * G * l * Math.sin(q[1] + b);
    const r2 = Q[2] + mh * G * L * Math.sin(q[2] - b);
    const den = M00 - M01 * M01 / M11 - M02 * M02 / M22;
    const a0 = (r0 - M01 * r1 / M11 - M02 * r2 / M22) / den;
    return [a0, (r1 - M01 * a0) / M11, (r2 - M02 * a0) / M22];
  }

  class CarySim {
    constructor(params) { this.p = Object.assign(defaultParams(), params || {}); this.reset(); }

    reset() {
      this.t = 0; this.betaP = 0; this.betaR = 0;
      this.pq = [0, 0, 0]; this.pqd = [0, 0, 0];   // pitch: x, theta (pendulum fwd +), alpha (head fwd +)
      this.rq = [0, 0, 0]; this.rqd = [0, 0, 0];   // roll:  y, chi (pendulum right +), zeta (head right +)
      this.wX = 0; this.wZ = 0; this.psi = 0; this.psiDot = 0; this.spinRate = 0; this.aLat = 0; this.trail = [[0, 0]]; this.trailAcc = 0;
      // stick values -1..1 (right / forward positive); spin: -1 L1, +1 R1
      this.input = { vel: 0, roll: 0, headP: 0, headR: 0, spin: 0 };
      this.pid1 = new PIDv1(0, 0, 0); this.pid2 = new PIDv1(0, 0, 0); this.pid3 = new PIDv1(0, 0, 0);
      this.applyParams();
      // command channels in Arduino "servo degrees" (90 = centre)
      this.drivePulse = new RateLimiter(90); this.spinPulse = new RateLimiter(90);
      this.tiltCmd = new RateLimiter(90); this.panCmd = new RateLimiter(90); this.htCmd = new RateLimiter(90);
      this.tiltTarget = 90; this.panTarget = 90; this.htTarget = 90; this.driveTarget = 90; this.spinTarget = 90; this.spinRampRate = 0;
      this.panVel = 0; this.htVel = 0;
      this.sp1 = 0; this.sp2 = 0; this.sp3 = 0; this.sp4 = 0; this.out1 = 0; this.out2 = 0; this.out3 = 0;
      this.ctrlAcc = 0; this.tiltLoad = 0; this.tiltLoadPeak = 0; this.tauM = 0; this.tauR = 0;
      this.shearP = this.shearR = this.shear = this.shearG = this.shearUse = this.shearPeak = 0;
      this.log = [];
    }

    applyParams() {
      const p = this.p;
      this.pid1.setTunings(p.kp1, p.ki1, p.kd1); this.pid2.setTunings(p.kp2, p.ki2, p.kd2); this.pid3.setTunings(p.kp3, p.ki3, p.kd3);
    }
    driveSpeed() { const p = this.p; return [p.driveSpeed1, p.driveSpeed2, p.driveSpeed3][p.speedLevel - 1] || p.driveSpeed1; }
    rampSpeed() { const p = this.p; return [p.ramping1, p.ramping2, p.ramping3][p.speedLevel - 1] || p.ramping1; }
    spinSpeed() { const p = this.p; return p.spinLevel === 2 ? p.spinSpeed2 : p.spinSpeed1; }

    pitchDeg() { return (this.pq[1] + this.betaP) * R2D; }   // IMU, relative to true vertical (incline, see sim.js)
    leanDeg() { return (this.rq[0] / this.p.R - this.betaR) * R2D; }
    headPitchDeg() { return (this.pq[2] - this.betaP) * R2D; }
    headRollDeg() { return (this.rq[2] - this.betaR) * R2D; }
    rollTiltRel() { return this.rq[1] + this.rq[0] / this.p.R; }
    // Sabertooth: 1000..2000 µs -> -1..+1. VarSpeedServo degrees -> µs: 1000 + deg*1000/180
    driveU() { return clamp((this.drivePulse.v - 90) / 90, -1, 1); }
    spinU() { return clamp((this.spinPulse.v - 90) / 90, -1, 1); }
    // servo command degrees -> joint angle [rad]
    tiltJoint(cmd) { const p = this.p; return (cmd - 90) * p.tiltServoTravel / 180 / p.tiltGear * D2R; }
    headJoint(cmd) { const p = this.p; return (cmd - 90) * p.headServoTravel / 180 / p.headGear * D2R; }

    // ------------------------------------------------------------ firmware, every 20 ms
    controlStep(dt) {
      const p = this.p, inp = this.input;
      const ds = this.driveSpeed();
      // readSticks(): Xbox raw values ±32767
      const JRX = Math.round(clamp(inp.roll, -1, 1) * 32767), JRY = Math.round(clamp(inp.vel, -1, 1) * 32767);
      const JLX = Math.round(clamp(inp.headR, -1, 1) * 32767), JLY = Math.round(clamp(inp.headP, -1, 1) * 32767);
      if (p.driveEnabled) {
        // Firmware maps JoyRightX 22000 -> -MAX (servo orientation); the sim uses "stick right = lean right"
        this.sp1 = Math.abs(JRX) > p.driveDeadzone ? -clamp(amap(JRX, 22000, -22000, -p.bodyTiltMax, p.bodyTiltMax), -p.bodyTiltMax, p.bodyTiltMax) : 0;
        this.sp2 = Math.abs(JRY) > p.driveDeadzone ? clamp(amap(JRY, -22000, 22000, -ds, ds), -ds, ds) : 0;
      } else { this.sp1 = 0; this.sp2 = 0; }
      this.sp3 = Math.abs(JLX) > p.headDeadzone ? -clamp(amap(JLX, 32768, -32767, -p.headTiltMax, p.headTiltMax), -p.headTiltMax, p.headTiltMax) : 0;
      this.sp4 = Math.abs(JLY) > p.headDeadzone ? -clamp(amap(JLY, 32768, -32767, -p.headPanMax, p.headPanMax), -p.headPanMax, p.headPanMax) : 0;

      // computeBodyPosition()
      const input1 = p.rollSign * this.leanDeg() + p.bodyRollOffset;
      const input2 = p.pitchSign * this.pitchDeg() + p.bodyPitchOffset;
      this.pid1.compute(input1, this.sp1); this.pid2.compute(input2, this.sp2);
      if (p.gyroEnabled) {
        this.out1 = clamp(this.pid1.output, -p.bodyTiltMax, p.bodyTiltMax);
        this.tiltTarget = Math.trunc(mapf(this.out1, -p.bodyTiltMax, p.bodyTiltMax, 90 - p.bodyTiltMax, 90 + p.bodyTiltMax));
        this.out2 = clamp(this.pid2.output, -ds, ds);
        this.driveTarget = amap(this.out2, -ds, ds, 90 - ds, 90 + ds);
      } else {
        this.out1 = this.sp1; this.out2 = this.sp2;
        this.tiltTarget = Math.trunc(90 + this.sp1);
        this.driveTarget = 90 + this.sp2;
      }

      // computeHeadPosition(): Input3 = Input1 * -1.5 (sign chosen so it counteracts body lean)
      const input3 = 1.5 * input1;
      this.pid3.compute(input3, this.sp3);
      if (p.headGyroEnabled) { this.out3 = clamp(this.pid3.output, -p.headTiltMax, p.headTiltMax); this.htTarget = Math.trunc(90 + this.out3); }
      else { this.out3 = this.sp3; this.htTarget = Math.trunc(90 + this.sp3); }
      this.panTarget = Math.trunc(90 + this.sp4);

      // Spin on spot (L1/R1, only with drive enabled): write(90±spin, 255) / release: write(90, SPINRAMPING)
      if (p.driveEnabled && inp.spin) { this.spinTarget = 90 + Math.sign(inp.spin) * this.spinSpeed(); this.spinRampRate = 255 * 0.5 / 0.02 * 180 / 1000; }
      else if (this.spinTarget !== 90 || this.spinPulse.v !== 90) { this.spinTarget = 90; this.spinRampRate = p.spinRamping * 0.5 / 0.02 * 180 / 1000; }

      this.logSample();
    }

    // ------------------------------------------------------------ physics
    servoTorque(target, rel, relRate, inertia, gear, stall, noLoad) {
      const p = this.p, w = 2 * Math.PI * p.servoF;
      const tau = inertia * (w * w * (target - rel) - 2 * p.servoZeta * w * relRate);
      const ts = stall * gear, ws = relRate * gear, w0 = noLoad * 2 * Math.PI / 60;
      const hi = Math.min(ts, ts * (1 - ws / w0)), lo = Math.max(-ts, -ts * (1 + ws / w0));
      return clamp(tau, lo, hi);
    }
    forcesPitch(q, qd) {
      const p = this.p, R = p.R;
      const wRel = qd[1] + qd[0] / R, w0 = p.vNoLoad / 1000 / R;
      const tauM = p.tauStall * (this.driveU() - wRel / w0) - p.bBear * wRel;
      const tauH = this.servoTorque(this.headJoint(this.panCmd.v), q[2] + q[1], qd[2] + qd[1], p.mh * p.L * p.L + 1e-3, p.headGear, p.headServoStall, p.headServoNoLoad);
      this.tauM = tauM;
      return [tauM / R - rollingResistance(p, qd[0]), tauM + tauH, tauH];
    }
    forcesRoll(q, qd) {
      const p = this.p, R = p.R;
      const tauR = this.servoTorque(this.tiltJoint(this.tiltCmd.v), q[1] + q[0] / R, qd[1] + qd[0] / R, p.mp * p.l * p.l + 1e-3, p.tiltGear, p.tiltServoStall, p.tiltServoNoLoad);
      const tauD = this.servoTorque(this.headJoint(this.htCmd.v), q[2] - q[0] / R, qd[2] - qd[0] / R, p.mh * p.L * p.L + 1e-3, p.headGear, p.headServoStall, p.headServoNoLoad);
      this.tauR = tauR;
      const a = p.turning ? this.aLat : 0;
      const fS = -(p.ms + p.mp + p.mh) * a, f1 = -p.mp * a * p.l * Math.cos(q[1]), f2 = -p.mh * a * p.L * Math.cos(q[2]);
      return [tauR / R - tauD / R - rollingResistance(p, qd[0]) + fS, tauR + f1, tauD + f2];
    }
    computeShear() {
      const p = this.p, m = p.domeShellMass, Lo = p.domeShellL, tauM = this.tauM, tauR = this.tauR;
      const ap = planarAcc(p, this.pq, this.pqd, this.forcesPitch(this.pq, this.pqd), this.betaP);
      const ar = planarAcc(p, this.rq, this.rqd, this.forcesRoll(this.rq, this.rqd), this.betaR);
      const al = this.pq[2], ze = this.rq[2], aLat = p.turning ? this.aLat : 0;
      this.shearP = m * (ap[0] * Math.cos(al) + Lo * ap[2]) - m * G * Math.sin(al - this.betaP);   // gravity relative to the slope
      this.shearR = m * ((ar[0] + aLat) * Math.cos(ze) + Lo * ar[2]) - m * G * Math.sin(ze - this.betaR);
      this.shear = Math.hypot(this.shearP, this.shearR);
      this.shearG = this.shear / (m * G);              // in units of the shell's weight (1 = holding it sideways, 90 deg)
      this.shearUse = this.shearG / p.domeHoldG;       // fraction of the magnet capacity
      this.shearPeak = Math.max(this.shearPeak, this.shearUse);
      this.tauM = tauM; this.tauR = tauR;              // forcesPitch/forcesRoll above overwrite these; keep the integrator's values
    }
    rk4(q, qd, forces, h, beta) {
      const f = (q, qd) => planarAcc(this.p, q, qd, forces.call(this, q, qd), beta);
      const add = (a, b, k) => a.map((v, i) => v + k * b[i]);
      const k1v = qd, k1a = f(q, qd);
      const q2 = add(q, k1v, h / 2), v2 = add(qd, k1a, h / 2), k2a = f(q2, v2);
      const q3 = add(q, v2, h / 2), v3 = add(qd, k2a, h / 2), k3a = f(q3, v3);
      const q4 = add(q, v3, h), v4 = add(qd, k3a, h), k4a = f(q4, v4);
      for (let i = 0; i < 3; i++) { q[i] += h / 6 * (k1v[i] + 2 * v2[i] + 2 * v3[i] + v4[i]); qd[i] += h / 6 * (k1a[i] + 2 * k2a[i] + 2 * k3a[i] + k4a[i]); }
    }

    advance(dt) {
      const h = 0.001, ctrlDt = 0.02, p = this.p;
      // Maestro speed unit 0.25 µs/10 ms, VarSpeedServo 0.5 µs/20 ms -> both = 4.5 servo°/s per unit
      const maestro = s => s * 4.5;
      let n = Math.round(dt / h);
      while (n-- > 0) {
        this.ctrlAcc += h;
        if (this.ctrlAcc >= ctrlDt - 1e-9) { this.ctrlAcc -= ctrlDt; this.controlStep(ctrlDt); }
        this.drivePulse.step(this.driveTarget, maestro(this.rampSpeed()), h);
        this.spinPulse.step(this.spinTarget, this.spinRampRate, h);
        this.tiltCmd.step(this.tiltTarget, maestro(p.leanSpeed), h);
        this.stepHeadCh(this.panCmd, 'panVel', this.panTarget, h);
        this.stepHeadCh(this.htCmd, 'htVel', this.htTarget, h);
        // yaw: lean-induced turning + spin on spot (kinematic)
        const v = this.pqd[0], rho = this.rq[0] / p.R;
        this.spinRate += (this.spinU() * p.spinRateMax * D2R - this.spinRate) * Math.min(1, h / Math.max(h, p.spinTau));
        const turn = p.turning ? v / p.R * Math.sin(rho) : 0;
        this.psiDot = turn + this.spinRate;
        this.aLat = v * this.psiDot; // spin while driving curves the path as well
        // Incline: world slope (inclineX uphill towards +X = initial forward, inclineY uphill towards +Z = initial right),
        // projected onto this droid's pitch plane (forward) and roll plane (right) with its current heading.
        { const bx = p.inclineX * D2R, by = p.inclineY * D2R, c = Math.cos(this.psi), sn = Math.sin(this.psi);
          this.betaP = bx * c + by * sn; this.betaR = -bx * sn + by * c; }
        this.rk4(this.pq, this.pqd, this.forcesPitch, h, this.betaP);
        this.rk4(this.rq, this.rqd, this.forcesRoll, h, this.betaR);
        this.computeShear();
        this.tiltLoad = Math.abs(this.tauR || 0) / (p.tiltServoStall * p.tiltGear);
        this.tiltLoadPeak = Math.max(this.tiltLoadPeak, this.tiltLoad);
        this.psi += this.psiDot * h;
        const vy = this.rqd[0], c = Math.cos(this.psi), sn = Math.sin(this.psi);
        this.wX += (v * c - vy * sn) * h; this.wZ += (v * sn + vy * c) * h;
        this.trailAcc += h;
        if (this.trailAcc >= 0.05) { this.trailAcc = 0; this.trail.push([this.wX, this.wZ]); if (this.trail.length > 1200) this.trail.shift(); }
        this.t += h;
      }
    }
    // Maestro channel with speed limit and optional acceleration limit (unit 0.25 µs/10 ms/80 ms)
    stepHeadCh(ch, vk, target, h) {
      const p = this.p, vmax = p.headSpeed > 0 ? p.headSpeed * 4.5 : Infinity;
      if (!(p.headAccel > 0)) { ch.step(target, vmax === Infinity ? 0 : vmax, h); return; }
      const amax = p.headAccel * 4.5 / 0.08, e = target - ch.v;
      const vdes = Math.sign(e) * Math.min(vmax, Math.sqrt(2 * amax * Math.abs(e)));
      this[vk] += clamp(vdes - this[vk], -amax * h, amax * h);
      let nv = ch.v + this[vk] * h;
      if ((target - ch.v) * (target - nv) < 0) { nv = target; this[vk] = 0; }
      ch.v = nv;
    }

    logSample() {
      const ds = this.driveSpeed();
      this.log.push({
        t: this.t, vGround: this.pqd[0] * 1000,
        sp2: this.sp2 / 90 * 100, out2: this.out2 / 90 * 100, drv: this.driveU() * 100,
        pitch: this.pitchDeg(), headP: this.headPitchDeg(),
        sp1: this.sp1, lean: this.leanDeg(), tilt: this.tiltJoint(this.tiltCmd.v) * R2D, ds,
      });
      if (this.log.length > 1500) this.log.splice(0, this.log.length - 1500);
    }
  }

  const api = { CarySim, defaultParams, PIDv1, amap, D2R, R2D };
  if (typeof module !== 'undefined' && module.exports) module.exports = api; else global.CARY = api;
})(typeof window !== 'undefined' ? window : globalThis);

