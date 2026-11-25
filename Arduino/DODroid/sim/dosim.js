// ============================================================================
// D-O simulation core (source for DO-Drive-Simulation.html, built by build.py)
//  - Planar pitch model, 3 DOF:
//      x   : wheel centre position over ground (rolling without slip)
//      th  : chassis absolute angle (chassis CoM below the axle; + = CoM swings forward)
//      phi : neck absolute angle (neck + head above the axle; + = forward)
//  - Yaw from the left/right wheel torque difference (1 DOF)
//  - Firmware control from DODroid / LibBB reproduced (100 Hz loop, head and drive alternate -> 50 Hz each)
// ============================================================================
(function (global) {
  'use strict';
  const D2R = Math.PI / 180, R2D = 180 / Math.PI, G = 9.81;

  // ---- LibBB LowPassFilter (bilinear Butterworth 2nd order, no prewarping)
  class LowPassFilter {
    constructor(cutoff, sampleFreq) { this.cutoff = cutoff; this.dt = 1 / sampleFreq; this.reset(); this.calc(); }
    setCutoff(c) { this.cutoff = c; this.calc(); }
    calc() {
      const al = 2 * Math.PI * this.cutoff * this.dt, al2 = al * al, b1 = Math.SQRT2, D = al2 + 2 * al * b1 + 4;
      this.b0 = al2 / D; this.b1c = 2 * this.b0; this.b2 = this.b0;
      this.a0 = -(2 * al2 - 8) / D; this.a1 = -(al2 - 2 * b1 * al + 4) / D;
    }
    filter(xn) {
      const x = this.x, y = this.y; x[0] = xn;
      y[0] = this.a0 * y[1] + this.b0 * x[0] + this.a1 * y[2] + this.b1c * x[1] + this.b2 * x[2];
      y[2] = y[1]; y[1] = y[0]; x[2] = x[1]; x[1] = x[0];
      return y[0];
    }
    reset() { this.x = [0, 0, 0]; this.y = [0, 0, 0]; }
  }

  // ---- LibBB PIDController (paths used by DODroid)
  class PID {
    constructor() {
      this.kp = this.ki = this.kd = 0; this.goal = 0; this.offset = 0; this.reverse = false;
      this.gain = 1; // input_.controlGain() in LibBB, multiplies the control term (encoder in mm: 1/mmPerTick)
      this.iBounded = false; this.iMin = this.iMax = 0; this.errDbMin = 0; this.errDbMax = 0;
      this.dFilter = new LowPassFilter(5.0, 100); this.reset();
    }
    reset() { this.lastErr = this.errI = this.lastErrD = this.lastControl = 0; this.dFilter.reset(); }
    setIBounds(a, b) { this.iBounded = true; this.iMin = a; this.iMax = b; }
    update(present, dt) {
      let err = this.reverse ? this.goal + present : this.goal - present;
      if (err > this.errDbMin && err < this.errDbMax) err = 0;
      this.errI += err * dt;
      if (this.iBounded) this.errI = Math.min(this.iMax, Math.max(this.iMin, this.errI));
      this.lastErrD = (err - this.lastErr) / dt; this.lastErr = err;
      const dF = this.dFilter.filter(this.lastErrD);
      this.lastControl = this.kp * err + this.ki * this.errI + this.kd * dF;
      this.lastControl *= this.gain;
      return this.reverse ? -(this.lastControl + this.offset) : this.lastControl + this.offset;
    }
  }

  // ---- Servo: position goal with optional profile velocity, PD position loop limited by the torque-speed line
  class Servo {
    constructor() { this.goal = 0; }
  }

  function defaultParams() {
    return {
      // Geometry and masses, derived from the CAD (JamesB D-OEvo V3) where noted
      r: 0.126,          // m   wheel radius (CAD: TPU rim 252.6 mm; firmware WHEEL_CIRCUMFERENCE assumes 0.115)
      b: 0.100,          // m   track width (CAD: wheel halves 100 mm apart; firmware WHEEL_DISTANCE 95)
      mw: 0.45,          // kg  both wheel halves incl. tyres (CAD volumes, printed 0.8 g/cm3)
      wheelK: 0.75,      //     wheel inertia = wheelK * mw * r^2 (ring-like)
      mc: 3.9,           // kg  chassis: 1.19 kg parts + 0.19 kg 4x18650 + 2.5 kg lead (Björn)
      lc: 0.057,         // m   chassis CoM below the axle (CAD; lead CoM assumed 85 mm below the axle)
      Ic: 0.012,         // kg m^2 chassis inertia about its own CoM (estimate)
      // Head assembly (not in the CAD): neck + head + head counterweight, pivoting about the main axle
      mHead: 1.2,        // kg  total mass of the head assembly incl. its counterweight (ESTIMATE)
      headMoment: 0.0,   // kg m  net m*d about the axle (+ = top-heavy); Björn: roughly balanced
      headInertia: 0.022,// kg m^2 about the axle (CAD head assembly + estimated servos/electronics)
      Ln: 0.19,          // m   axle to head pivot (CAD), for drawing only
      neckGear: 1.69,    //     neck servo degrees per degree of neck motion (CAD gear diameters 57.4/34 mm, ASSUMPTION)
      Iz: 0.02,          // kg m^2 yaw inertia (estimate)
      cRoll: 2.0,        // N/(m/s) rolling resistance (estimate)
      cYaw: 0.05,        // Nm/(rad/s) yaw scrub friction (estimate)
      bBear: 0.01,       // Nm/(rad/s) bearing friction per side (estimate)
      // Drive motors (per side, at the wheel)
      // Pololu 25D 20.4:1 (HP assumed) @ 4S, 97/18 to the wheel, ~80 % gear efficiency
      tauStall: 2.3,     // Nm  per side at the wheel
      vNoLoad: 1500,     // mm/s at PWM 255
      // Neck servo XL430-W250 (CAD) @ 12 V
      servoStall: 1.5, servoNoLoad: 61, servoF: 4.0, servoZeta: 0.8,
      pitchAtRest: 0,    // deg, IMU pitch measured at rest during calibration
      imuPitchSign: 1,   // +1: IMU pitch positive = chassis top forward (nose down); -1: opposite
      neckServoSign: 1,  // +1: positive neck servo angle = neck backward; -1: forward
      encoderQuantize: true,
      // Firmware parameters (DOParams)
      neckRange: 45, motorDeadband: 10,
      wheelKp: 0.06, wheelKi: 0.8, wheelKd: 0, wheelIBound: 255,
      balKp: 20, balKi: 0, balKd: 0, balErrDeadband: 1.0,
      autoPosKp: 6, autoPosKi: 0.2, autoPosKd: 0,
      posKp: 1.2, posKi: 0.2, posKd: 0,     // pos_k*: DRIVE_POS (secondary remote), firmware defaults
      posStickRange: 1000,  // mm position offset at full stick in DRIVE_POS (firmware: axis * 1000)
      // Drive mode: 'vel' (DRIVE_VEL), 'pos' (DRIVE_POS, stick = position offset), 'autopos' (DRIVE_VEL with input,
      // DRIVE_AUTO_POS after 500 ms idle; firmware default, auto_pos_control true)
      driveMode: 'autopos',
      maxSpeed: 1000, accel: 2500,
      speedAxisGain: 1.0, rotAxisGain: 0.4,
      leanHeadToBody: 0.0,
      faNeckIMUAccel: 0, faNeckIMUPitch: 0.7, faNeckSPAccel: -0.005, faNeckSpeed: -0.01, faNeckSpeedSP: -0.04,
      faHeadPitchSpeedSP: -0.01, faHeadRollTurn: 0.10, faHeadHeadingTurn: 0.14,
      turning: true,
    };
  }
  const TICKS_PER_TURN = 979.2 * (97.0 / 18.0), WHEEL_CIRC_FW = 722.566;
  const VEL = 0, AUTO_POS = 1, OFF = 2, POS = 3;
  const MODE_NAMES = ['VEL', 'AUTO_POS', 'OFF', 'POS'];

  class DOSim {
    constructor(params) {
      this.p = Object.assign(defaultParams(), params || {});
      // settings saved by the earlier page version only had the auto_pos_control checkbox
      if (params && params.driveMode === undefined && params.autoPosControl === false) this.p.driveMode = 'vel';
      delete this.p.autoPosControl;
      this.reset();
    }

    reset() {
      this.t = 0; this.seq = 0;
      this.q = [0, 0, 0]; this.qd = [0, 0, 0];        // x, th, phi
      this.psi = 0; this.psiDot = 0; this.wX = 0; this.wZ = 0; this.distL = 0; this.distR = 0; this.trail = [[0, 0]]; this.trailAcc = 0;
      this.input = { vel: 0, rot: 0, headP: 0, headH: 0, headR: 0, lean: 0, driveOn: true };
      this.lPID = new PID(); this.rPID = new PID(); this.bal = new PID(); this.pos = new PID();
      this.bal.reverse = true;
      this.balF = new LowPassFilter(100, 100);
      this.encL = { ticks: 0, speed: 0, filt: 0, f: new LowPassFilter(25, 100), posMM: 0 };
      this.encR = { ticks: 0, speed: 0, filt: 0, f: new LowPassFilter(25, 100), posMM: 0 };
      this.curVel = 0; this.curRot = 0; this.goalVel = 0; this.goalRot = 0;
      this.pwmL = 0; this.pwmR = 0; this.balOut = 0;
      this.mode = this.wantedMode(); this.modeKey = this.p.driveMode; this.msNoInput = 0; this.posZero = 0;
      this.neckGoal = 0; this.headPitch = 0; this.headHeading = 0; this.headRoll = 0;
      this.neckTarget = 0; // relative neck angle target (rad, + = forward)
      this.applyParams();
      this.switchDrive(this.mode);
      this.ctrlAcc = 0; this.log = []; this.tauL = this.tauR = this.tauN = 0;
    }

    applyParams() {
      const p = this.p;
      for (const c of [this.lPID, this.rPID]) { c.kp = p.wheelKp; c.ki = p.wheelKi; c.kd = p.wheelKd; c.setIBounds(-p.wheelIBound, p.wheelIBound); c.gain = TICKS_PER_TURN / WHEEL_CIRC_FW; }  // bb::Encoder controlGain() = 1/mmPerTick (~7.3): LibBB multiplies the control term by it
      this.bal.kp = p.balKp; this.bal.ki = p.balKi; this.bal.kd = p.balKd; this.bal.errDbMin = -p.balErrDeadband; this.bal.errDbMax = p.balErrDeadband;
      // autoPosController_ and posController_ share input/output; one PID object with the gains of the active mode
      const pg = this.mode === POS ? [p.posKp, p.posKi, p.posKd] : [p.autoPosKp, p.autoPosKi, p.autoPosKd];
      this.pos.kp = pg[0]; this.pos.ki = pg[1]; this.pos.kd = pg[2];
    }

    // derived
    pitchDeg() { return -this.p.imuPitchSign * this.q[1] * R2D; }                     // IMU pitch: + = chassis top forward (nose down)
    neckRel() { return this.q[2] + this.q[1]; }                 // neck relative to chassis [rad], + = forward
    wheelRelL() { return (this.qd[0] + this.psiDot * this.p.b / 2) / this.p.r + this.qd[1]; } // wheel vs chassis [rad/s]
    wheelRelR() { return (this.qd[0] - this.psiDot * this.p.b / 2) / this.p.r + this.qd[1]; }
    cogAboveAxle() { const p = this.p; const m = p.mw + p.mc + p.mHead;
      return (-p.mc * p.lc * Math.cos(this.q[1]) + p.headMoment * Math.cos(this.q[2])) / m; }

    // mode the drive switches to when enabled, as in NewDODroid: the primary drive button always switches OFF -> VEL
    // (auto position then follows via the 500 ms idle rule), the secondary drive button switches to POS.
    // (The older DODroid switched straight to AUTO_POS when auto_pos_control was set.)
    wantedMode() { return this.p.driveMode === 'pos' ? POS : VEL; }

    switchDrive(mode) {
      this.lPID.reset(); this.lPID.goal = 0; this.rPID.reset(); this.rPID.goal = 0;
      this.bal.reset(); this.bal.goal = -this.p.pitchAtRest;
      this.pos.reset(); this.pos.goal = (this.encL.posMM + this.encR.posMM) / 2; this.posZero = this.pos.goal;
      this.mode = mode; this.applyParams();
    }

    // --------------------------------------------------------------- firmware, called at 100 Hz
    controlTick(dt) {
      const p = this.p, inp = this.input;
      // packet handling (continuous stick input)
      const idle = Math.abs(inp.vel) < 1e-3 && Math.abs(inp.rot) < 1e-3;
      if (!inp.driveOn) { if (this.mode !== OFF) this.switchDrive(OFF); }
      else {
        if (this.mode === OFF || this.modeKey !== p.driveMode) { this.modeKey = p.driveMode; this.switchDrive(this.wantedMode()); }
        const auto = p.driveMode === 'autopos';
        if (idle && auto) { this.msNoInput += dt; if (this.msNoInput > 0.5 && this.mode === VEL) this.switchDrive(AUTO_POS); }
        else if (!idle) { this.msNoInput = 0; if (this.mode === AUTO_POS) this.switchDrive(VEL); }
        if (this.mode === POS) this.pos.goal = this.posZero + inp.vel * p.posStickRange;   // rotation is ignored in DRIVE_POS
        if (this.mode === VEL) {
          this.goalVel = Math.max(-p.maxSpeed, Math.min(p.maxSpeed, inp.vel * p.maxSpeed * p.speedAxisGain));
          this.goalRot = Math.max(-p.maxSpeed, Math.min(p.maxSpeed, inp.rot * p.maxSpeed * p.rotAxisGain));
        }
      }
      this.lean = inp.lean * p.neckRange;
      this.bal.goal = p.leanHeadToBody * this.lean - p.pitchAtRest;

      // encoders (relative wheel/chassis) and IMU, every tick
      // Firmware mm per tick (WHEEL_CIRCUMFERENCE 722.566 mm, i.e. r = 0.115 m): the firmware's speeds and positions
      // are about 9 % smaller than the real ones with the CAD wheel radius r.
      const mmPerTick = WHEEL_CIRC_FW / TICKS_PER_TURN;
      for (const [e, rel] of [[this.encL, this.relAngL || 0], [this.encR, this.relAngR || 0]]) {
        const tk = rel / (2 * Math.PI) * TICKS_PER_TURN;
        const ticks = p.encoderQuantize ? Math.floor(tk) : tk;
        e.speed = (ticks - e.ticks) / dt * mmPerTick; e.ticks = ticks; e.posMM = ticks * mmPerTick;
        e.filt = e.f.filter(e.speed);
      }
      const pitch = this.pitchDeg(), yawRate = this.psiDot * R2D;

      this.seq++;
      const dt2 = 2 * dt;
      if (this.seq % 2 === 0) this.stepHead(pitch, yawRate);
      else this.stepDrive(dt2, pitch);
    }

    stepDrive(dt, pitch) {
      const p = this.p;
      if (this.mode === OFF) { this.pwmL = this.pwmR = 0; return; }
      // balanceController_.update() -> velOutput_.set(value)
      const value = this.bal.update(this.balF.filter(pitch), dt);
      this.balOut = value;
      const ramp = (cur, goal) => cur < goal ? Math.min(goal, cur + p.accel * dt) : Math.max(goal, cur - p.accel * dt);
      this.curVel = ramp(this.curVel, this.goalVel); this.curRot = ramp(this.curRot, this.goalRot);
      const clamp = v => Math.max(-p.maxSpeed, Math.min(p.maxSpeed, v));
      this.lPID.goal = clamp(this.curVel + this.curRot - value);
      this.rPID.goal = clamp(this.curVel - this.curRot - value);
      // wheel speed controllers -> motors (DCMotor deadband, +-255)
      const motor = v => { v = Math.max(-255, Math.min(255, v)); return Math.abs(v) < p.motorDeadband ? 0 : v; };
      this.pwmL = motor(this.lPID.update(this.encL.filt, dt));
      this.pwmR = motor(this.rPID.update(this.encR.filt, dt));
      // position controller (after the others, as in the firmware) -> setGoalVelocity / setGoalRotation(0)
      if (this.mode === AUTO_POS || this.mode === POS) {
        const g = this.pos.update((this.encL.posMM + this.encR.posMM) / 2, dt);
        this.goalVel = g; this.goalRot = 0;
      }
    }

    stepHead(pitch, yawRate) {
      const p = this.p;
      const speed = (this.encL.filt + this.encR.filt) / 2, speedSP = this.goalVel;
      let accelSP = speedSP - speed; if (speedSP === 0) accelSP = 0;
      let nod = p.faNeckSPAccel * accelSP + p.faNeckSpeed * speed;   // faNeckIMUAccel*ax: ax not modelled (default 0)
      nod += speedSP < 0 ? p.faNeckSpeedSP * speedSP / 2 : p.faNeckSpeedSP * speedSP;
      nod += this.lean + p.pitchAtRest;
      let neck = nod + p.faNeckIMUPitch * pitch;
      neck = Math.max(-p.neckRange, Math.min(p.neckRange, neck));
      this.neckGoal = neck;                         // servo degrees, + = neck backward (see docs)
      this.neckTarget = -p.neckServoSign * neck / p.neckGear * D2R; // relative neck angle, + = forward
      this.headPitch = -nod + this.input.headP + p.faHeadPitchSpeedSP * speedSP;
      this.headHeading = p.faHeadHeadingTurn * yawRate + this.input.headH;
      this.headRoll = -p.faHeadRollTurn * yawRate + this.input.headR;
    }

    // --------------------------------------------------------------- physics
    motorTorque(pwm, wRel) {
      const p = this.p, w0 = p.vNoLoad / 1000 / p.r;
      return p.tauStall * (pwm / 255 - wRel / w0) - p.bBear * wRel;
    }

    deriv(s) {
      // s = [x, th, phi, xd, thd, phid, psiDot]
      const p = this.p, [x, th, phi, xd, thd, phid, pd] = s;
      const wl = (xd + pd * p.b / 2) / p.r + thd, wr = (xd - pd * p.b / 2) / p.r + thd;
      const on = this.mode !== OFF;
      const tL = this.motorTorque(on ? this.pwmL : 0, wl), tR = this.motorTorque(on ? this.pwmR : 0, wr);
      const tau = tL + tR;
      // neck servo: PD on relative angle, torque-speed limited
      const Mn = p.headMoment, In = p.headInertia + 1e-4;
      const w = 2 * Math.PI * p.servoF, rel = phi + th, relD = phid + thd;
      let tN = In * (w * w * (this.neckTarget - rel) - 2 * p.servoZeta * w * relD);
      const ts = p.servoStall * p.neckGear, w0s = p.servoNoLoad * 2 * Math.PI / 60 / p.neckGear;
      tN = Math.max(Math.max(-ts, -ts * (1 + relD / w0s)), Math.min(Math.min(ts, ts * (1 - relD / w0s)), tN));
      this.tauL = tL; this.tauR = tR; this.tauN = tN;
      // mass matrix (arrow shape)
      const Iw = p.wheelK * p.mw * p.r * p.r;
      const c1 = Math.cos(th), s1 = Math.sin(th), c2 = Math.cos(phi), s2 = Math.sin(phi);
      const M00 = p.mw + Iw / (p.r * p.r) + p.mc + p.mHead, M01 = p.mc * p.lc * c1, M02 = Mn * c2;
      const M11 = p.Ic + p.mc * p.lc * p.lc, M22 = In;
      const Q0 = tau / p.r - p.cRoll * xd, Q1 = tau + tN, Q2 = tN;
      const r0 = Q0 + p.mc * p.lc * s1 * thd * thd + Mn * s2 * phid * phid;
      const r1 = Q1 - p.mc * G * p.lc * s1;
      const r2 = Q2 + Mn * G * s2;
      const den = M00 - M01 * M01 / M11 - M02 * M02 / M22;
      const a0 = (r0 - M01 * r1 / M11 - M02 * r2 / M22) / den;
      const a1 = (r1 - M01 * a0) / M11, a2 = (r2 - M02 * a0) / M22;
      // yaw
      const pdd = p.turning ? (((tL - tR) / p.r) * (p.b / 2) - p.cYaw * pd) / p.Iz : 0;
      return [xd, thd, phid, a0, a1, a2, pdd];
    }

    advance(dt) {
      const h = 0.001, ctrlDt = 0.01;
      let n = Math.round(dt / h);
      while (n-- > 0) {
        this.ctrlAcc += h;
        if (this.ctrlAcc >= ctrlDt - 1e-9) { this.ctrlAcc -= ctrlDt; this.controlTick(ctrlDt); if (this.seq % 2 === 1) this.logSample(); }
        const s = [...this.q, ...this.qd, this.psiDot];
        const add = (a, b, k) => a.map((v, i) => v + k * b[i]);
        const k1 = this.deriv(s), k2 = this.deriv(add(s, k1, h / 2)), k3 = this.deriv(add(s, k2, h / 2)), k4 = this.deriv(add(s, k3, h));
        const ns = s.map((v, i) => v + h / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]));
        // relative wheel angles for the encoders
        const wl = this.wheelRelL(), wr = this.wheelRelR();
        this.relAngL = (this.relAngL || 0) + wl * h; this.relAngR = (this.relAngR || 0) + wr * h;
        this.q = ns.slice(0, 3); this.qd = ns.slice(3, 6); this.psiDot = ns[6];
        this.psi += this.psiDot * h;
        const v = this.qd[0];
        this.distL += (v + this.psiDot * this.p.b / 2) * h; this.distR += (v - this.psiDot * this.p.b / 2) * h;
        this.wX += v * Math.cos(this.psi) * h; this.wZ += v * Math.sin(this.psi) * h;
        this.trailAcc += h; if (this.trailAcc >= 0.05) { this.trailAcc = 0; this.trail.push([this.wX, this.wZ]); if (this.trail.length > 1200) this.trail.shift(); }
        this.t += h;
      }
    }

    // lateral tipping: centripetal acceleration vs. what the wheel track can hold
    lateralAccel() { return this.qd[0] * this.psiDot; }
    lateralLimit() { const p = this.p; return G * (p.b / 2) / Math.max(0.01, p.r + this.cogAboveAxle()); }

    logSample() {
      this.log.push({
        t: this.t, vSP: this.goalVel, vCmd: this.curVel, vGround: this.qd[0] * 1000, vEnc: (this.encL.filt + this.encR.filt) / 2,
        pitch: this.pitchDeg(), neckRel: this.neckRel() * R2D, neckAbs: this.q[2] * R2D, pwmL: this.pwmL, pwmR: this.pwmR,
        rotSP: this.goalRot / (this.p.b * 500) * R2D, yawRate: this.psiDot * R2D, latA: this.lateralAccel(), latLim: this.lateralLimit(),
      });
      if (this.log.length > 1000) this.log.splice(0, this.log.length - 1000);
    }
  }

  const api = { DOSim, defaultParams, LowPassFilter, PID, D2R, R2D, VEL, AUTO_POS, OFF, POS, MODE_NAMES };
  if (typeof module !== 'undefined' && module.exports) module.exports = api; else global.DO = api;
})(typeof window !== 'undefined' ? window : globalThis);

