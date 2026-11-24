// ============================================================================
// BB-8 simulation core
//  - Two decoupled planar models (pitch plane x-z, roll plane y-z), 3 DOF each:
//      s  : position of the sphere center over ground (rolling without slip)
//      a1 : absolute angle of the pendulum (point mass mp at distance l below the sphere center)
//      a2 : absolute angle of the head     (point mass mh at distance L above the sphere center)
//  - Firmware controllers from BB8Droid / LibBB replicated (100 Hz)
// ============================================================================
(function (global) {
  'use strict';
  const D2R = Math.PI / 180, R2D = 180 / Math.PI, G = 9.81;

  // ---- LibBB LowPassFilter (bilinear, 2nd order Butterworth, no prewarping)
  class LowPassFilter {
    constructor(cutoff, sampleFreq) { this.cutoff = cutoff; this.dt = 1 / sampleFreq; this.x = [0, 0, 0]; this.y = [0, 0, 0]; this.calc(); }
    setCutoff(c) { this.cutoff = c; this.calc(); }
    calc() {
      const al = 2 * Math.PI * this.cutoff * this.dt, al2 = al * al, b1 = Math.SQRT2;
      const D = al2 + 2 * al * b1 + 4;
      this.b0 = al2 / D; this.b1c = 2 * this.b0; this.b2 = this.b0;
      this.a0 = -(2 * al2 - 8) / D; this.a1 = -(al2 - 2 * b1 * al + 4) / D;
    }
    filter(xn) {
      const x = this.x, y = this.y;
      x[0] = xn;
      y[0] = this.a0 * y[1] + this.b0 * x[0] + this.a1 * y[2] + this.b1c * x[1] + this.b2 * x[2];
      y[2] = y[1]; y[1] = y[0]; x[2] = x[1]; x[1] = x[0];
      return y[0];
    }
    reset() { this.x = [0, 0, 0]; this.y = [0, 0, 0]; }
  }

  // ---- LibBB PIDController (only the code paths used here)
  class PID {
    constructor() {
      this.kp = this.ki = this.kd = 0; this.goal = 0; this.offset = 0;
      // Corresponds to input_.controlGain() in LibBB: multiplies the whole control term (before the offset).
      // 1 for IMU inputs, 1/mmPerTick (~3.0) for the encoder in UNIT_MILLIMETERS.
      this.gain = 1;
      this.iBounded = false; this.iMin = this.iMax = 0;
      this.dFilter = new LowPassFilter(5.0, 100); this.reset();
    }
    reset() { this.lastErr = this.errI = this.lastErrD = this.lastControl = 0; this.dFilter.reset(); }
    setIBounds(a, b) { this.iBounded = true; this.iMin = a; this.iMax = b; }
    // Returns the value the firmware passes to setControlOutput()
    update(present, dt) {
      const err = this.goal - present;
      this.errI += err * dt;
      if (this.iBounded) this.errI = Math.min(this.iMax, Math.max(this.iMin, this.errI));
      this.lastErrD = (err - this.lastErr) / dt;
      this.lastErr = err;
      const dF = this.dFilter.filter(this.lastErrD);
      this.lastControl = this.kp * err + this.ki * this.errI + this.kd * dF;
      this.lastControl *= this.gain;
      return this.lastControl + this.offset;
    }
  }

  // ---- Servo: Dynamixel-like profile generator (v/a limits) + PD position control
  class ServoProfile {
    constructor(p0 = 180) { this.pos = p0; this.vel = 0; }
    step(target, vmax, amax, dt) {
      // Dynamixel semantics: 0 = no limit
      if (!(vmax > 0)) vmax = Infinity;
      if (!(amax > 0)) amax = Infinity;
      if (vmax === Infinity && amax === Infinity) { this.vel = 0; this.pos = target; return this.pos; }
      if (amax === Infinity) { const e0 = target - this.pos, d = Math.sign(e0) * Math.min(Math.abs(e0), vmax * dt); this.vel = d / dt; this.pos += d; return this.pos; }
      const e = target - this.pos;
      if (e === 0 && this.vel === 0) return this.pos;
      let vdes = Math.sign(e) * Math.min(vmax, Math.sqrt(2 * amax * Math.abs(e)));
      const dv = Math.max(-amax * dt, Math.min(amax * dt, vdes - this.vel));
      this.vel += dv;
      let np = this.pos + this.vel * dt;
      if ((target - this.pos) * (target - np) < 0) { np = target; this.vel = 0; }
      this.pos = np;
      return this.pos;
    }
  }

  // ---- Defaults
  function defaultParams() {
    return {
      // Main parameters
      mp: 4.5,          // kg   pendulum mass (24 mono cells + motor + servo, Björn's estimate)
      l: 0.155,         // m    distance of the pendulum below the sphere center (Björn's estimate)
      mh: 1.0,          // kg   head mass
      // Further mechanics
      R: 0.253,         // m    sphere radius (BB8Config)
      ms: 3.0,          // kg   mass of the sphere shell (thin shell)
      L: 0.32,          // m    distance of the head CoM above the sphere center
      cRoll: 3.0,       // N/(m/s)   rolling resistance (viscous)
      bBear: 0.05,      // Nm/(rad/s) bearing friction of the drive
      // Drive motor (referred to the output, sphere relative to pendulum)
      tauStall: 30,     // Nm
      vNoLoad: 2600,    // mm/s no-load speed at PWM 255
      // Servos
      servoF: 4.0,      // Hz  bandwidth of the servo position control
      servoZeta: 0.8,
      servoStall: 4.1,  // Nm  servo stall torque (assumption XM430-W350 @12V)
      servoNoLoad: 46,  // rpm servo no-load speed
      // Firmware parameters (BB8Params)
      driveSpeedKp: 0.05, driveSpeedKi: 0.2, driveSpeedKd: 0.0, driveIBound: 255,
      balKp: 25, balKi: 0, balKd: 0.2,
      rollKp: 0.5, rollKi: 0.4, rollKd: 0.4, rollKff: 0.0, rollIMax: 50,
      rollDirect: false,
      driveSpeedMax: 3000, driveSpeedDeadband: 0.01, rollAngleMax: 20,
      bodyRollRange: 25, bodyRollVel: 500, bodyRollAccel: 60,
      domePitchRange: 45, domePitchVel: 100, domeRollRange: 20, domeRollVel: 100,
      domePitchReduction: 2.0, // assumption (only DOME_ROLL_REDUCTION is defined in the code)
      // Head compensation and feedforward: the "ideal" set, same defaults as the firmware (BB8Config.h).
      // faDomeRollToWeight / faDomePitchToWeight are the static ideals for the default masses
      // (see idealDomeRollToWeight() / idealDomePitchToWeight()); recompute them when the masses change.
      faSpeedSPToDome: 0.01, faRollSPToDome: -0.5, faDomeRollToWeight: -0.46, faDomePitchToWeight: -0.85, faPitchToDome: 0, domeCompRemoteOnly: true,
      driveAccelLimit: 2000, // mm/s^2 ramp on the speed setpoint from the stick (0 = off), firmware drive_accel_max
      faAccelToDome: 15,   // deg head (around the sphere center) per m/s^2 of commanded acceleration
      domeAccelTau: 0.5,   // s  low-pass on the commanded acceleration for the head
      rollReduction: 1.0,      // servo degrees per pendulum degree (firmware ROLL_REDUCTION, previously 80/22)
      rollGoalRamp: 0,     // deg/s ramp on the roll setpoint (0 = off), acts on the error AND the feedforward
      rollShaper: 0,       // 0 = off, 1 = ZV, 2 = ZVD (input shaper on the roll setpoint)
      rollShaperT: 2.0,    // s   oscillation period the shaper is tuned to
      rollKpReturn: 0,     // >0: different K_p while the setpoint is smaller in magnitude than the lean (way back to upright)
      rollKiReturn: 0,     // >0: different K_i on the way back
      rollReturnWindow: 1.5, // s  the way back only counts for this long after a stick move towards center
      rollIBleed: 0.4,       // s  >0: bleed the I state on the way back with this time constant
      rollKdSpeed: 1000,     // mm/s >0: above this measured speed, scale roll_kd by rollKdSpeed/|v| (cornering stiffens the roll plane, see README 3.7)
      rollRateCutoff: 15,
      imuFilterCutoff: 100,
      encoderQuantize: true,
      turning: true,       // cornering: yaw from axle lean + centrifugal force in the roll plane
    };
  }
  const ROLL_REDUCTION = 80 / 22, DOME_ROLL_REDUCTION = 2.0, TICKS_PER_TURN = 4776.384;
  const DXL_VEL = 0.229 * 6;               // Dynamixel unit -> deg/s
  const DXL_ACC = 214.577 * 360 / 3600;    // Dynamixel unit -> deg/s^2

  // ---- Planar dynamics: q=[s,a1,a2], qd; Q generalized forces
  function planarAcc(p, q, qd, Q) {
    const { mp, l, mh, L, ms, R } = p;
    const Is = 2 / 3 * ms * R * R, eps = 1e-4;
    const c1 = Math.cos(q[1]), s1 = Math.sin(q[1]), c2 = Math.cos(q[2]), s2 = Math.sin(q[2]);
    const M00 = ms + Is / (R * R) + mp + mh, M01 = mp * l * c1, M02 = mh * L * c2;
    const M11 = mp * l * l + eps, M22 = mh * L * L + eps;
    const r0 = Q[0] + mp * l * s1 * qd[1] * qd[1] + mh * L * s2 * qd[2] * qd[2];
    const r1 = Q[1] - mp * G * l * s1;
    const r2 = Q[2] + mh * G * L * s2;
    // M is "arrow" shaped -> solve directly
    const den = M00 - M01 * M01 / M11 - M02 * M02 / M22;
    const a0 = (r0 - M01 * r1 / M11 - M02 * r2 / M22) / den;
    return [a0, (r1 - M01 * a0) / M11, (r2 - M02 * a0) / M22];
  }

  class BB8Sim {
    constructor(params) { this.p = Object.assign(defaultParams(), params || {}); this.reset(); }

    reset() {
      const p = this.p;
      this.t = 0;
      // Pitch plane: s=x, a1=theta (pendulum forward +), a2=alpha (head forward +)
      this.pq = [0, 0, 0]; this.pqd = [0, 0, 0];
      // Roll plane: s=y, a1=chi (pendulum towards +y), a2=zeta (head towards +y)
      this.rq = [0, 0, 0]; this.rqd = [0, 0, 0];
      // World: position (X forward at start, Z right), heading psi (positive = to the right)
      this.wX = 0; this.wZ = 0; this.psi = 0; this.psiDot = 0; this.aLat = 0; this.trail = [[0, 0]]; this.trailAcc = 0;
      this.input = { vel: 0, roll: 0, headP: 0, headR: 0, driveOn: true };
      // Firmware state
      this.drive = new PID(); this.bal = new PID(); this.roll = new PID();
      this.imuPitchF = new LowPassFilter(p.imuFilterCutoff, 100);
      this.imuRollF = new LowPassFilter(p.imuFilterCutoff, 100);
      this.rateF = new LowPassFilter(p.rollRateCutoff, 104);
      this.encSpeedF = new LowPassFilter(25, 100);
      this.encTicks = 0; this.encSpeed = 0; this.encSpeedFilt = 0;
      this.pwm = 0; this.speedSP = 0; this.tilt = 0; this.rate = 0; this.headTilt = 0;
      this.rollServo = new ServoProfile(); this.domePServo = new ServoProfile(); this.domeRServo = new ServoProfile();
      this.rollServoGoal = 180; this.domePGoal = 180; this.domeRGoal = 180;
      this.applyParams();
      this.ctrlAcc = 0; this.lastPitchCmd = 0; this.velRamped = 0; this.accelCmdF = 0; this.retTimer = 0; this.prevRollGoal = 0; this.retGoalStart = 0; this.retIStart = 0; this.rollReturning = false; this.goalHist = []; this.goalRamped = 0; this.rollLoad = 0; this.rollLoadPeak = 0;
      this.log = [];
    }

    applyParams() {
      const p = this.p;
      this.drive.kp = p.driveSpeedKp; this.drive.ki = p.driveSpeedKi; this.drive.kd = p.driveSpeedKd;
      // Encoder in mm: controlGain = 1/mmPerTick as in bb::Encoder (the firmware uses BODY_CIRCUMFERENCE = 2*pi*253 mm)
      this.drive.gain = TICKS_PER_TURN / (2 * Math.PI * p.R * 1000);
      // Bound the I share (gain*ki*errI) to driveIBound PWM, as in the firmware
      const eiMax = p.driveSpeedKi > 0 ? p.driveIBound / (p.driveSpeedKi * this.drive.gain) : 0;
      this.drive.setIBounds(-eiMax, eiMax);
      this.bal.kp = p.balKp; this.bal.ki = p.balKi; this.bal.kd = p.balKd;
      this.roll.kp = p.rollKp; this.roll.ki = p.rollKi; this.roll.kd = 0;
      this.roll.setIBounds(-p.rollIMax, p.rollIMax);
      this.imuPitchF.setCutoff(p.imuFilterCutoff); this.imuRollF.setCutoff(p.imuFilterCutoff);
      this.rateF.setCutoff(p.rollRateCutoff);
    }

    // derived quantities
    pitchDeg() { return this.pq[1] * R2D; }                 // IMU pitch = frame/pendulum
    leanDeg() { return this.rq[0] / this.p.R * R2D; }  // IMU roll = axle lean
    relDrive() { return this.pq[1] + this.pq[0] / this.p.R; }      // sphere rel. pendulum [rad]
    domePitchRel() { return this.pq[2] + this.pq[1]; }      // head rel. frame (pitch) [rad]
    rollTiltRel() { return this.rq[1] + this.rq[0] / this.p.R; }   // pendulum rel. axle [rad]
    domeRollRel() { return this.rq[2] - this.rq[0] / this.p.R; }   // head rel. axle [rad]

    // ------------------------------------------------------------ Firmware 100 Hz
    controlStep(dt) {
      const p = this.p, inp = this.input;
      // IMU (Madgwick assumed ideal) and encoder
      const pitch = this.pitchDeg(), lean = this.leanDeg();
      const gyroRoll = this.rqd[0] / p.R * R2D;
      const mmPerTick = 2 * Math.PI * p.R * 1000 / TICKS_PER_TURN;
      const relMM = this.relDrive() * p.R * 1000;
      const ticks = p.encoderQuantize ? Math.floor(relMM / mmPerTick) : relMM / mmPerTick;
      this.encSpeed = (ticks - this.encTicks) / dt * mmPerTick; this.encTicks = ticks;
      this.encSpeedFilt = this.encSpeedF.filter(this.encSpeed);

      // --- stepDriveMotor (DRIVE_VEL)
      let vel = inp.vel;
      if (Math.abs(vel) <= p.driveSpeedDeadband) vel = 0; else vel *= p.driveSpeedMax;
      // Setpoint ramp and commanded acceleration (pure feedforward, no feedback)
      const velPrev = this.velRamped;
      if (p.driveAccelLimit > 0) { const m = p.driveAccelLimit * dt; this.velRamped += Math.max(-m, Math.min(m, vel - this.velRamped)); }
      else this.velRamped = vel;
      vel = this.velRamped;
      const aCmd = (this.velRamped - velPrev) / dt / 1000;
      this.accelCmdF += (aCmd - this.accelCmdF) * Math.min(1, dt / Math.max(dt, p.domeAccelTau));
      this.bal.offset = vel;
      // Dome pitch compensation: balance setpoint = expected pendulum position for the measured dome pitch
      // Compensation only on the remote share of the head (measured position minus automatic shares)
      const autoP = p.faSpeedSPToDome * this.bal.offset + p.faPitchToDome * p.domePitchReduction * this.lastPitchCmd + p.faAccelToDome * p.domePitchReduction * this.accelCmdF;
      const domePitchMeas = ((this.domePServo.pos - 180) - (p.domeCompRemoteOnly ? autoP : 0)) / p.domePitchReduction;
      this.bal.goal = p.faDomePitchToWeight * domePitchMeas;
      if (!inp.driveOn) {
        this.pwm = 0; this.bal.reset(); this.drive.reset(); this.speedSP = 0;
        this.imuPitchF.filter(pitch);
      } else {
        this.speedSP = this.bal.update(this.imuPitchF.filter(pitch), dt);
        this.drive.goal = this.speedSP;
        this.pwm = Math.max(-255, Math.min(255, this.drive.update(this.encSpeedFilt, dt)));
        if (Math.abs(this.pwm) < 1) this.pwm = 0;
      }

      // --- stepRollMotor
      const domeRollMeas = (this.domeRServo.pos - 180) / DOME_ROLL_REDUCTION; // measured dome roll position [deg]
      this.headTilt = p.faDomeRollToWeight * domeRollMeas;
      // Setpoint shaping (shaper is in the firmware, the ramp is a simulation-only experiment)
      let g = inp.roll * p.rollAngleMax;
      this.goalHist.push(g); if (this.goalHist.length > 600) this.goalHist.shift();
      const past = k => this.goalHist[Math.max(0, this.goalHist.length - 1 - k)];
      if (p.rollShaper === 1) { const k = Math.round(p.rollShaperT / 2 / dt); g = 0.5 * g + 0.5 * past(k); }
      else if (p.rollShaper === 2) { const k = Math.round(p.rollShaperT / 2 / dt); g = 0.25 * g + 0.5 * past(k) + 0.25 * past(2 * k); }
      if (p.rollGoalRamp > 0) { const m = p.rollGoalRamp * dt; this.goalRamped += Math.max(-m, Math.min(m, g - this.goalRamped)); g = this.goalRamped; }
      else this.goalRamped = g;
      this.roll.goal = g;
      if (p.rollDirect) {
        this.tilt = this.roll.goal + this.headTilt;
        this.rate = this.rateF.filter(gyroRoll);
      } else {
        this.rate = this.rateF.filter(gyroRoll);
        if (Math.abs(this.roll.goal) < Math.abs(this.prevRollGoal) - 1e-6) {
          if (this.retTimer <= 0) { this.retGoalStart = this.prevRollGoal; this.retIStart = this.roll.errI; }
          this.retTimer = p.rollReturnWindow;
        }
        this.prevRollGoal = this.roll.goal;
        if (this.retTimer > 0) this.retTimer -= dt;
        const returning = this.retTimer > 0 && Math.abs(this.roll.goal) < Math.abs(lean) - 0.5 && this.roll.goal * lean >= 0;
        this.roll.kp = returning && p.rollKpReturn > 0 ? p.rollKpReturn : p.rollKp;
        this.roll.ki = returning && p.rollKiReturn > 0 ? p.rollKiReturn : p.rollKi;
        this.rollReturning = returning;
        if (returning && p.rollIBleed > 0) {
          // Bleed the I state towards the share of the new setpoint (full return -> target 0)
          const target = Math.abs(this.retGoalStart) > 1e-6 ? this.retIStart * this.roll.goal / this.retGoalStart : 0;
          this.roll.errI = target + (this.roll.errI - target) * Math.exp(-dt / p.rollIBleed);
        }
        const v = this.roll.update(this.imuRollF.filter(lean), dt);
        // Speed-scheduled gyro damping, as in BB8RollController (measured encoder speed, not the setpoint)
        const vAbs = Math.abs(this.encSpeedFilt);
        this.rollKdEff = (p.rollKdSpeed > 0 && vAbs > p.rollKdSpeed) ? p.rollKd * p.rollKdSpeed / vAbs : p.rollKd;
        this.tilt = p.rollKff * this.roll.goal + this.headTilt + v - this.rollKdEff * this.rate;
      }
      this.rollServoGoal = 180 + p.rollReduction * this.tilt;

      // --- stepDome
      this.lastPitchCmd = pitch;
      this.domePGoal = 180 + inp.headP + p.faSpeedSPToDome * this.bal.offset + p.faPitchToDome * p.domePitchReduction * pitch
        + p.faAccelToDome * p.domePitchReduction * this.accelCmdF;
      this.domeRGoal = 180 + inp.headR + p.faRollSPToDome * this.roll.goal;
    }

    // ------------------------------------------------------------ Physics
    // PD position control of the servo, limited by the torque-speed curve.
    // red: reduction servo -> joint (servo degrees per joint degree)
    servoTorque(target, rel, relRate, inertia, red) {
      const p = this.p, w = 2 * Math.PI * p.servoF;
      let tau = inertia * (w * w * (target - rel) - 2 * p.servoZeta * w * relRate);
      const ts = p.servoStall * red, ws = relRate * red, w0 = p.servoNoLoad * 2 * Math.PI / 60;
      const hi = Math.min(ts, ts * (1 - ws / w0)), lo = Math.max(-ts, -ts * (1 + ws / w0));
      return Math.max(lo, Math.min(hi, tau));
    }

    forcesPitch(q, qd) {
      const p = this.p, R = p.R;
      const wRel = qd[1] + qd[0] / R;
      const w0 = p.vNoLoad / 1000 / R;
      const u = this.input.driveOn ? this.pwm / 255 : 0;
      const tauM = p.tauStall * (u - wRel / w0) - p.bBear * wRel;
      // Dome pitch servo: target angle (around the sphere center) from the servo position
      const target = (this.domePServo.pos - 180) / p.domePitchReduction * D2R;
      const tauH = this.servoTorque(target, q[2] + q[1], qd[2] + qd[1], p.mh * p.L * p.L + 1e-3, p.domePitchReduction);
      this.tauM = tauM;
      return [tauM / R - p.cRoll * qd[0], tauM + tauH, tauH];
    }

    forcesRoll(q, qd) {
      const p = this.p, R = p.R;
      const tTilt = (this.rollServo.pos - 180) / p.rollReduction * D2R;
      const tauR = this.servoTorque(tTilt, q[1] + q[0] / R, qd[1] + qd[0] / R, p.mp * p.l * p.l + 1e-3, p.rollReduction);
      const tDome = (this.domeRServo.pos - 180) / DOME_ROLL_REDUCTION * D2R;
      const tauD = this.servoTorque(tDome, q[2] - q[0] / R, qd[2] - qd[0] / R, p.mh * p.L * p.L + 1e-3, DOME_ROLL_REDUCTION);
      this.tauR = tauR;
      // Centrifugal force in the rotating frame (outwards = -aLat in y)
      const a = p.turning ? this.aLat : 0;
      const fS = -(p.ms + p.mp + p.mh) * a, f1 = -p.mp * a * p.l * Math.cos(q[1]), f2 = -p.mh * a * p.L * Math.cos(q[2]);
      return [tauR / R - tauD / R - p.cRoll * qd[0] + fS, tauR + f1, tauD + f2];
    }

    rk4(q, qd, forces, h) {
      const f = (q, qd) => planarAcc(this.p, q, qd, forces.call(this, q, qd));
      const add = (a, b, k) => a.map((v, i) => v + k * b[i]);
      const k1v = qd, k1a = f(q, qd);
      const q2 = add(q, k1v, h / 2), v2 = add(qd, k1a, h / 2), k2a = f(q2, v2);
      const q3 = add(q, v2, h / 2), v3 = add(qd, k2a, h / 2), k3a = f(q3, v3);
      const q4 = add(q, v3, h), v4 = add(qd, k3a, h), k4a = f(q4, v4);
      for (let i = 0; i < 3; i++) {
        q[i] += h / 6 * (k1v[i] + 2 * v2[i] + 2 * v3[i] + v4[i]);
        qd[i] += h / 6 * (k1a[i] + 2 * k2a[i] + 2 * k3a[i] + k4a[i]);
      }
    }

    // Advance simulation time by dt (fixed step 1 ms, controllers 10 ms)
    advance(dt) {
      const h = 0.001, ctrlDt = 0.01, p = this.p;
      let n = Math.round(dt / h);
      while (n-- > 0) {
        this.ctrlAcc += h;
        if (this.ctrlAcc >= ctrlDt - 1e-9) { this.ctrlAcc -= ctrlDt; this.controlStep(ctrlDt); this.logSample(); }
        // Servo profiles (position goals in servo degrees, limits as in BB8::setServoParameters)
        const rr = p.bodyRollRange * p.rollReduction;
        this.rollServo.step(Math.max(180 - rr, Math.min(180 + rr, this.rollServoGoal)), p.bodyRollVel * DXL_VEL, p.bodyRollAccel * DXL_ACC, h);
        this.domePServo.step(Math.max(180 - p.domePitchRange, Math.min(180 + p.domePitchRange, this.domePGoal)), p.domePitchVel * DXL_VEL, 3000, h);
        this.domeRServo.step(Math.max(180 - p.domeRollRange, Math.min(180 + p.domeRollRange, this.domeRGoal)), p.domeRollVel * DXL_VEL, 3000, h);
        // Cornering: yaw rate from axle lean (sphere without spin around the vertical at the contact point)
        const v = this.pqd[0], rho = this.rq[0] / p.R;
        this.psiDot = p.turning ? v / p.R * Math.sin(rho) : 0;
        this.aLat = v * this.psiDot;
        this.rk4(this.pq, this.pqd, this.forcesPitch, h);
        this.rk4(this.rq, this.rqd, this.forcesRoll, h);
        this.rollLoad = Math.abs(this.tauR || 0) / (p.servoStall * p.rollReduction);
        this.rollLoadPeak = Math.max(this.rollLoadPeak, this.rollLoad);
        this.psi += this.psiDot * h;
        const vy = this.rqd[0], c = Math.cos(this.psi), sn = Math.sin(this.psi);
        this.wX += (v * c - vy * sn) * h; this.wZ += (v * sn + vy * c) * h;
        this.trailAcc += h;
        if (this.trailAcc >= 0.05) { this.trailAcc = 0; this.trail.push([this.wX, this.wZ]); if (this.trail.length > 1200) this.trail.shift(); }
        this.t += h;
      }
    }

    logSample() {
      this.log.push({
        t: this.t, vSP: this.bal.offset, vCmd: this.speedSP, vGround: this.pqd[0] * 1000, vEnc: this.encSpeedFilt,
        pitch: this.pitchDeg(), pwm: this.pwm, rollGoal: this.roll.goal, lean: this.leanDeg(), tilt: this.tilt,
        headP: this.pq[2] * R2D, headR: this.rq[2] * R2D,
      });
      if (this.log.length > 2000) this.log.splice(0, this.log.length - 2000);
    }

    // Static equilibrium helpers.
    // Expected pendulum deflection [deg] per m/s^2 at quasi-stationary acceleration (no friction/head)
    pendulumPerAccel() { const p = this.p, R = p.R; const M = p.ms + 2 / 3 * p.ms + p.mp + p.mh; return (R * M - p.mp * p.l) / (p.mp * G * p.l) * R2D; }
    // Roll: ideal fa_dome_roll_to_weight (head on the axle, pendulum relative to the axle)
    idealDomeRollToWeight() { const p = this.p; return -(p.mh * p.L) / (p.mp * p.l); }
    // Pitch: the head sits on the pendulum frame and swings with it -> denominator mp*l - mh*L
    idealDomePitchToWeight() { const p = this.p; return -(p.mh * p.L) / (p.mp * p.l - p.mh * p.L); }
  }

  const api = { BB8Sim, defaultParams, LowPassFilter, PID, D2R, R2D, ROLL_REDUCTION };
  if (typeof module !== 'undefined' && module.exports) module.exports = api; else global.BB8 = api;
})(typeof window !== 'undefined' ? window : globalThis);
