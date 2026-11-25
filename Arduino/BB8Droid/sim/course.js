// ============================================================================
// Obstacle course for the BB-8 simulations (no three.js dependency; used by sim.js, cary.js and the pages)
//  - Layout in world coordinates, start at the origin facing +X:
//      slalom: 5 cones per lane, 1.5 m apart, from X = 3 m
//      ramps:  up 2 m, plateau 1 m, down 2 m at 5 / 10 / 12.5 / 15 % slope, 1.5 m flat in between
//      seesaw: 3 m plank, 10 % when resting on one end, tips under the droid's weight
//  - heightAt / gradientAt give the surface; the physics uses the gradient at the droid's contact point.
//  - The seesaw is a plank on a pivot with end stops; the droids' weight tips it (one-way coupling: the plank
//    does not push the droid, the droid only feels the changing slope).
// ============================================================================
(function (global) {
  'use strict';
  const G = 9.81;

  function makeCourse(opts) {
    const lanes = (opts && opts.lanes) || [0];
    const W = 3.2, bevel = 0.4, soft = 0.15;          // width, side bevel, edge rounding (m)
    const cones = [];
    for (const z of lanes) for (let i = 0; i < 5; i++) cones.push({x: 3 + 1.5 * i, z, r: 0.12, h: 0.45, hit: false});
    const ramps = [];
    let x = 12;
    for (const slope of [5, 10, 12.5, 15]) {
      const L = 2, P = 1, rise = L * slope / 100;
      ramps.push({x0: x, L, P, rise, slope, W});
      x += 2 * L + P + 1.5;
    }
    const aMax = Math.atan(0.10), plankL = 3;
    const seesaw = {xp: x + plankL / 2, L: plankL, W, slope: 10, aMax, hp: plankL / 2 * Math.tan(aMax),
                    a: aMax, w: 0, mPlank: 6, damping: 25};   // a > 0: the +X end is up (approached from -X)
    const length = seesaw.xp + plankL / 2 + 2;

    // ramp profile along X (piecewise linear), rounded at the kinks by averaging over +-soft
    const rampRaw = (r, u) => u <= 0 ? 0 : u < r.L ? r.rise * u / r.L : u < r.L + r.P ? r.rise
                            : u < 2 * r.L + r.P ? r.rise * (1 - (u - r.L - r.P) / r.L) : 0;
    const rampH = (r, u) => { let s = 0; for (let k = -2; k <= 2; k++) s += rampRaw(r, u + k * soft / 2); return s / 5; };
    // width profile: 1 inside, linear bevel to 0 at the sides
    const across = z => { const d = Math.abs(z) - W / 2; return d <= 0 ? 1 : d >= bevel ? 0 : 1 - d / bevel; };
    const onPlank = (X, Z) => Math.abs(X - seesaw.xp) <= plankL / 2 && Math.abs(Z) <= W / 2;
    const plankH = X => seesaw.hp + (X - seesaw.xp) * Math.tan(seesaw.a);

    function heightAt(X, Z) {
      let h = 0;
      const w = across(Z);
      if (w > 0) for (const r of ramps) { const u = X - r.x0; if (u > -soft && u < 2 * r.L + r.P + soft) h = Math.max(h, w * rampH(r, u)); }
      if (onPlank(X, Z)) h = Math.max(h, plankH(X));
      return h;
    }
    // surface gradient (rise per run in X and Z), limited to +-50 % so falling off an edge stays bounded
    function gradientAt(X, Z) {
      const e = 0.02, lim = 0.5;
      const gx = (heightAt(X + e, Z) - heightAt(X - e, Z)) / (2 * e), gz = (heightAt(X, Z + e) - heightAt(X, Z - e)) / (2 * e);
      return {gx: Math.max(-lim, Math.min(lim, gx)), gz: Math.max(-lim, Math.min(lim, gz))};
    }
    // seesaw dynamics: bodies = [{x, z, m}] (droid contact points and masses)
    function update(dt, bodies) {
      const s = seesaw; let tau = 0, I = s.mPlank * s.L * s.L / 12;
      for (const b of bodies) if (onPlank(b.x, b.z)) { const d = b.x - s.xp; tau += -b.m * G * d * Math.cos(s.a); I += b.m * d * d; }
      const n = Math.max(1, Math.ceil(dt / 0.002)), h = dt / n;
      for (let i = 0; i < n; i++) {
        s.w += (tau - s.damping * s.w) / I * h; s.a += s.w * h;
        if (s.a > s.aMax) { s.a = s.aMax; if (s.w > 0) s.w = 0; }
        if (s.a < -s.aMax) { s.a = -s.aMax; if (s.w < 0) s.w = 0; }
      }
    }
    // cone hits: a cone counts as hit when the sphere (radius R) touches its base
    function checkCones(X, Z, R) { let n = 0; for (const c of cones) if (!c.hit && Math.hypot(X - c.x, Z - c.z) < R + c.r) { c.hit = true; n++; } return n; }
    function reset() { for (const c of cones) c.hit = false; seesaw.a = seesaw.aMax; seesaw.w = 0; }
    const conesHit = () => cones.filter(c => c.hit).length;

    return {cones, ramps, seesaw, length, W, bevel, heightAt, gradientAt, update, checkCones, reset, conesHit};
  }

  // ---- 3D (three.js passed in, so this file stays usable without it)
  // Meshes for ramps, seesaw and cones in a group (add it to the scene/terrain group); call update() every frame.
  function buildMeshes(THREE, course) {
    const group = new THREE.Group();
    const rampMat = new THREE.MeshStandardMaterial({color: 0x9aa3ad, roughness: 0.9, flatShading: true});
    for (const r of course.ramps) {
      const x0 = r.x0 - 0.3, len = 2 * r.L + r.P + 0.6, wid = course.W + 2 * course.bevel;
      const geo = new THREE.PlaneGeometry(len, wid, Math.ceil(len / 0.05), 8); geo.rotateX(-Math.PI / 2);
      const pos = geo.attributes.position;
      for (let i = 0; i < pos.count; i++) { const x = pos.getX(i) + x0 + len / 2, z = pos.getZ(i); pos.setY(i, course.heightAt(x, z) + 0.003); }
      geo.computeVertexNormals();
      const m = new THREE.Mesh(geo, rampMat); m.position.set(x0 + len / 2, 0, 0); group.add(m);
      group.add(label(THREE, `${r.slope} %`, r.x0 + r.L + r.P / 2, r.rise + 0.45, -course.W / 2 - 0.2));
    }
    const s = course.seesaw;
    const plank = new THREE.Mesh(new THREE.BoxGeometry(s.L, 0.04, s.W), new THREE.MeshStandardMaterial({color: 0xb08a5a, roughness: 0.8}));
    const plankPivot = new THREE.Group(); plankPivot.position.set(s.xp, s.hp, 0); plank.position.set(0, -0.02, 0); plankPivot.add(plank); group.add(plankPivot);
    const base = new THREE.Mesh(new THREE.BoxGeometry(0.16, s.hp - 0.02, s.W), new THREE.MeshStandardMaterial({color: 0x55606b})); base.position.set(s.xp, (s.hp - 0.02) / 2, 0); group.add(base);
    group.add(label(THREE, `seesaw ${s.slope} %`, s.xp, s.hp + 0.75, -s.W / 2 - 0.2));
    const coneMat = new THREE.MeshStandardMaterial({color: 0xe8742a, roughness: 0.6}), hitMat = new THREE.MeshStandardMaterial({color: 0xd62828, roughness: 0.6});
    const coneMeshes = course.cones.map(c => { const m = new THREE.Mesh(new THREE.ConeGeometry(c.r, c.h, 24), coneMat); m.position.set(c.x, c.h / 2, c.z); group.add(m); return m; });
    group.add(label(THREE, 'slalom', course.cones[0].x + 3, 0.95, -1.4));
    function update() {
      plankPivot.rotation.set(0, 0, s.a);
      course.cones.forEach((c, i) => { coneMeshes[i].material = c.hit ? hitMat : coneMat; coneMeshes[i].rotation.z = c.hit ? 0.5 : 0; });
    }
    update();
    return {group, update};
  }
  function label(THREE, text, x, y, z) {
    const c = document.createElement('canvas'); c.width = 256; c.height = 64; const g = c.getContext('2d');
    g.fillStyle = 'rgba(27,31,36,0.75)'; g.fillRect(0, 0, 256, 64); g.fillStyle = '#fff'; g.font = '600 34px Barlow, system-ui, sans-serif';
    g.textAlign = 'center'; g.textBaseline = 'middle'; g.fillText(text, 128, 33);
    const sp = new THREE.Sprite(new THREE.SpriteMaterial({map: new THREE.CanvasTexture(c), depthTest: false}));
    sp.scale.set(0.8, 0.2, 1); sp.position.set(x, y, z); return sp;
  }
  // Pose of a droid standing on the surface at (X, Z): sphere centre and the tilt quaternion (surface normal),
  // to be combined with the yaw. Without a course: flat ground.
  function droidPose(THREE, course, X, Z, R) {
    const h = course ? course.heightAt(X, Z) : 0, g = course ? course.gradientAt(X, Z) : {gx: 0, gz: 0};
    const n = new THREE.Vector3(-g.gx, 1, -g.gz).normalize();
    const center = new THREE.Vector3(X, h, Z).addScaledVector(n, R);
    const tilt = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), n);
    return {h, n, center, tilt};
  }

  const api = {makeCourse, buildMeshes, droidPose};
  if (typeof module !== 'undefined' && module.exports) module.exports = api; else global.BB8COURSE = api;
})(typeof window !== 'undefined' ? window : globalThis);
