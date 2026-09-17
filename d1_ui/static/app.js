/* D1 reach console: draws the Go2 + D1 from their URDFs, animates the joints from
 * the server's live state, and turns a click on the hologram sphere into a
 * Cartesian target -- previewed first, sent only on SEND.
 *
 * Frames. The whole scene is the Go2 base frame, Z up, exactly as the URDFs and
 * the solver use it, so a point picked here is sent to the server unchanged.
 * three.js is Y-up by default; DefaultUp is flipped before anything is built.
 *
 * Scene graph. Each URDF joint becomes two nodes: a fixed one carrying the
 * joint's origin (xyz, rpy) and a rotating one carrying the joint angle about
 * the axis. That is the same composition `position_only.workspace.forward`
 * uses, and tests/test_d1_ui.py checks the two agree, since a browser cannot
 * run in the test suite. The small red sphere is the server's FK of the tool
 * point: if it sits on the pincer tip, the page and the solver agree live.
 *
 * Sim or hardware. The server decides at start-up and says which in every state
 * message. In sim mode the joints, fingers and legs come from the simulator, and
 * the arm controls are disabled here as well as refused by the server.
 *
 * Camera window. `/camera.mjpg` is the wrist camera with YOLO's boxes already
 * drawn on by the server, so a box is always on the frame it was detected in.
 * Whether to show it, and what was detected, comes with the state.
 */
(() => {
  'use strict';
  THREE.Object3D.DefaultUp.set(0, 0, 1);

  const $ = (id) => document.getElementById(id);
  const fmt = (v, d = 3) => (v === null || v === undefined || Number.isNaN(v)) ? '–' : Number(v).toFixed(d);
  const esc = (t) => String(t).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
  // Per-viewer layout only (where the camera window sits); the page works without it.
  const store = {
    get(k) { try { return JSON.parse(localStorage.getItem(k)); } catch (e) { return null; } },
    set(k, v) { try { localStorage.setItem(k, JSON.stringify(v)); } catch (e) { /* private window */ } },
  };

  // ------------------------------------------------------------ renderer
  const view = $('view');
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(window.devicePixelRatio);
  view.appendChild(renderer.domElement);
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0b0f14);
  const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 50);
  camera.up.set(0, 0, 1);
  camera.position.set(1.1, -1.0, 0.8);
  const controls = new THREE.OrbitControls(camera, renderer.domElement);
  controls.target.set(0.1, 0, 0.25);
  controls.enableDamping = true;

  scene.add(new THREE.HemisphereLight(0xdfe8f5, 0x202830, 0.9));
  const key = new THREE.DirectionalLight(0xffffff, 0.8); key.position.set(1, -1, 2); scene.add(key);
  const grid = new THREE.GridHelper(2, 20, 0x2a3644, 0x1b2430);
  grid.rotation.x = Math.PI / 2; grid.position.z = -0.20;   // nominal ground; the real height depends on posture
  scene.add(grid);
  scene.add(new THREE.AxesHelper(0.15));

  function resize() {
    const w = view.clientWidth, h = view.clientHeight;
    renderer.setSize(w, h); camera.aspect = w / h; camera.updateProjectionMatrix();
  }
  window.addEventListener('resize', resize); resize();

  // ------------------------------------------------------------ loaders
  function parseBinarySTL(buf) {
    const dv = new DataView(buf);
    const n = dv.getUint32(80, true);
    const pos = new Float32Array(n * 9), nor = new Float32Array(n * 9);
    let o = 84;
    for (let i = 0; i < n; i++) {
      const nx = dv.getFloat32(o, true), ny = dv.getFloat32(o + 4, true), nz = dv.getFloat32(o + 8, true);
      for (let v = 0; v < 3; v++) {
        const b = o + 12 + v * 12, k = i * 9 + v * 3;
        pos[k] = dv.getFloat32(b, true); pos[k + 1] = dv.getFloat32(b + 4, true); pos[k + 2] = dv.getFloat32(b + 8, true);
        nor[k] = nx; nor[k + 1] = ny; nor[k + 2] = nz;
      }
      o += 50;
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    g.setAttribute('normal', new THREE.BufferAttribute(nor, 3));
    return g;
  }
  const collada = new THREE.ColladaLoader();
  async function loadMesh(url, material) {
    if (url.toLowerCase().endsWith('.stl')) {
      const buf = await (await fetch(url)).arrayBuffer();
      return new THREE.Mesh(parseBinarySTL(buf), material);
    }
    return new Promise((resolve, reject) => collada.load(url, (c) => {
      // The files are metres and Z_UP, the same frame as the scene; undo any
      // up-axis correction the loader applied and recolour to the scene material.
      c.scene.rotation.set(0, 0, 0); c.scene.scale.set(1, 1, 1);
      c.scene.traverse((o) => { if (o.isMesh) { o.material = material; } });
      resolve(c.scene);
    }, undefined, reject));
  }

  // ------------------------------------------------------------ URDF -> scene graph
  const eulerZYX = (rpy) => new THREE.Euler(rpy[0], rpy[1], rpy[2], 'ZYX');   // Rz(y)Ry(p)Rx(r), as URDF
  const revolute = {};   // joint name -> {node, axis}
  const prismatic = {};  // joint name -> {node, axis}

  async function buildRobot(spec, parentNode, material, rootOffset) {
    const linkNodes = {};
    const root = new THREE.Object3D();
    if (rootOffset) root.position.fromArray(rootOffset);
    parentNode.add(root);
    linkNodes[spec.root] = root;

    // Attach visuals to every link node once it exists.
    const attach = async (linkName, node) => {
      for (const v of (spec.links[linkName] || [])) {
        try {
          const mesh = await loadMesh(v.mesh, material);
          mesh.position.fromArray(v.xyz); mesh.rotation.copy(eulerZYX(v.rpy)); mesh.scale.fromArray(v.scale);
          node.add(mesh);
        } catch (e) { console.warn('mesh failed', v.mesh, e); }
      }
    };
    await attach(spec.root, root);

    // Joints in file order are parent-before-child for both URDFs; loop until all placed.
    const pending = spec.joints.slice();
    let guard = 0;
    while (pending.length && guard++ < 50) {
      for (let i = pending.length - 1; i >= 0; i--) {
        const j = pending[i];
        const parent = linkNodes[j.parent];
        if (!parent) continue;
        const fixed = new THREE.Object3D();
        fixed.position.fromArray(j.xyz); fixed.rotation.copy(eulerZYX(j.rpy));
        parent.add(fixed);
        const moving = new THREE.Object3D();
        fixed.add(moving);
        const axis = new THREE.Vector3().fromArray(j.axis).normalize();
        if (j.type === 'revolute' || j.type === 'continuous') revolute[j.name] = { node: moving, axis };
        if (j.type === 'prismatic') prismatic[j.name] = { node: moving, axis };
        linkNodes[j.child] = moving;
        pending.splice(i, 1);
        await attach(j.child, moving);
      }
    }
    return linkNodes;
  }

  const setRevolute = (name, q) => { const j = revolute[name]; if (j) j.node.setRotationFromAxisAngle(j.axis, q); };
  const setPrismatic = (name, d) => { const j = prismatic[name]; if (j) j.node.position.copy(j.axis).multiplyScalar(d); };

  // ------------------------------------------------------------ hologram sphere + markers
  let sphere = null, model = null;
  const toolMarker = new THREE.Mesh(new THREE.SphereGeometry(0.008, 16, 12), new THREE.MeshBasicMaterial({ color: 0xff4d4d }));
  const candMarker = new THREE.Mesh(new THREE.SphereGeometry(0.012, 16, 12), new THREE.MeshBasicMaterial({ color: 0x35d3ff }));
  candMarker.visible = false;
  const candLine = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0x35d3ff, transparent: true, opacity: 0.6 }));
  candLine.visible = false;
  scene.add(toolMarker, candMarker, candLine);

  function buildSphere(sph) {
    // Polar angle measured from the sphere's pole; rotate so the pole is +Z, and
    // stop the surface where targets would be refused anyway (below min_z).
    const r = sph.radius;
    const cosMax = Math.max(-1, Math.min(1, (sph.min_z - sph.center[2]) / r));
    const thetaLength = Math.acos(cosMax);
    const geo = new THREE.SphereGeometry(r, 64, 40, 0, Math.PI * 2, 0, thetaLength);
    const solid = new THREE.Mesh(geo, new THREE.MeshPhongMaterial({
      color: 0x35d3ff, transparent: true, opacity: 0.10, side: THREE.DoubleSide, depthWrite: false, shininess: 80,
    }));
    const wire = new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ color: 0x35d3ff, wireframe: true, transparent: true, opacity: 0.18 }));
    const g = new THREE.Group();
    g.add(solid, wire);
    g.position.fromArray(sph.center);
    g.rotation.x = Math.PI / 2;   // +Y pole -> +Z
    scene.add(g);
    return solid;
  }

  // ------------------------------------------------------------ picking (click, not drag)
  const ray = new THREE.Raycaster();
  let down = null, candidate = null;
  renderer.domElement.addEventListener('pointerdown', (e) => { down = { x: e.clientX, y: e.clientY }; });
  renderer.domElement.addEventListener('pointerup', (e) => {
    if (!down || !sphere) return;
    const moved = Math.hypot(e.clientX - down.x, e.clientY - down.y);
    down = null;
    if (moved > 4) return;   // an orbit, not a pick
    const rect = renderer.domElement.getBoundingClientRect();
    const ndc = new THREE.Vector2(((e.clientX - rect.left) / rect.width) * 2 - 1, -((e.clientY - rect.top) / rect.height) * 2 + 1);
    ray.setFromCamera(ndc, camera);
    const hit = ray.intersectObject(sphere, false)[0];
    if (!hit) return;
    setCandidate(hit.point);
  });

  async function setCandidate(p) {
    candidate = [p.x, p.y, p.z];
    candMarker.position.copy(p); candMarker.visible = true;
    candLine.geometry.setFromPoints([toolMarker.position.clone(), p]); candLine.visible = true;
    $('cand').textContent = `[${fmt(p.x)}, ${fmt(p.y)}, ${fmt(p.z)}] m`;
    $('send').disabled = true;
    const pv = $('preview'); pv.className = 'preview'; pv.textContent = 'solving…';
    const r = await post('/target', { target: candidate, preview: true });
    if (r.ok) {
      pv.className = 'preview ok';
      const att = `pitch ${fmt(r.elevation_deg, 1)}° roll ${fmt(r.roll_deg, 1)}°` +
                  (r.level_requested ? (r.level_ok ? ' — level' : ' — NOT level') : '');
      pv.textContent = `IK ok: ${r.position_error_mm} mm in ${r.iterations} it\njoints ${r.servo_deg.map((v) => fmt(v, 1)).join(', ')}\nlargest move ${fmt(Math.max(...r.delta_deg.map(Math.abs)), 1)}°\n${att}` +
                       (mode === 'sim' ? '\n(sim mode: preview only)' : '');
      $('send').disabled = mode === 'sim';
    } else {
      pv.className = 'preview bad';
      pv.textContent = `refused: ${r.reason}`;
    }
  }

  // ------------------------------------------------------------ server I/O
  async function post(path, body) {
    try {
      const res = await fetch(path, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body || {}) });
      return await res.json();
    } catch (e) { return { ok: false, reason: String(e) }; }
  }
  let toastTimer = null;
  function toast(msg) {
    let t = $('toast');
    if (!t) { t = document.createElement('div'); t.id = 'toast'; document.body.appendChild(t); }
    t.textContent = msg; t.classList.add('show');
    clearTimeout(toastTimer); toastTimer = setTimeout(() => t.classList.remove('show'), 2500);
  }

  $('send').onclick = async () => {
    if (!candidate) return;
    const r = await post('/target', { target: candidate });
    toast(r.ok ? 'sent' : `refused: ${r.reason}`);
  };
  $('stop').onclick = async () => { const r = await post('/stop'); toast(r.ok ? 'stopping — arm holds' : r.reason); };
  $('park').onclick = async () => { const r = await post('/park'); toast(r.ok ? 'parking' : `refused: ${r.reason}`); };
  $('release').onclick = async () => {
    let r = await post('/release', {});
    if (!r.ok && /not folded/.test(r.reason || '')) {
      if (!confirm('The arm is NOT folded. Releasing removes torque and it WILL FALL to its stop.\n\nRelease anyway?')) return;
      r = await post('/release', { force: true });
    }
    toast(r.ok ? 'released' : `refused: ${r.reason}`);
  };
  $('live').onchange = async (e) => {
    if (e.target.checked && !confirm('Arm the console? Every SEND, PARK and RELEASE will move the real arm.')) {
      e.target.checked = false; return;
    }
    await post('/arm', { live: e.target.checked });
  };

  // ------------------------------------------------------------ sim or hardware
  let mode = null;
  function applyMode(s) {
    if (s.mode === mode) return;
    mode = s.mode;
    const sim = mode === 'sim';
    document.body.classList.toggle('sim', sim);
    $('mode').textContent = sim ? 'SIM' : 'HARDWARE';
    $('mode').className = 'pill ' + (sim ? 'sim' : 'hw');
    $('mode').title = s.mode_reason || '';
    $('live').disabled = sim;
    for (const id of ['stop', 'park', 'release']) $(id).disabled = sim;
    if (sim) $('send').disabled = true;
  }

  // ------------------------------------------------------------ camera window
  const cam = { win: $('camwin'), img: $('camimg'), msg: $('cammsg'), streaming: false };
  (function restoreCamWin() {
    const saved = store.get('d1ui.camwin');
    if (!saved) return;
    if (Number.isFinite(saved.left) && Number.isFinite(saved.top)) {
      cam.win.style.left = Math.min(Math.max(0, saved.left), window.innerWidth - 80) + 'px';
      cam.win.style.top = Math.min(Math.max(0, saved.top), window.innerHeight - 30) + 'px';
    }
    if (Number.isFinite(saved.width)) cam.win.style.width = saved.width + 'px';
    cam.win.classList.toggle('collapsed', !!saved.collapsed);
    $('camtoggle').textContent = saved.collapsed ? '+' : '–';
  })();
  function saveCamWin() {
    const r = cam.win.getBoundingClientRect();
    store.set('d1ui.camwin', { left: r.left, top: r.top, width: r.width, collapsed: cam.win.classList.contains('collapsed') });
  }
  (function dragByBar() {
    const bar = $('cambar');
    let start = null;
    bar.addEventListener('pointerdown', (e) => {
      if (e.target.closest('button')) return;
      const r = cam.win.getBoundingClientRect();
      start = { x: e.clientX, y: e.clientY, left: r.left, top: r.top };
      bar.setPointerCapture(e.pointerId);
    });
    bar.addEventListener('pointermove', (e) => {
      if (!start) return;
      cam.win.style.left = Math.max(0, Math.min(window.innerWidth - 80, start.left + e.clientX - start.x)) + 'px';
      cam.win.style.top = Math.max(0, Math.min(window.innerHeight - 30, start.top + e.clientY - start.y)) + 'px';
    });
    bar.addEventListener('pointerup', () => { if (start) { start = null; saveCamWin(); } });
  })();
  let resizeTimer = null;
  new ResizeObserver(() => { clearTimeout(resizeTimer); resizeTimer = setTimeout(saveCamWin, 300); }).observe(cam.win);
  $('camtoggle').onclick = () => {
    const collapsed = cam.win.classList.toggle('collapsed');
    $('camtoggle').textContent = collapsed ? '+' : '–';
    saveCamWin();
  };

  function applyCamera(c, sim) {
    if (!c) return;
    const fresh = !!c.available && c.frame_age_s !== null && c.frame_age_s !== undefined && c.frame_age_s < 2.5;
    if (fresh && !cam.streaming) { cam.streaming = true; cam.img.src = '/camera.mjpg?t=' + Date.now(); }
    cam.img.classList.toggle('on', fresh);
    cam.msg.style.display = fresh ? 'none' : 'flex';
    if (!fresh) cam.msg.textContent = c.message || (sim ? 'waiting for the simulator\'s camera' : 'no camera');
    $('camsrc').textContent = c.source || '–';
    $('camsrc').title = c.source || '';
    $('camfps').textContent = fresh && c.fps ? `${fmt(c.fps, 0)} fps` : '';
    const foot = $('camdet');
    foot.title = c.detector ? `detector: ${c.detector}` : '';
    if (!c.detector_ready) {
      foot.textContent = c.detector ? `YOLO ${c.detector}` : 'no detector';
    } else {
      const dets = c.detections || [];
      foot.innerHTML = (dets.length
        ? dets.map((d) => `<span class="hit">${esc(d.label)} ${fmt(d.confidence, 2)}</span>`).join(' · ')
        : `YOLO: no ${esc((c.targets || ['objects']).join('/'))} detected`) +
        (c.detect_ms !== null && c.detect_ms !== undefined ? ` · YOLO ${fmt(c.detect_ms, 0)} ms` : '');
    }
  }

  // ------------------------------------------------------------ live state
  function applyState(s) {
    applyMode(s);
    const sim = s.mode === 'sim';
    $('conn').textContent = s.connected ? (sim ? 'sim feed' : 'live feedback')
      : (sim ? (s.servo_deg ? 'sim not updating' : 'waiting for sim') : 'no feedback');
    $('conn').className = 'pill ' + (s.connected ? 'ok' : 'bad');
    $('power').textContent = s.power === null ? '–' : (s.power ? 'on' : 'off');
    $('enable').textContent = s.enable === null ? '–' : (s.enable ? 'holding' : 'released');
    $('error').textContent = s.error === null ? '–' : String(s.error);
    $('age').textContent = s.feedback_age_s === null ? '–' : fmt(s.feedback_age_s, 2) + ' s';
    $('legs').textContent = s.legs_live ? (sim ? 'from the sim' : 'live')
                                        : (sim ? 'nominal (the sim sends none)' : 'nominal (no rt/lowstate)');
    if (sim) {
      const m = s.sim || {};
      $('simstate').textContent = m.source ? `${m.source} · ${fmt(m.sim_time_s, 1)} s${m.status ? ' · ' + m.status : ''}` : '–';
    }
    $('busy').textContent = s.busy ? `${s.busy}${s.progress ? ` · step ${s.progress.step} · ${s.progress.error_mm} mm` : ''}` : 'idle';
    $('grip').textContent = s.finger_m ? `fingers ${fmt(1000 * s.finger_m[0], 1)} / ${fmt(1000 * s.finger_m[1], 1)} mm`
                                       : (s.gripper_units === null ? '–' : fmt(s.gripper_units, 1) + ' units');
    $('tool').textContent = s.tool_m ? `[${s.tool_m.map((v) => fmt(v)).join(', ')}]` : '–';
    $('live').checked = !!s.live;
    $('armbox').classList.toggle('live', !!s.live);
    $('livehint').textContent = sim ? 'SIM: the page follows the simulator; nothing is sent to any arm'
      : (s.live ? 'LIVE: commands go to the arm' : 'dry run: clicks and buttons rehearse only');
    if (s.last_result) $('result').textContent = JSON.stringify(s.last_result);
    if (s.log) $('log').textContent = s.log.join('\n');
    applyCamera(s.camera, sim);

    const tbl = $('joints');
    tbl.innerHTML = [0, 1, 2, 3, 4, 5].map((i) =>
      `<tr><td>J${i}</td><td>${fmt(s.servo_deg && s.servo_deg[i], 1)}</td><td>${fmt(s.q_rad && s.q_rad[i] * 180 / Math.PI, 1)}</td></tr>`).join('');

    if (model) {
      if (s.q_rad) model.d1.arm_joints.forEach((name, i) => setRevolute(name, s.q_rad[i]));
      if (s.finger_m) {
        // The simulator's finger joints, in metres along their URDF axes.
        setPrismatic('Joint7_1', s.finger_m[0]); setPrismatic('Joint7_2', s.finger_m[1]);
      } else if (s.gripper_units !== null) {
        // Servo 6 is 0..~65 units, closed..open; the stroke-to-units mapping is unverified,
        // so the fingers are a proportional guess for the picture only.
        const d = Math.max(0, Math.min(1, s.gripper_units / 65)) * model.d1.gripper_stroke_m;
        setPrismatic('Joint7_1', d); setPrismatic('Joint7_2', -d);
      }
      if (s.go2_q_rad) model.go2.motor_order.forEach((name, i) => setRevolute(name, s.go2_q_rad[i]));
      if (s.base_height_m !== null && s.base_height_m !== undefined) grid.position.z = -s.base_height_m;
      if (s.tool_m) toolMarker.position.fromArray(s.tool_m);
      if (candMarker.visible) candLine.geometry.setFromPoints([toolMarker.position.clone(), candMarker.position.clone()]);
    }
  }

  function connectEvents() {
    const es = new EventSource('/events');
    es.onmessage = (ev) => { try { applyState(JSON.parse(ev.data)); } catch (e) { console.warn(e); } };
    es.onerror = () => {
      $('conn').textContent = 'reconnecting…'; $('conn').className = 'pill bad';
      cam.streaming = false;   // the server may have restarted; reopen the stream with the next state
    };
  }

  // ------------------------------------------------------------ boot
  (async () => {
    model = await (await fetch('/model.json')).json();
    const go2Mat = new THREE.MeshStandardMaterial({ color: 0x3a4250, metalness: 0.2, roughness: 0.7 });
    const d1Mat = new THREE.MeshStandardMaterial({ color: 0xc8ced8, metalness: 0.3, roughness: 0.5 });
    await buildRobot(model.go2, scene, go2Mat, null);
    await buildRobot(model.d1, scene, d1Mat, model.d1.mount);
    sphere = buildSphere(model.sphere);
    // Sitting-ish nominal legs until rt/lowstate arrives, so the picture is not a standing dog.
    for (const leg of ['FR', 'FL', 'RR', 'RL']) {
      setRevolute(`${leg}_thigh_joint`, leg.startsWith('R') ? 1.4 : 0.9);
      setRevolute(`${leg}_calf_joint`, -2.4);
    }
    connectEvents();
  })().catch((e) => { console.error(e); toast('failed to load model: ' + e); });

  (function loop() { requestAnimationFrame(loop); controls.update(); renderer.render(scene, camera); })();
})();
