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
 * uses, and demos/cup/tests/test_d1_ui.py checks the two agree, since a browser cannot
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
  // Intel's D435 case is a binary PLY (demos/cup/pick_demo/assets/realsense). Parsed here for the same reason
  // parseBinarySTL exists: r128 ships PLYLoader as a separate example script, and vendoring one more
  // file to read one mesh is more than this needs. Only the layout that file actually has is handled --
  // binary little-endian, float x/y/z then any other per-vertex properties, uchar+int face lists -- and
  // anything else throws rather than being guessed at.
  function parseBinaryPLY(buf) {
    const bytes = new Uint8Array(buf);
    const headEnd = (() => {
      const needle = 'end_header\n';
      const text = new TextDecoder('ascii').decode(bytes.subarray(0, Math.min(bytes.length, 4096)));
      const at = text.indexOf(needle);
      if (at < 0) throw new Error('PLY: no end_header in the first 4 kB');
      return at + needle.length;
    })();
    const header = new TextDecoder('ascii').decode(bytes.subarray(0, headEnd)).split('\n');
    if (!header.some((l) => l.trim() === 'format binary_little_endian 1.0')) {
      throw new Error('PLY: only binary_little_endian 1.0 is handled');
    }
    const SIZES = { char: 1, uchar: 1, int8: 1, uint8: 1, short: 2, ushort: 2, int16: 2, uint16: 2,
                    int: 4, uint: 4, int32: 4, uint32: 4, float: 4, float32: 4, double: 8, float64: 8 };
    const elements = [];
    for (const line of header) {
      const t = line.trim().split(/\s+/);
      if (t[0] === 'element') elements.push({ name: t[1], count: parseInt(t[2], 10), props: [] });
      else if (t[0] === 'property' && elements.length) {
        const e = elements[elements.length - 1];
        if (t[1] === 'list') e.props.push({ list: true, countType: t[2], type: t[3], name: t[4] });
        else e.props.push({ list: false, type: t[1], name: t[2] });
      }
    }
    const dv = new DataView(buf);
    const read = (type, at) => {
      switch (type) {
        case 'float': case 'float32': return dv.getFloat32(at, true);
        case 'double': case 'float64': return dv.getFloat64(at, true);
        case 'int': case 'int32': return dv.getInt32(at, true);
        case 'uint': case 'uint32': return dv.getUint32(at, true);
        case 'short': case 'int16': return dv.getInt16(at, true);
        case 'ushort': case 'uint16': return dv.getUint16(at, true);
        case 'char': case 'int8': return dv.getInt8(at);
        default: return dv.getUint8(at);
      }
    };
    let at = headEnd, verts = null, faces = [];
    for (const e of elements) {
      if (e.name === 'vertex') {
        const stride = e.props.reduce((n, pr) => n + (SIZES[pr.type] || 0), 0);
        const offs = {}; let o = 0;
        for (const pr of e.props) { offs[pr.name] = { at: o, type: pr.type }; o += SIZES[pr.type] || 0; }
        if (!offs.x || !offs.y || !offs.z) throw new Error('PLY: vertex has no x/y/z');
        verts = new Float32Array(e.count * 3);
        for (let i = 0; i < e.count; i++) {
          const b = at + i * stride;
          verts[i * 3] = read(offs.x.type, b + offs.x.at);
          verts[i * 3 + 1] = read(offs.y.type, b + offs.y.at);
          verts[i * 3 + 2] = read(offs.z.type, b + offs.z.at);
        }
        at += e.count * stride;
      } else {
        for (let i = 0; i < e.count; i++) {
          for (const pr of e.props) {
            if (!pr.list) { at += SIZES[pr.type] || 0; continue; }
            const n = read(pr.countType, at); at += SIZES[pr.countType] || 1;
            const idx = [];
            for (let k = 0; k < n; k++) { idx.push(read(pr.type, at)); at += SIZES[pr.type] || 4; }
            if (e.name === 'face') for (let k = 2; k < n; k++) faces.push(idx[0], idx[k - 1], idx[k]);
          }
        }
      }
    }
    if (!verts) throw new Error('PLY: no vertex element');
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(verts, 3));
    g.setIndex(faces);
    g.computeVertexNormals();
    return g;
  }

  const collada = new THREE.ColladaLoader();
  async function loadMesh(url, material) {
    if (url.toLowerCase().endsWith('.ply')) {
      const buf = await (await fetch(url)).arrayBuffer();
      return new THREE.Mesh(parseBinaryPLY(buf), material);
    }
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
    if (mountGrabbed()) return;   // a click on the mount gizmo is not a click on the sphere
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
  $('pick').onclick = async () => {
    const live = $('live').checked;
    const warning = live
      ? 'LIVE is on: the scripted pick WILL MOVE THE REAL ARM through a reach, a descent and a lift.\n\n'
        + 'Make sure the arm is clear and you can reach STOP.\n\nRun it?'
      : 'Dry run: the pick will look, plan and log, but send nothing to the arm.\n\nRun it?';
    if (!confirm(warning)) return;
    const r = await post('/pick');
    toast(r.ok ? (live ? 'pick running — LIVE' : 'pick running — dry run') : `refused: ${r.reason}`);
  };

  $('gripval').oninput = (e) => { $('griplabel').textContent = e.target.value; };
  $('gripset').onclick = async () => {
    const units = Number($('gripval').value);
    if ($('live').checked && !confirm(
      `Move the gripper to ${units} units?\n\nThe fingers close under the drive's own effort. `
      + 'Keep hands out of the jaws.')) return;
    const r = await post('/gripper', { units });
    toast(r.ok ? (r.reason === 'dry run' ? 'dry run — not sent' : `gripper -> ${units}`) : `refused: ${r.reason}`);
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
    $('picksec').classList.toggle('off', sim);
    if (sim) $('send').disabled = true;
  }

  // ------------------------------------------------------------ the scripted pick
  function applyPick(s) {
    const p = s.pick;
    if (!p) return;
    const busy = s.busy === 'pick';
    $('pick').disabled = !p.available || !!s.busy;
    $('pick').textContent = busy ? 'PICKING…' : 'PICK';
    $('pick').className = (p.available && s.live) ? 'danger' : '';
    $('pickcam').textContent = p.camera_source || '–';
    $('pickmount').textContent = p.mount_source || '–';
    // An uncalibrated camera or mount is the difference between a demo and a measurement: say so.
    const caveats = [];
    if ((p.camera_source || '').includes('NOT this camera')) caveats.push('camera model is the datasheet preset');
    if ((p.mount_source || '').includes('assumed')) caveats.push('wrist mount is assumed, not measured');
    $('pickmsg').textContent = p.refusal
      ? `unavailable: ${p.refusal}`
      : (caveats.length ? `runs, but: ${caveats.join('; ')}` : '');
    $('pickmsg').className = 'hint' + (p.refusal ? ' warn' : '');
    const last = p.last;
    $('gripnow').textContent = s.gripper_units === null || s.gripper_units === undefined
      ? '–' : `${s.gripper_units} units`;
    $('gripset').disabled = s.mode === 'sim' || !!s.busy;
    $('gripset').className = s.live ? 'danger' : '';
    $('pickresult').textContent = last
      ? `${last.ok ? 'finished' : 'ended'}: ${last.reason} · ${last.elapsed_s}s · `
        + `${last.frames_looked_at} frames, ${last.frames_with_cup} with a cup · `
        + `${last.live ? `${last.commands_sent} commands sent` : 'dry run, nothing sent'}`
      : '';
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
    foot.title = [c.detector ? `detector: ${c.detector}` : '', c.tag_detector ? `AprilTag: ${c.tag_detector}` : '']
      .filter(Boolean).join('\n');
    let html;
    if (!c.detector_ready) {
      html = esc(c.detector ? `YOLO ${c.detector}` : 'no detector');
    } else {
      const dets = c.detections || [];
      html = (dets.length
        ? dets.map((d) => `<span class="hit">${esc(d.label)} ${fmt(d.confidence, 2)}</span>`).join(' · ')
        : `YOLO: no ${esc((c.targets || ['objects']).join('/'))} detected`) +
        (c.detect_ms !== null && c.detect_ms !== undefined ? ` · YOLO ${fmt(c.detect_ms, 0)} ms` : '');
    }
    if (c.tags_enabled) {
      const tags = c.tags || [];
      html += ' · ' + (tags.length
        ? tags.map((t) => `<span class="tag">tag ${esc(t.id)}${t.range_m === null || t.range_m === undefined
          ? '' : ' ' + fmt(t.range_m, 2) + ' m'}</span>`).join(' · ')
        : 'no AprilTag in view');
    }
    foot.innerHTML = html;
  }

  // ------------------------------------------------------------ live state
  function applyState(s) {
    applyMode(s);
    const sim = s.mode === 'sim';
    $('conn').textContent = s.connected ? (sim ? 'sim feed' : 'live feedback')
      : (sim ? (s.servo_deg ? 'sim not updating' : 'waiting for sim') : 'no feedback');
    $('conn').className = 'pill ' + (s.connected ? 'ok' : 'bad');
    $('power').textContent = s.power === null ? '–' : (s.power ? 'on' : 'off');
    applyPick(s);
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


  // ------------------------------------------------------------ wrist mount editor
  //
  // The camera's optical frame on Link6, as six numbers an operator can move. The RealSense CAD is
  // parented to the Link6 node, so it follows the arm exactly as the real camera would, and the axes
  // helper at its origin is the optical frame itself -- red x right, green y down, blue z forward,
  // the frame `deproject` works in.
  //
  // Moving it is a gizmo in the 3D view rather than a bank of sliders: MOVE IN 3D puts arrows and
  // rings on the camera and a label at the end of each axis reading that axis in cm and degrees. The
  // panel keeps the same six numbers, in cm and degrees, editable for an exact value.
  //
  // Editing is local first and posted after: the picture must not wait on a round trip, but the
  // server holds the mount the perception actually uses, so every change is sent. Only SAVE writes a
  // file. The page never invents a starting pose; it asks the server what the mount currently is.
  const mountEd = {
    node: null, group: null, live: null, start: null, inputs: {}, ready: false,
    pivot: null, gizmos: [], labels: [], dragging: false, dragFrom: null, on: false,
    active: null, turn: null,
    // name, limit, step -- limits in the units the panel shows, inside the server's own (it refuses
    // anything over half a metre from the wrist, which is what a metres/centimetres slip looks like).
    AXES: [['x', 20, 0.1], ['y', 20, 0.1], ['z', 20, 0.1],
           ['roll', 180, 0.5], ['pitch', 180, 0.5], ['yaw', 180, 0.5]],
    // live[] is metres and degrees, as the server takes them; the panel is cm and degrees.
    toPanel: (v, i) => (i < 3 ? v * 100 : v),
    fromPanel: (v, i) => (i < 3 ? v / 100 : v),
  };

  function mountApplyToScene() {
    if (!mountEd.group || !mountEd.live) return;
    const v = mountEd.live;
    mountEd.group.position.set(v[0], v[1], v[2]);
    mountEd.group.rotation.copy(eulerZYX([v[3], v[4], v[5]].map((d) => d * Math.PI / 180)));
    if (mountEd.pivot) {
      // The gizmo rides a pivot with no rotation of its own, so its arrows and rings are Link6's
      // axes -- the frame the six numbers are in. A drag turns the pivot; the turn is taken off it
      // again when the drag ends (mountGizmoDrop), so the rings never drift away from those axes.
      mountEd.pivot.position.copy(mountEd.group.position);
      if (!mountEd.dragging) mountEd.pivot.quaternion.identity();
    }
  }

  function mountRefreshInputs() {
    mountEd.AXES.forEach(([name], i) => {
      const row = mountEd.inputs[name];
      if (!row) return;
      const value = mountEd.toPanel(mountEd.live[i], i);
      if (document.activeElement !== row.num) row.num.value = value.toFixed(2);
      row.num.classList.toggle('mg-dirty', mountEd.start !== null
        && Math.abs(mountEd.live[i] - mountEd.start[i]) > (i < 3 ? 5e-5 : 0.01));
    });
    mountLabelsRefresh();
  }

  let mountPostTimer = null;
  function mountPost() {
    if (mountPostTimer) return;                    // at most one in flight per frame budget
    mountPostTimer = setTimeout(async () => {
      mountPostTimer = null;
      const v = mountEd.live.slice();
      try {
        const r = await fetch('/mount', {
          method: 'POST', headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ xyz_m: v.slice(0, 3), rpy_deg: v.slice(3) }),
        });
        const j = await r.json();
        if (!j.ok) { $('mountmsg').textContent = j.reason || 'refused'; return; }
        $('mountmsg').textContent = 'unsaved — the controller is using this now';
        if (j.mount) mountShow(j.mount);
      } catch (e) { $('mountmsg').textContent = 'could not reach the server'; }
    }, 120);
  }

  // The server checks the case against Link6's CAD shell on every change, so putting the camera
  // inside the wrist shows up here rather than later in a render.
  function mountShow(status) {
    $('mountsrc').textContent = status.source || '–';
    // No clearance where the case mesh cannot be read (the dog): the editor still works, the
    // warning simply is not offered, and saying nothing is better than implying "clear".
    const hit = status.clearance && status.clearance.intersects_shell;
    const msg = $('mountmsg');
    if (hit) {
      const [ox, oz] = status.clearance.overlap_m;
      msg.textContent = `the case is inside the Link6 shell by ${(ox * 1000).toFixed(0)} x ${(oz * 1000).toFixed(0)} mm`;
      msg.style.color = '#ff8a8a';
    } else {
      const clear = status.clearance ? ' — clear of the wrist' : '';
      msg.textContent = status.saved_to ? `saved: ${status.saved_to}` : `unsaved${clear}`;
      msg.style.color = '';
    }
  }

  // Six numbers in two rows -- cm across x y z, then degrees about the same three axes -- with the
  // headings coloured like the gizmo's arrows, so a number and the handle that moves it match.
  function mountBuildControls() {
    const grid = $('mountgrid');
    grid.textContent = '';
    const head = document.createElement('tr');
    head.innerHTML = '<td></td><td class="mg-ax ax-x">x</td><td class="mg-ax ax-y">y</td>'
                   + '<td class="mg-ax ax-z">z</td><td></td>';
    grid.appendChild(head);
    [[0, 'move'], [3, 'turn']].forEach(([base, label]) => {
      const tr = document.createElement('tr');
      const name = document.createElement('td');
      name.className = 'mg-name'; name.textContent = label;
      tr.appendChild(name);
      for (let k = 0; k < 3; k++) {
        const i = base + k;
        const [axis, limit, step] = mountEd.AXES[i];
        const td = document.createElement('td');
        const num = document.createElement('input');
        num.type = 'number'; num.min = -limit; num.max = limit; num.step = step;
        num.title = axis;
        td.appendChild(num); tr.appendChild(td);
        num.addEventListener('change', () => {
          const value = Math.max(-limit, Math.min(limit, Number(num.value)));
          if (!Number.isFinite(value)) { mountRefreshInputs(); return; }
          mountEd.live[i] = mountEd.fromPanel(value, i);
          num.value = value.toFixed(2);            // a typed 400 is refused: show the 180 that was taken
          mountApplyToScene(); mountRefreshInputs(); mountPost();
        });
        mountEd.inputs[axis] = { num };
      }
      const unit = document.createElement('td');
      unit.className = 'mg-unit'; unit.textContent = base === 0 ? 'cm' : '°';
      tr.appendChild(unit);
      grid.appendChild(tr);
    });
  }

  // ---- the gizmo: arrows to slide it, rings to turn it, a label per axis
  //
  // Two TransformControls on one pivot, translate and rotate at once, so there is no mode to switch:
  // the single button is on or off. They are sized apart (arrows inside the rings) to keep the two
  // sets of handles from overlapping, and whichever takes a drag switches the other off for its
  // duration, so a click can never grab both.
  const MOUNT_ARROW_SIZE = 0.55, MOUNT_RING_SIZE = 0.75;   // arrows inside the rings
  const MOUNT_AXES_3D = [
    { key: 'x', angle: 'roll', dir: new THREE.Vector3(1, 0, 0), colour: '#ff6b6b' },
    { key: 'y', angle: 'pitch', dir: new THREE.Vector3(0, 1, 0), colour: '#7ee787' },
    { key: 'z', angle: 'yaw', dir: new THREE.Vector3(0, 0, 1), colour: '#6bb6ff' },
  ];

  const MOUNT_LABEL_W = 1024, MOUNT_LABEL_H = 96, MOUNT_LABEL_PX = 13;   // canvas, and text height on screen

  function mountMakeLabel() {
    const canvas = document.createElement('canvas');
    canvas.width = MOUNT_LABEL_W; canvas.height = MOUNT_LABEL_H;
    const texture = new THREE.CanvasTexture(canvas);
    const sprite = new THREE.Sprite(new THREE.SpriteMaterial({
      map: texture, depthTest: false, depthWrite: false, sizeAttenuation: false, transparent: true,
    }));
    sprite.center.set(0, 0.5);         // the text starts at the end of the axis, not across it
    sprite.renderOrder = 10;
    sprite.visible = false;
    scene.add(sprite);
    return { sprite, canvas, texture, text: null, plate: 1 };
  }

  function mountDrawLabel(label, text, colour) {
    if (label.text === text) return;
    label.text = text;
    const ctx = label.canvas.getContext('2d');
    ctx.clearRect(0, 0, MOUNT_LABEL_W, MOUNT_LABEL_H);
    ctx.font = `bold ${Math.round(MOUNT_LABEL_H * 0.5)}px ui-monospace, Menlo, Consolas, monospace`;
    ctx.textBaseline = 'middle';
    // A plate only as wide as the text: the rest of the canvas stays clear of the picture behind it.
    const width = Math.min(MOUNT_LABEL_W, ctx.measureText(text).width + 28);
    label.plate = width / MOUNT_LABEL_W;           // the share of the canvas the text actually uses
    ctx.fillStyle = 'rgba(11, 15, 20, 0.82)';
    ctx.fillRect(0, MOUNT_LABEL_H * 0.2, width, MOUNT_LABEL_H * 0.6);
    ctx.fillStyle = colour;
    ctx.fillText(text, 14, MOUNT_LABEL_H * 0.5);
    label.texture.needsUpdate = true;
  }

  // Each axis reads "x -6.10 cm  roll 0.0°": how far along it the camera sits, and the angle of the
  // saved triple that belongs to it. The angle is *named*, because roll/pitch/yaw are a ZYX sequence
  // and not three independent turns -- a drag on one ring can move more than one of them, and a label
  // reading a bare "25.2°" next to the ring you just dragged would imply otherwise. While a ring is
  // being dragged that axis reads the turn the drag itself has applied, which is about that axis only.
  function mountLabelsRefresh() {
    if (!mountEd.labels.length || !mountEd.live) return;
    MOUNT_AXES_3D.forEach((axis, i) => {
      const cm = (mountEd.live[i] * 100).toFixed(2);
      const turn = mountEd.turn && mountEd.turn.axis === i
        ? `turned ${mountEd.turn.deg >= 0 ? '+' : ''}${mountEd.turn.deg.toFixed(1)}°`
        : `${axis.angle} ${mountEd.live[i + 3].toFixed(1)}°`;
      mountDrawLabel(mountEd.labels[i], `${axis.key} ${cm} cm  ${turn}`, axis.colour);
    });
  }

  // The handles scale with distance so they stay the same size on screen (TransformControls does
  // `factor * size / 7`); the labels sit at the end of each axis, so they follow the same scale.
  function mountLabelsPlace() {
    if (!mountEd.on || !mountEd.pivot || !mountEd.labels.length) return;
    const origin = mountEd.pivot.getWorldPosition(new THREE.Vector3());
    const factor = origin.distanceTo(camera.position)
                 * Math.min(1.9 * Math.tan(Math.PI * camera.fov / 360) / camera.zoom, 7);
    const reach = factor * MOUNT_RING_SIZE / 7 * 1.15;   // just past the rotate rings
    const basis = new THREE.Quaternion();
    mountEd.pivot.getWorldQuaternion(basis);
    // A sprite with sizeAttenuation off is scaled in viewport fractions, so the canvas's own aspect
    // has to be put back by hand or the text comes out stretched, differently on every window size.
    const wpx = renderer.domElement.clientWidth || 1, hpx = renderer.domElement.clientHeight || 1;
    const sy = (MOUNT_LABEL_PX / (MOUNT_LABEL_H * 0.5)) * MOUNT_LABEL_H / hpx;
    const sx = sy * hpx * (MOUNT_LABEL_W / MOUNT_LABEL_H) / wpx;
    // Behind the camera there is nothing sensible to draw: a projection there lands anywhere.
    const ahead = origin.clone().applyMatrix4(camera.matrixWorldInverse).z < -camera.near;
    MOUNT_AXES_3D.forEach((axis, i) => {
      const label = mountEd.labels[i];
      label.sprite.visible = ahead;
      label.sprite.scale.set(sx, sy, 1);
      if (!ahead) return;
      const dir = axis.dir.clone().applyQuaternion(basis).multiplyScalar(reach);
      // Kept inside the viewport: the panel covers the right of the window, and a label that runs
      // under it is a measurement nobody can read. Clamped in NDC, where the sprite's own size is
      // known -- its scale is a fraction of the viewport, so it spans 2*scale in NDC.
      const ndc = origin.clone().add(dir).project(camera);
      ndc.x = Math.max(-1, Math.min(ndc.x, 1 - 2 * sx * label.plate));
      ndc.y = Math.max(-1 + sy, Math.min(ndc.y, 1 - sy));
      label.sprite.position.copy(ndc.unproject(camera));
    });
  }

  // True while the pointer is on the gizmo: mid-drag, or hovering a handle it is about to take.
  // The sphere's click handler asks, so nudging the camera never also picks a target behind it.
  function mountGrabbed() {
    return mountEd.dragging || (mountEd.on && mountEd.gizmos.some((g) => g.axis));
  }

  function mountGizmoGrab(control) {
    mountEd.dragging = true;
    mountEd.active = control;
    mountEd.turn = null;
    controls.enabled = false;
    mountEd.gizmos.forEach((g) => { if (g !== control) g.enabled = false; });
    // Rotation is read as a change since the drag began: the pivot starts each drag square with
    // Link6, and what the ring adds to it is applied to the camera's own orientation.
    mountEd.dragFrom = {
      pivot: mountEd.pivot.quaternion.clone(),
      group: mountEd.group.quaternion.clone(),
    };
  }

  function mountGizmoDrop() {
    mountEd.dragging = false;
    mountEd.active = null;
    mountEd.turn = null;
    controls.enabled = true;
    mountEd.gizmos.forEach((g) => { g.enabled = mountEd.on; });
    mountEd.dragFrom = null;
    mountApplyToScene();                           // squares the pivot with Link6 again
    mountRefreshInputs();
  }

  // The pivot moved: read the six numbers back out of it and off we go. Position is already in the
  // Link6 frame because the pivot is a child of Link6; rotation is the drag's delta applied to the
  // orientation the camera had when the drag started, read back as the URDF's ZYX roll/pitch/yaw.
  function mountGizmoChanged() {
    if (!mountEd.dragFrom || !mountEd.live) return;
    const clamp = (v, lim) => Math.max(-lim, Math.min(lim, v));
    const p = mountEd.pivot.position;
    mountEd.live[0] = clamp(p.x, 0.20);
    mountEd.live[1] = clamp(p.y, 0.20);
    mountEd.live[2] = clamp(p.z, 0.20);
    const delta = mountEd.pivot.quaternion.clone().multiply(mountEd.dragFrom.pivot.clone().invert());
    mountEd.turn = mountTurnOf(delta);
    const q = delta.clone().multiply(mountEd.dragFrom.group);
    const e = new THREE.Euler().setFromQuaternion(q, 'ZYX');
    mountEd.live[3] = e.x * 180 / Math.PI;
    mountEd.live[4] = e.y * 180 / Math.PI;
    mountEd.live[5] = e.z * 180 / Math.PI;
    // Keep the drawn camera and the numbers the same thing, then tell the server.
    mountEd.group.position.set(mountEd.live[0], mountEd.live[1], mountEd.live[2]);
    mountEd.group.quaternion.copy(q);
    mountEd.pivot.position.copy(mountEd.group.position);
    mountRefreshInputs();
    mountPost();
  }

  // Remove handles by name from a TransformControls, picture and picker alike, so they can be
  // neither seen nor grabbed.
  function mountTrimGizmo(control, mode, names) {
    const parts = control._gizmo;
    if (!parts) return;                            // a future three.js: leave the gizmo as it comes
    for (const set of [parts.gizmo, parts.picker, parts.helper]) {
      const group = set && set[mode];
      if (!group) continue;
      group.children.filter((c) => names.includes(c.name)).forEach((c) => group.remove(c));
    }
  }

  // The turn a rotation drag has applied so far, as an angle about the ring's own axis. Signed by
  // which way the drag went; nothing to report for a translate drag, which leaves the pivot square.
  function mountTurnOf(delta) {
    const control = mountEd.active;
    if (!control || control.getMode() !== 'rotate') return null;
    const i = ['X', 'Y', 'Z'].indexOf(control.axis);
    if (i < 0) return null;
    const q = delta.clone().normalize();
    const sin = Math.hypot(q.x, q.y, q.z);
    if (sin < 1e-9) return { axis: i, deg: 0 };
    const deg = 2 * Math.atan2(sin, q.w) * 180 / Math.PI;
    const along = [q.x, q.y, q.z][i] / sin;        // +1 or -1: the ring's axis, one way or the other
    return { axis: i, deg: deg * Math.sign(along) };
  }

  function mountGizmoBuild() {
    mountEd.pivot = new THREE.Object3D();
    mountEd.node.add(mountEd.pivot);
    // Arrows inside the rings, both smaller than the default, so they read as one small handle on a
    // camera the size of a matchbox rather than filling the view.
    [['translate', MOUNT_ARROW_SIZE], ['rotate', MOUNT_RING_SIZE]].forEach(([mode, size]) => {
      const control = new THREE.TransformControls(camera, renderer.domElement);
      control.setMode(mode);
      control.setSpace('local');                   // the pivot is square with Link6: so are the handles
      control.setSize(size);
      control.setTranslationSnap(null);
      control.attach(mountEd.pivot);
      control.enabled = false;
      control.visible = false;
      // Only the named axes. TransformControls also offers free rotation -- a screen-space ring and an
      // invisible sphere filling the middle -- and a screen-plane translate blob, which would turn a
      // drag into a rotation about no axis in particular and leave the labels describing a number
      // nobody chose. Dropping those handles is what keeps "each axis reads that axis" true.
      mountTrimGizmo(control, mode, mode === 'rotate' ? ['E', 'XYZE'] : ['XYZ']);
      control.addEventListener('dragging-changed', (e) => (e.value ? mountGizmoGrab(control) : mountGizmoDrop()));
      control.addEventListener('objectChange', mountGizmoChanged);
      scene.add(control);
      mountEd.gizmos.push(control);
    });
    mountEd.labels = MOUNT_AXES_3D.map(() => mountMakeLabel());
  }

  function mountGizmoToggle(on) {
    mountEd.on = on;
    mountEd.gizmos.forEach((g) => { g.enabled = on; g.visible = on; });
    mountEd.labels.forEach((l) => { l.sprite.visible = on; });
    const button = $('mountmove');
    button.textContent = on ? 'DONE MOVING' : 'MOVE IN 3D';
    button.classList.toggle('on', on);
    $('mountmovehint').textContent = on
      ? 'Arrows slide it, rings turn it — both along the wrist\'s own axes. Each label reads that axis.'
      : 'Arrows slide it, rings turn it — both along the wrist\'s own axes.';
    if (on) { mountApplyToScene(); mountLabelsRefresh(); mountLabelsPlace(); }
    else if (mountEd.dragging) mountGizmoDrop();
  }

  async function mountInit(link6) {
    let status;
    try { status = await (await fetch('/mount')).json(); } catch (e) { return; }
    if (!status || !status.available) return;      // no pick configured here: no mount to edit
    mountEd.node = link6;
    mountEd.live = status.xyz_m.concat(status.rpy_deg);
    mountEd.start = mountEd.live.slice();

    mountEd.group = new THREE.Object3D();
    link6.add(mountEd.group);
    mountEd.group.add(new THREE.AxesHelper(0.03));
    try {
      const material = new THREE.MeshStandardMaterial({ color: 0x8b93a0, metalness: 0.8, roughness: 0.35 });
      const mesh = await loadMesh(status.mesh, material);
      // The CAD is in its own frame; the server sends the 4x4 that takes it into the optical frame,
      // which is the frame this group is. Row-major, as THREE.Matrix4.set wants.
      const m = status.mesh_to_optical;
      mesh.applyMatrix4(new THREE.Matrix4().set(...m[0], ...m[1], ...m[2], ...m[3]));
      mountEd.group.add(mesh);
    } catch (e) { console.warn('camera mesh failed', e); toast('camera mesh failed to load'); }

    mountBuildControls();
    mountGizmoBuild();
    mountApplyToScene(); mountRefreshInputs();
    mountShow(status);
    $('mountsec').hidden = false;
    mountEd.ready = true;

    $('mountmove').addEventListener('click', () => mountGizmoToggle(!mountEd.on));
    $('mountrevert').addEventListener('click', () => {
      mountEd.live = mountEd.start.slice();
      mountApplyToScene(); mountRefreshInputs(); mountPost();
    });
    $('mountsave').addEventListener('click', async () => {
      const name = prompt('Save the mount as (letters, digits, - and _):', 'wrist_mount');
      if (name === null) return;
      $('mountmsg').textContent = 'saving…';
      try {
        const r = await fetch('/mount/save', {
          method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ name }),
        });
        const j = await r.json();
        if (!j.ok) { $('mountmsg').textContent = j.reason || 'refused'; return; }
        mountEd.start = mountEd.live.slice();
        mountRefreshInputs();
        if (j.mount) mountShow(j.mount);
        $('mountmsg').textContent = `saved: ${j.path}`;
        toast('mount saved');
      } catch (e) { $('mountmsg').textContent = 'could not reach the server'; }
    });
  }


  // ------------------------------------------------------------ boot
  (async () => {
    model = await (await fetch('/model.json')).json();
    const go2Mat = new THREE.MeshStandardMaterial({ color: 0x3a4250, metalness: 0.2, roughness: 0.7 });
    const d1Mat = new THREE.MeshStandardMaterial({ color: 0xc8ced8, metalness: 0.3, roughness: 0.5 });
    await buildRobot(model.go2, scene, go2Mat, null);
    const d1Links = await buildRobot(model.d1, scene, d1Mat, model.d1.mount);
    sphere = buildSphere(model.sphere);
    // Sitting-ish nominal legs until rt/lowstate arrives, so the picture is not a standing dog.
    for (const leg of ['FR', 'FL', 'RR', 'RL']) {
      setRevolute(`${leg}_thigh_joint`, leg.startsWith('R') ? 1.4 : 0.9);
      setRevolute(`${leg}_calf_joint`, -2.4);
    }
    connectEvents();
    // The camera hangs off Link6, so it needs the arm built first. A failure here must not take the
    // console with it: the mount editor is a convenience, the arm controls are not.
    mountInit(d1Links.Link6).catch((e) => console.warn('mount editor unavailable', e));
  })().catch((e) => { console.error(e); toast('failed to load model: ' + e); });

  (function loop() {
    requestAnimationFrame(loop);
    controls.update();
    mountLabelsPlace();   // the axis labels follow the gizmo, which is sized off the camera
    renderer.render(scene, camera);
  })();
})();
