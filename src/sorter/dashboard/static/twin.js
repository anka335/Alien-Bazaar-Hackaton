// 3D view of the arm and the table: the reBot CAD meshes posed from FK, the layout, the items,
// and what the wrist camera sees. Data: /api/twin/layout once, /api/twin/state polled.
// `?embed` hides the title and help (the dashboard shows it in its main screen).
import * as THREE from "three";
import { STLLoader } from "/twin-assets/three/STLLoader.js";
import { OrbitControls } from "/twin-assets/three/OrbitControls.js";

const MM = 0.001;
const embed = new URLSearchParams(location.search).has("embed");
document.body.classList.toggle("embed", embed);
const $ = (id) => document.getElementById(id);

function badge(text, state) {
  $("badge").textContent = text;
  $("badge").dataset.state = state ?? "";
}

// --- renderer, camera, lights ---

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio || 1, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
document.body.prepend(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x121a24);
scene.fog = new THREE.Fog(0x121a24, 2.5, 6);
const camera = new THREE.PerspectiveCamera(38, 1, 0.01, 20);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
function resetView() {
  // robot frame (x fwd, y left, z up) → three (x, z, -y): from the front right, above
  camera.position.set(1.05, 0.85, 0.55);
  controls.target.set(0.08, 0.02, -0.02);
  controls.update();
}
resetView();
renderer.domElement.addEventListener("dblclick", resetView);

function resize() {
  renderer.setSize(innerWidth, innerHeight);
  camera.aspect = innerWidth / innerHeight;
  camera.updateProjectionMatrix();
}
addEventListener("resize", resize);
resize();

scene.add(new THREE.HemisphereLight(0xf2f6ff, 0x3a3228, 1.5));
const sun = new THREE.DirectionalLight(0xffffff, 2.3);
sun.position.set(0.6, 1.6, 0.9);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
Object.assign(sun.shadow.camera, { left: -0.8, right: 0.8, top: 0.8, bottom: -0.8, near: 0.3, far: 4 });
sun.shadow.bias = -0.0005;
scene.add(sun);
const fill = new THREE.DirectionalLight(0xcfe3ff, 0.6);
fill.position.set(-1.2, 0.8, -0.8);
scene.add(fill);

// everything robot-related lives in `world`: z up, metres
const world = new THREE.Group();
world.rotation.x = -Math.PI / 2;
scene.add(world);

const mat = (color, extra = {}) => new THREE.MeshStandardMaterial({ color, roughness: 0.85, metalness: 0, ...extra });

function block(sx, sy, sz, x, y, z, material) {
  const m = new THREE.Mesh(new THREE.BoxGeometry(sx, sy, sz), material);
  m.position.set(x, y, z);
  m.castShadow = m.receiveShadow = true;
  world.add(m);
  return m;
}

// an open box: floor + four walls; `inner` = inside size, `t` = wall thickness
function tray([cx, cy], [ix, iy], floorZ, wallH, t, material, floorMaterial = material) {
  const ox = ix + 2 * t, oy = iy + 2 * t, top = floorZ + wallH;
  block(ox, oy, floorZ, cx, cy, floorZ / 2, floorMaterial);
  block(t, oy, top, cx - ix / 2 - t / 2, cy, top / 2, material);
  block(t, oy, top, cx + ix / 2 + t / 2, cy, top / 2, material);
  block(ix, t, top, cx, cy - iy / 2 - t / 2, top / 2, material);
  block(ix, t, top, cx, cy + iy / 2 + t / 2, top / 2, material);
}

function label(text, x, y, z) {
  const c = document.createElement("canvas");
  const ctx = c.getContext("2d");
  const font = "600 44px Barlow, Arial, sans-serif";
  ctx.font = font;
  c.width = Math.ceil(ctx.measureText(text).width) + 36;
  c.height = 64;
  ctx.font = font;
  ctx.fillStyle = "rgba(15, 24, 35, 0.72)";
  ctx.beginPath();
  ctx.roundRect(0, 0, c.width, c.height, 32);
  ctx.fill();
  ctx.fillStyle = "#e9f7fb";
  ctx.textBaseline = "middle";
  ctx.fillText(text, 18, c.height / 2 + 2);
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: false }));
  s.scale.set((0.045 * c.width) / c.height, 0.045, 1);
  s.position.set(x, y, z);
  s.renderOrder = 10;
  world.add(s);
}

function outline(points, z, color) {
  const pts = points.map(([x, y]) => new THREE.Vector3(x * MM, y * MM, z));
  const line = new THREE.LineLoop(
    new THREE.BufferGeometry().setFromPoints(pts),
    new THREE.LineDashedMaterial({ color, dashSize: 0.012, gapSize: 0.008 }),
  );
  line.computeLineDistances();
  world.add(line);
}

const BIN_COLORS = { light: 0xeceeee, dark: 0x3a3c3e, colored: 0x248caa };
const BIN_NAMES = { light: "Light", dark: "Dark", colored: "Colored" };

function buildTable(L) {
  const { box, background: bg, bins } = L.table;
  // the table top under everything, z = 0
  const xs = [box.center_mm[0], bg.center_mm[0], ...Object.values(bins.centers_mm).map((c) => c[0])];
  const ys = [box.center_mm[1], bg.center_mm[1], ...Object.values(bins.centers_mm).map((c) => c[1])];
  const x0 = Math.min(...xs, 0) * MM - 0.3, x1 = Math.max(...xs) * MM + 0.3;
  const y0 = Math.min(...ys) * MM - 0.3, y1 = Math.max(...ys) * MM + 0.3;
  block(x1 - x0, y1 - y0, 0.03, (x0 + x1) / 2, (y0 + y1) / 2, -0.015, mat(0xcdb18d, { roughness: 0.7 }));

  // the gray mat
  block(bg.size_mm[0] * MM, bg.size_mm[1] * MM, 0.002, bg.center_mm[0] * MM, bg.center_mm[1] * MM, 0.001, mat(0x7e7c7a));
  label("Mat", bg.center_mm[0] * MM + (bg.size_mm[0] / 2) * MM + 0.05, bg.center_mm[1] * MM, 0.03);

  // the mixed box
  const bc = box.center_mm.map((v) => v * MM);
  tray(bc, box.size_mm.map((v) => v * MM), box.floor_z_mm * MM, box.wall_mm * MM, 0.01, mat(0xd2ac7e), mat(0xb88c5c));
  label("Box", bc[0], bc[1] - (box.size_mm[1] / 2) * MM - 0.05, (box.floor_z_mm + box.wall_mm) * MM + 0.03);

  // the bins
  const inner = (bins.size_mm - 24) * MM;
  for (const [color, [x, y]] of Object.entries(bins.centers_mm)) {
    const m = mat(BIN_COLORS[color], { roughness: 0.55 });
    tray([x * MM, y * MM], [inner, inner], bins.floor_z_mm * MM, bins.wall_mm * MM, 0.012, m);
    const r = Math.hypot(x, y);
    const out = (bins.size_mm / 2 + 45) / r;
    label(BIN_NAMES[color] ?? color, x * MM * (1 + out), y * MM * (1 + out), (bins.floor_z_mm + bins.wall_mm) * MM + 0.03);
  }

  // where a pick is allowed
  outline(L.zones.box ?? [], (box.floor_z_mm + 1) * MM, 0x3cc8e2);
  outline(L.zones.background ?? [], 0.003, 0x3cc8e2);
}

// --- the arm: CAD meshes per URDF link ---

const links = {};
let meshesToLoad = 0;
let meshesLoaded = 0;

async function buildArm() {
  const manifest = await (await fetch("/twin-assets/manifest.json")).json();
  const loader = new STLLoader();
  for (const [name, parts] of Object.entries(manifest.links)) {
    const g = new THREE.Group();
    g.matrixAutoUpdate = false;
    world.add(g);
    links[name] = g;
    for (const p of parts) {
      meshesToLoad++;
      loader.load(
        "/twin-assets/" + p.mesh,
        (geo) => {
          const m = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ color: p.color, metalness: 0.25, roughness: 0.55 }));
          m.position.set(...p.xyz);
          m.rotation.set(p.rpy[0], p.rpy[1], p.rpy[2], "ZYX"); // URDF rpy = Rz·Ry·Rx
          m.castShadow = m.receiveShadow = true;
          g.add(m);
          if (++meshesLoaded === meshesToLoad) badge(simulated ? "sim" : "live", "ready");
        },
        undefined,
        () => badge("mesh error", "error"),
      );
    }
  }
}

// --- items: crumpled cloth ---

function rng(seed) {
  let s = (seed * 2654435761) >>> 0 || 1;
  return () => ((s = (s * 1664525 + 1013904223) >>> 0) / 4294967296);
}

function clothGeometry(id) {
  const g = new THREE.IcosahedronGeometry(1, 4);
  const r = rng(id + 1);
  const waves = Array.from({ length: 6 }, () => [r() * 6.28, 2 + Math.floor(r() * 5), 0.04 + r() * 0.08, r() * 6.28]);
  const p = g.attributes.position;
  const v = new THREE.Vector3();
  for (let i = 0; i < p.count; i++) {
    v.fromBufferAttribute(p, i);
    const a = Math.atan2(v.y, v.x);
    let k = 1;
    for (const [ph, n, amp, ph2] of waves) k += amp * Math.sin(n * a + ph) * Math.cos(n * 0.7 * v.z * 3 + ph2);
    v.multiplyScalar(k);
    p.setXYZ(i, v.x, v.y, v.z);
  }
  g.computeVertexNormals();
  return g;
}

const items = new Map(); // id → {mesh, target, location}
let itemRadius = 0.03;

function updateItems(list) {
  const seen = new Set();
  for (const it of list) {
    seen.add(it.id);
    let e = items.get(it.id);
    if (!e) {
      const mesh = new THREE.Mesh(clothGeometry(it.id), mat(it.color, { roughness: 0.95 }));
      mesh.castShadow = mesh.receiveShadow = true;
      world.add(mesh);
      e = { mesh, target: new THREE.Vector3(), location: null };
      items.set(it.id, e);
    }
    const [x, y, z] = it.xyz.map((v) => v * MM);
    const held = it.location === "gripper";
    const r = itemRadius;
    if (held) {
      e.mesh.scale.set(r * 0.55, r * 0.55, r * 1.1); // hanging from the fingers
      e.target.set(x, y, z - r * 0.6);
    } else {
      e.mesh.scale.set(r * 1.2, r * 0.95, 0.011);
      e.target.set(x, y, z - 0.009);
    }
    if (held || e.location === null) e.mesh.position.copy(e.target); // grabbed: follows the TCP exactly
    e.location = it.location;
  }
  for (const [id, e] of items) {
    if (!seen.has(id)) {
      world.remove(e.mesh);
      items.delete(id);
    }
  }
}

// --- the wrist camera and what it sees on the table ---

const camBody = new THREE.Mesh(new THREE.BoxGeometry(0.025, 0.09, 0.025), mat(0x1b1e22, { roughness: 0.4 }));
camBody.matrixAutoUpdate = false;
camBody.visible = false;
world.add(camBody);
const frustum = new THREE.LineSegments(
  new THREE.BufferGeometry().setAttribute("position", new THREE.Float32BufferAttribute(new Float32Array(16 * 3), 3)),
  new THREE.LineBasicMaterial({ color: 0x3cc8e2, transparent: true, opacity: 0.55 }),
);
frustum.frustumCulled = false;
world.add(frustum);
const footprint = new THREE.Mesh(
  new THREE.BufferGeometry().setAttribute("position", new THREE.Float32BufferAttribute(new Float32Array(6 * 3), 3)),
  new THREE.MeshBasicMaterial({ color: 0x3cc8e2, transparent: true, opacity: 0.18, side: THREE.DoubleSide, depthWrite: false }),
);
footprint.frustumCulled = false;
world.add(footprint);
let camIntr = null;

function updateCamera(T) {
  const show = !!(T && camIntr);
  camBody.visible = frustum.visible = footprint.visible = show;
  if (!show) return;
  const M = new THREE.Matrix4().set(...T);
  camBody.matrix.copy(M);
  camBody.matrixWorldNeedsUpdate = true;
  const o = new THREE.Vector3().setFromMatrixPosition(M);
  const R = new THREE.Matrix3().setFromMatrix4(M);
  const { width: w, height: h, focal_px: f } = camIntr;
  const corners = [[0, 0], [w, 0], [w, h], [0, h]].map(([u, v]) => {
    const d = new THREE.Vector3((u - w / 2) / f, (v - h / 2) / f, 1).applyMatrix3(R);
    const t = d.z < -0.05 ? Math.min(-o.z / d.z, 1.5) : 0.35; // hit the table, or a short ray
    return o.clone().addScaledVector(d, t);
  });
  const fp = frustum.geometry.attributes.position;
  corners.forEach((c, i) => {
    const n = corners[(i + 1) % 4];
    fp.setXYZ(i * 4, o.x, o.y, o.z);
    fp.setXYZ(i * 4 + 1, c.x, c.y, c.z);
    fp.setXYZ(i * 4 + 2, c.x, c.y, c.z);
    fp.setXYZ(i * 4 + 3, n.x, n.y, n.z);
  });
  fp.needsUpdate = true;
  const tri = [0, 1, 2, 0, 2, 3].map((i) => corners[i]);
  const pp = footprint.geometry.attributes.position;
  tri.forEach((c, i) => pp.setXYZ(i, c.x, c.y, Math.max(c.z, 0) + 0.004));
  pp.needsUpdate = true;
}

// --- polling ---

let simulated = false;

function applyState(S) {
  for (const [name, m] of Object.entries(S.links)) {
    const g = links[name];
    if (!g) continue;
    g.matrix.set(...m);
    g.matrixWorldNeedsUpdate = true;
  }
  updateItems(S.items);
  updateCamera(S.camera);
}

async function pollState() {
  try {
    const r = await fetch("/api/twin/state", { cache: "no-store" });
    if (!r.ok) throw new Error(r.statusText);
    applyState(await r.json());
    if (meshesLoaded === meshesToLoad && meshesToLoad) badge(simulated ? "sim" : "live", "ready");
  } catch {
    badge("no connection", "error");
  }
  setTimeout(pollState, 40);
}

const PHASES = {
  idle: "Ready", starting: "Starting", look_bg: "Looking at the mat", sense_bg: "Checking color",
  pick_from_bg: "Picking up", drop_to_bin: "Into the bin", look_box: "Looking in the box",
  sense_box: "Choosing a grasp", pick_from_box: "Grabbing", place_on_bg: "Laying out",
  done: "Done", held: "Held", error: "Stopped",
};

async function pollStatus() {
  if (!embed) {
    try {
      const s = await (await fetch("/api/status", { cache: "no-store" })).json();
      $("phase").textContent = PHASES[s.phase] ?? s.phase;
      $("counts").textContent = Object.entries(s.counters).map(([k, n]) => `${BIN_NAMES[k] ?? k} ${n}`).join(" · ");
    } catch {
      /* the badge shows it */
    }
  }
  setTimeout(pollStatus, 500);
}

// Space or Esc holds the arm from here too (inside the dashboard, keys go to this frame)
document.addEventListener("keydown", (e) => {
  if (e.code === "Space" || e.key === "Escape") {
    e.preventDefault();
    fetch("/api/command", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ cmd: "hold" }) });
  }
});

async function main() {
  try {
    const L = await (await fetch("/api/twin/layout")).json();
    simulated = L.simulated;
    $("mode").textContent = simulated ? "simulator" : "live arm";
    camIntr = L.camera;
    itemRadius = L.item_radius_mm * MM;
    buildTable(L);
  } catch {
    badge("no layout", "error");
  }
  buildArm().catch(() => badge("mesh error", "error"));
  pollState();
  pollStatus();
}

// cloth that was dropped falls into place
const clock = new THREE.Clock();
renderer.setAnimationLoop(() => {
  const k = 1 - Math.exp(-clock.getDelta() * 12);
  for (const e of items.values()) e.mesh.position.lerp(e.target, k);
  controls.update();
  renderer.render(scene, camera);
});

main();
