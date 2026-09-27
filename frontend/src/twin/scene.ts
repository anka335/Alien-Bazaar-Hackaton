// 3D view of the arm and the table: the reBot CAD meshes posed from FK, the layout, the items,
// and what the wrist camera sees. Data: /api/twin/layout once, /api/twin/state polled while the
// view is mounted. The meshes come from /twin-assets (rebot_b601, D-012).
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";

const MM = 0.001;
const BIN_COLORS: Record<string, number> = { light: 0xeceeee, dark: 0x3a3c3e, colored: 0x248caa };
export const BIN_NAMES: Record<string, string> = { light: "Light", dark: "Dark", colored: "Colored" };

export type Badge = { text: string; state: "" | "ready" | "error" };

interface Rect {
  center_mm: [number, number];
  size_mm: [number, number];
}
interface Layout {
  simulated: boolean;
  table: {
    box: Rect & { floor_z_mm: number; wall_mm: number };
    background: Rect;
    bins: { centers_mm: Record<string, [number, number]>; size_mm: number; floor_z_mm: number; wall_mm: number };
    edge_x_mm?: number;
  };
  zones: Record<string, [number, number][]>;
  camera: { width: number; height: number; focal_px: number };
  item_radius_mm: number;
  cloth_n: number | null;
}
interface Item {
  id: number;
  color: string;
  location: string;
  xyz: [number, number, number];
  vertices?: number[];
}
interface TwinState {
  links: Record<string, number[]>;
  items: Item[];
  camera: number[] | null;
}
interface ItemEntry {
  mesh: THREE.Mesh;
  target: THREE.Vector3 | null;
  location: string | null;
}

const mat = (color: THREE.ColorRepresentation, extra: THREE.MeshStandardMaterialParameters = {}) =>
  new THREE.MeshStandardMaterial({ color, roughness: 0.85, metalness: 0, ...extra });

function rng(seed: number) {
  let s = (seed * 2654435761) >>> 0 || 1;
  return () => (s = (s * 1664525 + 1013904223) >>> 0) / 4294967296;
}

function clothGeometry(id: number) {
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

export class TwinScene {
  private renderer = new THREE.WebGLRenderer({ antialias: true });
  private scene = new THREE.Scene();
  private camera = new THREE.PerspectiveCamera(38, 1, 0.01, 20);
  private controls: OrbitControls;
  private world = new THREE.Group(); // robot frame: z up, metres
  private links: Record<string, THREE.Group> = {};
  private meshesToLoad = 0;
  private meshesLoaded = 0;
  private items = new Map<number, ItemEntry>();
  private itemRadius = 0.03;
  private clothN: number | null = null;
  private camIntr: Layout["camera"] | null = null;
  private simulated = false;
  private camBody: THREE.Mesh;
  private frustum: THREE.LineSegments;
  private footprint: THREE.Mesh;
  private resizeObserver: ResizeObserver;
  private timer: ReturnType<typeof setTimeout> | undefined;
  private disposed = false;
  private clock = new THREE.Clock();

  constructor(
    private container: HTMLElement,
    private onBadge: (b: Badge) => void,
  ) {
    const r = this.renderer;
    r.setPixelRatio(Math.min(devicePixelRatio || 1, 2));
    r.shadowMap.enabled = true;
    r.shadowMap.type = THREE.PCFSoftShadowMap;
    container.prepend(r.domElement);

    this.scene.background = new THREE.Color(0x121a24);
    this.scene.fog = new THREE.Fog(0x121a24, 2.5, 6);
    this.controls = new OrbitControls(this.camera, r.domElement);
    this.controls.enableDamping = true;
    this.resetView();
    r.domElement.addEventListener("dblclick", this.resetView);

    this.scene.add(new THREE.HemisphereLight(0xf2f6ff, 0x3a3228, 1.5));
    const sun = new THREE.DirectionalLight(0xffffff, 2.3);
    sun.position.set(0.6, 1.6, 0.9);
    sun.castShadow = true;
    sun.shadow.mapSize.set(2048, 2048);
    Object.assign(sun.shadow.camera, { left: -0.8, right: 0.8, top: 0.8, bottom: -0.8, near: 0.3, far: 4 });
    sun.shadow.bias = -0.0005;
    this.scene.add(sun);
    const fill = new THREE.DirectionalLight(0xcfe3ff, 0.6);
    fill.position.set(-1.2, 0.8, -0.8);
    this.scene.add(fill);

    this.world.rotation.x = -Math.PI / 2;
    this.scene.add(this.world);

    this.camBody = new THREE.Mesh(new THREE.BoxGeometry(0.025, 0.09, 0.025), mat(0x1b1e22, { roughness: 0.4 }));
    this.camBody.matrixAutoUpdate = false;
    this.camBody.visible = false;
    this.world.add(this.camBody);
    this.frustum = new THREE.LineSegments(
      new THREE.BufferGeometry().setAttribute("position", new THREE.Float32BufferAttribute(new Float32Array(16 * 3), 3)),
      new THREE.LineBasicMaterial({ color: 0x3cc8e2, transparent: true, opacity: 0.55 }),
    );
    this.frustum.frustumCulled = false;
    this.world.add(this.frustum);
    this.footprint = new THREE.Mesh(
      new THREE.BufferGeometry().setAttribute("position", new THREE.Float32BufferAttribute(new Float32Array(6 * 3), 3)),
      new THREE.MeshBasicMaterial({ color: 0x3cc8e2, transparent: true, opacity: 0.18, side: THREE.DoubleSide, depthWrite: false }),
    );
    this.footprint.frustumCulled = false;
    this.world.add(this.footprint);

    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(container);
    this.resize();

    // cloth that was dropped falls into place
    r.setAnimationLoop(() => {
      const k = 1 - Math.exp(-this.clock.getDelta() * 12);
      for (const e of this.items.values()) if (e.target) e.mesh.position.lerp(e.target, k);
      this.controls.update();
      r.render(this.scene, this.camera);
    });
    this.onBadge({ text: "loading", state: "" });
    this.start();
  }

  dispose(): void {
    this.disposed = true;
    clearTimeout(this.timer);
    this.resizeObserver.disconnect();
    this.renderer.setAnimationLoop(null);
    this.controls.dispose();
    this.scene.traverse((o) => {
      const m = o as THREE.Mesh;
      m.geometry?.dispose();
      const mats = Array.isArray(m.material) ? m.material : m.material ? [m.material] : [];
      for (const x of mats) {
        (x as THREE.MeshBasicMaterial).map?.dispose();
        x.dispose();
      }
    });
    this.renderer.dispose();
    this.renderer.domElement.remove();
  }

  resetView = (): void => {
    // robot frame (x fwd, y left, z up) → three (x, z, -y): from the front right, above
    this.camera.position.set(1.05, 0.85, 0.55);
    this.controls.target.set(0.08, 0.02, -0.02);
    this.controls.update();
  };

  private resize(): void {
    const w = this.container.clientWidth || 1;
    const h = this.container.clientHeight || 1;
    this.renderer.setSize(w, h);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  private badgeReady(): void {
    if (this.meshesToLoad && this.meshesLoaded === this.meshesToLoad) {
      this.onBadge({ text: this.simulated ? "sim" : "live", state: "ready" });
    }
  }

  private async start(): Promise<void> {
    try {
      const L = (await (await fetch("/api/twin/layout")).json()) as Layout;
      if (this.disposed) return;
      this.simulated = L.simulated;
      this.camIntr = L.camera;
      this.itemRadius = L.item_radius_mm * MM;
      this.clothN = L.cloth_n;
      this.buildTable(L);
    } catch {
      this.onBadge({ text: "no layout", state: "error" });
    }
    this.buildArm().catch(() => this.onBadge({ text: "mesh error", state: "error" }));
    this.poll();
  }

  private async poll(): Promise<void> {
    try {
      const r = await fetch("/api/twin/state", { cache: "no-store" });
      if (!r.ok) throw new Error(r.statusText);
      const S = (await r.json()) as TwinState;
      if (this.disposed) return;
      this.applyState(S);
      this.badgeReady();
    } catch {
      if (this.disposed) return;
      this.onBadge({ text: "no connection", state: "error" });
    }
    this.timer = setTimeout(() => this.poll(), 40);
  }

  // --- the table ---

  private block(sx: number, sy: number, sz: number, x: number, y: number, z: number, material: THREE.Material) {
    const m = new THREE.Mesh(new THREE.BoxGeometry(sx, sy, sz), material);
    m.position.set(x, y, z);
    m.castShadow = m.receiveShadow = true;
    this.world.add(m);
    return m;
  }

  // an open box: floor + four walls; `inner` = inside size, `t` = wall thickness
  private tray(
    [cx, cy]: number[],
    [ix, iy]: number[],
    floorZ: number,
    wallH: number,
    t: number,
    material: THREE.Material,
    floorMaterial = material,
  ) {
    const ox = ix + 2 * t,
      oy = iy + 2 * t,
      top = floorZ + wallH;
    this.block(ox, oy, floorZ, cx, cy, floorZ / 2, floorMaterial);
    this.block(t, oy, top, cx - ix / 2 - t / 2, cy, top / 2, material);
    this.block(t, oy, top, cx + ix / 2 + t / 2, cy, top / 2, material);
    this.block(ix, t, top, cx, cy - iy / 2 - t / 2, top / 2, material);
    this.block(ix, t, top, cx, cy + iy / 2 + t / 2, top / 2, material);
  }

  private label(text: string, x: number, y: number, z: number) {
    const c = document.createElement("canvas");
    const ctx = c.getContext("2d");
    if (!ctx) return;
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
    this.world.add(s);
  }

  private outline(points: [number, number][], z: number, color: number) {
    const pts = points.map(([x, y]) => new THREE.Vector3(x * MM, y * MM, z));
    const line = new THREE.LineLoop(
      new THREE.BufferGeometry().setFromPoints(pts),
      new THREE.LineDashedMaterial({ color, dashSize: 0.012, gapSize: 0.008 }),
    );
    line.computeLineDistances();
    this.world.add(line);
  }

  private buildTable(L: Layout) {
    const { box, background: bg, bins } = L.table;
    const centers = Object.values(bins.centers_mm);
    const xs = [box.center_mm[0], bg.center_mm[0], ...centers.map((c) => c[0])];
    const ys = [box.center_mm[1], bg.center_mm[1], ...centers.map((c) => c[1])];
    const x0 = (L.table.edge_x_mm ?? Math.min(...xs, 0) - 300) * MM,
      x1 = Math.max(...xs) * MM + 0.3;
    const y0 = Math.min(...ys) * MM - 0.3,
      y1 = Math.max(...ys) * MM + 0.3;
    this.block(x1 - x0, y1 - y0, 0.03, (x0 + x1) / 2, (y0 + y1) / 2, -0.015, mat(0xcdb18d, { roughness: 0.7 }));

    // the gray mat
    this.block(bg.size_mm[0] * MM, bg.size_mm[1] * MM, 0.002, bg.center_mm[0] * MM, bg.center_mm[1] * MM, 0.001, mat(0x7e7c7a));
    this.label("Mat", bg.center_mm[0] * MM + (bg.size_mm[0] / 2) * MM + 0.05, bg.center_mm[1] * MM, 0.03);

    // the mixed box
    const bc = box.center_mm.map((v) => v * MM);
    this.tray(bc, box.size_mm.map((v) => v * MM), box.floor_z_mm * MM, box.wall_mm * MM, 0.01, mat(0xd2ac7e), mat(0xb88c5c));
    this.label("Box", bc[0], bc[1] - (box.size_mm[1] / 2) * MM - 0.05, (box.floor_z_mm + box.wall_mm) * MM + 0.03);

    // the bins
    const inner = (bins.size_mm - 24) * MM;
    for (const [color, [x, y]] of Object.entries(bins.centers_mm)) {
      const m = mat(BIN_COLORS[color] ?? 0x888888, { roughness: 0.55 });
      this.tray([x * MM, y * MM], [inner, inner], bins.floor_z_mm * MM, bins.wall_mm * MM, 0.012, m);
      const r = Math.hypot(x, y);
      const out = (bins.size_mm / 2 + 45) / r;
      this.label(BIN_NAMES[color] ?? color, x * MM * (1 + out), y * MM * (1 + out), (bins.floor_z_mm + bins.wall_mm) * MM + 0.03);
    }

    // where a pick is allowed
    this.outline(L.zones.box ?? [], (box.floor_z_mm + 1) * MM, 0x3cc8e2);
    this.outline(L.zones.background ?? [], 0.003, 0x3cc8e2);
  }

  // --- the arm: CAD meshes per URDF link ---

  private async buildArm() {
    type Part = { mesh: string; color: string; xyz: [number, number, number]; rpy: [number, number, number] };
    const manifest = (await (await fetch("/twin-assets/manifest.json")).json()) as { links: Record<string, Part[]> };
    if (this.disposed) return;
    const loader = new STLLoader();
    for (const [name, parts] of Object.entries(manifest.links)) {
      const g = new THREE.Group();
      g.matrixAutoUpdate = false;
      this.world.add(g);
      this.links[name] = g;
      for (const p of parts) {
        this.meshesToLoad++;
        loader.load(
          "/twin-assets/" + p.mesh,
          (geo) => {
            if (this.disposed) return geo.dispose();
            const m = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ color: p.color, metalness: 0.25, roughness: 0.55 }));
            m.position.set(...p.xyz);
            m.rotation.set(p.rpy[0], p.rpy[1], p.rpy[2], "ZYX"); // URDF rpy = Rz·Ry·Rx
            m.castShadow = m.receiveShadow = true;
            g.add(m);
            this.meshesLoaded++;
            this.badgeReady();
          },
          undefined,
          () => this.onBadge({ text: "mesh error", state: "error" }),
        );
      }
    }
  }

  // --- items: crumpled cloth ---

  private clothMesh(color: string, n: number) {
    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.Float32BufferAttribute(new Float32Array(n * n * 3), 3));
    const idx: number[] = [];
    for (let i = 0; i < n - 1; i++) {
      for (let j = 0; j < n - 1; j++) {
        const a = i * n + j,
          b = a + 1,
          c = a + n,
          e = c + 1;
        idx.push(a, b, e, a, e, c);
      }
    }
    g.setIndex(idx);
    const m = new THREE.Mesh(g, mat(color, { roughness: 0.95, side: THREE.DoubleSide }));
    m.castShadow = m.receiveShadow = true;
    m.frustumCulled = false;
    return m;
  }

  private updateCloth(list: Item[], n: number) {
    for (const it of list) {
      const verts = it.vertices;
      if (!verts) continue;
      let e = this.items.get(it.id);
      if (!e) {
        e = { mesh: this.clothMesh(it.color, n), target: null, location: null };
        this.world.add(e.mesh);
        this.items.set(it.id, e);
      }
      const pos = e.mesh.geometry.attributes.position as THREE.BufferAttribute;
      for (let i = 0; i < verts.length; i++) (pos.array as Float32Array)[i] = verts[i] * MM;
      pos.needsUpdate = true;
      e.mesh.geometry.computeVertexNormals();
      e.location = it.location;
    }
  }

  private updateItems(list: Item[]) {
    if (this.clothN && list.length && list[0].vertices) return this.updateCloth(list, this.clothN);
    const seen = new Set<number>();
    for (const it of list) {
      seen.add(it.id);
      let e = this.items.get(it.id);
      if (!e) {
        const mesh = new THREE.Mesh(clothGeometry(it.id), mat(it.color, { roughness: 0.95 }));
        mesh.castShadow = mesh.receiveShadow = true;
        this.world.add(mesh);
        e = { mesh, target: new THREE.Vector3(), location: null };
        this.items.set(it.id, e);
      }
      const target = e.target ?? (e.target = new THREE.Vector3());
      const [x, y, z] = it.xyz.map((v) => v * MM);
      const held = it.location === "gripper";
      const r = this.itemRadius;
      if (held) {
        e.mesh.scale.set(r * 0.55, r * 0.55, r * 1.1); // hanging from the fingers
        target.set(x, y, z - r * 0.6);
      } else {
        e.mesh.scale.set(r * 1.2, r * 0.95, 0.011);
        target.set(x, y, z - 0.009);
      }
      if (held || e.location === null) e.mesh.position.copy(target); // grabbed: follows the TCP exactly
      e.location = it.location;
    }
    for (const [id, e] of this.items) {
      if (!seen.has(id)) {
        this.world.remove(e.mesh);
        e.mesh.geometry.dispose();
        this.items.delete(id);
      }
    }
  }

  // --- the wrist camera and what it sees on the table ---

  private updateCamera(T: number[] | null) {
    const intr = this.camIntr;
    const show = !!(T && intr);
    this.camBody.visible = this.frustum.visible = this.footprint.visible = show;
    if (!T || !intr) return;
    const M = new THREE.Matrix4().set(...(T as Parameters<THREE.Matrix4["set"]>));
    this.camBody.matrix.copy(M);
    this.camBody.matrixWorldNeedsUpdate = true;
    const o = new THREE.Vector3().setFromMatrixPosition(M);
    const R = new THREE.Matrix3().setFromMatrix4(M);
    const { width: w, height: h, focal_px: f } = intr;
    const corners = [
      [0, 0],
      [w, 0],
      [w, h],
      [0, h],
    ].map(([u, v]) => {
      const d = new THREE.Vector3((u - w / 2) / f, (v - h / 2) / f, 1).applyMatrix3(R);
      const t = d.z < -0.05 ? Math.min(-o.z / d.z, 1.5) : 0.35; // hit the table, or a short ray
      return o.clone().addScaledVector(d, t);
    });
    const fp = this.frustum.geometry.attributes.position as THREE.BufferAttribute;
    corners.forEach((c, i) => {
      const n = corners[(i + 1) % 4];
      fp.setXYZ(i * 4, o.x, o.y, o.z);
      fp.setXYZ(i * 4 + 1, c.x, c.y, c.z);
      fp.setXYZ(i * 4 + 2, c.x, c.y, c.z);
      fp.setXYZ(i * 4 + 3, n.x, n.y, n.z);
    });
    fp.needsUpdate = true;
    const tri = [0, 1, 2, 0, 2, 3].map((i) => corners[i]);
    const pp = this.footprint.geometry.attributes.position as THREE.BufferAttribute;
    tri.forEach((c, i) => pp.setXYZ(i, c.x, c.y, Math.max(c.z, 0) + 0.004));
    pp.needsUpdate = true;
  }

  private applyState(S: TwinState) {
    for (const [name, m] of Object.entries(S.links)) {
      const g = this.links[name];
      if (!g) continue;
      g.matrix.set(...(m as Parameters<THREE.Matrix4["set"]>));
      g.matrixWorldNeedsUpdate = true;
    }
    this.updateItems(S.items);
    this.updateCamera(S.camera);
  }
}
