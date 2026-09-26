// Camera calibration page (setup mode, `python -m sorter manual`). State is polled from
// /api/calibrate; every button posts one action there. Hold goes through /api/command, release
// through /api/manual, like on the manual page.
"use strict";

const $ = (id) => document.getElementById(id);
const SVG = "http://www.w3.org/2000/svg";
let state = null;
let built = false;
// the wizard, kept per browser: step 1..4, the mark of step 1, the view of step 2
const saved = (() => { try { return JSON.parse(localStorage.getItem("calibrate") ?? "{}"); } catch { return {}; } })();
let step = Number(new URLSearchParams(location.search).get("step")) || saved.step || 1; // ?step=N opens a step
let markIdx = saved.markIdx ?? 0;
let viewIdx = saved.viewIdx ?? 0;
let skipped = new Set(); // marks skipped from the current view
let pick = null; // the mark the next click in the image is for
let lastSeen; // the arm's last action when step 2 last rendered

function remember() {
  try { localStorage.setItem("calibrate", JSON.stringify({ step, markIdx, viewIdx })); } catch { /* private window */ }
}

async function post(url, body) {
  try {
    const r = await fetch(url, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    });
    const data = await r.json().catch(() => ({}));
    if (!r.ok) showError(data.detail ?? r.statusText);
    return r.ok ? data : null;
  } catch {
    $("conn").hidden = false;
    return null;
  }
}

const act = (action, extra = {}) => post("/api/calibrate", { action, ...extra });

function hold() {
  $("hold").dataset.engaged = "";
  post("/api/command", { cmd: "hold" });
}

let localError = null;
function showError(msg) {
  localError = msg;
  setTimeout(() => { if (localError === msg) localError = null; }, 6000);
  render();
}

function el(tag, attrs = {}, text) {
  const e = document.createElementNS(SVG, tag);
  for (const [k, v] of Object.entries(attrs)) e.setAttribute(k, v);
  if (text !== undefined) e.textContent = text;
  return e;
}

function button(label, onClick, busy = true) {
  const b = document.createElement("button");
  b.type = "button";
  b.textContent = label;
  if (busy) b.dataset.busy = "";
  b.addEventListener("click", onClick);
  return b;
}

function goStep(n) {
  step = n;
  remember();
  render();
}

// --- build once ---

function buildMap(s) {
  // seen from behind the arm: +x (away from the arm) is up, +y (left) is left
  const xs = s.marks.map((m) => m.xyz[0]);
  const ys = s.marks.map((m) => m.xyz[1]);
  const pad = 22;
  const [x0, x1, y0, y1] = [Math.min(...xs) - pad, Math.max(...xs) + pad, Math.min(...ys) - pad, Math.max(...ys) + pad];
  const [w, h] = [y1 - y0, x1 - x0]; // 1 unit = 1 mm
  const at = (x, y) => [y1 - y, x1 - x];
  const map = $("map");
  map.setAttribute("viewBox", `-2 -2 ${w + 4} ${h + 18}`);
  map.append(el("rect", { x: 0, y: 0, width: w, height: h, rx: 8, class: "mat" }));
  map.append(el("text", { x: w / 2, y: h + 14, class: "arm" }, "arm ↓"));
  s.marks.forEach((m, i) => {
    const [cx, cy] = at(m.xyz[0], m.xyz[1]);
    const g = el("g", { class: "map-mark", "data-mark": m.name });
    g.append(el("rect", { x: cx - 6, y: cy - 6, width: 12, height: 12, rx: 2 }));
    g.append(el("text", { x: cx, y: cy - 10 }, m.name));
    g.addEventListener("click", () => { markIdx = i; remember(); render(); });
    map.append(g);
  });
}

function build(s) {
  buildMap(s);
  $("hover").textContent = s.hover_mm;
  $("he-file").textContent = s.hand_eye_file;
  $("rig-file").textContent = s.rig_file;
  markIdx = Math.min(markIdx, s.marks.length - 1);
  viewIdx = Math.min(viewIdx, s.views - 1);
  built = true;
}

// --- render ---

function renderOverlay(s) {
  const svg = $("overlay");
  if (!s.image) {
    svg.replaceChildren();
    return;
  }
  const { width: w, height: h } = s.image;
  svg.setAttribute("viewBox", `0 0 ${w} ${h}`);
  const clip = el("clipPath", { id: "img-clip" });
  clip.append(el("rect", { x: 0, y: 0, width: w, height: h }));
  const layer = el("g", { "clip-path": "url(#img-clip)" });
  const parts = [];
  for (const [zone, pts] of Object.entries(s.overlay.zones)) {
    parts.push(el("polygon", { points: pts.map((p) => p.join(",")).join(" "), class: `zone zone-${zone}` }));
  }
  const clicked = new Set(s.clicks.filter((c) => c.here).map((c) => c.mark));
  for (const m of s.overlay.marks) {
    if (!m.px) continue;
    const [u, v] = m.px;
    const g = el("g", { class: "mark" });
    if (m.name === pick) g.classList.add("picked");
    if (clicked.has(m.name)) g.classList.add("done");
    g.append(el("circle", { cx: u, cy: v, r: 11 }));
    g.append(el("text", { x: u + 14, y: v - 10 }, m.name));
    parts.push(g);
  }
  for (const c of s.clicks.filter((c) => c.here)) {
    const [u, v] = c.px;
    const g = el("g", { class: "click" });
    g.append(el("path", { d: `M${u - 9} ${v}h18M${u} ${v - 9}v18` }));
    parts.push(g);
  }
  layer.append(...parts);
  svg.replaceChildren(el("defs"), layer);
  svg.firstChild.append(clip);
  $("legend").textContent =
    `Circles: marks where the ${s.overlay_mount === "fit" ? "new fit" : "mount in use"} expects them · crosses: your clicks here`;
}

const inImage = (s, px) => px && px[0] >= 20 && px[1] >= 20 && px[0] < s.image.width - 20 && px[1] < s.image.height - 20;

function nextPick(s) {
  if (!s.image) return null;
  const done = new Set(s.clicks.filter((c) => c.here).map((c) => c.mark));
  const m = s.overlay.marks.find((o) => !done.has(o.name) && !skipped.has(o.name) && inImage(s, o.px));
  return m ? m.name : null;
}

function fitText(s) {
  const f = s.fit;
  const views = new Set(s.clicks.map((c) => c.pose)).size;
  if (!f) return `No fit yet: click at least 3 marks (${new Set(s.clicks.map((c) => c.mark)).size} so far).`;
  let t = `Fit over ${f.marks} marks from ${views} view${views === 1 ? "" : "s"}: RMSE ${f.rmse_mm.toFixed(1)} mm` +
    ` (${f.rmse_mm <= 3 ? "good" : f.rmse_mm <= 6 ? "fair: check the clicks with the biggest error" : "poor: a wrong click?"}).` +
    ` ${f.change_mm.toFixed(0)} mm, ${f.change_deg.toFixed(1)}° from the mount in use.`;
  if (f.true_error_mm !== undefined) t += ` Sim: ${f.true_error_mm.toFixed(1)} mm, ${f.true_error_deg.toFixed(2)}° off the true mount.`;
  if (views < 2) t += " Add a second view for a reliable rotation.";
  const per = (f.views ?? []).filter((v) => v.rmse_mm !== null);
  if (per.length) t += ` Per view, without the arm: ${per.map((v) => `view ${v.view} ${v.rmse_mm.toFixed(1)} mm`).join(", ")}` +
    (f.rmse_mm > 6 && per.every((v) => v.rmse_mm <= 3) ? " (each view agrees with the tape: the arm pose differs between views)." : ".");
  if (!f.plausible) t = `Wrong fit: it puts the camera far from where it is mounted, so a click is on the wrong mark. Delete the clicks with the biggest error (All clicks). ${t}`;
  if (f.fixed_views?.length) t += ` View ${f.fixed_views.join(", ")} was clicked mirrored (M2↔M3, M4↔M5 look alike): relabeled.`;
  return t;
}

function renderStep1(s, a) {
  const m = s.marks[markIdx];
  const placed = s.marks.filter((x) => x.placed).length;
  $("mark-count").textContent = `${placed} of ${s.marks.length} recorded`;
  const here = a.last === `to mark ${m.name}` && !a.busy && !a.error;
  $("mark-task").textContent = here
    ? `The tip is over ${m.name}. Put the tape right under it (or check it is there), then Next mark.`
    : a.busy && a.action === `to mark ${m.name}`
      ? `Moving to ${m.name}…`
      : `Mark ${markIdx + 1} of ${s.marks.length}: ${m.name}${m.placed ? " (recorded)" : ""}. Press Move.`;
  $("mark-go").textContent = `Move tip to ${m.name}`;
  $("mark-next").textContent = markIdx === s.marks.length - 1 ? "All done" : "Next mark";
  $("mark-back").disabled = markIdx === 0;
  $("to-2").textContent = placed === s.marks.length ? "Continue" : `Continue (${placed} of ${s.marks.length})`;
  for (const g of document.querySelectorAll(".map-mark")) {
    g.classList.toggle("picked", g.dataset.mark === m.name);
    g.classList.toggle("placed", s.marks.some((x) => x.name === g.dataset.mark && x.placed));
  }
}

function renderStep2(s, a) {
  // the arm just arrived at a view (from here or another tab): follow it, once per arrival
  const at = a.last !== lastSeen ? /^view (\d+)$/.exec(a.last ?? "") : null;
  lastSeen = a.last;
  if (at && Number(at[1]) - 1 !== viewIdx) {
    viewIdx = Number(at[1]) - 1;
    skipped = new Set();
    remember();
  }
  $("view-count").textContent = `view ${viewIdx + 1} of ${s.views}`;
  $("view-go").textContent = `Move camera to view ${viewIdx + 1}`;
  const atView = a.last === `view ${viewIdx + 1}` && !a.busy;
  const clickedHere = s.clicks.filter((c) => c.here).map((c) => c.mark);
  let task;
  if (a.busy) task = `Moving: ${a.action}…`;
  else if (!atView && !clickedHere.length && !a.last?.startsWith("view")) task = `Press "Move camera to view ${viewIdx + 1}".`;
  else if (clickedHere.length && s.detected && s.detected.marks.length) {
    task = `Found ${s.detected.marks.join(", ")} by themselves. Check that every circle sits on its tape square.` +
      (pick ? ` ${pick} was not found: click its square, or skip it.` : " Then Next view.");
  } else if (s.detected && !s.detected.marks.length && atView) {
    task = `No marks found (${s.detected.squares} dark squares). Click them by hand, starting with ${pick ?? "any"}, or move the camera.`;
  } else if (pick && !s.fit) {
    task = `Click the tape square of ${pick} in the image. The circles are only a guess until 3 marks are clicked: find ${pick} by the map above (M1 and M6 are the close pair in the middle, M6 toward M2 / M3).`;
  } else if (pick) task = `Click the tape square of ${pick} (its circle should be on it now).`;
  else task = viewIdx < s.views - 1 ? "Every mark in sight is clicked. Next view." : "Every mark in sight is clicked. Continue.";
  $("click-task").textContent = task;
  $("click-skip").disabled = !pick;
  $("view-next").disabled = viewIdx >= s.views - 1 || a.busy || a.held || !!a.fault;
  $("view-next").textContent = viewIdx >= s.views - 1 ? "Last view" : `Next view (${viewIdx + 2})`;
  $("clicked").replaceChildren(
    ...clickedHere.map((n) => {
      const span = document.createElement("span");
      span.textContent = `✓ ${n}`;
      return span;
    }),
  );
  $("fit").textContent = fitText(s);
  $("fit").dataset.quality = !s.fit ? "" : !s.fit.plausible ? "bad" : s.fit.rmse_mm <= 3 ? "good" : s.fit.rmse_mm <= 6 ? "ok" : "bad";
  $("click-n").textContent = s.clicks.length;
  $("clicks").replaceChildren(
    ...s.clicks.map((c, i) => {
      const li = document.createElement("li");
      if (c.here) li.dataset.here = "";
      const res = c.residual_mm === null ? "" : ` · ${c.residual_mm.toFixed(1)} mm`;
      const text = document.createElement("span");
      text.textContent = `${c.mark}, view ${c.pose + 1}${res}`;
      const del = button("×", () => act("delete_click", { index: i }), false);
      del.setAttribute("aria-label", `Delete click ${i + 1} (${c.mark})`);
      li.append(text, del);
      return li;
    }),
  );
  $("clear-clicks").disabled = !s.clicks.length;
  $("to-3").disabled = !s.fit || !s.fit.plausible;
}

function renderStep3(s, a) {
  $("fit-3").textContent = fitText(s);
  $("fit-3").dataset.quality = !s.fit ? "" : s.fit.rmse_mm <= 3 ? "good" : s.fit.rmse_mm <= 6 ? "ok" : "bad";
  $("calibrate").disabled = !s.fit || !s.fit.plausible || a.busy || a.held || !!a.fault;
  $("calibrate").textContent = a.busy ? "Wait for the arm…" : "Calibrate";
}

function renderStep4(s, a) {
  $("done").textContent = s.mount.saved && s.look_saved
    ? `Calibrated: ${s.mount.saved} and ${s.look_saved}.`
    : "Not calibrated yet in this session.";
  $("look").hidden = !s.look;
  if (!s.look) return;
  $("look-rows").replaceChildren(
    ...Object.entries(s.look).map(([name, c]) => {
      const tr = document.createElement("tr");
      const zone = c.fits ? "all" : `${Math.round(c.coverage * 100)}%${c.depth_ok ? "" : ", too close"}`;
      for (const t of [name, `${c.camera_mm} mm`, c.sees_mm.join(" × "), zone]) {
        const td = document.createElement("td");
        td.textContent = t;
        tr.append(td);
      }
      if (!c.fits) tr.dataset.warn = "";
      const td = document.createElement("td");
      const b = button(a.last === `preview ${name}` ? "Again" : "Preview", () => act("goto_look", { pose: name }));
      b.disabled = a.busy || a.held || !!a.fault;
      td.append(b);
      tr.append(td);
      return tr;
    }),
  );
}

function render() {
  const s = state;
  if (!s) return;
  if (!built) build(s);
  const a = s.arm;

  const stuck = a.held || !!a.fault;
  let phase = "Ready";
  if (a.fault) phase = "Fault";
  else if (a.held) phase = "Held";
  else if (a.busy) phase = `Moving: ${a.action}`;
  $("phase").textContent = phase;
  $("phase").toggleAttribute("data-alarm", stuck);
  $("mode").textContent = a.fault ? "Fault" : a.held ? "Held" : a.busy ? "Moving" : "Calibrate";
  $("mode").dataset.mode = stuck ? "paused" : a.busy ? "running" : "idle";
  $("where").textContent = a.at ? `at ${a.at}` : a.last ? `last: ${a.last}` : "";
  $("hold").toggleAttribute("data-engaged", a.held);
  $("tcp").textContent = a.tcp_mm.map((v) => v.toFixed(0)).join(" · ");
  $("mount").textContent = s.mount.method;

  for (const sec of document.querySelectorAll(".step")) sec.hidden = Number(sec.dataset.step) !== step;
  $("map-card").hidden = step > 2;
  for (const b of document.querySelectorAll("#stepper [data-goto]")) {
    b.toggleAttribute("aria-current", Number(b.dataset.goto) === step);
  }
  pick = step === 2 ? nextPick(s) : null;
  if (step === 2) {
    const clicked = new Set(s.clicks.filter((c) => c.here).map((c) => c.mark));
    for (const g of document.querySelectorAll(".map-mark")) {
      g.classList.toggle("picked", g.dataset.mark === pick);
      g.classList.toggle("placed", clicked.has(g.dataset.mark));
    }
  }
  if (step === 1) renderStep1(s, a);
  if (step === 2) renderStep2(s, a);
  if (step === 3) renderStep3(s, a);
  if (step === 4) renderStep4(s, a);

  for (const b of document.querySelectorAll("[data-busy]")) b.disabled = a.busy || stuck;
  $("overlay").toggleAttribute("data-disabled", a.busy || stuck || !pick);
  $("fault").hidden = !a.fault;
  $("fault-msg").textContent = a.fault ?? "";
  $("clear-fault").disabled = a.busy;
  $("held").hidden = !a.held || !!a.fault;
  renderOverlay(s);

  const err = localError ?? (a.fault || a.held ? null : a.error);
  $("error").hidden = !err;
  $("error").textContent = err ?? "";
}

async function poll() {
  try {
    const r = await fetch("/api/calibrate", { cache: "no-store" });
    if (r.status === 404) {
      $("phase").textContent = "Calibration is off";
      $("error").hidden = false;
      $("error").textContent = "Start the sorter with: python -m sorter manual";
    } else if (r.ok) {
      state = await r.json();
      $("conn").hidden = true;
      render();
    } else {
      throw new Error(r.statusText);
    }
  } catch {
    $("conn").hidden = false;
  }
  setTimeout(poll, 250);
}

// A click in the image: the picked mark is there; the next one is picked by render().
$("overlay").addEventListener("click", async (e) => {
  const s = state;
  if (!s || !s.image || !pick || s.arm.busy || s.arm.held) return;
  const svg = $("overlay");
  const p = new DOMPoint(e.clientX, e.clientY).matrixTransform(svg.getScreenCTM().inverse());
  if (p.x < 0 || p.y < 0 || p.x >= s.image.width || p.y >= s.image.height) return;
  await act("click", { mark: pick, u: p.x, v: p.y });
});

for (const b of document.querySelectorAll("[data-goto]")) {
  b.addEventListener("click", () => goStep(Number(b.dataset.goto)));
}
$("hold").addEventListener("click", hold);
$("release").addEventListener("click", () => post("/api/manual", { action: "release" }));
$("clear-fault").addEventListener("click", () => {
  const ok = confirm(
    "Check the arm first: nothing blocks it, the camera cable is slack and the joints turn freely.\n\n" +
      "Clear the fault? The arm stays powered and holds where it is now.",
  );
  if (ok) post("/api/manual", { action: "clear_fault" });
});

$("mark-go").addEventListener("click", () => act("goto_mark", { mark: state.marks[markIdx].name }));
$("mark-next").addEventListener("click", () => {
  if (markIdx < state.marks.length - 1) {
    markIdx += 1;
    remember();
    render();
  } else goStep(2);
});
$("mark-back").addEventListener("click", () => { markIdx = Math.max(markIdx - 1, 0); remember(); render(); });
$("to-2").addEventListener("click", () => goStep(2));

$("view-go").addEventListener("click", () => { skipped = new Set(); act("goto_view", { index: viewIdx }); });
$("view-next").addEventListener("click", () => {
  viewIdx = Math.min(viewIdx + 1, state.views - 1);
  skipped = new Set();
  remember();
  render();
  act("goto_view", { index: viewIdx });
});
$("detect").addEventListener("click", () => act("detect"));
$("click-skip").addEventListener("click", () => { if (pick) skipped.add(pick); render(); });
$("clear-clicks").addEventListener("click", () => {
  if (confirm("Delete every click?")) { viewIdx = 0; remember(); act("clear_clicks"); }
});
$("to-3").addEventListener("click", () => goStep(3));

$("calibrate").addEventListener("click", async () => {
  const r = await act("calibrate");
  if (r) goStep(4);
});

// Space or Esc anywhere holds the arm. preventDefault keeps Space from also clicking a focused button.
document.addEventListener("keydown", (e) => {
  if (e.code === "Space" || e.key === "Escape") {
    e.preventDefault();
    hold();
  }
});

// The MJPEG stream ends when the server restarts; reconnect it.
const live = $("live");
live.addEventListener("error", () => {
  setTimeout(() => { live.src = `/stream/live.mjpg?t=${Date.now()}`; }, 1000);
});

poll();
