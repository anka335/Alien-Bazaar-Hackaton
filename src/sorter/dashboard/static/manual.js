// Manual control page (setup mode, `python -m sorter manual`). State is polled from /api/manual;
// every button posts one action there. Hold goes through /api/command like on the main page.
"use strict";

const $ = (id) => document.getElementById(id);
const deg = (rad) => (rad * 180) / Math.PI;
let state = null;
let built = false;

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

const act = (action, extra = {}) => post("/api/manual", { action, ...extra });

function hold() {
  $("hold").dataset.engaged = "";
  post("/api/command", { cmd: "hold" });
}

let localError = null;
function showError(msg) {
  localError = msg;
  setTimeout(() => { if (localError === msg) localError = null; }, 4000);
  render();
}

function build(s) {
  const poses = $("poses");
  const save = $("save-name");
  for (const name of Object.keys(s.poses)) {
    const b = document.createElement("button");
    b.type = "button";
    b.textContent = name;
    b.dataset.pose = name;
    b.dataset.busy = "";
    b.addEventListener("click", () => act("go", { pose: name }));
    poses.append(b);
    save.append(new Option(name, name));
  }
  save.value = "look_box";

  const jog = $("jog");
  for (let j = 0; j < 6; j++) {
    const name = document.createElement("span");
    name.className = "name";
    name.textContent = `J${j + 1}`;
    const val = document.createElement("span");
    val.className = "val";
    val.id = `j${j}`;
    const [minus, plus] = [-1, 1].map((sign) => {
      const b = document.createElement("button");
      b.type = "button";
      b.textContent = sign < 0 ? "−" : "+";
      b.setAttribute("aria-label", `J${j + 1} ${sign < 0 ? "minus" : "plus"}`);
      b.dataset.busy = "";
      b.addEventListener("click", () => {
        const step = Number(document.querySelector("input[name=step]:checked").value);
        act("jog", { joint: j, delta_deg: sign * step });
      });
      return b;
    });
    jog.append(name, minus, val, plus);
  }
  $("rig").textContent = s.rig_file;
  built = true;
}

function renderTour(s) {
  const next = s.tour.indexOf(s.tour_next);
  $("tour").replaceChildren(
    ...s.tour.map((name, i) => {
      const li = document.createElement("li");
      li.dataset.state = i === next ? "next" : i < next ? "done" : "";
      li.append(document.createElement("i"), name);
      return li;
    }),
  );
  $("tour-next").textContent = `Next: ${s.tour_next}`;
}

function render() {
  const s = state;
  if (!s) return;
  if (!built) build(s);

  const stuck = s.held || !!s.fault;
  let phase = "Ready";
  if (s.fault) phase = "Fault";
  else if (s.held) phase = "Held";
  else if (s.busy) phase = `Moving: ${s.action}`;
  $("phase").textContent = phase;
  $("phase").toggleAttribute("data-alarm", stuck);
  $("mode").textContent = s.fault ? "Fault" : s.held ? "Held" : s.busy ? "Moving" : "Manual";
  $("mode").dataset.mode = stuck ? "paused" : s.busy ? "running" : "idle";
  $("where").textContent = s.at ? `at ${s.at}` : s.last ? `last: ${s.last}` : "";
  $("hold").toggleAttribute("data-engaged", s.held);

  $("tcp").textContent = s.tcp_mm.map((v) => v.toFixed(0)).join(" · ");
  $("grip").textContent = s.gripper.toFixed(2);
  s.joints.forEach((q, j) => { $(`j${j}`).textContent = `${deg(q).toFixed(1)}°`; });

  for (const b of document.querySelectorAll("[data-busy]")) b.disabled = s.busy || stuck;
  $("release").disabled = !s.held || !!s.fault || s.busy;
  $("fault").hidden = !s.fault;
  $("fault-msg").textContent = s.fault ?? "";
  $("clear-fault").disabled = s.busy;
  for (const b of document.querySelectorAll("[data-pose]")) b.toggleAttribute("data-at", b.dataset.pose === s.at);
  renderTour(s);

  // A fault has its own banner; the failed action's error just repeats it.
  const err = localError ?? (s.fault ? null : s.held ? "Arm held. Press Release hold, then move on." : s.error);
  $("error").hidden = !err;
  $("error").textContent = err ?? "";
}

async function poll() {
  try {
    const r = await fetch("/api/manual", { cache: "no-store" });
    if (r.status === 404) {
      $("phase").textContent = "Manual control is off";
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

$("hold").addEventListener("click", hold);
$("tour-next").addEventListener("click", () => act("tour_next"));
$("tour-reset").addEventListener("click", () => act("tour_reset"));
$("grip-open").addEventListener("click", () => act("gripper", { open: true }));
$("grip-close").addEventListener("click", () => act("gripper", { open: false }));
$("release").addEventListener("click", () => act("release"));
$("clear-fault").addEventListener("click", () => {
  const ok = confirm(
    "Check the arm first: nothing blocks it and the joints turn freely.\n\n" +
      "Clear the fault? The arm stays powered and holds where it is now.",
  );
  if (ok) act("clear_fault");
});
$("save").addEventListener("click", async () => {
  const name = $("save-name").value;
  if (!confirm(`Overwrite pose "${name}" with the current joints?`)) return;
  const r = await act("save", { pose: name });
  if (r) $("save-out").textContent = `Saved: ${r.line.trim()}`;
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
