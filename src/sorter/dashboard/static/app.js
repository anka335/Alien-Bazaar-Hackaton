// Laundry sorter dashboard. Status arrives over /ws; if the socket is down, /api/status is polled.
// PHASE_LABELS is injected by the server (sorter/dashboard/render.py).
"use strict";

const $ = (id) => document.getElementById(id);
const MODES = { idle: "Idle", running: "Running", paused: "Paused" };
const STOPPED = ["idle", "done", "held", "error"];

// When each command does something; mirrors StateMachine._apply.
const ENABLED = {
  start: (s) => s.mode === "idle",
  pause: (s) => s.mode === "running",
  resume: (s) => s.mode === "paused" && s.phase !== "held",
  step: (s) => s.mode === "paused" && !STOPPED.includes(s.phase),
  stop: (s) => s.mode !== "idle",
  reset: (s) => s.phase === "held" || s.phase === "error",
};

// Folded clothes in each bin's stack, cycled per item.
const CLOTH = {
  light: ["#f8f5ef", "#e6ecf2", "#f3e7d6", "#dde2e8"],
  dark: ["#2b2e33", "#22314a", "#3d3631", "#1b1d22"],
  colored: ["#e0473f", "#2f7fd6", "#3fa35a", "#f0c23b", "#ef7d2d", "#8a4fa0", "#e889b8"],
};

let socketOpen = false;
let lastCounters = null;
let lastPhase = null;
const visited = new Set(); // program lights already passed for the current item

async function send(cmd) {
  try {
    const r = await fetch("/api/command", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ cmd }),
    });
    if (!r.ok) throw new Error(r.statusText);
  } catch {
    setOnline(false);
  }
}

function hold() {
  $("hold").dataset.engaged = "";
  send("hold");
}

function setOnline(online) {
  $("conn").hidden = online;
}

const phaseLabel = (p) => PHASE_LABELS[p] ?? p;

function formatAge(s) {
  if (s < 60) return `${Math.round(s)} s`;
  if (s < 3600) return `${Math.round(s / 60)} min`;
  return `${Math.round(s / 3600)} h`;
}

function renderProgram(phase) {
  if (phase !== lastPhase) {
    // a new item starts at the box, or at the work area when it isn't coming from the box
    if (phase === "look_box" || (phase === "look_bg" && lastPhase !== "place_on_bg")) visited.clear();
    if (STOPPED.includes(phase) && phase !== "held") visited.clear();
    visited.add(phase);
    lastPhase = phase;
  }
  for (const li of document.querySelectorAll("#program li[data-phase]")) {
    const p = li.dataset.phase;
    li.dataset.state = p === phase ? "now" : visited.has(p) ? "done" : "";
  }
}

function renderBins(counters) {
  for (const [bin, n] of Object.entries(counters)) {
    const count = $(`count-${bin}`);
    const stack = $(`stack-${bin}`);
    if (!count || !stack) continue;
    count.textContent = n;
    while (stack.children.length > n) stack.lastChild.remove();
    while (stack.children.length < n) {
      const cloth = document.createElement("span");
      const colors = CLOTH[bin];
      cloth.style.background = colors[stack.children.length % colors.length];
      if (lastCounters) cloth.className = "new";
      stack.append(cloth);
    }
    if (lastCounters && n > (lastCounters[bin] ?? 0)) bump(count.closest(".bin"));
  }
  lastCounters = counters;
}

function renderEvents(s) {
  $("events").replaceChildren(
    ...s.events.slice().reverse().map((e) => {
      const li = document.createElement("li");
      li.dataset.level = e.level;
      const age = document.createElement("span");
      age.className = "age";
      age.textContent = formatAge(Math.max(0, s.now - e.t));
      const msg = document.createElement("span");
      msg.className = "msg";
      msg.title = `${e.source}: ${e.msg}`;
      msg.textContent = e.msg;
      li.append(age, msg);
      return li;
    }),
  );
}

function render(s) {
  const alarm = s.phase === "held" || s.phase === "error";
  $("phase").textContent = phaseLabel(s.phase);
  $("phase").toggleAttribute("data-alarm", alarm);
  $("next").textContent = s.next_phase && s.mode !== "running" && s.next_phase !== s.phase
    ? `Next: ${phaseLabel(s.next_phase).toLowerCase()}`
    : "";
  $("mode").textContent = MODES[s.mode] ?? s.mode;
  $("mode").dataset.mode = s.mode;
  $("hold").toggleAttribute("data-engaged", s.phase === "held");
  renderProgram(s.phase);

  $("error").hidden = !s.error;
  $("error").textContent = s.error ? `Stopped: ${s.error}. Fix the cause, then press Reset or Resume.` : "";
  $("health").hidden = !s.health.length;
  $("health").textContent = s.health.join(" ");

  renderBins(s.counters);
  $("cycle").textContent = s.cycle;
  $("cycle-time").textContent = s.last_cycle_s == null ? "–" : `${s.last_cycle_s.toFixed(1)} s`;
  $("failures").textContent = s.failures;
  $("failures").toggleAttribute("data-alarm", s.failures > 0);

  for (const b of document.querySelectorAll("[data-cmd]")) {
    b.disabled = !ENABLED[b.dataset.cmd](s);
  }
  renderEvents(s);
}

function bump(el) {
  el.classList.remove("bump");
  void el.offsetWidth; // restart the animation
  el.classList.add("bump");
}

function connect() {
  const ws = new WebSocket(`${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/ws`);
  ws.onopen = () => { socketOpen = true; setOnline(true); };
  ws.onmessage = (e) => render(JSON.parse(e.data));
  ws.onclose = () => { socketOpen = false; setTimeout(connect, 1000); };
}

// Fallback while the socket is down, and the way we notice the server is gone.
setInterval(async () => {
  if (socketOpen) return;
  try {
    const r = await fetch("/api/status", { cache: "no-store" });
    if (!r.ok) throw new Error(r.statusText);
    render(await r.json());
    setOnline(true);
  } catch {
    setOnline(false);
  }
}, 700);

// MJPEG streams end when the server restarts; reconnect them.
for (const id of ["decision", "live"]) {
  const img = $(id);
  const base = img.getAttribute("src");
  img.addEventListener("error", () => {
    setTimeout(() => { img.src = `${base}?t=${Date.now()}`; }, 1000);
  });
}

$("hold").addEventListener("click", hold);
for (const b of document.querySelectorAll("[data-cmd]")) {
  b.addEventListener("click", () => send(b.dataset.cmd));
}
// Space or Esc anywhere holds the arm. preventDefault keeps Space from also clicking a focused button.
document.addEventListener("keydown", (e) => {
  if (e.code === "Space" || e.key === "Escape") {
    e.preventDefault();
    hold();
  }
});

connect();
