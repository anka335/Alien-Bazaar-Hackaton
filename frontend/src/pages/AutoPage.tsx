// The sorting tab (auto mode): the state machine's phase and program lights, the decision frame
// or the 3D view, the wrist camera, the bins, the run controls and the warnings.
import { useEffect, useRef, useState, type ReactNode } from "react";
import { MODE_LABELS, isRunMode, postJSON, type Bin, type OperatorMode, type Status } from "../api";
import { LiveImage } from "../components/common";
import { usePath, useStored } from "../hooks";
import { useSorter } from "../sorter";
import { TwinView } from "../twin/TwinView";

const MODES: Record<string, string> = { idle: "Idle", running: "Running", paused: "Paused" };
const STOPPED = ["idle", "done", "held", "error"];

// When each command does something; mirrors StateMachine._apply.
const COMMANDS: { cmd: string; label: string; title: string; icon: ReactNode; enabled: (s: Status) => boolean }[] = [
  { cmd: "start", label: "Start", title: "Start a new run", icon: <path d="M4 2.5v11l9-5.5z" />, enabled: (s) => s.mode === "idle" },
  { cmd: "pause", label: "Pause", title: "Pause after the current step", icon: <path d="M4 2.5h3v11H4zM9 2.5h3v11H9z" />, enabled: (s) => s.mode === "running" },
  { cmd: "resume", label: "Resume", title: "Continue the run", icon: <path d="M4 2.5v11l9-5.5z" />, enabled: (s) => s.mode === "paused" && s.phase !== "held" },
  { cmd: "step", label: "Step", title: "Run one step, then pause", icon: <path d="M3 2.5v11l7-5.5zM11 2.5h2.5v11H11z" />, enabled: (s) => s.mode === "paused" && !STOPPED.includes(s.phase) },
  { cmd: "stop", label: "Stop", title: "Stop the arm now, go home, end the run", icon: <path d="M3.5 3.5h9v9h-9z" />, enabled: (s) => s.mode !== "idle" },
  {
    cmd: "reset",
    label: "Reset",
    title: "After a hold or error: lift, open the gripper, go home, continue",
    icon: <path d="M8 2.5a5.5 5.5 0 1 1-5.2 3.7l1.9.7A3.5 3.5 0 1 0 8 4.5V7L4.5 3.5 8 0z" />,
    enabled: (s) => s.phase === "held" || s.phase === "error",
  },
];

const PROGRAM = [
  { group: "From the box", steps: [["look_box", "Look"], ["sense_box", "Choose"], ["pick_from_box", "Grab"], ["place_on_bg", "Lay out"]] },
  { group: "Sorting", steps: [["look_bg", "Look"], ["sense_bg", "Check color"], ["pick_from_bg", "Pick up"], ["drop_to_bin", "Into bin"]] },
];

// Folded clothes in each bin's stack, cycled per item.
const CLOTH: Record<Bin, string[]> = {
  light: ["#f8f5ef", "#e6ecf2", "#f3e7d6", "#dde2e8"],
  dark: ["#2b2e33", "#22314a", "#3d3631", "#1b1d22"],
  colored: ["#e0473f", "#2f7fd6", "#3fa35a", "#f0c23b", "#ef7d2d", "#8a4fa0", "#e889b8"],
};
const BINS: [Bin, string][] = [["light", "Light"], ["dark", "Dark"], ["colored", "Colored"]];

function formatAge(s: number) {
  if (s < 60) return `${Math.round(s)} s`;
  if (s < 3600) return `${Math.round(s / 60)} min`;
  return `${Math.round(s / 3600)} h`;
}

/** Program lights already passed for the current item. */
function useVisited(phase: string | undefined): Set<string> {
  const visited = useRef(new Set<string>());
  const last = useRef<string | null>(null);
  if (phase !== undefined && phase !== last.current) {
    // a new item starts at the box, or at the work area when it isn't coming from the box
    if (phase === "look_box" || (phase === "look_bg" && last.current !== "place_on_bg")) visited.current.clear();
    if (STOPPED.includes(phase) && phase !== "held") visited.current.clear();
    visited.current.add(phase);
    last.current = phase;
  }
  return visited.current;
}

function BinStack({ bin, name, n, animate }: { bin: Bin; name: string; n: number; animate: number | null }) {
  const colors = CLOTH[bin];
  return (
    <div className="bin" data-bin={bin}>
      <div className="stack">
        {Array.from({ length: n }, (_, i) => (
          <span key={i} className={animate !== null && i >= animate ? "new" : undefined} style={{ background: colors[i % colors.length] }} />
        ))}
      </div>
      <span className={animate !== null ? "count bump" : "count"} key={n}>{n}</span>
      <span className="bin-name">{name}</span>
    </div>
  );
}

function Bins({ counters }: { counters: Record<Bin, number> }) {
  // counts before the last change; null until the first status (no animation on load)
  const [before, setBefore] = useState<Record<Bin, number> | null>(null);
  const last = useRef<Record<Bin, number> | null>(null);
  useEffect(() => {
    const prev = last.current;
    if (prev && BINS.some(([b]) => counters[b] !== prev[b])) setBefore(prev);
    last.current = counters;
  }, [counters]);
  return (
    <section className="bins" aria-label="Items sorted into each bin">
      {BINS.map(([bin, name]) => {
        const from = before !== null && counters[bin] > before[bin] ? before[bin] : null;
        return <BinStack key={bin} bin={bin} name={name} n={counters[bin]} animate={from} />;
      })}
    </section>
  );
}

export function AutoPage(): React.JSX.Element {
  const { status: s, online, run, setMode, phaseLabel } = useSorter();
  const [view, setView] = useStored<"decision" | "twin">("screen-view", "decision");
  const visited = useVisited(s?.phase);
  // the Load and Unload tabs share this page; the tab says which loop it switches to
  const tabMode: OperatorMode = usePath() === "/unload" ? "unload" : "load";
  const auto = isRunMode(s?.operator);
  const send = (cmd: string) => run(postJSON("/api/command", { cmd }));

  const alarm = s?.phase === "held" || s?.phase === "error";
  const next =
    s && s.next_phase && s.mode !== "running" && s.next_phase !== s.phase ? `Next: ${phaseLabel(s.next_phase).toLowerCase()}` : "";

  return (
    <div className="page page-auto">
      <header className="panel" aria-label="Control panel">
        <div className="display">
          <div className="display-top">
            <span className="mode" data-mode={s?.mode ?? "idle"}>{s ? MODES[s.mode] ?? s.mode : "…"}</span>
            <span className="next">{next}</span>
          </div>
          <h1 className="phase" aria-live="polite" data-alarm={alarm || undefined}>
            {s ? phaseLabel(s.phase) : "Connecting…"}
          </h1>
          <ol className="program" aria-label="Steps for one item">
            {PROGRAM.map((g) => (
              <li className="group" key={g.group}>
                <span className="group-name">{g.group}</span>
                <ol>
                  {g.steps.map(([p, label]) => (
                    <li key={p} data-state={p === s?.phase ? "now" : visited.has(p) ? "done" : ""}>
                      <i />
                      {label}
                    </li>
                  ))}
                </ol>
              </li>
            ))}
          </ol>
        </div>
        <dl className="readouts">
          <div><dt>Cycle</dt><dd>{s?.cycle ?? 0}</dd></div>
          <div><dt>Last cycle</dt><dd>{s?.last_cycle_s == null ? "–" : `${s.last_cycle_s.toFixed(1)} s`}</dd></div>
          <div><dt>Misses in a row</dt><dd data-alarm={(s?.failures ?? 0) > 0 || undefined}>{s?.failures ?? 0}</dd></div>
        </dl>
      </header>

      <main className="stage">
        <figure className="screen">
          {view === "twin" ? <TwinView compact /> : <LiveImage src="/stream/decision.mjpg" alt="The last frame the robot decided on, with what it found drawn on top" />}
          <figcaption className="screen-tabs" role="tablist" aria-label="Main screen">
            <button type="button" role="tab" aria-selected={view === "decision"} onClick={() => setView("decision")}>Last decision</button>
            <button type="button" role="tab" aria-selected={view === "twin"} onClick={() => setView("twin")}>3D view</button>
          </figcaption>
          <div className="alerts">
            {!online && <p className="alert alert-conn">No connection to the sorter. Hold may not reach the arm. Retrying…</p>}
            {s?.error && <p className="alert alert-error">Stopped: {s.error}. Fix the cause, then press Reset or Resume.</p>}
            {!!s?.health.length && <p className="alert alert-health">{s.health.join(" ")}</p>}
          </div>
        </figure>

        <aside className="side">
          <figure className="porthole">
            <div className="porthole-glass">
              <LiveImage src="/stream/live.mjpg" alt="Live view from the camera on the robot's wrist" />
            </div>
            <figcaption><span className="live-dot" />Wrist camera</figcaption>
          </figure>
          <Bins counters={s?.counters ?? { light: 0, dark: 0, colored: 0 }} />
        </aside>
      </main>

      <footer className="bottom">
        {auto || !s ? (
          <div className="controls" role="group" aria-label="Run controls">
            {COMMANDS.map((c) => (
              <button key={c.cmd} type="button" title={c.title} disabled={!s || !c.enabled(s)} onClick={() => send(c.cmd)}>
                <svg viewBox="0 0 16 16" aria-hidden="true">{c.icon}</svg>
                {c.label}
              </button>
            ))}
          </div>
        ) : (
          <div className="mode-strip">
            <span>The arm is in the <b>{s.operator}</b> mode: the sorter can't run.</span>
            <button type="button" className="primary" onClick={() => setMode(tabMode)}>Switch to {MODE_LABELS[tabMode]}</button>
          </div>
        )}
        <ol className="events" aria-label="Recent warnings">
          {s?.events
            .slice()
            .reverse()
            .map((e, i) => (
              <li key={`${e.t}-${i}`} data-level={e.level}>
                <span className="age">{formatAge(Math.max(0, s.now - e.t))}</span>
                <span className="msg" title={`${e.source}: ${e.msg}`}>{e.msg}</span>
              </li>
            ))}
        </ol>
      </footer>
    </div>
  );
}
