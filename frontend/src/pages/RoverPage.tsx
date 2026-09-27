// The Rover tab: the Leo Rover navigation sim (sorter.nav). The OAK-D's view is the operator's
// screen: click a pixel, then Face / Go to it. Chase and overview are debug views (ground truth).
import { useEffect, useRef, useState } from "react";
import { getJSON, postJSON } from "../api";
import { NavCommands } from "../components/NavCommands";

interface NavResult {
  command: string;
  args?: Record<string, unknown>;
  moved_m?: number;
  turned_deg?: number;
  elapsed_s?: number;
  blocked?: string | null;
  note?: string;
  auto?: boolean;
  t: number;
}

interface NavDetection {
  u: number;
  v: number;
  score: number;
  distance_m: number | null;
  bearing_deg: number | null;
  source: string;
}

interface NavState {
  episode: { scenario: string; seed: number };
  busy: string | null;
  error: string | null;
  log: NavResult[];
  detections: NavDetection[];
  scenarios: string[];
  jpeg_seq: number;
  real?: boolean;
  hardware?: { rosbridge: string; camera: string; odometry: string | null; fault: string | null; max_mps: number; max_rps: number };
  odom?: { x: number; y: number; yaw_deg: number; v: number; w: number; distance: number };
  truth?: { sock_rover: [number, number] };
  score?: { success: boolean; in_zone: boolean; collisions: number; sock_pushed_m: number };
  commands_done?: number;
  sim_t?: number;
  frame_size?: [number, number];
}

const DETECTORS = ["classic", "sam3", "seg"] as const;

export function RoverPage(): React.JSX.Element {
  const [s, setS] = useState<NavState | null>(null);
  const [err, setErr] = useState<string | null>(null);
  const [pixel, setPixel] = useState<[number, number] | null>(null);
  const [dist, setDist] = useState(0.5);
  const [angle, setAngle] = useState(45);
  const [speed, setSpeed] = useState(0.25);
  const [scenario, setScenario] = useState("easy");
  const [seed, setSeed] = useState(0);
  const [detector, setDetector] = useState<(typeof DETECTORS)[number]>("classic");
  const [truth, setTruth] = useState(false);
  const [gapCm, setGapCm] = useState(30);
  const img = useRef<HTMLImageElement>(null);

  useEffect(() => {
    let alive = true;
    const poll = async () => {
      try {
        const st = await getJSON<NavState>("/api/nav/state");
        if (alive) {
          setS(st);
          setErr(null);
        }
      } catch (e) {
        if (alive) setErr(String(e));
      }
    };
    poll();
    const id = setInterval(poll, 400);
    return () => {
      alive = false;
      clearInterval(id);
    };
  }, []);

  // the real rover: SAM3 by default (the classic detector needs good light)
  const isReal = !!s?.real;
  useEffect(() => {
    if (isReal) setDetector((d) => (d === "classic" ? "sam3" : d));
  }, [isReal]);

  const send = async (url: string, body: unknown) => {
    try {
      await postJSON(url, body);
      setErr(null);
    } catch (e) {
      setErr(String(e));
    }
  };
  const cmd = (name: string, args: Record<string, unknown> = {}) => send("/api/nav/command", { name, args });

  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if ((e.target as HTMLElement).tagName === "INPUT" || (e.target as HTMLElement).tagName === "SELECT") return;
      const keys: Record<string, string> = { ArrowUp: "forward", ArrowDown: "back", ArrowLeft: "left", ArrowRight: "right" };
      if (keys[e.key]) {
        e.preventDefault();
        cmd("nudge", { direction: keys[e.key] });
      } else if (e.key === "Escape") send("/api/nav/stop", {});
    };
    document.addEventListener("keydown", onKey);
    return () => document.removeEventListener("keydown", onKey);
  });

  const onClick = (e: React.MouseEvent<HTMLImageElement>) => {
    const el = img.current;
    if (!el || !s?.frame_size) return;
    const r = el.getBoundingClientRect();
    const [w, h] = s.frame_size;
    setPixel([Math.round(((e.clientX - r.left) / r.width) * w), Math.round(((e.clientY - r.top) / r.height) * h)]);
  };

  const busy = s?.busy ?? null;
  const fw = s?.frame_size?.[0] ?? 640;
  const fh = s?.frame_size?.[1] ?? 480;

  return (
    <div className="page page-rover">
      <section className="rover-main">
        <div className="rover-screen">
          <img ref={img} src="/api/nav/stream/rgb.mjpg" alt="OAK-D RGB" onClick={onClick} />
          <svg viewBox={`0 0 ${fw} ${fh}`} className="rover-overlay">
            {s?.detections.map((d, i) => (
              <g key={i}>
                <circle cx={d.u} cy={d.v} r={10} className={i === 0 ? "det det-best" : "det"} />
                <text x={d.u + 12} y={d.v - 8}>
                  {d.source} {d.score.toFixed(2)}
                  {d.distance_m !== null ? ` ${d.distance_m.toFixed(2)} m` : ""}
                </text>
              </g>
            ))}
            {pixel && (
              <g className="pick">
                <line x1={pixel[0] - 14} y1={pixel[1]} x2={pixel[0] + 14} y2={pixel[1]} />
                <line x1={pixel[0]} y1={pixel[1] - 14} x2={pixel[0]} y2={pixel[1] + 14} />
              </g>
            )}
          </svg>
          <div className="rover-hud">
            <span>
              {s?.real ? <b className="real-badge">REAL ROVER</b> : s ? `${s.episode.scenario} #${s.episode.seed}` : "…"} ·{" "}
              {busy ? `running: ${busy}` : "ready"}
            </span>
            <span>
              {s?.odom
                ? `odom ${s.odom.distance.toFixed(2)} m · yaw ${s.odom.yaw_deg.toFixed(0)}° · ${s.commands_done} cmds · ${s.sim_t?.toFixed(1)} s`
                : ""}
            </span>
          </div>
        </div>
        <div className="rover-side">
          <img src="/api/nav/stream/depth.mjpg" alt="OAK-D depth" />
          {!s?.real && <img src="/api/nav/stream/chase.mjpg" alt="Chase camera (debug)" />}
          {!s?.real && <img src="/api/nav/stream/overview.mjpg" alt="Overview (debug)" />}
        </div>
      </section>

      <section className="rover-run">
        <button className="run-robot" disabled={!!busy} onClick={() => send("/api/nav/run", { detector, gap_m: gapCm / 100 })}>
          RUN ROBOT
        </button>
        <label>
          stop <input type="number" step={1} min={0} max={100} value={gapCm} onChange={(e) => setGapCm(+e.target.value)} /> cm before the sock
        </label>
        <span className="hint">detector: {detector} · finds the nearest sock, drives up fast, stops in front of it</span>
        <button className="stop" onClick={() => send("/api/nav/stop", {})}>
          Stop (Esc)
        </button>
      </section>

      <section className="rover-controls">
        <div className="rover-group">
          <h3>Drive</h3>
          <label>
            distance m <input type="number" step={0.05} value={dist} onChange={(e) => setDist(+e.target.value)} />
          </label>
          <label>
            speed <input type="number" step={0.05} min={0.05} max={0.4} value={speed} onChange={(e) => setSpeed(+e.target.value)} />
          </label>
          <button disabled={!!busy} onClick={() => cmd("forward", { distance_m: dist, speed })}>Forward</button>
          <button disabled={!!busy} onClick={() => cmd("forward", { distance_m: -dist, speed })}>Back</button>
          <label>
            angle ° <input type="number" step={5} value={angle} onChange={(e) => setAngle(+e.target.value)} />
          </label>
          <button disabled={!!busy} onClick={() => cmd("turn", { angle_deg: angle })}>Turn left</button>
          <button disabled={!!busy} onClick={() => cmd("turn", { angle_deg: -angle })}>Turn right</button>
          <button disabled={!!busy} onClick={() => cmd("scan", { step_deg: 60 })}>Scan</button>
          <button disabled={!!busy} onClick={() => cmd("seek")}>Seek sock</button>
          <button disabled={!!busy} onClick={() => cmd("clearance")}>Clearance</button>
          <button disabled={!!busy} onClick={() => cmd("look")}>Look</button>
          <button className="stop" onClick={() => send("/api/nav/stop", {})}>Stop (Esc)</button>
          <p className="hint">Arrow keys: nudge 5 cm / 5°</p>
        </div>
        <div className="rover-group">
          <h3>Pixel {pixel ? `(${pixel[0]}, ${pixel[1]})` : "— click the camera view"}</h3>
          <button disabled={!!busy || !pixel} onClick={() => pixel && cmd("face", { u: pixel[0], v: pixel[1] })}>Face</button>
          <button disabled={!!busy || !pixel} onClick={() => pixel && cmd("go_to_pixel", { u: pixel[0], v: pixel[1] })}>
            Go to pixel
          </button>
        </div>
        <div className="rover-group">
          <h3>Algorithm</h3>
          <select value={detector} onChange={(e) => setDetector(e.target.value as (typeof DETECTORS)[number])}>
            {DETECTORS.map((d) => (
              <option key={d}>{d}</option>
            ))}
          </select>
          <button disabled={!!busy} onClick={() => send("/api/nav/detect", { detector })}>Detect</button>
          <button disabled={!!busy} onClick={() => send("/api/nav/auto", { detector })}>Approach sock</button>
        </div>
        {s?.real ? (
          <div className="rover-group">
            <h3>Real rover</h3>
            {s.hardware && (
              <p className={s.hardware.fault ? "warn" : "ok"}>
                {s.hardware.rosbridge} · camera: {s.hardware.camera} · odometry: {s.hardware.odometry ?? "–"} · max{" "}
                {s.hardware.max_mps} m/s, {s.hardware.max_rps} rad/s
                {s.hardware.fault ? ` · ${s.hardware.fault}` : ""}
              </p>
            )}
            <button onClick={() => send("/api/nav/reset", {})}>Reconnect</button>
            <p className="hint">Speeds are capped by nav.real. Esc or Stop halts it.</p>
          </div>
        ) : (
        <div className="rover-group">
          <h3>Scenario</h3>
          <select value={scenario} onChange={(e) => setScenario(e.target.value)}>
            {(s?.scenarios ?? ["easy"]).map((n) => (
              <option key={n}>{n}</option>
            ))}
          </select>
          <label>
            seed <input type="number" value={seed} onChange={(e) => setSeed(+e.target.value)} />
          </label>
          <button onClick={() => send("/api/nav/reset", { scenario, seed })}>Reset</button>
          <label className="check">
            <input type="checkbox" checked={truth} onChange={(e) => setTruth(e.target.checked)} /> show ground truth
          </label>
          {truth && s?.score && s.truth && (
            <p className={s.score.success ? "ok" : "warn"}>
              sock at x {s.truth.sock_rover[0].toFixed(2)} y {s.truth.sock_rover[1].toFixed(2)} m ·{" "}
              {s.score.in_zone ? "in zone" : "not in zone"} · collisions {s.score.collisions} · pushed{" "}
              {(s.score.sock_pushed_m * 100).toFixed(0)} cm
            </p>
          )}
        </div>
        )}
      </section>

      {(err || s?.error) && <p className="rover-error">{err ?? s?.error}</p>}

      <NavCommands
        busy={busy}
        pixel={pixel}
        log={s?.log ?? []}
        run={(name, args) => (name === "stop" && busy ? send("/api/nav/stop", {}) : cmd(name, args))}
      />

      <section className="rover-log">
        <table>
          <thead>
            <tr>
              <th>t s</th>
              <th>command</th>
              <th>moved m</th>
              <th>turned °</th>
              <th>note</th>
            </tr>
          </thead>
          <tbody>
            {[...(s?.log ?? [])].reverse().map((r, i) => (
              <tr key={i} className={r.blocked ? "blocked" : undefined}>
                <td>{r.t.toFixed(1)}</td>
                <td>
                  {r.auto ? "⟳ " : ""}
                  {r.command} {r.args ? Object.values(r.args).map((v) => (typeof v === "number" ? +v.toFixed(2) : v)).join(" ") : ""}
                </td>
                <td>{r.moved_m?.toFixed(2) ?? ""}</td>
                <td>{r.turned_deg?.toFixed(0) ?? ""}</td>
                <td>{[r.blocked, r.note].filter(Boolean).join(" · ")}</td>
              </tr>
            ))}
          </tbody>
        </table>
      </section>
    </div>
  );
}
