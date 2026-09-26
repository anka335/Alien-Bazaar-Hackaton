// The calibration tab (calibrate mode): tape marks, clicks from a few views, the camera mount
// fit, then the look poses. State is polled from /api/calibrate; every button posts one action
// there. Release and clear fault go through /api/manual, like on the manual tab.
import { useEffect, useRef, useState, type MouseEvent } from "react";
import { postJSON, type CalibrateState, type Fit } from "../api";
import { ArmAlerts, LiveImage, ModeGate, armPhase } from "../components/common";
import { usePoll, useStored } from "../hooks";
import { useSorter } from "../sorter";
import { TwinView } from "../twin/TwinView";

type S = CalibrateState;
const STEPS = ["Marks", "Click", "Calibrate", "Check"];

const inImage = (s: S, px: [number, number] | null) =>
  !!px && !!s.image && px[0] >= 20 && px[1] >= 20 && px[0] < s.image.width - 20 && px[1] < s.image.height - 20;

function nextPick(s: S, skipped: Set<string>): string | null {
  if (!s.image) return null;
  const done = new Set(s.clicks.filter((c) => c.here).map((c) => c.mark));
  const m = s.overlay.marks.find((o) => !done.has(o.name) && !skipped.has(o.name) && inImage(s, o.px));
  return m ? m.name : null;
}

function quality(f: Fit | null, checkPlausible = true): string {
  if (!f) return "";
  if (checkPlausible && !f.plausible) return "bad";
  return f.rmse_mm <= 3 ? "good" : f.rmse_mm <= 6 ? "ok" : "bad";
}

function fitText(s: S): string {
  const f = s.fit;
  const views = new Set(s.clicks.map((c) => c.pose)).size;
  if (!f) return `No fit yet: click at least 3 marks (${new Set(s.clicks.map((c) => c.mark)).size} so far).`;
  let t =
    `Fit over ${f.marks} marks from ${views} view${views === 1 ? "" : "s"}: RMSE ${f.rmse_mm.toFixed(1)} mm` +
    ` (${f.rmse_mm <= 3 ? "good" : f.rmse_mm <= 6 ? "fair: check the clicks with the biggest error" : "poor: a wrong click?"}).` +
    ` ${f.change_mm.toFixed(0)} mm, ${f.change_deg.toFixed(1)}° from the mount in use.`;
  if (f.true_error_mm !== undefined) t += ` Sim: ${f.true_error_mm.toFixed(1)} mm, ${(f.true_error_deg ?? 0).toFixed(2)}° off the true mount.`;
  if (views < 2) t += " Add a second view for a reliable rotation.";
  const per = (f.views ?? []).flatMap((v) => (v.rmse_mm === null ? [] : [{ view: v.view, rmse: v.rmse_mm }]));
  if (per.length)
    t +=
      ` Per view, without the arm: ${per.map((v) => `view ${v.view} ${v.rmse.toFixed(1)} mm`).join(", ")}` +
      (f.rmse_mm > 6 && per.every((v) => v.rmse <= 3) ? " (each view agrees with the tape: the arm pose differs between views)." : ".");
  if (!f.plausible)
    t = `Wrong fit: it puts the camera far from where it is mounted, so a click is on the wrong mark. Delete the clicks with the biggest error (All clicks). ${t}`;
  if (f.fixed_views?.length) t += ` View ${f.fixed_views.join(", ")} was clicked mirrored (M2↔M3, M4↔M5 look alike): relabeled.`;
  return t;
}

/** The marks on the mat, seen from behind the arm: +x (away from the arm) up, +y (left) left. */
function MarkMap({ s, current, lit, onPick }: { s: S; current: string | null; lit: Set<string>; onPick: (i: number) => void }) {
  const xs = s.marks.map((m) => m.xyz[0]);
  const ys = s.marks.map((m) => m.xyz[1]);
  const pad = 22;
  const [x1, y0, y1] = [Math.max(...xs) + pad, Math.min(...ys) - pad, Math.max(...ys) + pad];
  const x0 = Math.min(...xs) - pad;
  const [w, h] = [y1 - y0, x1 - x0]; // 1 unit = 1 mm
  return (
    <svg className="map" viewBox={`-2 -2 ${w + 4} ${h + 18}`} aria-label="Where the marks are on the mat, seen from behind the arm">
      <rect x={0} y={0} width={w} height={h} rx={8} className="mat" />
      <text x={w / 2} y={h + 14} className="arm">arm ↓</text>
      {s.marks.map((m, i) => {
        const [cx, cy] = [y1 - m.xyz[1], x1 - m.xyz[0]];
        const cls = ["map-mark", m.name === current && "picked", lit.has(m.name) && "placed"].filter(Boolean).join(" ");
        return (
          <g key={m.name} className={cls} onClick={() => onPick(i)}>
            <rect x={cx - 6} y={cy - 6} width={12} height={12} rx={2} />
            <text x={cx} y={cy - 10}>{m.name}</text>
          </g>
        );
      })}
    </svg>
  );
}

function Overlay({ s, pick, disabled, onClick }: { s: S; pick: string | null; disabled: boolean; onClick: (u: number, v: number) => void }) {
  const ref = useRef<SVGSVGElement>(null);
  if (!s.image) return null;
  const { width: w, height: h } = s.image;
  const clicked = new Set(s.clicks.filter((c) => c.here).map((c) => c.mark));
  const handle = (e: MouseEvent) => {
    const svg = ref.current;
    const ctm = svg?.getScreenCTM();
    if (!svg || !ctm || disabled) return;
    const p = new DOMPoint(e.clientX, e.clientY).matrixTransform(ctm.inverse());
    if (p.x < 0 || p.y < 0 || p.x >= w || p.y >= h) return;
    onClick(p.x, p.y);
  };
  return (
    <svg
      ref={ref}
      className="overlay"
      viewBox={`0 0 ${w} ${h}`}
      preserveAspectRatio="xMidYMid meet"
      role="img"
      aria-label="Click a tape mark in the image"
      data-disabled={disabled || undefined}
      onClick={handle}
    >
      <defs>
        <clipPath id="img-clip">
          <rect x={0} y={0} width={w} height={h} />
        </clipPath>
      </defs>
      <g clipPath="url(#img-clip)">
        {Object.entries(s.overlay.zones).map(([zone, pts]) => (
          <polygon key={zone} points={pts.map((p) => p.join(",")).join(" ")} className={`zone zone-${zone}`} />
        ))}
        {s.overlay.marks.map(
          (m) =>
            m.px && (
              <g key={m.name} className={["mark", m.name === pick && "picked", clicked.has(m.name) && "done"].filter(Boolean).join(" ")}>
                <circle cx={m.px[0]} cy={m.px[1]} r={11} />
                <text x={m.px[0] + 14} y={m.px[1] - 10}>{m.name}</text>
              </g>
            ),
        )}
        {s.clicks
          .filter((c) => c.here)
          .map((c, i) => (
            <g key={i} className="click">
              <path d={`M${c.px[0] - 9} ${c.px[1]}h18M${c.px[0]} ${c.px[1] - 9}v18`} />
            </g>
          ))}
      </g>
    </svg>
  );
}

function Calibrate() {
  const { run } = useSorter();
  const { data: s, refresh } = usePoll<S>("/api/calibrate", 250);
  // the wizard, kept per browser: the step, the mark of step 1, the view of step 2
  const [wiz, setWiz] = useStored("calibrate", { step: 1, markIdx: 0, viewIdx: 0 });
  const [skipped, setSkipped] = useState<Set<string>>(new Set());
  const lastSeen = useRef<string | null | undefined>(undefined);
  const set = (patch: Partial<typeof wiz>) => setWiz((w) => ({ ...w, ...patch }));

  const act = async <T = { ok: boolean },>(action: string, extra: object = {}) => {
    const r = await run(postJSON<T>("/api/calibrate", { action, ...extra }));
    refresh();
    return r;
  };

  // the arm just arrived at a view (from here or another browser): follow it, once per arrival
  const last = s?.arm.last;
  useEffect(() => {
    if (last === undefined) return;
    const at = last !== lastSeen.current ? /^view (\d+)$/.exec(last ?? "") : null;
    lastSeen.current = last;
    if (at && wiz.step === 2 && Number(at[1]) - 1 !== wiz.viewIdx) {
      set({ viewIdx: Number(at[1]) - 1 });
      setSkipped(new Set());
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [last]);

  const a = s?.arm ?? null;
  const p = armPhase(a, "Calibrate");
  if (!s || !a) {
    return (
      <div className="page page-setup">
        <header className="panel"><div className="display"><h1 className="phase">{p.phase}</h1></div></header>
      </div>
    );
  }

  const step = Math.min(Math.max(wiz.step, 1), 4);
  const markIdx = Math.min(wiz.markIdx, s.marks.length - 1);
  const viewIdx = Math.min(wiz.viewIdx, s.views - 1);
  const stuck = a.held || !!a.fault;
  const off = a.busy || stuck;
  const pick = step === 2 ? nextPick(s, skipped) : null;
  const m = s.marks[markIdx];
  const clickedHere = s.clicks.filter((c) => c.here).map((c) => c.mark);

  let mapCurrent: string | null = null;
  let mapLit = new Set<string>();
  if (step === 1) {
    mapCurrent = m.name;
    mapLit = new Set(s.marks.filter((x) => x.placed).map((x) => x.name));
  } else if (step === 2) {
    mapCurrent = pick;
    mapLit = new Set(clickedHere);
  }

  return (
    <div className="page page-setup page-calibrate">
      <header className="panel" aria-label="Arm status">
        <div className="display">
          <div className="display-top">
            <span className="mode" data-mode={p.chipMode}>{p.chip}</span>
            <span className="next">{a.at ? `at ${a.at}` : a.last ? `last: ${a.last}` : ""}</span>
          </div>
          <h1 className="phase" aria-live="polite" data-alarm={p.alarm || undefined}>{p.phase}</h1>
        </div>
        <dl className="readouts">
          <div><dt>TCP x · y · z, mm</dt><dd>{a.tcp_mm.map((v) => v.toFixed(0)).join(" · ")}</dd></div>
          <div><dt>Camera mount</dt><dd>{s.mount.method}</dd></div>
        </dl>
      </header>

      <main className="stage setup-stage">
        <div className="views">
          <figure className="screen cam">
            <LiveImage src="/stream/live.mjpg" alt="Live view from the camera on the robot's wrist" />
            <Overlay
              s={s}
              pick={pick}
              disabled={off || !pick}
              onClick={(u, v) => {
                if (pick) act("click", { mark: pick, u, v });
              }}
            />
            <figcaption className="tag"><span className="live-dot" />Wrist camera</figcaption>
            {s.image && (
              <p className="legend">
                Circles: marks where the {s.overlay_mount === "fit" ? "new fit" : "mount in use"} expects them · crosses: your clicks here
              </p>
            )}
            <ArmAlerts arm={a} onChange={refresh} />
          </figure>
          <figure className="screen twin-screen">
            <TwinView compact />
          </figure>
        </div>

        <aside className="deck" aria-label="Calibration steps">
          <ol className="stepper" aria-label="Steps">
            {STEPS.map((label, i) => (
              <li key={label}>
                <button type="button" aria-current={step === i + 1 || undefined} onClick={() => set({ step: i + 1 })}>
                  <b>{i + 1}</b>
                  {label}
                </button>
              </li>
            ))}
          </ol>

          {step <= 2 && (
            <section className="card">
              <h2>The marks on the mat <span className="count">seen from behind the arm</span></h2>
              <MarkMap s={s} current={mapCurrent} lit={mapLit} onPick={(i) => set({ markIdx: i })} />
            </section>
          )}

          {step === 1 && <Step1 s={s} markIdx={markIdx} off={off} act={act} set={set} />}
          {step === 2 && (
            <Step2
              s={s}
              viewIdx={viewIdx}
              pick={pick}
              clickedHere={clickedHere}
              act={act}
              set={set}
              skip={() => pick && setSkipped((x) => new Set(x).add(pick))}
              clearSkipped={() => setSkipped(new Set())}
            />
          )}
          {step === 3 && (
            <section className="card">
              <h2>Calibrate</h2>
              <p className="fit big" data-quality={quality(s.fit, false)}>{fitText(s)}</p>
              <p className="hint">
                Saves the camera mount to <code>{s.hand_eye_file}</code>, recomputes <code>look_box</code> and <code>look_bg</code> for it (the lowest
                pose that sees the whole zone with depth) and writes them with the zone ROIs to <code>{s.rig_file}</code>. Everything is used at once.
              </p>
              <div className="row">
                <button
                  type="button"
                  className="primary big"
                  disabled={!s.fit || !s.fit.plausible || off}
                  onClick={async () => {
                    if (await act("calibrate")) set({ step: 4 });
                  }}
                >
                  {a.busy ? "Wait for the arm…" : "Calibrate"}
                </button>
              </div>
              <div className="row foot">
                <button type="button" onClick={() => set({ step: 2 })}>Back</button>
              </div>
            </section>
          )}
          {step === 4 && (
            <section className="card">
              <h2>Check</h2>
              <p className="task">
                {s.mount.saved && s.look_saved ? `Calibrated: ${s.mount.saved} and ${s.look_saved}.` : "Not calibrated yet in this session."}
              </p>
              {s.look && (
                <table className="look">
                  <thead>
                    <tr><th>Pose</th><th>Camera</th><th>Sees, mm</th><th>Zone</th><th /></tr>
                  </thead>
                  <tbody>
                    {Object.entries(s.look).map(([name, c]) => (
                      <tr key={name} data-warn={!c.fits || undefined}>
                        <td>{name}</td>
                        <td>{c.camera_mm} mm</td>
                        <td>{c.sees_mm.join(" × ")}</td>
                        <td>{c.fits ? "all" : `${Math.round(c.coverage * 100)}%${c.depth_ok ? "" : ", too close"}`}</td>
                        <td>
                          <button type="button" disabled={off} onClick={() => act("goto_look", { pose: name })}>
                            {a.last === `preview ${name}` ? "Again" : "Preview"}
                          </button>
                        </td>
                      </tr>
                    ))}
                  </tbody>
                </table>
              )}
              <p className="hint">
                Preview each pose: the dashed outline must sit on the edges of the box or the mat. Then commit <code>config/hand_eye.yaml</code> and{" "}
                <code>config/rig.yaml</code>.
              </p>
              <div className="row foot">
                <button type="button" onClick={() => set({ step: 2 })}>Recalibrate</button>
              </div>
            </section>
          )}
        </aside>
      </main>
    </div>
  );
}

type Act = <T = { ok: boolean }>(action: string, extra?: object) => Promise<T | null>;
type SetWiz = (patch: Partial<{ step: number; markIdx: number; viewIdx: number }>) => void;

function Step1({ s, markIdx, off, act, set }: { s: S; markIdx: number; off: boolean; act: Act; set: SetWiz }) {
  const a = s.arm;
  const m = s.marks[markIdx];
  const placed = s.marks.filter((x) => x.placed).length;
  const here = a.last === `to mark ${m.name}` && !a.busy && !a.error;
  const task = here
    ? `The tip is over ${m.name}. Put the tape right under it (or check it is there), then Next mark.`
    : a.busy && a.action === `to mark ${m.name}`
      ? `Moving to ${m.name}…`
      : `Mark ${markIdx + 1} of ${s.marks.length}: ${m.name}${m.placed ? " (recorded)" : ""}. Press Move.`;
  const lastMark = markIdx === s.marks.length - 1;
  return (
    <section className="card">
      <h2>Tape marks <span className="count">{placed} of {s.marks.length} recorded</span></h2>
      <p className="task">{task}</p>
      <p className="hint">
        The tip stops {s.hover_mm} mm above the spot. Stick a small dark tape square (~1 cm) right under it. Already taped: the visit checks the tape
        is still under the tip and records where the tip really is.
      </p>
      <div className="row">
        <button type="button" className="primary" disabled={off} onClick={() => act("goto_mark", { mark: m.name })}>
          Move tip to {m.name}
        </button>
        <button type="button" onClick={() => (lastMark ? set({ step: 2 }) : set({ markIdx: markIdx + 1 }))}>
          {lastMark ? "All done" : "Next mark"}
        </button>
      </div>
      <div className="row foot">
        <button type="button" disabled={markIdx === 0} onClick={() => set({ markIdx: Math.max(markIdx - 1, 0) })}>Back</button>
        <button type="button" className="primary" onClick={() => set({ step: 2 })}>
          {placed === s.marks.length ? "Continue" : `Continue (${placed} of ${s.marks.length})`}
        </button>
      </div>
    </section>
  );
}

function Step2(props: {
  s: S;
  viewIdx: number;
  pick: string | null;
  clickedHere: string[];
  act: Act;
  set: SetWiz;
  skip: () => void;
  clearSkipped: () => void;
}) {
  const { s, viewIdx, pick, clickedHere, act, set, skip, clearSkipped } = props;
  const a = s.arm;
  const off = a.busy || a.held || !!a.fault;
  const atView = a.last === `view ${viewIdx + 1}` && !a.busy;
  let task: string;
  if (a.busy) task = `Moving: ${a.action}…`;
  else if (!atView && !clickedHere.length && !a.last?.startsWith("view")) task = `Press "Move camera to view ${viewIdx + 1}".`;
  else if (clickedHere.length && s.detected && s.detected.marks.length) {
    task =
      `Found ${s.detected.marks.join(", ")} by themselves. Check that every circle sits on its tape square.` +
      (pick ? ` ${pick} was not found: click its square, or skip it.` : " Then Next view.");
  } else if (s.detected && !s.detected.marks.length && atView) {
    task = `No marks found (${s.detected.squares} dark squares). Click them by hand, starting with ${pick ?? "any"}, or move the camera.`;
  } else if (pick && !s.fit) {
    task = `Click the tape square of ${pick} in the image. The circles are only a guess until 3 marks are clicked: find ${pick} by the map above (M1 and M6 are the close pair in the middle, M6 toward M2 / M3).`;
  } else if (pick) task = `Click the tape square of ${pick} (its circle should be on it now).`;
  else task = viewIdx < s.views - 1 ? "Every mark in sight is clicked. Next view." : "Every mark in sight is clicked. Continue.";
  const lastView = viewIdx >= s.views - 1;

  return (
    <section className="card">
      <h2>Click the marks <span className="count">view {viewIdx + 1} of {s.views}</span></h2>
      <div className="row">
        <button
          type="button"
          className="primary"
          disabled={off}
          onClick={() => {
            clearSkipped();
            act("goto_view", { index: viewIdx });
          }}
        >
          Move camera to view {viewIdx + 1}
        </button>
      </div>
      <p className="task">{task}</p>
      <div className="row">
        <button type="button" disabled={off} onClick={() => act("detect")}>Find marks again</button>
        <button type="button" disabled={!pick} onClick={skip}>Not visible, skip</button>
        <button
          type="button"
          disabled={lastView || off}
          onClick={() => {
            const next = Math.min(viewIdx + 1, s.views - 1);
            set({ viewIdx: next });
            clearSkipped();
            act("goto_view", { index: next });
          }}
        >
          {lastView ? "Last view" : `Next view (${viewIdx + 2})`}
        </button>
      </div>
      <div className="chips" aria-label="Marks clicked from this view">
        {clickedHere.map((n) => <span key={n}>✓ {n}</span>)}
      </div>
      <p className="fit" data-quality={quality(s.fit)}>{fitText(s)}</p>
      <details className="clicks-box">
        <summary>All clicks ({s.clicks.length})</summary>
        <ol className="clicks">
          {s.clicks.map((c, i) => (
            <li key={i} data-here={c.here || undefined}>
              <span>
                {c.mark}, view {c.pose + 1}
                {c.residual_mm === null ? "" : ` · ${c.residual_mm.toFixed(1)} mm`}
              </span>
              <button type="button" aria-label={`Delete click ${i + 1} (${c.mark})`} onClick={() => act("delete_click", { index: i })}>×</button>
            </li>
          ))}
        </ol>
        <button
          type="button"
          disabled={!s.clicks.length}
          onClick={() => {
            if (confirm("Delete every click?")) {
              set({ viewIdx: 0 });
              act("clear_clicks");
            }
          }}
        >
          Clear all
        </button>
      </details>
      <div className="row foot">
        <button type="button" onClick={() => set({ step: 1 })}>Back</button>
        <button type="button" className="primary" disabled={!s.fit || !s.fit.plausible} onClick={() => set({ step: 3 })}>Continue</button>
      </div>
    </section>
  );
}

export function CalibratePage(): React.JSX.Element {
  return (
    <ModeGate modes={["calibrate"]} what="Camera calibration">
      <Calibrate />
    </ModeGate>
  );
}
