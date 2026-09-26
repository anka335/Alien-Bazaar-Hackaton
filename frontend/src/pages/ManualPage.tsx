// The manual tab (manual mode): named poses with a tour through them, joint jog, gripper,
// re-teaching a pose into rig.yaml. State is polled from /api/manual; every button posts one
// action there.
import { useState } from "react";
import { postJSON, type ManualState } from "../api";
import { ArmAlerts, LiveImage, ModeGate, armPhase } from "../components/common";
import { usePoll } from "../hooks";
import { useSorter } from "../sorter";
import { TwinView } from "../twin/TwinView";

const deg = (rad: number) => (rad * 180) / Math.PI;

function Manual() {
  const { run } = useSorter();
  const { data: s, refresh } = usePoll<ManualState>("/api/manual", 250);
  const [step, setStep] = useState(5);
  const [saveName, setSaveName] = useState("look_box");
  const [saved, setSaved] = useState<string | null>(null);
  const act = async (action: string, extra: object = {}) => {
    const r = await run(postJSON<{ ok: boolean; line?: string }>("/api/manual", { action, ...extra }));
    refresh();
    return r;
  };
  const p = armPhase(s, "Manual");
  const stuck = !s || s.held || !!s.fault;
  const off = !s || s.busy || stuck; // buttons that move the arm
  const next = s ? s.tour.indexOf(s.tour_next) : -1;

  return (
    <div className="page page-setup">
      <header className="panel" aria-label="Arm status">
        <div className="display">
          <div className="display-top">
            <span className="mode" data-mode={p.chipMode}>{p.chip}</span>
            <span className="next">{s?.at ? `at ${s.at}` : s?.last ? `last: ${s.last}` : ""}</span>
          </div>
          <h1 className="phase" aria-live="polite" data-alarm={p.alarm || undefined}>{p.phase}</h1>
        </div>
        <dl className="readouts">
          <div><dt>TCP x · y · z, mm</dt><dd>{s ? s.tcp_mm.map((v) => v.toFixed(0)).join(" · ") : "–"}</dd></div>
          <div><dt>Gripper</dt><dd>{s ? s.gripper.toFixed(2) : "–"}</dd></div>
        </dl>
      </header>

      <main className="stage setup-stage">
        <div className="views">
          <figure className="screen cam">
            <LiveImage src="/stream/live.mjpg" alt="Live view from the camera on the robot's wrist" />
            <figcaption className="tag"><span className="live-dot" />Wrist camera</figcaption>
            <ArmAlerts arm={s} onChange={refresh} />
          </figure>
          <figure className="screen twin-screen">
            <TwinView compact />
          </figure>
        </div>

        <aside className="deck" aria-label="Manual controls">
          <section className="card">
            <h2>Tour</h2>
            <ol className="tour">
              {s?.tour.map((name, i) => (
                <li key={name} data-state={i === next ? "next" : i < next ? "done" : ""}>
                  <i />
                  {name}
                </li>
              ))}
            </ol>
            <div className="row">
              <button type="button" className="primary" disabled={off} onClick={() => act("tour_next")}>
                Next: {s?.tour_next ?? "…"}
              </button>
              <button type="button" title="Start the tour over from the box" onClick={() => act("tour_reset")}>Restart</button>
            </div>
          </section>

          <section className="card">
            <h2>Go to pose</h2>
            <div className="poses">
              {s &&
                Object.keys(s.poses).map((name) => (
                  <button key={name} type="button" disabled={off} data-at={name === s.at || undefined} onClick={() => act("go", { pose: name })}>
                    {name}
                  </button>
                ))}
            </div>
            <p className="hint">Bins go via home, like a real drop.</p>
          </section>

          <section className="card">
            <h2>Gripper</h2>
            <div className="row">
              <button type="button" disabled={off} onClick={() => act("gripper", { open: true })}>Open</button>
              <button type="button" disabled={off} onClick={() => act("gripper", { open: false })}>Close</button>
              <button type="button" title="Leave hold without moving" disabled={!s?.held || !!s?.fault || s?.busy} onClick={() => act("release")}>
                Release hold
              </button>
            </div>
          </section>

          <section className="card">
            <h2>
              Jog joints
              <span className="steps" role="radiogroup" aria-label="Step">
                {[1, 5, 15].map((v) => (
                  <label key={v}>
                    <input type="radio" name="step" checked={step === v} onChange={() => setStep(v)} />
                    {v}°
                  </label>
                ))}
              </span>
            </h2>
            <div className="jog">
              {Array.from({ length: 6 }, (_, j) => (
                <JogRow key={j} j={j} q={s?.joints[j]} disabled={off} onJog={(sign) => act("jog", { joint: j, delta_deg: sign * step })} />
              ))}
            </div>
          </section>

          <section className="card">
            <h2>Save pose</h2>
            <div className="row">
              <select aria-label="Pose to overwrite" value={saveName} onChange={(e) => setSaveName(e.target.value)}>
                {s && Object.keys(s.poses).map((n) => <option key={n} value={n}>{n}</option>)}
              </select>
              <button
                type="button"
                disabled={off}
                onClick={async () => {
                  if (!confirm(`Overwrite pose "${saveName}" with the current joints?`)) return;
                  const r = await act("save", { pose: saveName });
                  if (r?.line) setSaved(r.line.trim());
                }}
              >
                Save current
              </button>
            </div>
            <p className="hint">{saved ? `Saved: ${saved}` : <>Writes the current joints to <code>{s?.rig_file ?? "rig.yaml"}</code>.</>}</p>
          </section>
        </aside>
      </main>
    </div>
  );
}

function JogRow({ j, q, disabled, onJog }: { j: number; q: number | undefined; disabled: boolean; onJog: (sign: number) => void }) {
  return (
    <>
      <span className="name">J{j + 1}</span>
      <button type="button" aria-label={`J${j + 1} minus`} disabled={disabled} onClick={() => onJog(-1)}>−</button>
      <span className="val">{q === undefined ? "–" : `${deg(q).toFixed(1)}°`}</span>
      <button type="button" aria-label={`J${j + 1} plus`} disabled={disabled} onClick={() => onJog(1)}>+</button>
    </>
  );
}

export function ManualPage(): React.JSX.Element {
  return (
    <ModeGate modes={["manual", "calibrate"]} what="Manual control">
      <Manual />
    </ModeGate>
  );
}
