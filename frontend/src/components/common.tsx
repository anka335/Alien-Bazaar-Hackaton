import { useEffect, useState, type ReactNode } from "react";
import { MODE_LABELS, isRunMode, postJSON, type ManualState, type OperatorMode } from "../api";
import { useSorter } from "../sorter";

/** An MJPEG stream; it ends when the server restarts, so it reconnects. */
export function LiveImage({ src, alt }: { src: string; alt: string }): React.JSX.Element {
  const [url, setUrl] = useState(src);
  useEffect(() => setUrl(src), [src]);
  return (
    <img
      src={url}
      alt={alt}
      onError={() => setTimeout(() => setUrl(`${src}?t=${Date.now()}`), 1000)}
    />
  );
}

/** The tab's content only in one of the `modes`; otherwise a card to switch to the first. */
export function ModeGate({ modes, what, children }: { modes: OperatorMode[]; what: string; children: ReactNode }): React.JSX.Element {
  const { status, meta, setMode } = useSorter();
  const [busy, setBusy] = useState(false);
  if (!status) return <div className="gate"><p className="gate-text">Connecting…</p></div>;
  if (modes.includes(status.operator)) return <>{children}</>;
  const target = modes[0];
  const available = meta === null || meta.modes.includes(target);
  const running = isRunMode(status.operator) && status.mode !== "idle";
  return (
    <div className="gate">
      <div className="gate-card">
        <p className="gate-kicker">Mode: {MODE_LABELS[status.operator]}</p>
        <h2>{what} works in the {MODE_LABELS[target]} mode</h2>
        {!available ? (
          <p className="gate-text">This server has no manual control.</p>
        ) : running ? (
          <p className="gate-text">A run is going. Stop it on the Sorting tab first: the arm finishes its step and goes home.</p>
        ) : (
          <p className="gate-text">
            {isRunMode(target)
              ? "The state machine takes the arm; the manual controls turn off."
              : "The state machine lets go of the arm; it moves only on your buttons."}
          </p>
        )}
        <button
          type="button"
          className="primary big"
          disabled={!available || running || busy}
          onClick={async () => {
            setBusy(true);
            await setMode(target);
            setBusy(false);
          }}
        >
          Switch to {MODE_LABELS[target]}
        </button>
      </div>
    </div>
  );
}

/** Phase line of the arm in a setup mode (manual, calibrate). */
export function armPhase(a: ManualState | null, idle: string): { phase: string; chip: string; chipMode: string; alarm: boolean } {
  if (!a) return { phase: "Connecting…", chip: idle, chipMode: "idle", alarm: false };
  const stuck = a.held || !!a.fault;
  return {
    phase: a.fault ? "Fault" : a.held ? "Held" : a.busy ? `Moving: ${a.action}` : "Ready",
    chip: a.fault ? "Fault" : a.held ? "Held" : a.busy ? "Moving" : idle,
    chipMode: stuck ? "paused" : a.busy ? "running" : "idle",
    alarm: stuck,
  };
}

/** Fault, hold and last error of the arm in a setup mode, over the camera image. */
export function ArmAlerts({ arm, onChange }: { arm: ManualState | null; onChange: () => void }): React.JSX.Element {
  const { run, online } = useSorter();
  const act = async (action: string) => {
    await run(postJSON("/api/manual", { action }));
    onChange();
  };
  const err = arm && !arm.fault && !arm.held ? arm.error : null;
  return (
    <div className="alerts">
      {arm?.fault && (
        <div className="alert alert-error alert-fault">
          <p><b>Arm fault.</b> {arm.fault}</p>
          <button
            type="button"
            disabled={arm.busy}
            title="Hold where the arm is now and allow motion again; the motors stay on"
            onClick={() => {
              const ok = confirm(
                "Check the arm first: nothing blocks it, the camera cable is slack and the joints turn freely.\n\n" +
                  "Clear the fault? The arm stays powered and holds where it is now.",
              );
              if (ok) act("clear_fault");
            }}
          >
            Clear fault
          </button>
        </div>
      )}
      {arm?.held && !arm.fault && (
        <div className="alert alert-error alert-fault">
          <p><b>Arm held.</b> Nothing moves until you release it.</p>
          <button type="button" disabled={arm.busy} onClick={() => act("release")}>Release hold</button>
        </div>
      )}
      {!online && <p className="alert alert-conn">No connection to the sorter. Hold may not reach the arm. Retrying…</p>}
      {err && <p className="alert alert-error">{err}</p>}
    </div>
  );
}

export function Toasts(): React.JSX.Element {
  const { toasts } = useSorter();
  return (
    <div className="toasts" aria-live="assertive">
      {toasts.map((t) => (
        <p key={t.id} className="toast" data-level={t.level}>{t.text}</p>
      ))}
    </div>
  );
}
