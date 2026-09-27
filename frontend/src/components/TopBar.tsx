// The admin panel's bar, on every tab: the tabs (a mode's tab switches the operator mode), the
// arm's speed, the connection and Hold.
import { useEffect, useRef, useState } from "react";
import { MODE_LABELS, TAB_MODES, postJSON, type OperatorMode, type Speed } from "../api";
import { navigate, usePath } from "../hooks";
import { useSorter } from "../sorter";

export const TABS: { path: string; label: string; mode: OperatorMode | null }[] = [
  { path: "/auto", label: "Auto", mode: "auto" },
  { path: "/manual", label: "Manual", mode: "manual" },
  { path: "/calibrate", label: "Calibrate", mode: "calibrate" },
  { path: "/3d", label: "3D view", mode: null },
  { path: "/rover", label: "Rover", mode: null },
];

export function currentTab(path: string, operator: OperatorMode | undefined): string {
  if (TABS.some((t) => t.path === path)) return path;
  return TAB_MODES[operator ?? "auto"];
}

/** The tabs. A mode's tab is its switch too: opening it switches the operator mode, and stays
 * on the current tab if the mode can't change now (the reason goes to the toasts). */
function Tabs() {
  const path = usePath();
  const { status, meta, setMode } = useSorter();
  const [pending, setPending] = useState<OperatorMode | null>(null);
  const tab = currentTab(path, status?.operator);
  const open = async (path: string, mode: OperatorMode | null) => {
    if (mode === null || mode === status?.operator) return navigate(path);
    setPending(mode);
    if (await setMode(mode)) navigate(path);
    setPending(null);
  };
  return (
    <nav className="tabs" aria-label="Tabs and operator mode">
      {TABS.map((t) => {
        const off = t.mode !== null && meta !== null && !meta.modes.includes(t.mode);
        const live = t.mode !== null && t.mode === status?.operator;
        return (
          <a
            key={t.path}
            href={t.path}
            aria-current={tab === t.path ? "page" : undefined}
            aria-disabled={off || pending !== null || undefined}
            data-pending={pending === t.mode || undefined}
            title={off ? "Not on this server" : t.mode && !live ? `Switch to the ${MODE_LABELS[t.mode]} mode` : undefined}
            onClick={(e) => {
              e.preventDefault();
              if (!off && pending === null) open(t.path, t.mode);
            }}
          >
            {t.label}
            {live && <i className="tab-live" title="Active mode" />}
          </a>
        );
      })}
    </nav>
  );
}

function SpeedControl({ speed }: { speed: Speed }) {
  const { run } = useSorter();
  const [value, setValue] = useState(speed.speed_scale);
  const editing = useRef(false);
  const timer = useRef<ReturnType<typeof setTimeout>>(undefined);
  useEffect(() => {
    if (!editing.current) setValue(speed.speed_scale);
  }, [speed.speed_scale]);
  const commit = (v: number) => {
    clearTimeout(timer.current);
    timer.current = setTimeout(async () => {
      await run(postJSON("/api/speed", { speed_scale: v }));
      editing.current = false;
    }, 150);
  };
  return (
    <label className="speed" title="Arm speed, from the next motion on">
      <span>Speed</span>
      <input
        type="range"
        min={0.05}
        max={speed.max_speed_scale}
        step={0.01}
        value={value}
        onChange={(e) => {
          editing.current = true;
          const v = Number(e.target.value);
          setValue(v);
          commit(v);
        }}
      />
      <b>{Math.round((value / speed.max_speed_scale) * 100)}%</b>
    </label>
  );
}

export function HoldButton(): React.JSX.Element {
  const { hold, holdSent } = useSorter();
  return (
    <button
      type="button"
      className="hold"
      data-engaged={holdSent || undefined}
      title="Freeze the arm where it is (Space or Esc)"
      onClick={hold}
    >
      <span className="hold-word">Hold</span>
      <span className="hold-key">Space</span>
    </button>
  );
}

export function TopBar(): React.JSX.Element {
  const { status, online } = useSorter();
  return (
    <header className="topbar">
      <div className="brand">
        <svg viewBox="0 0 32 32" aria-hidden="true">
          <rect x="3" y="2" width="26" height="28" rx="5" fill="currentColor" opacity="0.18" />
          <circle cx="16" cy="18" r="8" fill="var(--water)" />
          <circle cx="16" cy="18" r="5" fill="var(--glass)" />
        </svg>
        <span>Laundry sorter</span>
      </div>
      <Tabs />
      <div className="bar-right">
        {status?.speed && <SpeedControl speed={status.speed} />}
        <span className="conn" data-online={online || undefined} title={online ? "Connected" : "No connection"}>
          {online ? "Online" : "Offline"}
        </span>
        <HoldButton />
      </div>
    </header>
  );
}
