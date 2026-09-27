// The Mission tab: Start drives to the socks, loads each one, then drives to the station
// (sorter.mission.control). The OAK-D's last frame big, the wrist camera beside it, the steps below.
import { useEffect, useState } from "react";
import { getJSON, postJSON } from "../api";

interface MissionState {
  phase: string;
  note: string;
  running: boolean;
  real: boolean;
  cargo: Record<string, number>;
  loaded: number;
  stops: number;
  events: Record<string, unknown>[];
}

const PHASES: Record<string, string> = {
  idle: "ready",
  searching: "looking for a sock / driving to it",
  loading: "the arm is loading",
  to_station: "driving to the station",
  done: "at the station",
  stopped: "stopped",
  error: "error",
};

export function MissionPage(): React.JSX.Element {
  const [s, setS] = useState<MissionState | null>(null);
  const [err, setErr] = useState<string | null>(null);
  const [capacity, setCapacity] = useState(6);
  const [gapCm, setGapCm] = useState(10);
  const [tick, setTick] = useState(0);

  useEffect(() => {
    let alive = true;
    const poll = async () => {
      try {
        const st = await getJSON<MissionState>("/api/mission/state");
        if (alive) setS(st);
      } catch (e) {
        if (alive) setErr(String(e));
      }
      if (alive) setTick((t) => t + 1);
    };
    poll();
    const id = setInterval(poll, 700);
    return () => {
      alive = false;
      clearInterval(id);
    };
  }, []);

  const send = async (url: string, body: unknown) => {
    setErr(null);
    try {
      await postJSON(url, body);
    } catch (e) {
      setErr(e instanceof Error ? e.message : String(e));
    }
  };

  const running = s?.running ?? false;
  return (
    <div className="page page-rover">
      <section className="rover-main">
        <div className="rover-screen">
          <img src={`/api/mission/view.jpg?t=${tick}`} alt="OAK-D: the rover's view" onError={(e) => (e.currentTarget.style.visibility = "hidden")} onLoad={(e) => (e.currentTarget.style.visibility = "visible")} />
          <div className="rover-hud">
            <span>
              {s?.real ? <b className="real-badge">REAL ROVER</b> : "SIM"} · {s ? PHASES[s.phase] ?? s.phase : "…"}
              {s?.note ? ` · ${s.note}` : ""}
            </span>
            <span>
              in the box: {s?.loaded ?? 0}
              {s && Object.keys(s.cargo).length > 0 ? ` (${Object.entries(s.cargo).map(([c, n]) => `${c} ${n}`).join(", ")})` : ""} · stops {s?.stops ?? 0}
            </span>
          </div>
        </div>
        <div className="rover-side">
          <img src="/stream/live.mjpg" alt="Wrist camera" />
          <img src="/stream/decision.mjpg" alt="The arm's last decision" />
        </div>
      </section>

      <section className="rover-run">
        <button className="run-robot" disabled={running} onClick={() => send("/api/mission/start", { capacity, gap_m: gapCm / 100 })}>
          START MISSION
        </button>
        <label>
          socks before the station <input type="number" min={1} max={20} value={capacity} onChange={(e) => setCapacity(+e.target.value)} />
        </label>
        <label>
          stop <input type="number" min={0} max={50} value={gapCm} onChange={(e) => setGapCm(+e.target.value)} /> cm before a sock
        </label>
        <button className="stop" onClick={() => send("/api/mission/stop", {})}>
          STOP
        </button>
        {err && <span className="hint">{err}</span>}
      </section>

      <section className="rover-log">
        <ol>
          {[...(s?.events ?? [])].reverse().map((e, i) => (
            <li key={i}>
              <code>{JSON.stringify(e)}</code>
            </li>
          ))}
        </ol>
      </section>
    </div>
  );
}
