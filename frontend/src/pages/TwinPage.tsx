// The 3D tab: the arm, the table and the items full size, with the run's phase and counts.
import { MODE_LABELS, isRunMode } from "../api";
import { useSorter } from "../sorter";
import { BIN_NAMES } from "../twin/scene";
import { TwinView } from "../twin/TwinView";

export function TwinPage(): React.JSX.Element {
  const { status: s, phaseLabel } = useSorter();
  const hud = s && (
    <>
      <span className="twin-mode">{MODE_LABELS[s.operator]} mode</span>
      <span className="twin-phase">{isRunMode(s.operator) ? phaseLabel(s.phase) : "Manual control"}</span>
      <span className="twin-counts">
        {Object.entries(s.counters)
          .map(([k, n]) => `${BIN_NAMES[k] ?? k} ${n}`)
          .join(" · ")}
      </span>
    </>
  );
  return (
    <div className="page page-twin">
      <TwinView hud={hud} />
    </div>
  );
}
