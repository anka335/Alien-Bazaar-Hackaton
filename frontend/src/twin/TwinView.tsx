import { useEffect, useRef, useState, type ReactNode } from "react";
import { TwinScene, type Badge } from "./scene";

/** The 3D view, sized to its box. `compact`: the small version embedded in another tab. */
export function TwinView({ compact = false, hud }: { compact?: boolean; hud?: ReactNode }): React.JSX.Element {
  const box = useRef<HTMLDivElement>(null);
  const [badge, setBadge] = useState<Badge>({ text: "loading", state: "" });
  useEffect(() => {
    if (!box.current) return;
    const scene = new TwinScene(box.current, setBadge);
    return () => scene.dispose();
  }, []);
  return (
    <div className="twin" data-compact={compact || undefined} ref={box}>
      <div className="twin-hud">
        {hud}
        <b className="badge" data-state={badge.state}>{badge.text}</b>
      </div>
      <div className="twin-legend">
        <span><i style={{ background: "#3cc8e2" }} />pick workspace</span>
        <span><i style={{ background: "rgb(60 200 226 / 0.35)" }} />what the wrist camera sees</span>
        {!compact && <span className="help">drag: orbit · right-drag: pan · wheel: zoom · double-click: reset</span>}
      </div>
    </div>
  );
}
