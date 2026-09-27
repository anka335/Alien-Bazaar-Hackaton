// Every navigation command as a card: its parameters as fields and a Run button. The list comes
// from the server (`GET /api/nav/commands`, the commands' own signatures), so it never goes stale.
import { useEffect, useState } from "react";
import { getJSON } from "../api";

interface Param {
  name: string;
  type: "number" | "bool" | "str";
  default: number | boolean | string | null;
  required: boolean;
}

interface Spec {
  name: string;
  params: Param[];
  doc: string;
}

export interface NavLogEntry {
  command: string;
  moved_m?: number;
  turned_deg?: number;
  blocked?: string | null;
  note?: string;
  ok?: boolean;
}

// starting values for parameters without a default
const START: Record<string, number | string> = {
  distance_m: 0.5,
  angle_deg: 45,
  direction: "forward",
  v: 0.2,
  w: 0,
  duration_s: 1,
};
const DIRECTIONS = ["forward", "back", "left", "right"];

type Values = Record<string, Record<string, string | boolean>>;

function initial(specs: Spec[]): Values {
  const out: Values = {};
  for (const c of specs) {
    out[c.name] = {};
    for (const p of c.params) {
      const d = p.default ?? START[p.name] ?? "";
      out[c.name][p.name] = p.type === "bool" ? Boolean(d) : String(d);
    }
  }
  return out;
}

export function NavCommands({
  busy,
  pixel,
  log,
  run,
}: {
  busy: string | null;
  pixel: [number, number] | null;
  log: NavLogEntry[];
  run: (name: string, args: Record<string, unknown>) => void;
}): React.JSX.Element {
  const [specs, setSpecs] = useState<Spec[]>([]);
  const [values, setValues] = useState<Values>({});
  const [err, setErr] = useState<string | null>(null);

  useEffect(() => {
    getJSON<Spec[]>("/api/nav/commands")
      .then((s) => {
        setSpecs(s);
        setValues(initial(s));
      })
      .catch((e) => setErr(String(e)));
  }, []);

  // a click on the camera view fills u / v of every pixel command
  useEffect(() => {
    if (!pixel) return;
    setValues((vals) => {
      const next = { ...vals };
      for (const c of specs) {
        if (c.params.some((p) => p.name === "u")) {
          next[c.name] = { ...next[c.name], u: String(pixel[0]), v: String(pixel[1]) };
        }
      }
      return next;
    });
  }, [pixel, specs]);

  const set = (cmd: string, name: string, value: string | boolean) =>
    setValues((vals) => ({ ...vals, [cmd]: { ...vals[cmd], [name]: value } }));

  const submit = (c: Spec) => {
    const args: Record<string, unknown> = {};
    for (const p of c.params) {
      const raw = values[c.name]?.[p.name];
      if (p.type === "bool") args[p.name] = Boolean(raw);
      else if (raw === "" || raw === undefined) {
        if (p.required) return setErr(`${c.name}: ${p.name} is required`);
      } else if (p.type === "number") {
        const n = Number(raw);
        if (Number.isNaN(n)) return setErr(`${c.name}: ${p.name} must be a number`);
        args[p.name] = n;
      } else args[p.name] = raw;
    }
    setErr(null);
    run(c.name, args);
  };

  const lastOf = (name: string) => [...log].reverse().find((r) => r.command === name);

  return (
    <section className="nav-commands">
      <h2>Commands</h2>
      {err && <p className="rover-error">{err}</p>}
      <div className="nav-command-grid">
        {specs.map((c) => {
          const last = lastOf(c.name);
          const needsPixel = c.params.some((p) => p.name === "u");
          return (
            <form
              key={c.name}
              className="nav-command"
              onSubmit={(e) => {
                e.preventDefault();
                submit(c);
              }}
            >
              <header>
                <code>{c.name}</code>
                <button type="submit" disabled={!!busy && c.name !== "stop"} className={c.name === "stop" ? "stop" : undefined}>
                  Run
                </button>
              </header>
              {c.params.map((p) => (
                <label key={p.name}>
                  <span>
                    {p.name}
                    {p.required ? " *" : ""}
                  </span>
                  {p.type === "bool" ? (
                    <input
                      type="checkbox"
                      checked={Boolean(values[c.name]?.[p.name])}
                      onChange={(e) => set(c.name, p.name, e.target.checked)}
                    />
                  ) : p.name === "direction" ? (
                    <select value={String(values[c.name]?.[p.name] ?? "")} onChange={(e) => set(c.name, p.name, e.target.value)}>
                      {DIRECTIONS.map((d) => (
                        <option key={d}>{d}</option>
                      ))}
                    </select>
                  ) : (
                    <input
                      type="text"
                      inputMode="decimal"
                      placeholder={p.required ? "required" : "default"}
                      value={String(values[c.name]?.[p.name] ?? "")}
                      onChange={(e) => set(c.name, p.name, e.target.value)}
                    />
                  )}
                </label>
              ))}
              {needsPixel && !pixel && <p className="hint">Click the camera view to fill u, v</p>}
              <p className="doc">{c.doc}</p>
              {last && (
                <p className={last.blocked || last.ok === false ? "last warn" : "last"}>
                  last: moved {last.moved_m?.toFixed(2) ?? "–"} m, turned {last.turned_deg?.toFixed(0) ?? "–"}°
                  {last.blocked ? ` · ${last.blocked}` : ""}
                  {last.note ? ` · ${last.note}` : ""}
                </p>
              )}
            </form>
          );
        })}
      </div>
    </section>
  );
}
