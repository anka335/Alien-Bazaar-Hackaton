// The live sorter state shared by every tab: status over /ws (polled from /api/status while the
// socket is down), /api/meta once, and actions every tab uses (hold, mode switch, messages).
import {
  createContext,
  useCallback,
  useContext,
  useEffect,
  useMemo,
  useRef,
  useState,
  type ReactNode,
} from "react";
import { getJSON, postJSON, type Meta, type OperatorMode, type Status } from "./api";

export interface Toast {
  id: number;
  text: string;
  level: "error" | "info";
}

interface Sorter {
  status: Status | null;
  meta: Meta | null;
  online: boolean;
  holdSent: boolean;
  hold: () => void;
  setMode: (mode: OperatorMode) => Promise<boolean>;
  toasts: Toast[];
  notify: (text: string, level?: Toast["level"]) => void;
  /** Run a request; its error goes to the toasts. Returns the result, or null on error. */
  run: <T>(p: Promise<T>) => Promise<T | null>;
  phaseLabel: (phase: string) => string;
}

const Ctx = createContext<Sorter | null>(null);

export function useSorter(): Sorter {
  const s = useContext(Ctx);
  if (!s) throw new Error("useSorter outside SorterProvider");
  return s;
}

export function SorterProvider({ children }: { children: ReactNode }): React.JSX.Element {
  const [status, setStatus] = useState<Status | null>(null);
  const [meta, setMeta] = useState<Meta | null>(null);
  const [online, setOnline] = useState(true);
  const [holdSent, setHoldSent] = useState(false);
  const [toasts, setToasts] = useState<Toast[]>([]);
  const socketOpen = useRef(false);
  const nextId = useRef(1);

  const notify = useCallback((text: string, level: Toast["level"] = "error") => {
    const id = nextId.current++;
    setToasts((t) => [...t.filter((x) => x.text !== text), { id, text, level }]);
    setTimeout(() => setToasts((t) => t.filter((x) => x.id !== id)), level === "error" ? 6000 : 3000);
  }, []);

  const run = useCallback(
    async <T,>(p: Promise<T>): Promise<T | null> => {
      try {
        return await p;
      } catch (e) {
        if (e instanceof TypeError) setOnline(false); // fetch failed: no server
        else notify(e instanceof Error ? e.message : String(e));
        return null;
      }
    },
    [notify],
  );

  // status: the socket, and a poll while it is down (also how a dead server shows)
  useEffect(() => {
    let ws: WebSocket | null = null;
    let closed = false;
    let retry: ReturnType<typeof setTimeout>;
    const connect = () => {
      ws = new WebSocket(`${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/ws`);
      ws.onopen = () => {
        socketOpen.current = true;
        setOnline(true);
      };
      ws.onmessage = (e) => setStatus(JSON.parse(e.data) as Status);
      ws.onclose = () => {
        socketOpen.current = false;
        if (!closed) retry = setTimeout(connect, 1000);
      };
    };
    connect();
    const poll = setInterval(async () => {
      if (socketOpen.current) return;
      try {
        setStatus(await getJSON<Status>("/api/status"));
        setOnline(true);
      } catch {
        setOnline(false);
      }
    }, 700);
    return () => {
      closed = true;
      clearTimeout(retry);
      clearInterval(poll);
      ws?.close();
    };
  }, []);

  useEffect(() => {
    let stop = false;
    let t: ReturnType<typeof setTimeout>;
    const load = async () => {
      try {
        const m = await getJSON<Meta>("/api/meta");
        if (!stop) setMeta(m);
      } catch {
        if (!stop) t = setTimeout(load, 1000);
      }
    };
    load();
    return () => {
      stop = true;
      clearTimeout(t);
    };
  }, []);

  // the engaged look lasts until the status (or the arm's state) says otherwise
  useEffect(() => {
    if (!holdSent) return;
    const t = setTimeout(() => setHoldSent(false), 1500);
    return () => clearTimeout(t);
  }, [holdSent]);

  const hold = useCallback(() => {
    setHoldSent(true);
    postJSON("/api/command", { cmd: "hold" }).catch(() => setOnline(false));
  }, []);

  const setMode = useCallback(
    async (mode: OperatorMode) => {
      const r = await run(postJSON<{ mode: OperatorMode }>("/api/mode", { mode }));
      if (r) setStatus((s) => (s ? { ...s, operator: r.mode } : s));
      return r !== null;
    },
    [run],
  );

  const phaseLabel = useCallback((p: string) => meta?.phase_labels[p] ?? p, [meta]);

  const value = useMemo(
    () => ({ status, meta, online, holdSent, hold, setMode, toasts, notify, run, phaseLabel }),
    [status, meta, online, holdSent, hold, setMode, toasts, notify, run, phaseLabel],
  );
  return <Ctx.Provider value={value}>{children}</Ctx.Provider>;
}
