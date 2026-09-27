import { useCallback, useEffect, useRef, useState, useSyncExternalStore } from "react";
import { ApiError, getJSON } from "./api";

// --- routing: the tabs are paths, switched with the History API (no reload) ---

const listeners = new Set<() => void>();
const subscribe = (fn: () => void) => {
  listeners.add(fn);
  addEventListener("popstate", fn);
  return () => {
    listeners.delete(fn);
    removeEventListener("popstate", fn);
  };
};

export function navigate(path: string): void {
  if (path === location.pathname) return;
  history.pushState(null, "", path);
  for (const fn of listeners) fn();
}

export function usePath(): string {
  return useSyncExternalStore(subscribe, () => location.pathname);
}

// --- polling a JSON endpoint while `enabled` ---

export interface Polled<T> {
  data: T | null;
  error: ApiError | null;
  refresh: () => void;
}

export function usePoll<T>(url: string, periodMs: number, enabled = true): Polled<T> {
  const [data, setData] = useState<T | null>(null);
  const [error, setError] = useState<ApiError | null>(null);
  const kick = useRef<() => void>(() => {});

  useEffect(() => {
    if (!enabled) {
      setData(null);
      return;
    }
    let stop = false;
    let t: ReturnType<typeof setTimeout>;
    const tick = async () => {
      clearTimeout(t);
      try {
        const d = await getJSON<T>(url);
        if (!stop) {
          setData(d);
          setError(null);
        }
      } catch (e) {
        if (!stop) setError(e instanceof ApiError ? e : new ApiError("no connection", 0));
      }
      if (!stop) t = setTimeout(tick, periodMs);
    };
    kick.current = tick;
    tick();
    return () => {
      stop = true;
      clearTimeout(t);
      kick.current = () => {};
    };
  }, [url, periodMs, enabled]);

  const refresh = useCallback(() => kick.current(), []);
  return { data, error, refresh };
}

// --- a value kept in localStorage (per browser; private windows just don't remember) ---

export function useStored<T>(key: string, initial: T): [T, (v: T | ((old: T) => T)) => void] {
  const [value, setValue] = useState<T>(() => {
    try {
      const raw = localStorage.getItem(key);
      return raw === null ? initial : (JSON.parse(raw) as T);
    } catch {
      return initial;
    }
  });
  const set = useCallback(
    (v: T | ((old: T) => T)) => {
      setValue((old) => {
        const next = typeof v === "function" ? (v as (old: T) => T)(old) : v;
        try {
          localStorage.setItem(key, JSON.stringify(next));
        } catch {
          /* private window: not remembered */
        }
        return next;
      });
    },
    [key],
  );
  return [value, set];
}
