// The sorter's HTTP API (docs/architecture.md → Dashboard HTTP API): types and small helpers.

export type OperatorMode = "auto" | "manual" | "calibrate";
export type Bin = "light" | "dark" | "colored";

export interface SorterEvent {
  t: number;
  level: "info" | "warning" | "error";
  source: string;
  msg: string;
}

export interface Speed {
  speed_scale: number;
  max_speed_scale: number;
}

export interface Status {
  phase: string;
  next_phase: string | null;
  mode: "idle" | "running" | "paused";
  run_id: string | null;
  cycle: number;
  counters: Record<Bin, number>;
  failures: number;
  last_cycle_s: number | null;
  error: string | null;
  health: string[];
  events: SorterEvent[];
  now: number;
  speed: Speed | null;
  operator: OperatorMode;
}

export interface Meta {
  phase_labels: Record<string, string>;
  modes: OperatorMode[];
  calibrate: boolean;
}

export interface ManualState {
  busy: boolean;
  action: string | null;
  last: string | null;
  error: string | null;
  held: boolean;
  fault: string | null;
  at: string | null;
  joints: number[];
  gripper: number;
  tcp_mm: number[];
  poses: Record<string, number[]>;
  tour: string[];
  tour_next: string;
  rig_file: string;
}

export interface Fit {
  rmse_mm: number;
  change_mm: number;
  change_deg: number;
  marks: number;
  fixed_views: number[];
  views: { view: number; marks: string[]; rmse_mm: number | null }[];
  plausible: boolean;
  true_error_mm?: number;
  true_error_deg?: number;
}

export interface LookPose {
  tcp_z: number;
  camera_mm: number;
  coverage: number;
  depth_ok: boolean;
  fits: boolean;
  sees_mm: [number, number];
  top: number;
}

export interface CalibrateState {
  arm: ManualState;
  image: { width: number; height: number } | null;
  overlay: {
    marks: { name: string; px: [number, number] | null }[];
    zones: Record<string, [number, number][]>;
  };
  overlay_mount: "fit" | "in use";
  marks: { name: string; xyz: [number, number, number]; placed: boolean }[];
  hover_mm: number;
  detected: { squares: number; marks: string[] } | null;
  views: number;
  clicks: {
    mark: string;
    px: [number, number];
    here: boolean;
    pose: number;
    residual_mm: number | null;
  }[];
  fit: Fit | null;
  mount: { method: string; saved: string | null };
  hand_eye_file: string;
  look: Record<string, LookPose> | null;
  look_saved: string | null;
  rig_file: string;
}

export class ApiError extends Error {
  constructor(
    message: string,
    readonly status: number,
  ) {
    super(message);
  }
}

async function detail(r: Response): Promise<string> {
  const body = await r.json().catch(() => ({}));
  const d = (body as { detail?: unknown }).detail;
  return typeof d === "string" ? d : r.statusText || `HTTP ${r.status}`;
}

export async function getJSON<T>(url: string): Promise<T> {
  const r = await fetch(url, { cache: "no-store" });
  if (!r.ok) throw new ApiError(await detail(r), r.status);
  return (await r.json()) as T;
}

export async function postJSON<T = { ok: boolean }>(url: string, body: unknown): Promise<T> {
  const r = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  if (!r.ok) throw new ApiError(await detail(r), r.status);
  return (await r.json()) as T;
}

export const TAB_MODES: Record<OperatorMode, string> = {
  auto: "/auto",
  manual: "/manual",
  calibrate: "/calibrate",
};

export const MODE_LABELS: Record<OperatorMode, string> = {
  auto: "Auto",
  manual: "Manual",
  calibrate: "Calibrate",
};
