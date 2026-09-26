"""status_web: a small web page showing where the cloth task stands.

    ros2 run cloth_task status_web            # http://localhost:8080  (task.launch.py starts it)

Reads only (never commands anything):
  /task_supervisor/status     JSON snapshot from task_supervisor_node (phase, counters, cloth, …)
  /rosout                     log lines of the task, the arm bridge and the detector
  /joint_states               arm joints and finger opening
  /cloth_detector/debug_image the last SAM3 detection (masks, grasp point)
  /camera/camera/color/image_raw  the live wrist camera

Serves: /  (the page), /api/status (JSON), /image/live.jpg, /image/detection.jpg.
Plain http.server in a thread + OpenCV for JPEG, no extra dependencies.
"""

from __future__ import annotations

import contextlib
import json
import math
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rcl_interfaces.msg import Log
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String

from cloth_task.core import ARM_JOINTS

# the supervisor's phases in the order a pick goes through them
PIPELINE = (
    "init",
    "go_to_view",
    "detect",
    "approach",
    "trial_grasp",
    "validate",
    "holding",
    "place",
    "done",
)
WATCHED_NODES = ("task_supervisor_node", "arm_bridge", "cloth_detector_node", "sim_gripper")
LEVELS = {10: "debug", 20: "info", 30: "warn", 40: "error", 50: "fatal"}
IMAGE_MAX_AGE_S = 3.0  # an older live frame is reported as stale


def snapshot(state: dict, now: float) -> dict:
    """What /api/status returns: the supervisor's status plus derived fields for the page."""
    st = dict(state.get("status") or {})
    phase = st.get("phase")
    since = st.get("phase_since")
    started = st.get("started")
    joints = state.get("joints") or {}
    return {
        "connected": bool(st) and now - state.get("status_t", 0.0) < 3600,
        "status": st,
        "phase": phase,
        "phase_index": PIPELINE.index(phase) if phase in PIPELINE else -1,
        "phase_for_s": round(now - since, 1) if since else None,
        "running_for_s": round(now - started, 1) if started else None,
        "pipeline": PIPELINE,
        "joints_deg": [round(math.degrees(joints[j]), 1) for j in ARM_JOINTS if j in joints],
        "finger_mm": round(joints["joint_left"] * 1000, 1) if "joint_left" in joints else None,
        "joints_age_s": round(now - state["joints_t"], 1) if state.get("joints_t") else None,
        "live_age_s": round(now - state["live_t"], 1) if state.get("live_t") else None,
        "detection_age_s": round(now - state["det_t"], 1) if state.get("det_t") else None,
        "logs": list(state.get("logs") or [])[-60:],
    }


class StatusWeb(Node):
    def __init__(self):
        super().__init__("status_web")
        self.declare_parameter("host", "127.0.0.1")  # 0.0.0.0 to open it from another device
        self.declare_parameter("port", 8080)
        self.declare_parameter("live_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("detection_topic", "/cloth_detector/debug_image")
        p = lambda n: self.get_parameter(n).value  # noqa: E731

        self.lock = threading.Lock()
        self.state: dict = {"logs": deque(maxlen=300)}
        self._jpeg_cache: dict[str, tuple[float, bytes]] = {}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/task_supervisor/status", self._on_status, latched)
        self.create_subscription(Log, "/rosout", self._on_log, 100)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self.create_subscription(
            Image, p("live_topic"), lambda m: self._on_image("live", m), qos_profile_sensor_data
        )
        self.create_subscription(Image, p("detection_topic"), lambda m: self._on_image("det", m), 1)
        self.host, self.port = p("host"), int(p("port"))

    # --- ROS side ---

    def _on_status(self, msg: String) -> None:
        with contextlib.suppress(ValueError):
            status = json.loads(msg.data)
            with self.lock:
                self.state["status"], self.state["status_t"] = status, time.time()

    def _on_log(self, msg: Log) -> None:
        if msg.name not in WATCHED_NODES:
            return
        entry = {
            "t": msg.stamp.sec + msg.stamp.nanosec * 1e-9,
            "level": LEVELS.get(msg.level, str(msg.level)),
            "node": msg.name,
            "msg": msg.msg,
        }
        with self.lock:
            self.state["logs"].append(entry)

    def _on_joints(self, msg: JointState) -> None:
        if len(msg.name) != len(msg.position):
            return
        with self.lock:
            joints = dict(self.state.get("joints") or {})
            joints.update(zip(msg.name, msg.position, strict=True))
            self.state["joints"], self.state["joints_t"] = joints, time.time()

    def _on_image(self, which: str, msg: Image) -> None:
        with self.lock:  # keep the raw message; encode only when someone asks
            self.state[which], self.state[f"{which}_t"] = msg, time.time()

    def jpeg(self, which: str) -> bytes | None:
        """Latest image as JPEG, encoded at most 5×/s per stream."""
        import cv2
        from cv_bridge import CvBridge

        with self.lock:
            msg, t = self.state.get(which), self.state.get(f"{which}_t", 0.0)
            cached = self._jpeg_cache.get(which)
        if msg is None:
            return None
        if cached and cached[0] >= t and time.time() - cached[0] < 0.2:
            return cached[1]
        img = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        if img.shape[1] > 640:
            img = cv2.resize(img, (640, int(img.shape[0] * 640 / img.shape[1])))
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            return None
        data = buf.tobytes()
        with self.lock:
            self._jpeg_cache[which] = (time.time(), data)
        return data

    def api(self) -> dict:
        with self.lock:
            state = dict(self.state)
            state["logs"] = list(self.state["logs"])
        return snapshot(state, time.time())

    # --- HTTP side ---

    def serve(self) -> ThreadingHTTPServer:
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_):  # quiet: the terminal is for the robot
                pass

            def _send(self, code: int, body: bytes, ctype: str) -> None:
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):  # noqa: N802 (http.server API)
                path = self.path.split("?", 1)[0]
                if path in ("/", "/index.html"):
                    self._send(200, PAGE.encode(), "text/html; charset=utf-8")
                elif path == "/api/status":
                    self._send(200, json.dumps(node.api()).encode(), "application/json")
                elif path in ("/image/live.jpg", "/image/detection.jpg"):
                    data = node.jpeg("live" if "live" in path else "det")
                    if data is None:
                        self._send(404, b"no image yet", "text/plain")
                    else:
                        self._send(200, data, "image/jpeg")
                else:
                    self._send(404, b"not found", "text/plain")

        server = ThreadingHTTPServer((self.host, self.port), Handler)
        server.daemon_threads = True
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.get_logger().info(f"status page: http://{self.host}:{self.port}")
        return server


PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Cloth task status</title>
<style>
:root{--bg:#f6f7f9;--card:#fff;--ink:#1d2330;--mute:#6b7385;--line:#e2e5ea;--ok:#1f9d55;
--warn:#b7791f;--err:#c53030;--acc:#2b6cb0;--accbg:#e6f0fb}
@media (prefers-color-scheme:dark){:root{--bg:#12151b;--card:#1b2029;--ink:#e6e9ef;--mute:#9aa3b5;
--line:#2a313d;--ok:#48bb78;--warn:#ecc94b;--err:#fc8181;--acc:#63b3ed;--accbg:#1d2b3d}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);
font:14px/1.45 system-ui,-apple-system,Segoe UI,Roboto,sans-serif}
header{display:flex;align-items:center;gap:12px;padding:14px 20px;border-bottom:1px solid var(--line);
background:var(--card)}h1{font-size:17px;margin:0}#conn{font-size:12px;color:var(--mute)}
main{max-width:1200px;margin:0 auto;padding:16px;display:grid;gap:14px;
grid-template-columns:repeat(auto-fit,minmax(320px,1fr))}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:14px}
.wide{grid-column:1/-1}h2{font-size:12px;text-transform:uppercase;letter-spacing:.06em;
color:var(--mute);margin:0 0 10px}
.pipe{display:flex;flex-wrap:wrap;gap:6px}.step{padding:5px 10px;border-radius:999px;
border:1px solid var(--line);color:var(--mute);font-size:13px}
.step.done{color:var(--ok);border-color:var(--ok)}.step.now{background:var(--accbg);
color:var(--acc);border-color:var(--acc);font-weight:600}
#phase{font-size:26px;font-weight:650;margin:10px 0 2px}#phasefor{color:var(--mute)}
.err{display:none;margin-top:10px;padding:10px;border-radius:8px;background:#c5303018;
color:var(--err);border:1px solid var(--err)}
dl{display:grid;grid-template-columns:auto 1fr;gap:4px 14px;margin:0}dt{color:var(--mute)}
dd{margin:0;font-variant-numeric:tabular-nums}
img{width:100%;border-radius:8px;background:var(--bg);min-height:120px;display:block}
.none{color:var(--mute);font-size:13px;padding:30px 0;text-align:center}
#logs{font:12px/1.5 ui-monospace,SFMono-Regular,Menlo,monospace;max-height:340px;overflow:auto}
.log{padding:2px 0;border-bottom:1px solid var(--line);word-break:break-word}
.log .n{color:var(--mute)}.warn{color:var(--warn)}.error,.fatal{color:var(--err)}
</style></head><body>
<header><h1>Cloth task</h1><span id="conn">connecting…</span></header>
<main>
 <section class="card wide"><h2>Where we stand</h2>
  <div class="pipe" id="pipe"></div>
  <div id="phase">–</div><div id="phasefor"></div>
  <div class="err" id="err"></div></section>
 <section class="card"><h2>Run</h2><dl id="run"></dl></section>
 <section class="card"><h2>Cloth</h2><dl id="cloth"></dl></section>
 <section class="card"><h2>Arm</h2><dl id="arm"></dl></section>
 <section class="card"><h2>Wrist camera (live)</h2><img id="live" alt="">
  <div class="none" id="livenone">no camera image</div></section>
 <section class="card"><h2>Last detection (SAM3)</h2><img id="det" alt="">
  <div class="none" id="detnone">no detection yet</div></section>
 <section class="card wide"><h2>Log</h2><div id="logs"></div></section>
</main>
<script>
const $=id=>document.getElementById(id);
const NAMES={init:"Start",go_to_view:"To box view",search:"Search",halting:"Halt",detect:"Detect",
approach:"Approach",at_approach:"Above cloth",trial_grasp:"Grasp",validate:"Lift & check",
holding:"Holding",place:"Place",done:"Done",error:"Error"};
const fmt=s=>s==null?"–":s<90?s.toFixed(0)+" s":(s/60).toFixed(1)+" min";
function dl(el,rows){el.innerHTML=rows.map(([k,v])=>`<dt>${k}</dt><dd>${v??"–"}</dd>`).join("")}
function esc(s){return String(s).replace(/[&<>]/g,c=>({"&":"&amp;","<":"&lt;",">":"&gt;"}[c]))}
async function tick(){
 let d;try{d=await (await fetch("/api/status")).json()}catch(e){$("conn").textContent="page server not reachable";return}
 const s=d.status||{};
 $("conn").textContent=d.connected?"live":"waiting for task_supervisor_node…";
 $("pipe").innerHTML=d.pipeline.map((p,i)=>`<span class="step ${i<d.phase_index?"done":i==d.phase_index?"now":""}">${NAMES[p]||p}</span>`).join("");
 $("phase").textContent=NAMES[d.phase]||d.phase||"–";
 $("phase").style.color=d.phase=="error"?"var(--err)":d.phase=="done"?"var(--ok)":"";
 $("phasefor").textContent=d.phase?`for ${fmt(d.phase_for_s)} · running ${fmt(d.running_for_s)}`:"";
 $("err").style.display=s.error?"block":"none";$("err").textContent=s.error||"";
 dl($("run"),[["Placed",s.placed],["Attempt",s.attempt?`${s.attempt} / ${s.max_attempts}`:null],
  ["Cloths",s.cycles===0?"until the box is empty":s.cycles],["Place target",s.place_target],
  ["Mode",s.mode],["Speed",s.execution_speed]]);
 dl($("cloth"),[["Position (m)",s.cloth?s.cloth.join(", "):null],["Class",s.color],
  ["Going to bin",s.target]]);
 dl($("arm"),[["Joints (°)",d.joints_deg.length?d.joints_deg.join(", "):null],
  ["Fingers",d.finger_mm==null?null:d.finger_mm+" mm"],
  ["Joint states",d.joints_age_s==null?"none":d.joints_age_s<1?"live":`${fmt(d.joints_age_s)} old`]]);
 $("logs").innerHTML=d.logs.slice().reverse().map(l=>`<div class="log ${l.level}"><span class="n">${new Date(l.t*1000).toLocaleTimeString()} ${l.node}</span> ${esc(l.msg)}</div>`).join("");
 img("live",d.live_age_s);img("det",d.detection_age_s);
}
function img(id,age){const el=$(id),none=$(id+"none");
 if(age==null){el.style.display="none";none.style.display="block";return}
 el.style.display="block";none.style.display="none";
 const src=`/image/${id=="live"?"live":"detection"}.jpg?${Date.now()}`;
 const pre=new Image();pre.onload=()=>el.src=src;pre.src=src;}
tick();setInterval(tick,700);
</script></body></html>
"""


def main(args=None):
    rclpy.init(args=args)
    node = StatusWeb()
    try:
        server = node.serve()
    except OSError as e:
        node.get_logger().fatal(f"cannot open {node.host}:{node.port}: {e}")
        node.destroy_node()
        rclpy.try_shutdown()
        raise SystemExit(1) from e
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        server.shutdown()
        with contextlib.suppress(KeyboardInterrupt):
            node.destroy_node()
            rclpy.try_shutdown()
