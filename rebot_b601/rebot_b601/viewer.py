"""Browser 3D viewer ("virtual clone") of the arm.

Serves a single self-contained page on http://127.0.0.1:<port>/ that polls
``/state`` (a JSON snapshot from :meth:`Arm.snapshot`) and draws the arm from
its forward kinematics.  It is read-only: it never sends commands.  It shows
whatever the connected :class:`Arm` is doing, simulated or real (then it is a
digital twin of the real arm, fed by the measured joint angles).

With the CAD meshes fetched (``python -m rebot_b601 fetch-assets``) the page
renders the real parts with three.js; without them it falls back to a
kinematic skeleton (joint origins, gripper fingers, TCP axes).
"""

from __future__ import annotations

import json
import logging
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote

from .assets import ASSETS_DIR

log = logging.getLogger("rebot_b601.viewer")

SKELETON_PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>reBot B601 twin</title>
<style>
:root{--bg:#f4f5f7;--fg:#1c1f24;--mut:#6b7280;--grid:#d5d9e0;--arm:#1f5fbf;--joint:#0b2e63;--cmd:#9db8e8;--ok:#1a7f4b;--bad:#c62828;--tcp:#e07b00;--tgt:#8e24aa;--panel:#ffffffcc}
@media (prefers-color-scheme:dark){:root{--bg:#14171c;--fg:#e6e8eb;--mut:#8b93a1;--grid:#2c313a;--arm:#5aa2ff;--joint:#cfe3ff;--cmd:#3a557f;--ok:#4cc38a;--bad:#ff6b6b;--tcp:#ffb347;--tgt:#d28bff;--panel:#1c2027cc}}
html,body{margin:0;height:100%;background:var(--bg);color:var(--fg);font:14px/1.4 system-ui,sans-serif}
canvas{position:fixed;inset:0;width:100%;height:100%;touch-action:none;cursor:grab}
#hud{position:fixed;left:16px;top:16px;background:var(--panel);backdrop-filter:blur(6px);padding:10px 14px;border-radius:10px;max-width:calc(100vw - 32px);font-variant-numeric:tabular-nums}
#hud h1{font-size:14px;margin:0 0 6px;font-weight:600}
#hud .row{display:flex;gap:10px;justify-content:space-between}
#hud .k{color:var(--mut)}
#badge{display:inline-block;padding:1px 8px;border-radius:99px;font-size:12px;color:#fff;background:var(--mut)}
#help{position:fixed;right:16px;bottom:12px;color:var(--mut);font-size:12px}
</style></head><body>
<canvas id="c"></canvas>
<div id="hud"><h1>reBot B601-RS twin <span id="badge">connecting</span></h1><div id="rows"></div></div>
<div id="help">drag: rotate · wheel: zoom · double-click: reset · trail = TCP path</div>
<script>
const cv=document.getElementById('c'),ctx=cv.getContext('2d');
let W=0,H=0,dpr=1;function resize(){dpr=window.devicePixelRatio||1;W=innerWidth;H=innerHeight;cv.width=W*dpr;cv.height=H*dpr;ctx.setTransform(dpr,0,0,dpr,0,0)}
addEventListener('resize',resize);resize();
const css=n=>getComputedStyle(document.documentElement).getPropertyValue(n).trim();
// camera (z up)
const cam0={yaw:-0.9,pitch:0.45,dist:1.5,tx:0.2,ty:0.0,tz:0.2};let cam={...cam0};
let drag=null;
cv.addEventListener('pointerdown',e=>{drag={x:e.clientX,y:e.clientY};cv.setPointerCapture(e.pointerId);cv.style.cursor='grabbing'});
cv.addEventListener('pointerup',()=>{drag=null;cv.style.cursor='grab'});
cv.addEventListener('pointermove',e=>{if(!drag)return;cam.yaw-=(e.clientX-drag.x)*0.006;cam.pitch=Math.max(-1.4,Math.min(1.5,cam.pitch+(e.clientY-drag.y)*0.006));drag={x:e.clientX,y:e.clientY}});
cv.addEventListener('wheel',e=>{e.preventDefault();cam.dist=Math.max(0.4,Math.min(5,cam.dist*Math.exp(e.deltaY*0.001)))},{passive:false});
cv.addEventListener('dblclick',()=>{cam={...cam0}});
function proj(p){ // world -> screen, camera looks at target from yaw/pitch/dist
  const cy=Math.cos(cam.yaw),sy=Math.sin(cam.yaw),cp=Math.cos(cam.pitch),sp=Math.sin(cam.pitch);
  const x=p[0]-cam.tx,y=p[1]-cam.ty,z=p[2]-cam.tz;
  const x1=cy*x+sy*y, y1=-sy*x+cy*y;               // rotate about z
  // eye looks along +y1 after pitch: rotate about x
  const yy=cp*y1 - sp*z, zz=sp*y1 + cp*z;
  const d=cam.dist+yy; const f=Math.min(W,H)*1.1/Math.max(0.05,d);
  return [W/2+x1*f, H/2 - zz*f, d];
}
function line(a,b,col,w,dash){const A=proj(a),B=proj(b);ctx.strokeStyle=col;ctx.lineWidth=w;ctx.setLineDash(dash||[]);ctx.beginPath();ctx.moveTo(A[0],A[1]);ctx.lineTo(B[0],B[1]);ctx.stroke();ctx.setLineDash([])}
function dot(p,r,col){const A=proj(p);ctx.fillStyle=col;ctx.beginPath();ctx.arc(A[0],A[1],r,0,7);ctx.fill()}
function ring(c,r,z,col,n=48){ctx.strokeStyle=col;ctx.lineWidth=1;ctx.beginPath();for(let i=0;i<=n;i++){const a=i/n*6.2832,A=proj([c[0]+r*Math.cos(a),c[1]+r*Math.sin(a),z]);i?ctx.lineTo(A[0],A[1]):ctx.moveTo(A[0],A[1])}ctx.stroke()}
const add=(a,b)=>[a[0]+b[0],a[1]+b[1],a[2]+b[2]];const mul=(a,k)=>[a[0]*k,a[1]*k,a[2]*k];
let S=null,trail=[];
async function poll(){try{const r=await fetch('/state',{cache:'no-store'});S=await r.json();
  if(S.moving||S.connected){const t=S.tcp;const l=trail[trail.length-1];if(!l||Math.hypot(t[0]-l[0],t[1]-l[1],t[2]-l[2])>0.004){trail.push(t);if(trail.length>600)trail.shift()}}
  hud()}catch(e){S=null;badge('no server','var(--bad)')}setTimeout(poll,80)}
function badge(t,c){const b=document.getElementById('badge');b.textContent=t;b.style.background=c}
function hud(){let t,c;
  if(!S.connected){t='not connected (home pose)';c='var(--mut)'}else if(S.fault){t='FAULT';c='var(--bad)'}else if(S.moving){t='moving';c='var(--tcp)'}else{t=(S.simulated?'simulated · ':'live · ')+(S.torque_enabled?'holding':'torque off');c='var(--ok)'}
  badge(t,c);const f=(a,n=1)=>a.map(v=>v.toFixed(n)).join(', ');
  const rows=[['joints °',f(S.joints_deg)],['TCP m',f(S.tcp,3)],['gripper',(S.gripper_opening*100).toFixed(0)+'% open']];
  if(S.target)rows.push(['target m',f(S.target,3)]);if(S.fault)rows.push(['fault',S.fault]);
  document.getElementById('rows').innerHTML=rows.map(r=>`<div class="row"><span class="k">${r[0]}</span><span>${r[1]}</span></div>`).join('')}
function draw(){requestAnimationFrame(draw);ctx.clearRect(0,0,W,H);if(!S)return;
  const grid=css('--grid'),arm=css('--arm'),joint=css('--joint'),tcp=css('--tcp'),tgt=css('--tgt');
  // table grid (z=0) and workspace hint
  for(let i=-6;i<=6;i++){const v=i*0.1;line([v,-0.6,0],[v,0.6,0],grid,i?0.5:1.2);line([-0.6,v,0],[0.6,v,0],grid,i?0.5:1.2)}
  ring([0,0,0],0.06,0,grid);ring([0,0,0],0.06,0.075,grid);
  line([0,0,0],[0.15,0,0],'#d33',2);line([0,0,0],[0,0.15,0],'#2a2',2);line([0,0,0],[0,0,0.15],'#33d',2);
  // trail
  if(trail.length>1){ctx.strokeStyle=tcp;ctx.globalAlpha=0.45;ctx.lineWidth=1.5;ctx.beginPath();trail.forEach((p,i)=>{const A=proj(p);i?ctx.lineTo(A[0],A[1]):ctx.moveTo(A[0],A[1])});ctx.stroke();ctx.globalAlpha=1}
  // target marker
  if(S.target){const t=S.target;line([t[0]-.02,t[1],t[2]],[t[0]+.02,t[1],t[2]],tgt,2);line([t[0],t[1]-.02,t[2]],[t[0],t[1]+.02,t[2]],tgt,2);line([t[0],t[1],t[2]-.02],[t[0],t[1],t[2]+.02],tgt,2);ring([t[0],t[1]],0.025,t[2],tgt);line([t[0],t[1],0],t,tgt,1,[3,4])}
  // shadow on the table
  S.points.forEach((p,i)=>{if(i){const q=S.points[i-1];line([q[0],q[1],0],[p[0],p[1],0],grid,2)}});
  // links + joints
  const P=S.points;for(let i=1;i<P.length;i++)line(P[i-1],P[i],arm,7);
  for(let i=0;i<P.length-1;i++)dot(P[i],i?6:8,joint);
  // gripper: TCP frame rows are the axes (x = approach); fingers slide along +-y of the TCP frame
  const T=S.tcp,ax=S.tcp_axes[0],ay=S.tcp_axes[1],az=S.tcp_axes[2];const o=S.gripper_opening*0.05;
  const at=(x,y)=>add(add(T,mul(ax,x)),mul(ay,y));
  const fl=at(-0.042,o+0.004),fr=at(-0.042,-o-0.004),tl=at(0,o+0.004),tr=at(0,-o-0.004);
  line(at(-0.042,-0.06),at(-0.042,0.06),arm,5);line(fl,tl,tcp,4);line(fr,tr,tcp,4);
  dot(T,4,tcp);line(T,add(T,mul(ax,0.06)),'#d33',2);line(T,add(T,mul(ay,0.04)),'#2a2',2);line(T,add(T,mul(az,0.04)),'#33d',2);
}
poll();draw();
</script></body></html>
"""


MESH_PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>reBot B601 twin</title>
<style>
:root{--bg:#f4f5f7;--fg:#1c1f24;--mut:#6b7280;--ok:#1a7f4b;--bad:#c62828;--busy:#e07b00;--panel:#ffffffcc}
@media (prefers-color-scheme:dark){:root{--bg:#14171c;--fg:#e6e8eb;--mut:#8b93a1;--ok:#4cc38a;--bad:#ff6b6b;--busy:#ffb347;--panel:#1c2027cc}}
html,body{margin:0;height:100%;background:var(--bg);color:var(--fg);font:14px/1.4 system-ui,sans-serif;overflow:hidden}
canvas{display:block;touch-action:none}
#hud{position:fixed;left:16px;top:16px;background:var(--panel);backdrop-filter:blur(6px);padding:10px 14px;border-radius:10px;max-width:calc(100vw - 32px);font-variant-numeric:tabular-nums}
#hud h1{font-size:14px;margin:0 0 6px;font-weight:600}
#hud .row{display:flex;gap:10px;justify-content:space-between}
#hud .k{color:var(--mut)}
#badge{display:inline-block;padding:1px 8px;border-radius:99px;font-size:12px;color:#fff;background:var(--mut)}
#help{position:fixed;right:16px;bottom:12px;color:var(--mut);font-size:12px}
</style>
<script type="importmap">{"imports":{"three":"/assets/three/three.module.min.js"}}</script>
</head><body>
<div id="hud"><h1>reBot B601-RS twin <span id="badge">loading</span></h1><div id="rows"></div></div>
<div id="help">drag: orbit · right-drag: pan · wheel: zoom · double-click: reset view</div>
<script type="module">
import * as THREE from 'three';
import {STLLoader} from '/assets/three/STLLoader.js';
import {OrbitControls} from '/assets/three/OrbitControls.js';

const dark=matchMedia('(prefers-color-scheme: dark)').matches;
const renderer=new THREE.WebGLRenderer({antialias:true});
renderer.setPixelRatio(Math.min(devicePixelRatio||1,2));
renderer.setClearColor(dark?0x14171c:0xf4f5f7);
document.body.prepend(renderer.domElement);
const scene=new THREE.Scene();
const camera=new THREE.PerspectiveCamera(45,1,0.01,30);
function resize(){renderer.setSize(innerWidth,innerHeight);camera.aspect=innerWidth/innerHeight;camera.updateProjectionMatrix()}
addEventListener('resize',resize);resize();
// robot frame is z-up (URDF); three.js is y-up, so everything robot-related lives in `world`
const world=new THREE.Group();world.rotation.x=-Math.PI/2;scene.add(world);
const cam0=()=>{camera.position.set(0.95,0.75,0.95);controls.target.set(0.2,0.18,0);controls.update()};
const controls=new OrbitControls(camera,renderer.domElement);controls.enableDamping=true;cam0();
renderer.domElement.addEventListener('dblclick',cam0);
scene.add(new THREE.HemisphereLight(0xffffff,dark?0x30343c:0x8a8f99,1.6));
const sun=new THREE.DirectionalLight(0xffffff,2.2);sun.position.set(1.2,2.0,0.8);scene.add(sun);
const fill=new THREE.DirectionalLight(0xffffff,0.8);fill.position.set(-1.5,1.0,-1.0);scene.add(fill);
const grid=new THREE.GridHelper(1.2,12,dark?0x55606f:0x9aa3b0,dark?0x2c313a:0xd5d9e0);scene.add(grid);
world.add(new THREE.AxesHelper(0.15));

// ---- meshes -----------------------------------------------------------------
const linkGroups={};
let toLoad=0,loaded=0;const badge=(t,c)=>{const b=document.getElementById('badge');b.textContent=t;b.style.background=c||'var(--mut)'};
const manifest=await (await fetch('/assets/manifest.json')).json();
const loader=new STLLoader();
for(const [name,parts] of Object.entries(manifest.links)){
  const g=new THREE.Group();g.matrixAutoUpdate=false;world.add(g);linkGroups[name]=g;
  for(const p of parts){
    toLoad++;
    loader.load('/assets/'+p.mesh,geo=>{
      const m=new THREE.Mesh(geo,new THREE.MeshStandardMaterial({color:p.color,metalness:0.25,roughness:0.55}));
      m.position.set(...p.xyz);m.rotation.set(p.rpy[0],p.rpy[1],p.rpy[2],'ZYX');   // URDF rpy = Rz*Ry*Rx
      g.add(m);loaded++;if(loaded===toLoad)badge('ready');
    },undefined,e=>{console.error('mesh failed',p.mesh,e);badge('mesh error','var(--bad)')});
  }
}
// TCP frame axes on the gripper (x = approach), trail, target marker
if(linkGroups.gripper_end)linkGroups.gripper_end.add(new THREE.AxesHelper(0.07));
const MAXT=800;const trailPos=new Float32Array(MAXT*3);const trailGeo=new THREE.BufferGeometry();
trailGeo.setAttribute('position',new THREE.BufferAttribute(trailPos,3));trailGeo.setDrawRange(0,0);
const trail=new THREE.Line(trailGeo,new THREE.LineBasicMaterial({color:0xe07b00}));trail.frustumCulled=false;world.add(trail);
let nTrail=0;
const target=new THREE.Mesh(new THREE.SphereGeometry(0.012,16,12),new THREE.MeshBasicMaterial({color:0x8e24aa}));target.visible=false;world.add(target);
const stem=new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(),new THREE.Vector3()]),new THREE.LineDashedMaterial({color:0x8e24aa,dashSize:0.01,gapSize:0.01}));stem.visible=false;world.add(stem);

// ---- state polling ----------------------------------------------------------
let S=null;
function applyState(){
  for(const [name,m] of Object.entries(S.links)){const g=linkGroups[name];if(!g)continue;g.matrix.set(...m);g.matrixWorldNeedsUpdate=true}
  const t=S.tcp,last=nTrail?[trailPos[(nTrail-1)*3],trailPos[(nTrail-1)*3+1],trailPos[(nTrail-1)*3+2]]:null;
  if(S.connected&&(!last||Math.hypot(t[0]-last[0],t[1]-last[1],t[2]-last[2])>0.004)){
    if(nTrail===MAXT){trailPos.copyWithin(0,3);nTrail--}
    trailPos.set(t,nTrail*3);nTrail++;trailGeo.setDrawRange(0,nTrail);trailGeo.attributes.position.needsUpdate=true}
  if(S.target){target.visible=stem.visible=true;target.position.set(...S.target);
    stem.geometry.setFromPoints([new THREE.Vector3(S.target[0],S.target[1],0),new THREE.Vector3(...S.target)]);stem.computeLineDistances()}
  else target.visible=stem.visible=false;
  hud();
}
function hud(){
  if(loaded<toLoad){badge(`loading meshes ${loaded}/${toLoad}`,'var(--mut)')}
  else if(!S.connected)badge('not connected (home pose)','var(--mut)');
  else if(S.fault)badge('FAULT','var(--bad)');
  else if(S.moving)badge('moving','var(--busy)');
  else badge((S.simulated?'simulated · ':'live · ')+(S.torque_enabled?'holding':'torque off'),'var(--ok)');
  const f=(a,n=1)=>a.map(v=>v.toFixed(n)).join(', ');
  const rows=[['joints °',f(S.joints_deg)],['TCP m',f(S.tcp,3)],['gripper',(S.gripper_opening*100).toFixed(0)+'% open']];
  if(S.target)rows.push(['target m',f(S.target,3)]);if(S.fault)rows.push(['fault',S.fault]);
  document.getElementById('rows').innerHTML=rows.map(r=>`<div class="row"><span class="k">${r[0]}</span><span>${r[1]}</span></div>`).join('');
}
async function poll(){try{S=await (await fetch('/state',{cache:'no-store'})).json();applyState()}catch(e){badge('no server','var(--bad)')}setTimeout(poll,80)}
poll();
renderer.setAnimationLoop(()=>{controls.update();renderer.render(scene,camera)});
</script></body></html>
"""


_MIME = {".js": "text/javascript", ".json": "application/json", ".stl": "model/stl", ".urdf": "application/xml"}


def make_handler(arm, assets_dir: Path):
    assets_dir = Path(assets_dir).resolve()

    def page() -> bytes:
        # CAD view when the meshes have been fetched, otherwise the skeleton fallback
        return (MESH_PAGE if (assets_dir / "manifest.json").is_file() else SKELETON_PAGE).encode()

    class Handler(BaseHTTPRequestHandler):
        def _send(self, code: int, body: bytes, ctype: str, cache: str = "no-store") -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", cache)
            self.end_headers()
            self.wfile.write(body)

        def _asset(self, rel: str) -> None:
            try:
                target = (assets_dir / rel).resolve()
                inside = target.is_relative_to(assets_dir)
            except (OSError, ValueError):
                inside = False
            ctype = _MIME.get(target.suffix.lower()) if inside else None
            if not inside or ctype is None or not target.is_file():
                self._send(404, b"not found", "text/plain")
                return
            self._send(200, target.read_bytes(), ctype, cache="public, max-age=3600")

        def do_GET(self):  # noqa: N802
            path = unquote(self.path.split("?", 1)[0])
            if path in ("/", "/index.html"):
                self._send(200, page(), "text/html; charset=utf-8")
            elif path.startswith("/assets/"):
                self._asset(path[len("/assets/"):])
            elif path == "/favicon.ico":
                self._send(204, b"", "image/x-icon")
            elif path == "/state":
                try:
                    self._send(200, json.dumps(arm.snapshot()).encode(), "application/json")
                except Exception as e:  # never crash the server thread
                    self._send(500, json.dumps({"error": repr(e)}).encode(), "application/json")
            else:
                self._send(404, b"not found", "text/plain")

        def log_message(self, *args):  # silence per-request logging (stdout may be the MCP channel)
            pass

    return Handler


def start_viewer(arm, port: int = 8765, host: str = "127.0.0.1", assets_dir: Path | None = None) -> tuple[ThreadingHTTPServer, str]:
    """Start the viewer in a daemon thread.  ``port=0`` picks a free port.  Returns (server, url)."""
    assets = ASSETS_DIR if assets_dir is None else Path(assets_dir)
    server = ThreadingHTTPServer((host, port), make_handler(arm, assets))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, name="rebot-viewer", daemon=True).start()
    url = f"http://{host}:{server.server_address[1]}/"
    if (assets / "manifest.json").is_file():
        log.info("3D viewer (CAD meshes) at %s", url)
    else:
        log.info("3D viewer (skeleton; run `python -m rebot_b601 fetch-assets` for the CAD meshes) at %s", url)
    return server, url
