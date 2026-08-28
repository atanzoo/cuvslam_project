#!/usr/bin/env python3
"""Simple dependency-free Mac monitor for JSON telemetry sent over UDP.

UDP input: 0.0.0.0:8876
Web dashboard: http://127.0.0.1:8875
"""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import socket
import threading


HTTP_PORT = 8875
UDP_PORT = 8876
state = {"status": "waiting", "message": "等待 Ubuntu／Jetson 傳送 D1 telemetry"}
lock = threading.Lock()


def udp_receiver() -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", UDP_PORT))
    while True:
        payload, _ = sock.recvfrom(65535)
        try:
            incoming = json.loads(payload.decode("utf-8"))
            with lock:
                state.clear()
                state.update(incoming)
                state["status"] = "connected"
        except (UnicodeDecodeError, json.JSONDecodeError):
            continue


PAGE = r'''<!doctype html>
<html lang="zh-Hant"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>D1 Edu Mac Monitor</title>
<style>
body{font-family:-apple-system,BlinkMacSystemFont,"PingFang TC",sans-serif;background:#101820;color:#eef5f4;margin:0;padding:32px}
main{max-width:900px;margin:auto}h1{font-size:28px;font-weight:500}.status{padding:12px 16px;border-radius:10px;background:#20343b;margin-bottom:20px}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(190px,1fr));gap:14px}.card{background:#17262c;border:1px solid #31515b;border-radius:10px;padding:18px}.label{color:#9fb7bc;font-size:13px}.value{font-size:26px;margin-top:8px}.wide{grid-column:1/-1}.mono{font-family:ui-monospace,monospace;white-space:pre-wrap;word-break:break-word;color:#c8d7d8}@media(max-width:600px){body{padding:18px}}
</style><main><h1>智元 D1 Edu · Mac 監看</h1><div class="status" id="status">等待資料…</div><div class="grid">
<div class="card"><div class="label">位置 X</div><div class="value" id="x">—</div></div>
<div class="card"><div class="label">位置 Y</div><div class="value" id="y">—</div></div>
<div class="card"><div class="label">機身高度 Z</div><div class="value" id="z">—</div></div>
<div class="card"><div class="label">速度 vx</div><div class="value" id="vx">—</div></div>
<div class="card"><div class="label">Roll / Pitch / Yaw</div><div class="value" id="rpy">—</div></div>
<div class="card"><div class="label">電池</div><div class="value" id="battery">—</div></div>
<div class="card wide"><div class="label">關節狀態</div><div class="mono" id="joints">—</div></div>
</div></main><script>
const f=(v)=>typeof v==='number'?v.toFixed(3):'—';
async function poll(){try{const r=await fetch('/state?'+Date.now());const s=await r.json();
document.querySelector('#status').textContent=(s.status||'connected')+' · '+new Date().toLocaleTimeString();
for(const k of ['x','y','z','vx','battery'])document.querySelector('#'+k).textContent=f(s[k]);
document.querySelector('#rpy').textContent=Array.isArray(s.rpy)?s.rpy.map(f).join(' / '):'—';
document.querySelector('#joints').textContent=Array.isArray(s.joints)?s.joints.map(f).join(', '):(s.joints||'—');
}catch(e){document.querySelector('#status').textContent='尚未收到資料';}}setInterval(poll,250);poll();
</script></html>'''


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):  # noqa: N802
        if self.path.startswith("/state"):
            with lock:
                payload = json.dumps(state, ensure_ascii=False).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(payload)
            return
        body = PAGE.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *_):
        return


if __name__ == "__main__":
    threading.Thread(target=udp_receiver, daemon=True).start()
    print(f"Mac monitor: http://127.0.0.1:{HTTP_PORT}")
    print(f"Listening for UDP telemetry on 0.0.0.0:{UDP_PORT}")
    ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), Handler).serve_forever()
