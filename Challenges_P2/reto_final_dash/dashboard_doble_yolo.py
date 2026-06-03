import cv2
import base64
import time
import math
import threading
import numpy as np
import requests

from nicegui import ui, app
from fastapi.responses import StreamingResponse

# CONFIGURACIÓN

ROBOT_IP       = "10.241.140.114"
ROBOT_PORT     = 5000

# Streams de video del robot (HTTP MJPEG desde camera_web_full.py)
RAW_FEED       = f"http://{ROBOT_IP}:{ROBOT_PORT}/video_raw"       # stream completo
YOLO_FEED      = f"http://{ROBOT_IP}:{ROBOT_PORT}/video_yolo"       # mismo stream
LINE_FEED      = f"http://{ROBOT_IP}:{ROBOT_PORT}/video_image"       # mismo stream

# Endpoints de datos del robot
LINE_ERROR_URL     = f"http://{ROBOT_IP}:{ROBOT_PORT}/line_error"
YOLO_COMMAND_URL   = f"http://{ROBOT_IP}:{ROBOT_PORT}/yolo_command"
YOLO_SIGN_AREA_URL = f"http://{ROBOT_IP}:{ROBOT_PORT}/yolo_sign_area"
COLOR_URL          = f"http://{ROBOT_IP}:{ROBOT_PORT}/color"
FINISH_LINE_URL    = f"http://{ROBOT_IP}:{ROBOT_PORT}/finish_line"
INTERSECTION_URL   = f"http://{ROBOT_IP}:{ROBOT_PORT}/intersection_line"


# ESTADO GLOBAL
state = {
    # Error de línea
    "line_error":       0.0,
    "line_error_px":    0,
    "error_values":     [],
    "desired_values":   [],
    "error_avg":        0.0,
    # YOLO
    "yolo_command":     "none",
    "yolo_sign_area":   0.0,
    # Color semáforo
    "color_raw":        0.0,
    "color_str":        "Nada",
    "color_css":        "#64748b",
    # Flags
    "finish_line":      False,
    "intersection":     False,
    # Retroalimentación
    "feedback_title":   "Esperando datos",
    "feedback_text":    "Conectando con el robot...",
    # Tiempo
    "t0":               time.time(),
    # Velocidad simulada (hasta que llegue /cmd_vel real)
    "speed":            0.0,
    "angular_speed":    0.0,
}

COLOR_MAP = {
    0.0: ("Nada",     "#64748b"),
    1.0: ("Amarillo", "#eab308"),
    2.0: ("Verde",    "#22c55e"),
    3.0: ("Rojo",     "#ef4444"),
}

COMMAND_ICONS = {
    "turn_right":      "→ Girar derecha",
    "turn_left":       "← Girar izquierda",
    "stop":            "⛔ Detenerse",
    "roadwork_ahead":  "⚠ Obra en camino",
    "give_way":        "⚡ Ceder paso",
    "straight":        "↑ Seguir recto",
    "none":            "— Sin señal",
}



# LECTOR DE DATOS POR HTTP


def fetch_json(url, key, default):
    try:
        r = requests.get(url, timeout=0.25)
        return r.json().get(key, default)
    except Exception:
        return default

def fetch_float(url, key, default=0.0):
    try:
        r = requests.get(url, timeout=0.25)
        d = r.json()
        return float(d.get(key, default))
    except Exception:
        return default

def fetch_bool(url, key, default=False):
    try:
        r = requests.get(url, timeout=0.25)
        d = r.json()
        return bool(d.get(key, default))
    except Exception:
        return default

def fetch_str(url, key, default="none"):
    try:
        r = requests.get(url, timeout=0.25)
        d = r.json()
        return str(d.get(key, default))
    except Exception:
        return default

def data_loop():
    """Hilo que lee todos los tópicos del robot por HTTP cada 150ms."""
    t = 0
    while True:
        try:
            # /line_error  → {"line_error": -24}
            err_px = fetch_float(LINE_ERROR_URL, "line_error", 0.0)
            state["line_error_px"] = int(err_px)
            state["line_error"]    = max(-1.0, min(1.0, err_px / 320.0))

            state["error_values"].append(round(state["line_error"], 3))
            state["desired_values"].append(0)
            if len(state["error_values"]) > 130:
                state["error_values"].pop(0)
                state["desired_values"].pop(0)

            vals = state["error_values"]
            state["error_avg"] = sum(abs(e) for e in vals) / max(1, len(vals))

            # /yolo/command  → {"yolo_command": "stop"}
            state["yolo_command"] = fetch_str(YOLO_COMMAND_URL, "yolo_command", "none")

            # /yolo/sign_area → {"yolo_sign_area": 1234.5}
            state["yolo_sign_area"] = fetch_float(YOLO_SIGN_AREA_URL, "yolo_sign_area", 0.0)

            # /color → {"color": 2.0}
            c = fetch_float(COLOR_URL, "color", 0.0)
            state["color_raw"] = c
            label, css = COLOR_MAP.get(c, ("?", "#64748b"))
            state["color_str"] = label
            state["color_css"] = css

            # /finish_line → {"finish_line": true}
            state["finish_line"] = fetch_bool(FINISH_LINE_URL, "finish_line", False)

            # /intersection_line → {"intersection_line": false}
            state["intersection"] = fetch_bool(INTERSECTION_URL, "intersection_line", False)

            # Velocidad simulada hasta conectar /cmd_vel
            t += 0.15
            state["speed"]         = 0.24 + 0.06 * math.sin(t / 2)
            state["angular_speed"] = 0.12 * math.cos(t / 3)

            # Retroalimentación basada en error
            err = abs(state["line_error"])
            if state["finish_line"]:
                state["feedback_title"] = "🏁 Meta detectada"
                state["feedback_text"]  = "Se detectó la franja de meta."
            elif state["intersection"]:
                state["feedback_title"] = "✦ Intersección detectada"
                state["feedback_text"]  = "Línea punteada horizontal — evaluando acción."
            elif err < 0.15:
                state["feedback_title"] = "Siguiendo trayectoria correctamente"
                state["feedback_text"]  = "El robot se encuentra dentro del margen aceptable."
            elif err < 0.30:
                state["feedback_title"] = "Corrigiendo trayectoria"
                state["feedback_text"]  = "Desviación moderada, puede corregir."
            else:
                state["feedback_title"] = "Desviación alta"
                state["feedback_text"]  = "Revisar control, velocidad o detección visual."

        except Exception as e:
            print(f"[data_loop] {e}")

        time.sleep(0.15)

threading.Thread(target=data_loop, daemon=True).start()




# PROXY DE STREAMS MJPEG
# (re-expone los 3 streams del robot para el browser)

def _proxy_stream(url):
    """Abre el stream MJPEG del robot y lo retransmite al browser."""
    def generate():
        while True:
            try:
                resp = requests.get(url, stream=True, timeout=5)
                buf  = b""
                for chunk in resp.iter_content(chunk_size=4096):
                    buf += chunk
                    a = buf.find(b"\xff\xd8")
                    b_ = buf.find(b"\xff\xd9")
                    if a != -1 and b_ != -1 and b_ > a:
                        jpg = buf[a: b_ + 2]
                        buf = buf[b_ + 2:]
                        yield (
                            b"--frame\r\n"
                            b"Content-Type: image/jpeg\r\n\r\n" +
                            jpg +
                            b"\r\n"
                        )
            except Exception as e:
                print(f"[proxy] {url} → {e}")
                blank = np.zeros((240, 320, 3), dtype=np.uint8)
                blank[:] = (15, 23, 42)
                cv2.putText(blank, "Sin señal", (70, 120),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (100, 120, 150), 2)
                _, enc = cv2.imencode(".jpg", blank)
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n" +
                    enc.tobytes() +
                    b"\r\n"
                )
                time.sleep(1)
    return StreamingResponse(generate(), media_type="multipart/x-mixed-replace; boundary=frame")


@app.get("/stream/raw")
def stream_raw():
    return _proxy_stream(RAW_FEED)

@app.get("/stream/yolo")
def stream_yolo():
    return _proxy_stream(YOLO_FEED)

@app.get("/stream/line")
def stream_line():
    return _proxy_stream(LINE_FEED)









def track_svg(progress=0.0, lap=1):
    points = [
        (95,440),(210,440),(330,440),(500,440),(680,430),
        (760,365),(760,220),(725,105),(600,80),(470,105),
        (340,85),(190,100),(105,160),(100,270),(190,305),
        (300,275),(355,205),(420,280),(475,335),(550,315),
        (610,215),(690,200),(720,285),(695,370),(570,380),
        (440,335),(310,295),(180,320),(105,375),(95,440)
    ]
    prog = 1.0 if lap >= 2 else progress
    n = max(2, int(prog * len(points)))
    vp = points[:n]
    cx, cy = points[n] if n < len(points) else points[-1]

    def path(p):
        s = f"M {p[0][0]} {p[0][1]} "
        for x, y in p[1:]: s += f"L {x} {y} "
        return s

    pct = int(prog * 100)
    return f"""
    <svg width="100%" height="100%" viewBox="0 0 860 520" xmlns="http://www.w3.org/2000/svg">
      <defs>
        <linearGradient id="g" x1="0" y1="0" x2="1" y2="1">
          <stop offset="0%" stop-color="#166534"/>
          <stop offset="100%" stop-color="#22c55e"/>
        </linearGradient>
        <filter id="sh"><feDropShadow dx="0" dy="4" stdDeviation="4" flood-opacity="0.4"/></filter>
      </defs>
      <rect x="0" y="0" width="860" height="520" rx="14" fill="url(#g)"/>
      <path d="{path(points)}" fill="none" stroke="#f59e0b" stroke-width="90" stroke-linecap="round" stroke-linejoin="round" opacity="0.25"/>
      <path d="{path(points)}" fill="none" stroke="#111827" stroke-width="72" stroke-linecap="round" stroke-linejoin="round" opacity="0.3"/>
      <path d="{path(vp)}" fill="none" stroke="#f59e0b" stroke-width="90" stroke-linecap="round" stroke-linejoin="round" filter="url(#sh)"/>
      <path d="{path(vp)}" fill="none" stroke="#1f2937" stroke-width="72" stroke-linecap="round" stroke-linejoin="round"/>
      <path d="{path(vp)}" fill="none" stroke="#374151" stroke-width="48" stroke-linecap="round" stroke-linejoin="round"/>
      <path d="{path(vp)}" fill="none" stroke="white" stroke-width="3" stroke-dasharray="14 18" stroke-linecap="round" opacity="0.8"/>
      <g transform="translate({cx-20},{cy-20})" filter="url(#sh)">
        <path d="M4 28 L38 10 L42 40 Z" fill="#facc15" stroke="#ca8a04" stroke-width="2.5"/>
        <circle cx="20" cy="25" r="3.5" fill="#b45309"/>
        <circle cx="32" cy="31" r="2.5" fill="#b45309"/>
        <circle cx="29" cy="17" r="2.5" fill="#b45309"/>
      </g>
      <text x="18" y="32" fill="#d9f99d" font-size="20" font-weight="800">VUELTA: {lap}</text>
      <rect x="18" y="490" width="220" height="10" rx="5" fill="#14532d"/>
      <rect x="18" y="490" width="{max(6, int(220*prog))}" height="10" rx="5" fill="#22c55e"/>
      <text x="18" y="482" fill="white" font-size="14" font-weight="700">Progreso: {pct}%</text>
    </svg>"""






# ESTILO + INTERFAZ

ui.add_head_html("""
<link rel="preconnect" href="https://fonts.googleapis.com">
<link href="https://fonts.googleapis.com/css2?family=Space+Grotesk:wght@400;500;600;700&family=JetBrains+Mono:wght@400;700&display=swap" rel="stylesheet">
<style>
*{box-sizing:border-box;margin:0;padding:0}
html,body{height:100%;overflow:hidden}
body{background:#070b14;font-family:"Space Grotesk",sans-serif;color:#e2e8f0}
#splash{position:fixed;inset:0;z-index:9999;background:#070b14;display:flex;flex-direction:column;align-items:center;justify-content:center;gap:22px;transition:opacity .5s}
#splash.hide{opacity:0;pointer-events:none}
.sp-cheese{font-size:72px;animation:bounce 1s infinite alternate}
@keyframes bounce{from{transform:translateY(0) rotate(-5deg)}to{transform:translateY(-16px) rotate(5deg)}}
.sp-title{font-size:26px;font-weight:700;color:#fff;letter-spacing:.5px}
.sp-msg{font-family:"JetBrains Mono",monospace;font-size:13px;color:#6366f1;min-height:20px}
.sp-barw{width:260px;height:3px;background:#1e293b;border-radius:2px;overflow:hidden}
.sp-bar{height:3px;background:linear-gradient(90deg,#6366f1,#a78bfa,#38bdf8);border-radius:2px;animation:ld 2.2s ease-in-out forwards}
@keyframes ld{from{width:0}to{width:100%}}
.sp-dots span{display:inline-block;width:7px;height:7px;border-radius:50%;background:#a78bfa;margin:0 3px;animation:dt .8s infinite alternate}
.sp-dots span:nth-child(2){animation-delay:.2s}
.sp-dots span:nth-child(3){animation-delay:.4s}
@keyframes dt{from{opacity:.2;transform:scale(.7)}to{opacity:1;transform:scale(1.2)}}
.db-wrap{display:grid;grid-template-rows:56px 1fr;height:100vh;width:100vw}
.db-header{display:flex;align-items:center;justify-content:space-between;padding:0 20px;background:rgba(7,11,20,0.97);border-bottom:1px solid rgba(99,102,241,0.2)}
.hlogo{font-size:17px;font-weight:700;color:#fff;display:flex;align-items:center;gap:10px}
.hlogo-e{font-size:28px}
.hsub{font-size:10px;font-weight:400;color:#6366f1;letter-spacing:.5px;margin-top:1px}
.hbadge{font-family:"JetBrains Mono",monospace;font-size:10px;padding:3px 11px;border-radius:20px;background:rgba(34,197,94,0.1);color:#4ade80;border:1px solid rgba(34,197,94,0.25);display:flex;align-items:center;gap:6px}
.hbadge::before{content:"";width:6px;height:6px;border-radius:50%;background:#4ade80;animation:pu 1.4s infinite}
@keyframes pu{0%,100%{opacity:1;transform:scale(1)}50%{opacity:.3;transform:scale(.6)}}
.hclock{font-family:"JetBrains Mono",monospace;font-size:15px;color:#94a3b8;letter-spacing:1px}
.db-body{display:grid;grid-template-columns:270px 1fr 270px;height:100%;overflow:hidden}
.db-col{display:flex;flex-direction:column;gap:8px;overflow-y:auto;overflow-x:hidden;padding:10px 8px;scrollbar-width:thin;scrollbar-color:rgba(99,102,241,0.25) transparent}
.db-col.center{border-left:1px solid rgba(99,102,241,0.12);border-right:1px solid rgba(99,102,241,0.12);padding:10px}
::-webkit-scrollbar{width:3px}
::-webkit-scrollbar-thumb{background:rgba(99,102,241,0.25);border-radius:2px}
.card{background:linear-gradient(145deg,rgba(15,23,42,0.92),rgba(8,14,28,0.96));border:1px solid rgba(99,102,241,0.16);border-radius:13px;padding:11px 13px;position:relative;overflow:hidden;flex-shrink:0}
.card::after{content:"";position:absolute;top:0;left:0;right:0;height:1px;background:linear-gradient(90deg,transparent,rgba(167,139,250,0.35),transparent)}
.pt{font-family:"JetBrains Mono",monospace;font-size:9px;font-weight:700;letter-spacing:1.5px;text-transform:uppercase;color:#6366f1;margin-bottom:8px;display:flex;align-items:center;gap:5px}
.pt-d{width:5px;height:5px;border-radius:50%;background:#6366f1;flex-shrink:0}
.vframe{width:100%;border-radius:9px;background:#030609;display:block;object-fit:contain;border:1px solid rgba(99,102,241,0.1)}
.vpair{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.irow{display:flex;justify-content:space-between;align-items:center;padding:4px 0;border-bottom:1px solid rgba(148,163,184,0.06);font-size:11px}
.irow:last-child{border:none}
.ikey{color:#64748b}
.ival{color:#e2e8f0;font-weight:600}
.g{color:#4ade80}.y{color:#fbbf24}.r{color:#f87171}.b{color:#60a5fa}.p{color:#c084fc}
.cmd{background:rgba(99,102,241,0.1);border:1px solid rgba(99,102,241,0.28);border-radius:9px;padding:8px 12px;text-align:center;font-size:14px;font-weight:700;color:#c084fc;margin:3px 0;transition:background .3s}
.semp{display:flex;align-items:center;gap:9px;background:rgba(2,6,23,0.65);border:1px solid rgba(148,163,184,0.1);border-radius:9px;padding:7px 11px;margin-top:5px}
.semd{width:13px;height:13px;border-radius:50%;flex-shrink:0;transition:background .4s,box-shadow .4s}
.seml{font-size:13px;font-weight:700;transition:color .4s}
.flag-row{display:flex;gap:7px;margin-top:5px}
.flag{flex:1;text-align:center;padding:6px;border-radius:8px;font-size:11px;font-weight:700;border:1px solid rgba(148,163,184,0.08);background:rgba(2,6,23,0.5);color:#334155;transition:all .3s}
.flag.of{background:#052e16;color:#4ade80;border-color:#166534;box-shadow:0 0 10px rgba(74,222,128,0.15)}
.flag.oi{background:#0c1a40;color:#60a5fa;border-color:#1d4ed8;box-shadow:0 0 10px rgba(96,165,250,0.15)}
.vel-n{font-size:36px;font-weight:700;text-align:center;color:#4ade80;line-height:1;font-family:"JetBrains Mono",monospace}
.vel-u{font-size:12px;color:#64748b;font-weight:400}
.vel-a{text-align:center;font-size:12px;color:#c084fc;font-weight:600;margin-top:3px}
.ebar{position:relative;height:8px;background:#0f172a;border-radius:4px;margin:8px 0;border:1px solid rgba(99,102,241,0.12)}
.efill{position:absolute;top:0;height:8px;border-radius:4px;transition:width .25s,left .25s,background .3s}
.emid{position:absolute;left:50%;top:-4px;width:1px;height:16px;background:rgba(99,102,241,0.35)}
.mms{display:grid;grid-template-columns:1fr 1fr;gap:6px}
.mm{background:rgba(2,6,23,0.65);border:1px solid rgba(99,102,241,0.13);border-radius:9px;padding:7px 8px;text-align:center}
.mml{font-size:8px;color:#475569;font-weight:700;letter-spacing:.8px;text-transform:uppercase;margin-bottom:1px}
.mmv{font-size:20px;font-weight:700;line-height:1.1;font-family:"JetBrains Mono",monospace}
.fbt{font-size:13px;font-weight:700;margin-bottom:4px;transition:color .3s}
.fbs{font-size:11px;color:#94a3b8;line-height:1.5}
</style>
""")

ui.add_body_html("""
<div id="splash">
  <div class="sp-cheese">&#x1F9C0;</div>
  <div class="sp-title">PuzzleBot Dashboard</div>
  <div class="sp-msg" id="spMsg">Inicializando sistema...</div>
  <div class="sp-barw"><div class="sp-bar"></div></div>
  <div class="sp-dots"><span></span><span></span><span></span></div>
</div>
<script>
const msgs=["Conectando con el robot...","Cargando streams de video...","Ligando t\u00f3picos ROS 2...","Calibrando detectores...","Quesito listo!"];
let i=0;const el=document.getElementById("spMsg");
const iv=setInterval(()=>{i++;if(i<msgs.length)el.textContent=msgs[i];else clearInterval(iv);},440);
setTimeout(()=>{document.getElementById("splash").classList.add("hide");setTimeout(()=>document.getElementById("splash").remove(),600);},2400);
</script>
""")

with ui.element("div").classes("db-wrap"):

    with ui.element("div").classes("db-header"):
        ui.html("<div class=\"hlogo\"><span class=\"hlogo-e\">&#x1F9C0;</span><div><div>PuzzleBot Dashboard</div><div class=\"hsub\">Challenge 5 &middot; ROS 2 Live</div></div></div>")
        with ui.row().classes("items-center gap-3"):
            ui.html("<div class=\"hbadge\">CONECTADO</div>")
            clock_label = ui.label("00:00:00").classes("hclock")

    with ui.element("div").classes("db-body"):

        with ui.element("div").classes("db-col"):

            with ui.element("div").classes("card"):
                ui.html("<div class=\"pt\"><span class=\"pt-d\"></span>/yolo/debug</div>")
                ui.html("<img src=\"/stream/yolo\" class=\"vframe\" style=\"height:170px;\">")

            with ui.element("div").classes("card"):
                ui.html("<div class=\"pt\"><span class=\"pt-d\"></span>/yolo/command</div>")
                yolo_command_label = ui.html("<div class=\"cmd\">&#8212; Sin se&#241;al</div>")
                ui.html("<div class=\"pt\" style=\"margin-top:9px\"><span class=\"pt-d\"></span>/yolo/sign_area</div>")
                with ui.element("div").classes("irow"):
                    ui.html("<span class=\"ikey\">&#xC1;rea BB (px&#xB2;)</span>")
                    sign_area_label = ui.html("<span class=\"ival y\">0</span>")
                ui.html("<div class=\"pt\" style=\"margin-top:9px\"><span class=\"pt-d\"></span>/color</div>")
                semaforo_html = ui.html("<div class=\"semp\"><div class=\"semd\" style=\"background:#475569\"></div><span class=\"seml\" style=\"color:#64748b\">Nada</span></div>")

            with ui.element("div").classes("card"):
                ui.html("<div class=\"pt\"><span class=\"pt-d\"></span>Detecciones especiales</div>")
                finish_html = ui.html("<div class=\"flag-row\"><div class=\"flag\">&#x1F3C1; Finish Line</div><div class=\"flag\">&#x2736; Intersecci&#xF3;n</div></div>")

            with ui.element("div").classes("card"):
                ui.html("<div class=\"pt\"><span class=\"pt-d\"></span>Velocidad</div>")
                speed_label   = ui.html("<div class=\"vel-n\">0.00 <span class=\"vel-u\">m/s</span></div>")
                angular_label = ui.html("<div class=\"vel-a\">&omega;: 0.00 rad/s</div>")

        with ui.element("div").classes("db-col center"):

            with ui.element("div").classes("card"):
                with ui.element("div").classes("vpair"):
                    with ui.element("div"):
                        ui.html("<div class=\"pt\"><span class=\"pt-d\"></span>/camera/raw</div>")
                        ui.html("<img src=\"/stream/raw\" class=\"vframe\" style=\"height:230px;\">")
                    with ui.element("div"):
                        ui.html("<div class=\"pt\"><span class=\"pt-d\"></span>/camera/image</div>")
                        ui.html("<img src=\"/stream/line\" class=\"vframe\" style=\"height:230px;\">")

            with ui.element("div").classes("card"):
                ui.html("<div class=\"pt\"><span class=\"pt-d\"></span>/line_error &#x2014; Error vs trayectoria</div>")
                error_chart = ui.echart({
                    "backgroundColor": "transparent",
                    "tooltip": {"trigger": "axis"},
                    "legend": {"data": ["Error","Deseado"],"textStyle":{"color":"#94a3b8","fontSize":10},"top":0,"right":0},
                    "grid": {"top":24,"bottom":18,"left":36,"right":10},
                    "xAxis": {"type":"category","data":[],"axisLabel":{"color":"#475569","fontSize":9},"axisLine":{"lineStyle":{"color":"#1e293b"}}},
                    "yAxis": {"type":"value","min":-1,"max":1,"axisLabel":{"color":"#475569","fontSize":9},"splitLine":{"lineStyle":{"color":"rgba(99,102,241,0.1)"}}},
                    "series": [
                        {"name":"Error","type":"line","data":[],"smooth":True,"lineStyle":{"width":2.5,"color":"#f87171"},"areaStyle":{"color":"rgba(248,113,113,0.07)"},"symbol":"none"},
                        {"name":"Deseado","type":"line","data":[],"smooth":False,"lineStyle":{"width":1,"type":"dashed","color":"#6366f1"},"symbol":"none"},
                    ],
                }).classes("w-full").style("height:150px")

        with ui.element("div").classes("db-col"):

            with ui.element("div").classes("card"):
                ui.html("<div class=\"pt\"><span class=\"pt-d\"></span>/line_error</div>")
                with ui.element("div").classes("irow"):
                    ui.html("<span class=\"ikey\">Error (px)</span>")
                    error_px_label = ui.html("<span class=\"ival g\">0</span>")
                with ui.element("div").classes("irow"):
                    ui.html("<span class=\"ikey\">Error normalizado</span>")
                    error_norm_label = ui.html("<span class=\"ival g\">0.000</span>")
                ui.html("<div class=\"ebar\"><div class=\"emid\"></div><div id=\"ebarFill\" class=\"efill\" style=\"width:0%;left:50%;background:#4ade80\"></div></div>")

            with ui.element("div").classes("card"):
                ui.html("<div class=\"pt\"><span class=\"pt-d\"></span>Retroalimentaci&#xF3;n</div>")
                feedback_title_label = ui.html("<div class=\"fbt\" style=\"color:#4ade80\">Esperando datos</div>")
                feedback_text_label  = ui.html("<div class=\"fbs\">Conectando con el robot...</div>")

            with ui.element("div").classes("card"):
                with ui.element("div").classes("mms"):
                    with ui.element("div").classes("mm"):
                        ui.html("<div class=\"mml\">Error actual</div>")
                        error_now_label = ui.html("<div class=\"mmv g\">0.00</div>")
                    with ui.element("div").classes("mm"):
                        ui.html("<div class=\"mml\">Error prom.</div>")
                        error_avg_label = ui.html("<div class=\"mmv y\">0.00</div>")
                    with ui.element("div").classes("mm"):
                        ui.html("<div class=\"mml\">Tiempo</div>")
                        mission_time_label = ui.html("<div class=\"mmv b\">00:00</div>")
                    with ui.element("div").classes("mm"):
                        ui.html("<div class=\"mml\">Vuelta</div>")
                        lap_label = ui.html("<div class=\"mmv p\">1</div>")

            with ui.element("div").classes("card").style("flex:1;min-height:200px"):
                ui.html("<div class=\"pt\"><span class=\"pt-d\"></span>Mapa / Ruta</div>")
                map_html = ui.html("").style("width:100%;height:calc(100% - 26px)")



# UPDATE UI




_progress = 0.0
_lap      = 1

def update_dashboard():
    global _progress, _lap

    _progress += 0.003
    if _progress >= 1.0 and _lap == 1:
        _lap = 2
        _progress = 1.0

    # Comando YOLO
    cmd = state["yolo_command"]
    action_text = COMMAND_ICONS.get(cmd, f"? {cmd}")
    yolo_command_label.set_content(
        f'<div class="ival purple" style="font-size:15px;text-align:center;padding:4px 0">{action_text}</div>'
    )

    # Área señal
    area = state["yolo_sign_area"]
    sign_area_label.set_content(f'<span class="ival yellow">{area:.0f}</span>')

    # Semáforo
    css   = state["color_css"]
    clabel = state["color_str"]
    semaforo_html.set_content(f'''
      <div class="semaforo-pill">
        <div class="semaforo-dot" style="background:{css};color:{css}"></div>
        <span class="semaforo-text" style="color:{css}">{clabel}</span>
      </div>
    ''')

    # Flags
    fc = "flag-chip active-finish" if state["finish_line"]  else "flag-chip"
    ic = "flag-chip active-inter"  if state["intersection"] else "flag-chip"
    finish_html.set_content(
        f'<div class="flag-row"><div class="{fc}">🏁 Finish Line</div>'
        f'<div class="{ic}">✦ Intersección</div></div>'
    )

    # Velocidades
    sp  = state["speed"]
    ang = state["angular_speed"]
    speed_label.set_content(
        f'<div class="vel-big">{sp:.2f} <span style="font-size:14px;color:#94a3b8">m/s</span></div>'
    )
    angular_label.set_content(f'<div class="vel-sub">ω: {ang:.2f} rad/s</div>')

    # Error px / norm
    epx  = state["line_error_px"]
    enrm = state["line_error"]
    ecol = "#4ade80" if abs(enrm) < 0.15 else ("#facc15" if abs(enrm) < 0.30 else "#f87171")
    error_px_label.set_content(f'<span class="ival" style="color:{ecol}">{epx}</span>')
    error_norm_label.set_content(f'<span class="ival" style="color:{ecol}">{enrm:.3f}</span>')

    # Barra de error
    pct = min(abs(enrm), 1.0) * 50
    side = "right" if enrm >= 0 else "left"
    left_pct = 50.0 if enrm >= 0 else (50.0 - pct)
    # usamos JS para la barra (set_content no actualiza atributos inline fácilmente)

    # Retroalimentación
    ft_col = "#4ade80" if "correctamente" in state["feedback_title"] else (
             "#facc15" if "Corrigiendo" in state["feedback_title"] else
             "#60a5fa" if "Intersección" in state["feedback_title"] else
             "#c084fc" if "Meta" in state["feedback_title"] else "#f87171"
    )
    feedback_title_label.set_content(
        f'<div style="color:{ft_col};font-size:13px;font-weight:800;margin-bottom:4px">{state["feedback_title"]}</div>'
    )
    feedback_text_label.set_content(
        f'<div style="color:#cbd5e1;font-size:11px">{state["feedback_text"]}</div>'
    )

    # Métricas
    error_now_label.set_content(f'<div class="mm-val" style="color:{ecol}">{enrm:.2f}</div>')
    error_avg_label.set_content(f'<div class="mm-val" style="color:#facc15">{state["error_avg"]:.2f}</div>')
    elapsed = int(time.time() - state["t0"])
    mm, ss = elapsed // 60, elapsed % 60
    mission_time_label.set_content(f'<div class="mm-val" style="color:#60a5fa">{mm:02d}:{ss:02d}</div>')
    lap_label.set_content(f'<div class="mm-val" style="color:#c084fc">{_lap}</div>')

    # Reloj header
    clock_label.set_text(time.strftime("%H:%M:%S"))

    # Gráfica error
    error_chart.options["xAxis"]["data"]     = list(range(len(state["error_values"])))
    error_chart.options["series"][0]["data"] = state["error_values"]
    error_chart.options["series"][1]["data"] = state["desired_values"]
    error_chart.update()

    # Mapa
    map_html.set_content(track_svg(_progress, _lap))


ui.timer(0.5, update_dashboard, active=True)

ui.run(
    host="0.0.0.0",
    port=8080,
    title="PuzzleBot Dashboard",
    reload=False
)
