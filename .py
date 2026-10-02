import asyncio
import json
import time
import threading
import random
import os
from http.server import HTTPServer, BaseHTTPRequestHandler
import websockets
from foxglove_websocket.server import FoxgloveServer

import socket
PORT = 9000

# ==========================================
# 1. THE FRONTEND UI
# ==========================================
# NOTE: all colours live in the :root block below. To match the exact FSFEUP
# brand palette, only those few variables need to be changed.
HTML_PAGE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>FS FEUP | Control Panel</title>
    <link rel="preconnect" href="https://fonts.googleapis.com">
    <link href="https://fonts.googleapis.com/css2?family=Rajdhani:wght@500;600;700&family=Inter:wght@400;500;600&display=swap" rel="stylesheet">
    <style>
        :root {
            /* ---- FSFEUP palette (edit here) ---- */
            --bg:        #070B14;
            --surface:   #0E1524;
            --surface-2: #151E33;
            --border:    #24304A;
            --accent:    #FF6A13;
            --accent-dk: #D4550B;
            --text:      #F4F6FB;
            --muted:     #8A96B2;
            --danger:    #FF4D4D;
            --ok:        #3DDC84;
            /* ------------------------------------ */
            --radius: 12px;
        }
        * { box-sizing: border-box; }
        body { font-family: 'Inter','Segoe UI',sans-serif; background: var(--bg); color: var(--text); margin: 0; min-height: 100vh; }

        /* ---------- top bar ---------- */
        .topbar { position: sticky; top: 0; z-index: 10; display: flex; align-items: center; justify-content: space-between; padding: 14px 28px; background: rgba(7,11,20,0.9); backdrop-filter: blur(8px); border-bottom: 1px solid var(--border); }
        .topbar::after { content: ''; position: absolute; left: 0; bottom: -1px; width: 140px; height: 2px; background: var(--accent); }
        .brand { font-family: 'Rajdhani',sans-serif; font-size: 1.5em; font-weight: 700; letter-spacing: 3px; text-transform: uppercase; }
        .brand span { color: var(--accent); }
        .brand small { font-family: 'Inter',sans-serif; font-size: 0.45em; letter-spacing: 2px; color: var(--muted); margin-left: 12px; font-weight: 500; }
        .status { display: flex; align-items: center; gap: 8px; font-size: 0.8em; color: var(--muted); }
        .dot { width: 9px; height: 9px; border-radius: 50%; background: var(--danger); box-shadow: 0 0 8px var(--danger); }
        .status.online .dot { background: var(--ok); box-shadow: 0 0 8px var(--ok); }

        /* ---------- layout ---------- */
        .layout { display: grid; grid-template-columns: 290px 1fr; gap: 24px; padding: 24px 28px 60px; max-width: 1400px; margin: auto; align-items: start; }
        @media (max-width: 860px) { .layout { grid-template-columns: 1fr; padding: 16px; } }
        .sidebar { position: sticky; top: 84px; background: var(--surface); border: 1px solid var(--border); border-radius: var(--radius); padding: 20px; }
        @media (max-width: 860px) { .sidebar { position: static; } }
        .section-title { font-family: 'Rajdhani',sans-serif; font-size: 1.05em; letter-spacing: 2px; text-transform: uppercase; color: var(--accent); margin: 0 0 14px; display: flex; align-items: center; justify-content: space-between; }
        .count { background: var(--surface-2); color: var(--muted); font-size: 0.8em; padding: 1px 9px; border-radius: 999px; letter-spacing: 0; }

        /* ---------- forms ---------- */
        .field { display: flex; flex-direction: column; gap: 5px; margin-bottom: 12px; }
        .field label { font-size: 0.7em; color: var(--muted); text-transform: uppercase; letter-spacing: 1px; }
        .row { display: flex; gap: 10px; }
        .row .field { flex: 1; }
        input[type=text], input[type=number], select { width: 100%; background: var(--surface-2); color: var(--text); border: 1px solid var(--border); padding: 9px 11px; border-radius: 6px; outline: none; font-size: 14px; font-family: inherit; }
        input:focus, select:focus { border-color: var(--accent); box-shadow: 0 0 0 3px rgba(255,106,19,0.18); }
        button { font-family: inherit; cursor: pointer; border: none; border-radius: 6px; transition: all .15s; }
        .btn-primary { width: 100%; background: var(--accent); color: #0B0B0B; font-weight: 700; padding: 11px; letter-spacing: 1px; text-transform: uppercase; font-size: 13px; }
        .btn-primary:hover { background: var(--accent-dk); color: #fff; }
        .btn-ghost { background: transparent; color: var(--text); border: 1px solid var(--border); padding: 10px 16px; font-size: 13px; }
        .btn-ghost:hover { background: var(--surface-2); }
        .btn-danger { background: transparent; color: var(--danger); border: 1px solid var(--danger); padding: 10px 16px; font-size: 13px; }
        .btn-danger:hover { background: var(--danger); color: #fff; }

        /* ---------- main grids ---------- */
        .group { margin-bottom: 30px; }
        .grid-sliders { display: grid; grid-template-columns: repeat(auto-fill, minmax(300px, 1fr)); gap: 16px; }
        .grid-buttons { display: grid; grid-template-columns: repeat(auto-fill, minmax(150px, 1fr)); gap: 16px; }
        .empty { text-align: center; color: var(--muted); padding: 60px 10px; border: 1px dashed var(--border); border-radius: var(--radius); }

        .tile { position: relative; background: var(--surface); border: 1px solid var(--border); border-radius: var(--radius); padding: 16px 18px; transition: border-color .15s; }
        .tile:hover { border-color: #34436a; }
        .tile-head { display: flex; justify-content: space-between; align-items: center; margin-bottom: 6px; }
        .tile-name { font-family: 'Rajdhani',sans-serif; font-weight: 600; font-size: 1.1em; letter-spacing: 1px; text-transform: uppercase; color: var(--muted); overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
        .gear { background: transparent; color: var(--muted); font-size: 16px; padding: 2px 7px; }
        .gear:hover { color: var(--accent); }

        /* slider tile */
        .big-val { font-family: 'Rajdhani',sans-serif; font-weight: 700; font-size: 3em; line-height: 1; color: var(--text); font-variant-numeric: tabular-nums; }
        .big-val.noisy { color: var(--accent); }
        .slider-wrap { margin-top: 14px; }
        input[type=range] { -webkit-appearance: none; appearance: none; width: 100%; height: 8px; border-radius: 4px; outline: none; cursor: pointer;
            background: linear-gradient(to right, var(--accent) var(--pct,50%), var(--surface-2) var(--pct,50%)); }
        input[type=range]::-webkit-slider-thumb { -webkit-appearance: none; width: 20px; height: 20px; border-radius: 50%; background: var(--text); border: 3px solid var(--accent); box-shadow: 0 0 10px rgba(255,106,19,0.5); }
        input[type=range]::-moz-range-thumb { width: 14px; height: 14px; border-radius: 50%; background: var(--text); border: 3px solid var(--accent); }
        .bounds { display: flex; justify-content: space-between; font-size: 0.72em; color: var(--muted); margin-top: 6px; }
        .noise-row { display: flex; align-items: center; gap: 10px; margin-top: 14px; padding-top: 12px; border-top: 1px solid var(--border); font-size: 0.8em; color: var(--muted); }
        .switch { position: relative; width: 36px; height: 20px; flex-shrink: 0; }
        .switch input { opacity: 0; width: 0; height: 0; }
        .switch span { position: absolute; inset: 0; background: var(--surface-2); border: 1px solid var(--border); border-radius: 999px; cursor: pointer; transition: .2s; }
        .switch span::before { content: ''; position: absolute; width: 14px; height: 14px; left: 2px; top: 2px; background: var(--muted); border-radius: 50%; transition: .2s; }
        .switch input:checked + span { background: var(--accent); border-color: var(--accent); }
        .switch input:checked + span::before { transform: translateX(16px); background: #fff; }
        .noise-row .sd { display: flex; align-items: center; gap: 6px; margin-left: auto; opacity: .35; pointer-events: none; }
        .noise-row .sd.on { opacity: 1; pointer-events: auto; }
        .noise-row .sd input { width: 64px; padding: 4px 7px; font-size: 12px; }

        /* button tile (pad) */
        .tile.pad { padding: 14px; text-align: center; }
        .pad-btn { width: 100%; aspect-ratio: 1; border-radius: 50%; background: var(--surface-2); border: 2px solid var(--accent); color: var(--accent); font-family: 'Rajdhani',sans-serif; font-weight: 700; letter-spacing: 2px; font-size: 0.95em; user-select: none; -webkit-user-select: none; touch-action: none; max-width: 120px; }
        .pad-btn:hover { background: rgba(255,106,19,0.12); }
        .pad-btn.held { background: var(--accent); color: #0B0B0B; box-shadow: 0 0 28px rgba(255,106,19,0.6); transform: scale(0.96); }
        .pad .tile-head { margin-bottom: 10px; }

        /* ---------- modal ---------- */
        .overlay { position: fixed; inset: 0; background: rgba(0,0,0,0.65); display: none; align-items: center; justify-content: center; z-index: 50; padding: 16px; }
        .overlay.open { display: flex; }
        .modal { background: var(--surface); border: 1px solid var(--border); border-top: 3px solid var(--accent); border-radius: var(--radius); padding: 22px; width: 100%; max-width: 380px; }
        .modal-actions { display: flex; gap: 10px; margin-top: 18px; }
        .modal-actions .spacer { flex: 1; }
    </style>
</head>
<body>
    <div class="topbar">
        <div class="brand">FS <span>FEUP</span><small>CONTROL PANEL</small></div>
        <div class="status" id="status"><span class="dot"></span><span id="status-text">Disconnected</span></div>
    </div>

    <div class="layout">
        <aside class="sidebar">
            <h2 class="section-title">New component</h2>
            <div class="field">
                <label>Type</label>
                <select id="comp-type">
                    <option value="slider">Slider (numeric)</option>
                    <option value="button">Button (boolean)</option>
                </select>
            </div>
            <div class="field">
                <label>Name</label>
                <input type="text" id="comp-name" placeholder="e.g. Throttle"/>
            </div>
            <div class="row" id="range-inputs">
                <div class="field"><label>Min</label><input type="number" id="comp-min" value="0"/></div>
                <div class="field"><label>Max</label><input type="number" id="comp-max" value="100"/></div>
            </div>
            <button class="btn-primary" id="add-btn">+ Add</button>
        </aside>

        <main>
            <div id="empty" class="empty">No components yet. Create your first slider or button in the sidebar.</div>
            <section class="group" id="group-sliders" style="display:none">
                <h2 class="section-title">Sliders <span class="count" id="count-sliders">0</span></h2>
                <div class="grid-sliders" id="grid-sliders"></div>
            </section>
            <section class="group" id="group-buttons" style="display:none">
                <h2 class="section-title">Buttons <span class="count" id="count-buttons">0</span></h2>
                <div class="grid-buttons" id="grid-buttons"></div>
            </section>
        </main>
    </div>

    <!-- edit modal -->
    <div class="overlay" id="overlay">
        <div class="modal">
            <h2 class="section-title" id="modal-title">Edit</h2>
            <div class="field"><label>Name</label><input type="text" id="m-name"></div>
            <div class="row" id="m-range">
                <div class="field"><label>Min</label><input type="number" id="m-min"></div>
                <div class="field"><label>Max</label><input type="number" id="m-max"></div>
            </div>
            <div class="modal-actions">
                <button class="btn-danger" id="m-delete">Remove</button>
                <span class="spacer"></span>
                <button class="btn-ghost" id="m-cancel">Cancel</button>
                <button class="btn-primary" id="m-save" style="width:auto">Save</button>
            </div>
        </div>
    </div>

    <script>
        let state = { sliders: {}, buttons: {} };
        let ws;
        let isInitialized = false;
        let editing = null; // {type, name}

        const $ = (id) => document.getElementById(id);
        const clean = (s) => s.trim().replace(/[^a-zA-Z0-9_]/g, '_');
        const pct = (v, min, max) => max === min ? 0 : ((v - min) / (max - min)) * 100;

        // ---------- connection ----------
        function setStatus(online) {
            $('status').classList.toggle('online', online);
            $('status-text').innerText = online ? 'Connected' : 'Reconnecting...';
        }
        function connectWS() {
            ws = new WebSocket("ws://" + window.location.hostname + ":8001");
            ws.onopen = () => setStatus(true);
            ws.onmessage = (event) => {
                if (isInitialized) return;
                const loaded = JSON.parse(event.data);
                state = { sliders: loaded.sliders || {}, buttons: loaded.buttons || {} };
                isInitialized = true;
                render();
            };
            ws.onclose = () => { isInitialized = false; setStatus(false); setTimeout(connectWS, 1000); };
        }
        connectWS();

        function sendState() {
            if (ws && ws.readyState === WebSocket.OPEN && isInitialized) ws.send(JSON.stringify(state));
        }

        // ---------- rendering (structure only; live updates touch single elements) ----------
        function render() {
            const gs = $('grid-sliders'), gb = $('grid-buttons');
            gs.innerHTML = ''; gb.innerHTML = '';
            const sNames = Object.keys(state.sliders), bNames = Object.keys(state.buttons);

            sNames.forEach(name => gs.appendChild(sliderTile(name, state.sliders[name])));
            bNames.forEach(name => gb.appendChild(buttonTile(name)));

            $('group-sliders').style.display = sNames.length ? 'block' : 'none';
            $('group-buttons').style.display = bNames.length ? 'block' : 'none';
            $('empty').style.display = (sNames.length || bNames.length) ? 'none' : 'block';
            $('count-sliders').innerText = sNames.length;
            $('count-buttons').innerText = bNames.length;
        }

        function el(html) { const t = document.createElement('template'); t.innerHTML = html.trim(); return t.content.firstChild; }

        function sliderTile(name, d) {
            const min = d.min ?? 0, max = d.max ?? 100;
            const val = d.value ?? (min + max) / 2;
            const sd = d.std_dev ?? 1.0;
            const tile = el(`
                <div class="tile">
                    <div class="tile-head"><span class="tile-name">${name}</span><button class="gear" title="Edit">⚙</button></div>
                    <div class="big-val ${d.noise ? 'noisy' : ''}">${parseFloat(val).toFixed(2)}</div>
                    <div class="slider-wrap">
                        <input type="range" min="${min}" max="${max}" step="any" value="${val}" style="--pct:${pct(val,min,max)}%">
                        <div class="bounds"><span>${min}</span><span>${max}</span></div>
                    </div>
                    <div class="noise-row">
                        <label class="switch"><input type="checkbox" ${d.noise ? 'checked' : ''}><span></span></label>
                        <span>White noise</span>
                        <span class="sd ${d.noise ? 'on' : ''}">σ <input type="number" step="0.1" value="${sd}"></span>
                    </div>
                </div>`);
            const range = tile.querySelector('input[type=range]');
            const big = tile.querySelector('.big-val');
            range.addEventListener('input', () => {
                const v = parseFloat(range.value);
                state.sliders[name].value = v;
                big.innerText = v.toFixed(2);
                range.style.setProperty('--pct', pct(v, min, max) + '%');
                sendState();
            });
            const sw = tile.querySelector('.switch input');
            sw.addEventListener('change', () => {
                state.sliders[name].noise = sw.checked;
                big.classList.toggle('noisy', sw.checked);
                tile.querySelector('.sd').classList.toggle('on', sw.checked);
                sendState();
            });
            tile.querySelector('.sd input').addEventListener('input', (e) => {
                state.sliders[name].std_dev = parseFloat(e.target.value) || 0;
                sendState();
            });
            tile.querySelector('.gear').addEventListener('click', () => openModal('slider', name));
            return tile;
        }

        function buttonTile(name) {
            const tile = el(`
                <div class="tile pad">
                    <div class="tile-head"><span class="tile-name">${name}</span><button class="gear" title="Edit">⚙</button></div>
                    <button class="pad-btn">HOLD</button>
                </div>`);
            const pad = tile.querySelector('.pad-btn');
            const set = (pressed) => {
                pad.classList.toggle('held', pressed);
                if (state.buttons[name] && state.buttons[name].value !== pressed) {
                    state.buttons[name].value = pressed;
                    sendState();
                }
            };
            pad.addEventListener('pointerdown', (e) => { pad.setPointerCapture(e.pointerId); set(true); });
            ['pointerup', 'pointercancel', 'lostpointercapture'].forEach(ev => pad.addEventListener(ev, () => set(false)));
            tile.querySelector('.gear').addEventListener('click', () => openModal('button', name));
            return tile;
        }

        // ---------- add ----------
        $('comp-type').addEventListener('change', (e) => {
            $('range-inputs').style.display = e.target.value === 'slider' ? 'flex' : 'none';
        });
        function addComponent() {
            const type = $('comp-type').value;
            const raw = $('comp-name').value;
            if (!raw.trim()) return alert("Please provide a name!");
            const name = clean(raw);
            if (state.sliders[name] || state.buttons[name]) return alert("Name already exists!");

            if (type === 'slider') {
                const min = parseFloat($('comp-min').value), max = parseFloat($('comp-max').value);
                if (isNaN(min) || isNaN(max) || min >= max) return alert("Min must be lower than max!");
                state.sliders[name] = { value: (min + max) / 2, min, max, noise: false, std_dev: 1.0 };
            } else {
                state.buttons[name] = { value: false };
            }
            sendState(); render();
            $('comp-name').value = '';
        }
        $('add-btn').addEventListener('click', addComponent);
        $('comp-name').addEventListener('keydown', (e) => { if (e.key === 'Enter') addComponent(); });

        // ---------- edit modal ----------
        function openModal(type, name) {
            editing = { type, name };
            $('modal-title').innerText = 'Edit ' + type;
            $('m-name').value = name;
            $('m-range').style.display = type === 'slider' ? 'flex' : 'none';
            if (type === 'slider') { $('m-min').value = state.sliders[name].min; $('m-max').value = state.sliders[name].max; }
            $('overlay').classList.add('open');
        }
        function closeModal() { $('overlay').classList.remove('open'); editing = null; }

        function saveEdit() {
            const { type, name: oldName } = editing;
            const newName = clean($('m-name').value);
            if (!newName) return alert("Name cannot be empty!");
            if (newName !== oldName && (state.sliders[newName] || state.buttons[newName])) return alert("Name already exists!");

            const group = type === 'slider' ? state.sliders : state.buttons;
            const data = group[oldName];
            if (type === 'slider') {
                const min = parseFloat($('m-min').value), max = parseFloat($('m-max').value);
                if (isNaN(min) || isNaN(max) || min >= max) return alert("Min must be lower than max!");
                data.min = min; data.max = max;
                data.value = Math.max(min, Math.min(max, data.value));
            }
            if (newName !== oldName) {
                // rebuild to keep original ordering
                const rebuilt = {};
                for (const [k, v] of Object.entries(group)) rebuilt[k === oldName ? newName : k] = v;
                if (type === 'slider') state.sliders = rebuilt; else state.buttons = rebuilt;
            }
            closeModal(); sendState(); render();
        }
        function deleteEdit() {
            const { type, name } = editing;
            if (type === 'slider') delete state.sliders[name]; else delete state.buttons[name];
            closeModal(); sendState(); render();
        }
        $('m-save').addEventListener('click', saveEdit);
        $('m-cancel').addEventListener('click', closeModal);
        $('m-delete').addEventListener('click', deleteEdit);
        $('overlay').addEventListener('click', (e) => { if (e.target.id === 'overlay') closeModal(); });
        document.addEventListener('keydown', (e) => { if (e.key === 'Escape' && editing) closeModal(); });
    </script>
</body>
</html>
"""

# ==========================================
# 2. HTTP SERVER (Serves the UI)
# ==========================================
class UIHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/html; charset=utf-8')
        self.end_headers()
        self.wfile.write(HTML_PAGE.encode('utf-8'))

    def log_message(self, format, *args):
        pass

def start_http_server():
    server = HTTPServer(('0.0.0.0', 8000), UIHandler)
    print("🌍 Web UI is running! Open http://localhost:8000 in your browser")
    server.serve_forever()


# ==========================================
# 3. ASYNC WEBSOCKETS (Data Logic & JSON Save)
# ==========================================
CONFIG_FILE = "ui_config.json"
UI_STATE = {"sliders": {}, "buttons": {}}

# Load existing layout/values if the JSON file exists
if os.path.exists(CONFIG_FILE):
    try:
        with open(CONFIG_FILE, "r") as f:
            UI_STATE = json.load(f)
            # Ensure buttons don't load "stuck" in a pressed state
            for btn in UI_STATE.get("buttons", {}).values():
                btn["value"] = False
        print(f"💾 Loaded existing UI layout from {CONFIG_FILE}")
    except Exception as e:
        print(f"⚠️ Failed to load {CONFIG_FILE}: {e}")

async def ui_ws_handler(websocket, *args, **kwargs):
    global UI_STATE

    # 1. Immediately send the loaded state to the browser
    await websocket.send(json.dumps(UI_STATE))

    # 2. Listen for UI updates from the browser
    try:
        async for message in websocket:
            try:
                UI_STATE = json.loads(message)

                # 3. Save the new state to disk
                with open(CONFIG_FILE, "w") as f:
                    json.dump(UI_STATE, f, indent=4)

            except json.JSONDecodeError:
                pass
    except websockets.exceptions.ConnectionClosed:
        pass


# Global variable to store the latest data from C++
CPP_STATE = {}

class CppUdpListener(asyncio.DatagramProtocol):
    def __init__(self):
        self.first_message = True

    def datagram_received(self, data, addr):
        global CPP_STATE
        try:
            CPP_STATE = json.loads(data.decode('utf-8'))
            
            # Print ONLY the first time we get data so it doesn't spam your terminal
            if self.first_message:
                print(f"✅ Success! Received data from C++: {CPP_STATE}")
                self.first_message = False
                
        except json.JSONDecodeError:
            pass

async def main():
    threading.Thread(target=start_http_server, daemon=True).start()
    await websockets.serve(ui_ws_handler, "0.0.0.0", 8001)
    
    # ---> ADD THIS: Start the UDP listener on port 9001
    loop = asyncio.get_running_loop()
    await loop.create_datagram_endpoint(
        lambda: CppUdpListener(),
        local_addr=('127.0.0.1', 9001)
    )
    
    # Same Foxglove UDP socket we used before for sending to C++
    import socket
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    async with FoxgloveServer("0.0.0.0", 8765, "Dynamic UI Server") as fg_server:
        print("🦊 Foxglove streaming live at ws://localhost:8765")
        
        chan_id = None
        current_sliders = set()
        current_buttons = set()
        
        # ---> EXPLICIT SCHEMA SO FOXGLOVE CAN SEE THE VARIABLES
        cpp_schema = {
            "type": "object",
            "properties": {
                "rpm": {"type": "number"},
                "temperature_c": {"type": "number"},
                "system_status": {"type": "number"}
            }
        }
        
        cpp_chan_id = await fg_server.add_channel({
            "topic": "/cpp_telemetry",
            "encoding": "json",
            "schemaName": "CppData",
            "schema": json.dumps(cpp_schema)
        })

        while True:
            # 1. UI Channel Logic (Same as before)
            new_sliders = set(UI_STATE.get("sliders", {}).keys())
            new_buttons = set(UI_STATE.get("buttons", {}).keys())
            
            if new_sliders != current_sliders or new_buttons != current_buttons:
                if chan_id is not None:
                    await fg_server.remove_channel(chan_id)
                
                schema = {
                    "type": "object",
                    "properties": {
                        "sliders": { "type": "object", "properties": {n: {"type": "number"} for n in new_sliders} },
                        "buttons": { "type": "object", "properties": {n: {"type": "boolean"} for n in new_buttons} }
                    }
                }
                chan_id = await fg_server.add_channel({
                    "topic": "/custom_ui_controls",
                    "encoding": "json",
                    "schemaName": "CustomControls",
                    "schema": json.dumps(schema)
                })
                current_sliders = new_sliders
                current_buttons = new_buttons

            # 2. Send UI Data to Foxglove & C++
            if chan_id is not None and (current_sliders or current_buttons):
                payload = {"sliders": {}, "buttons": {}}
                
                for name, data in UI_STATE.get("sliders", {}).items():
                    val = float(data.get("value", 0.0))
                    if data.get("noise"):
                        val += random.gauss(0, float(data.get("std_dev", 1.0)))
                        val = max(float(data.get("min", val)), min(float(data.get("max", val)), val))
                    payload["sliders"][name] = val
                    
                for name, data in UI_STATE.get("buttons", {}).items():
                    payload["buttons"][name] = bool(data.get("value", False))
                
                ui_json = json.dumps(payload).encode("utf8")
                await fg_server.send_message(chan_id, time.time_ns(), ui_json)
                udp_sock.sendto(ui_json, ("127.0.0.1", 9000)) # Send to C++
                
            # ---> ADD THIS: 3. Forward C++ Data to Foxglove
            if CPP_STATE:
                await fg_server.send_message(
                    cpp_chan_id,
                    time.time_ns(),
                    json.dumps(CPP_STATE).encode("utf8")
                )
                
            await asyncio.sleep(0.1)

if __name__ == "__main__":
    try:
        print("Starting Foxglove Dynamic UI Backend...")
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nShutting down.")