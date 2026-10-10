#!/usr/bin/env python3
"""
vm_emulator.py  -  the full Vehicle Model (vm_server.py) running all the time, plus manual OVERRIDES.

 * The VM runs exactly as in vm_server.py: same physics, shutdown circuit, noise, topics and UDP packets.
   vm_server.py must be in the same folder (it is imported), so any change you make there applies here too.
 * Every sensor of the VM is a slider (raw ADC value 0..65535) with an OVERRIDE switch:
       switch off : the slider follows the live value computed by the VM
       switch on  : the slider value replaces that sensor's reading (the VM keeps running underneath)
   Overrides are always off when the emulator starts.
 * ATS / R2D are hold-buttons: they are the physical inputs, so they always set the reading.
 * "Vehicle inputs" (throttle, steering, brake) and the 3 shutdown relays are here too, as in vm_server.py.
 * You can still add your own sliders / buttons and choose the Foxglove topic (and JSON key) of each one.
   Custom components are always active. Components that share a topic are merged in one message;
   a key with dots is nested: "wheel_speed.fl" -> {"wheel_speed": {"fl": v}}
   (a custom component on /sensors/vm or /sensors/other adds / replaces that field).
 * UDP to C++ (127.0.0.1:9000, 200 Hz) and from C++ (127.0.0.1:9001) are the same as vm_server.py.

Run:  python3 vm_emulator.py        (UI on http://localhost:8000)
Settings are saved in emulator_config.json. Signals that are in vm_signals.json but missing in the
saved settings are added at start-up. If vm_signals.json changes version, the pre-filled components
are rebuilt (your own ones are kept).
"""
import asyncio
import json
import math
import os
import random
import re
import socket
import threading
import time
from http.server import HTTPServer, BaseHTTPRequestHandler

import websockets
from foxglove_websocket.server import FoxgloveServer

import vm_server as vm                                  # the Vehicle Model
from vm_server import DynamicChannel, set_in_tree

# ==========================================
# 0. PARAMETERS
# ==========================================
HERE = os.path.dirname(os.path.abspath(__file__))
SIGNALS_FILE = os.path.join(HERE, "vm_signals.json")
CONFIG_FILE = "emulator_config.json"

SIM_HZ = vm.SIM_HZ              # same rate as the VM
BROWSER_DIV = 10                # live values to the browser: SIM_HZ/10 = 20 Hz
UDP_TO_CPP = vm.UDP_TO_CPP
UDP_FROM_CPP = vm.UDP_FROM_CPP


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def clean_topic(t, default):
    t = re.sub(r"[^A-Za-z0-9_/\-]", "_", str(t or "").strip())
    if not t:
        t = default
    if not t.startswith("/"):
        t = "/" + t
    return re.sub(r"/{2,}", "/", t)


def clean_name(n, default):
    n = re.sub(r"[^A-Za-z0-9_]", "_", str(n or "").strip())
    return n or default


def clean_key(k, default):
    """JSON key; dots make it nested ('wheel_speed.fl')."""
    k = re.sub(r"[^A-Za-z0-9_.]", "_", str(k or "").strip()).strip(".")
    return re.sub(r"\.{2,}", ".", k) or default


def set_path(d, path, value):
    parts = path.split(".")
    for p in parts[:-1]:
        nxt = d.get(p)
        if not isinstance(nxt, dict):
            nxt = d[p] = {}
        d = nxt
    d[parts[-1]] = value


def get_path(d, path):
    for p in path.split("."):
        if not isinstance(d, dict) or p not in d:
            return None
        d = d[p]
    return d


# ==========================================
# 2. STATE (sliders / buttons) + vm_signals.json
# ==========================================
def normalise_slider(name, d):
    d = d if isinstance(d, dict) else {}
    try:
        lo = float(d.get("min", 0.0))
        hi = float(d.get("max", 100.0))
    except (TypeError, ValueError):
        lo, hi = 0.0, 100.0
    if not (math.isfinite(lo) and math.isfinite(hi)) or lo >= hi:
        lo, hi = 0.0, 100.0
    try:
        val = float(d.get("value", (lo + hi) / 2))
    except (TypeError, ValueError):
        val = (lo + hi) / 2
    if not math.isfinite(val):
        val = (lo + hi) / 2
    try:
        sd = float(d.get("std_dev", 1.0))
    except (TypeError, ValueError):
        sd = 1.0
    return {
        "topic": clean_topic(d.get("topic"), f"/emulator/{name}"),
        "key": clean_key(d.get("key"), name),
        "value": clamp(val, lo, hi),
        "min": lo, "max": hi,
        "noise": bool(d.get("noise", False)),
        "std_dev": sd if math.isfinite(sd) else 1.0,
        "integer": bool(d.get("integer", False)),
        "from_vm": bool(d.get("from_vm", False)),
        "override": bool(d.get("override", False)),      # only used by the pre-filled VM sensors
    }


def normalise_button(name, d):
    d = d if isinstance(d, dict) else {}
    return {
        "topic": clean_topic(d.get("topic"), f"/emulator/{name}"),
        "key": clean_key(d.get("key"), name),
        "value": bool(d.get("value", False)),
        "integer": bool(d.get("integer", False)),
        "from_vm": bool(d.get("from_vm", False)),
    }


def normalise_state(s):
    s = s if isinstance(s, dict) else {}
    sl = s.get("sliders") if isinstance(s.get("sliders"), dict) else {}
    bt = s.get("buttons") if isinstance(s.get("buttons"), dict) else {}
    base = vm.normalise_state(s)                         # vehicle inputs + shutdown relays
    return {
        "inputs": base["inputs"],
        "relays": base["relays"],
        "sliders": {clean_name(n, "x"): normalise_slider(clean_name(n, "x"), d) for n, d in sl.items()},
        "buttons": {clean_name(n, "x"): normalise_button(clean_name(n, "x"), d) for n, d in bt.items()},
    }


def load_signals():
    try:
        with open(SIGNALS_FILE, "r") as f:
            return json.load(f)
    except Exception as e:
        print(f"⚠️ Could not read {SIGNALS_FILE}: {e}")
        return None


def merge_vm_signals(state, desc):
    """Add a component for every VM signal that is not in the state yet."""
    added = 0
    for sig in desc.get("signals", []):
        for key in sig.get("keys", [sig["name"]]):
            name = clean_name(key.replace(".", "_"), "x")
            if name in state["sliders"] or name in state["buttons"]:
                continue
            if sig.get("kind") == "digital":
                state["buttons"][name] = normalise_button(name, {
                    "topic": sig["topic"], "key": key, "value": False,
                    "integer": True, "from_vm": True})
            else:
                state["sliders"][name] = normalise_slider(name, {
                    "topic": sig["topic"], "key": key,
                    "min": sig.get("raw_min", 0), "max": sig.get("raw_max", 65535),
                    "value": sig.get("default_raw", 0), "integer": True, "from_vm": True})
            added += 1
    if added:
        print(f"➕ Added {added} components from {os.path.basename(SIGNALS_FILE)}")
    return state


SIGNALS_DESC = load_signals()
SIGNALS_VERSION = (SIGNALS_DESC or {}).get("version", 0)


def save_state(state):
    with open(CONFIG_FILE, "w") as f:
        json.dump(dict(state, _signals_version=SIGNALS_VERSION), f, indent=4)


UI_STATE = normalise_state({})
if os.path.exists(CONFIG_FILE):
    try:
        with open(CONFIG_FILE, "r") as f:
            raw_cfg = json.load(f)
        UI_STATE = normalise_state(raw_cfg)
        for b in UI_STATE["buttons"].values():
            b["value"] = False
        for d in UI_STATE["sliders"].values():
            d["override"] = False                        # never start with a sensor silently overridden
        fresh = vm.normalise_state({})                    # pedals released, relays closed
        UI_STATE["inputs"], UI_STATE["relays"] = fresh["inputs"], fresh["relays"]
        if SIGNALS_DESC and raw_cfg.get("_signals_version") != SIGNALS_VERSION:
            # vm_signals.json changed: rebuild the pre-filled components, keep the user's own
            for group in ("sliders", "buttons"):
                UI_STATE[group] = {n: d for n, d in UI_STATE[group].items() if not d.get("from_vm")}
            print("♻️  vm_signals.json changed, pre-filled components rebuilt")
        print(f"💾 Loaded existing settings from {CONFIG_FILE}")
    except Exception as e:
        print(f"⚠️ Failed to load {CONFIG_FILE}: {e}")
if SIGNALS_DESC:
    UI_STATE = merge_vm_signals(UI_STATE, SIGNALS_DESC)
save_state(UI_STATE)

CLIENTS = set()


async def ui_ws_handler(websocket, *args, **kwargs):
    global UI_STATE
    await websocket.send(json.dumps(UI_STATE))
    CLIENTS.add(websocket)
    try:
        async for message in websocket:
            try:
                UI_STATE = normalise_state(json.loads(message))
                save_state(UI_STATE)
            except json.JSONDecodeError:
                pass
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        CLIENTS.discard(websocket)


async def broadcast_to_browsers(msg):
    for ws in list(CLIENTS):
        try:
            await ws.send(msg)
        except Exception:
            CLIENTS.discard(ws)


# ==========================================
# 4. WEB UI
# ==========================================
HTML_PAGE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>FS FEUP | VM Emulator</title>
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

        .topbar { position: sticky; top: 0; z-index: 10; display: flex; align-items: center; justify-content: space-between; padding: 14px 28px; background: rgba(7,11,20,0.9); backdrop-filter: blur(8px); border-bottom: 1px solid var(--border); }
        .topbar::after { content: ''; position: absolute; left: 0; bottom: -1px; width: 140px; height: 2px; background: var(--accent); }
        .brand { font-family: 'Rajdhani',sans-serif; font-size: 1.5em; font-weight: 700; letter-spacing: 3px; text-transform: uppercase; }
        .brand span { color: var(--accent); }
        .brand small { font-family: 'Inter',sans-serif; font-size: 0.45em; letter-spacing: 2px; color: var(--muted); margin-left: 12px; font-weight: 500; }
        .status { display: flex; align-items: center; gap: 8px; font-size: 0.8em; color: var(--muted); }
        .dot { width: 9px; height: 9px; border-radius: 50%; background: var(--danger); box-shadow: 0 0 8px var(--danger); }
        .status.online .dot { background: var(--ok); box-shadow: 0 0 8px var(--ok); }

        .layout { display: grid; grid-template-columns: 290px 1fr; gap: 24px; padding: 24px 28px 60px; max-width: 1400px; margin: auto; align-items: start; }
        @media (max-width: 860px) { .layout { grid-template-columns: 1fr; padding: 16px; } }
        .sidebar { position: sticky; top: 84px; background: var(--surface); border: 1px solid var(--border); border-radius: var(--radius); padding: 20px; }
        @media (max-width: 860px) { .sidebar { position: static; } }
        .section-title { font-family: 'Rajdhani',sans-serif; font-size: 1.05em; letter-spacing: 2px; text-transform: uppercase; color: var(--accent); margin: 0 0 14px; display: flex; align-items: center; justify-content: space-between; }
        .count { background: var(--surface-2); color: var(--muted); font-size: 0.8em; padding: 1px 9px; border-radius: 999px; letter-spacing: 0; }

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

        .group { margin-bottom: 30px; }
        .grid-sliders { display: grid; grid-template-columns: repeat(auto-fill, minmax(300px, 1fr)); gap: 16px; }
        .grid-buttons { display: grid; grid-template-columns: repeat(auto-fill, minmax(150px, 1fr)); gap: 16px; }
        .empty { text-align: center; color: var(--muted); padding: 60px 10px; border: 1px dashed var(--border); border-radius: var(--radius); }

        .tile { position: relative; background: var(--surface); border: 1px solid var(--border); border-radius: var(--radius); padding: 16px 18px; transition: border-color .15s; }
        .tile:hover { border-color: #34436a; }
        .tile-head { display: flex; justify-content: space-between; align-items: center; margin-bottom: 2px; }
        .tile-name { font-family: 'Rajdhani',sans-serif; font-weight: 600; font-size: 1.1em; letter-spacing: 1px; text-transform: uppercase; color: var(--muted); overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
        .topic { font-size: 0.68em; color: var(--muted); opacity: .8; margin-bottom: 8px; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; font-family: 'Inter',monospace; }
        .gear { background: transparent; color: var(--muted); font-size: 16px; padding: 2px 7px; }
        .gear:hover { color: var(--accent); }

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

        .tile.pad { padding: 14px; text-align: center; }
        .pad-btn { width: 100%; aspect-ratio: 1; border-radius: 50%; background: var(--surface-2); border: 2px solid var(--accent); color: var(--accent); font-family: 'Rajdhani',sans-serif; font-weight: 700; letter-spacing: 2px; font-size: 0.95em; user-select: none; -webkit-user-select: none; touch-action: none; max-width: 120px; }
        .pad-btn:hover { background: rgba(255,106,19,0.12); }
        .pad-btn.held { background: var(--accent); color: #0B0B0B; box-shadow: 0 0 28px rgba(255,106,19,0.6); transform: scale(0.96); }
        .pad .tile-head { margin-bottom: 2px; }
        .pad .topic { margin-bottom: 10px; }

        /* vehicle inputs / relays / override */
        .tile.fixed { border-top: 2px solid var(--accent); }
        .tile.fixed .tile-head { min-height: 28px; margin-bottom: 6px; }
        .big-val small { font-size: 0.4em; color: var(--muted); margin-left: 6px; font-weight: 600; }
        .relay-state.closed { color: var(--ok); }
        .relay-state.open { color: var(--danger); }
        .btn-ghost.small { padding: 4px 12px; font-size: 11px; letter-spacing: 1px; text-transform: uppercase; color: var(--muted); }
        .ovr-row { display: flex; align-items: center; gap: 10px; margin-bottom: 8px; font-size: 0.72em; letter-spacing: 1px; text-transform: uppercase; }
        .ovr-label { color: var(--ok); font-weight: 600; }
        .tile.overridden { border-color: var(--accent); }
        .tile.overridden .ovr-label { color: var(--accent); }
        .tile.live input[type=range] { opacity: .5; cursor: not-allowed; }
        .tile.live .big-val { color: var(--muted); }
        .overlay { position: fixed; inset: 0; background: rgba(0,0,0,0.65); display: none; align-items: center; justify-content: center; z-index: 50; padding: 16px; }
        .overlay.open { display: flex; }
        .modal { background: var(--surface); border: 1px solid var(--border); border-top: 3px solid var(--accent); border-radius: var(--radius); padding: 22px; width: 100%; max-width: 380px; }
        .modal-actions { display: flex; gap: 10px; margin-top: 18px; }
        .modal-actions .spacer { flex: 1; }
    </style>
</head>
<body>
    <div class="topbar">
        <div class="brand">FS <span>FEUP</span><small>VM EMULATOR</small></div>
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
                <input type="text" id="comp-name" placeholder="e.g. Battery_temp"/>
            </div>
            <div class="field">
                <label>Foxglove topic</label>
                <input type="text" id="comp-topic" placeholder="/sensors/other"/>
            </div>
            <div class="field">
                <label>JSON key (default: name, dots = nested)</label>
                <input type="text" id="comp-key" placeholder="battery_temp"/>
            </div>
            <div class="row" id="range-inputs">
                <div class="field"><label>Min</label><input type="number" id="comp-min" value="0"/></div>
                <div class="field"><label>Max</label><input type="number" id="comp-max" value="100"/></div>
            </div>
            <button class="btn-primary" id="add-btn">+ Add</button>
        </aside>

        <main>
            <section class="group">
                <h2 class="section-title">Vehicle inputs <button class="btn-ghost small" id="zero-all">Zero all</button></h2>
                <div class="grid-sliders" id="grid-inputs"></div>
            </section>
            <section class="group">
                <h2 class="section-title">Shutdown line (latching)</h2>
                <div class="grid-sliders" id="grid-relays"></div>
            </section>

            <div id="empty" class="empty">No components yet.</div>
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

    <div class="overlay" id="overlay">
        <div class="modal">
            <h2 class="section-title" id="modal-title">Edit</h2>
            <div class="field"><label>Name</label><input type="text" id="m-name"></div>
            <div class="field"><label>Foxglove topic</label><input type="text" id="m-topic"></div>
            <div class="field"><label>JSON key</label><input type="text" id="m-key"></div>
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
        const INPUT_DEFS = [
            { key: 'throttle', label: 'Throttle',           unit: '%', min: 0,  max: 100 },
            { key: 'steering', label: 'Steering',           unit: '',  min: -1, max: 1   },
            { key: 'brake',    label: 'Brake pedal travel', unit: '%', min: 0,  max: 100 },
        ];
        const RELAY_DEFS = [
            { key: 'dash',         label: 'Dash' },
            { key: 'master_panel', label: 'Master panel' },
            { key: 'bms',          label: 'BMS' },
        ];
        let state = { inputs: {}, relays: {}, sliders: {}, buttons: {} };
        const liveValues = {};      // last live value of each pre-filled VM sensor
        let liveRefs = {};          // name -> elements to update
        let ws;
        let isInitialized = false;
        let editing = null;

        const $ = (id) => document.getElementById(id);
        const clean = (s) => s.trim().replace(/[^a-zA-Z0-9_]/g, '_');
        const cleanKey = (s) => s.trim().replace(/[^a-zA-Z0-9_.]/g, '_').replace(/^\\.+|\\.+$/g, '').replace(/\\.{2,}/g, '.');
        const cleanTopic = (s, def) => {
            let t = s.trim().replace(/[^a-zA-Z0-9_\\/\\-]/g, '_');
            if (!t) t = def;
            if (t[0] !== '/') t = '/' + t;
            return t.replace(/\\/{2,}/g, '/');
        };
        const pct = (v, min, max) => max === min ? 0 : ((v - min) / (max - min)) * 100;
        function el(html) { const t = document.createElement('template'); t.innerHTML = html.trim(); return t.content.firstChild; }

        function setStatus(online) {
            $('status').classList.toggle('online', online);
            $('status-text').innerText = online ? 'Connected' : 'Reconnecting...';
        }
        function normaliseLoaded(loaded) {
            const out = { inputs: {}, relays: {}, sliders: loaded.sliders || {}, buttons: loaded.buttons || {} };
            INPUT_DEFS.forEach(f => {
                const src = loaded.inputs && loaded.inputs[f.key];
                const v = src ? parseFloat(src.value) : 0;
                out.inputs[f.key] = { value: isNaN(v) ? 0 : v, min: f.min, max: f.max };
            });
            RELAY_DEFS.forEach(r => { out.relays[r.key] = !(loaded.relays && loaded.relays[r.key] === false); });
            return out;
        }

        // ---------- vehicle inputs and relays (same as the VM page) ----------
        function buildInputs() {
            INPUT_DEFS.forEach(f => {
                const tile = el(`
                    <div class="tile fixed" id="input-${f.key}">
                        <div class="tile-head"><span class="tile-name">${f.label}</span><button class="gear" title="Zero">⟲</button></div>
                        <div class="big-val"><span class="v">0.00</span><small>${f.unit}</small></div>
                        <div class="slider-wrap">
                            <input type="range" min="${f.min}" max="${f.max}" step="any" value="0">
                            <div class="bounds"><span>${f.min}</span><span>${f.max}</span></div>
                        </div>
                    </div>`);
                tile.querySelector('input[type=range]').addEventListener('input', (e) => {
                    if (!isInitialized) return;
                    setInput(f.key, parseFloat(e.target.value));
                    sendState();
                });
                tile.querySelector('.gear').addEventListener('click', () => {
                    if (!isInitialized) return;
                    setInput(f.key, 0);
                    sendState();
                });
                $('grid-inputs').appendChild(tile);
            });
        }
        function setInput(key, v) {
            const f = INPUT_DEFS.find(x => x.key === key);
            v = Math.max(f.min, Math.min(f.max, v));
            state.inputs[key].value = v;
            const tile = $('input-' + key);
            tile.querySelector('.v').innerText = v.toFixed(2);
            const range = tile.querySelector('input[type=range]');
            range.value = v;
            range.style.setProperty('--pct', pct(v, f.min, f.max) + '%');
        }
        $('zero-all').addEventListener('click', () => {
            if (!isInitialized) return;
            INPUT_DEFS.forEach(f => setInput(f.key, 0));
            sendState();
        });
        function buildRelays() {
            RELAY_DEFS.forEach(r => {
                const tile = el(`
                    <div class="tile fixed" id="relay-${r.key}">
                        <div class="tile-head"><span class="tile-name">${r.label} relay</span>
                            <label class="switch"><input type="checkbox" checked><span></span></label></div>
                        <div class="big-val relay-state closed">CLOSED</div>
                    </div>`);
                const cb = tile.querySelector('input');
                cb.addEventListener('change', () => {
                    if (!isInitialized) { cb.checked = !cb.checked; return; }
                    setRelay(r.key, cb.checked);
                    sendState();
                });
                $('grid-relays').appendChild(tile);
            });
        }
        function setRelay(key, closed) {
            state.relays[key] = closed;
            const tile = $('relay-' + key);
            tile.querySelector('input').checked = closed;
            const st = tile.querySelector('.relay-state');
            st.innerText = closed ? 'CLOSED' : 'OPEN';
            st.classList.toggle('closed', closed);
            st.classList.toggle('open', !closed);
        }
        function syncFixed() {
            INPUT_DEFS.forEach(f => setInput(f.key, state.inputs[f.key].value));
            RELAY_DEFS.forEach(r => setRelay(r.key, state.relays[r.key]));
        }

        // ---------- live values of the VM sensors (only for sensors that are NOT overridden) ----------
        function showSliderValue(ref, v) {
            ref.range.value = v;
            ref.big.innerText = v.toFixed(ref.dec);
            ref.range.style.setProperty('--pct', pct(v, ref.min, ref.max) + '%');
        }
        function updateLive(live) {
            Object.keys(live).forEach(name => {
                liveValues[name] = live[name];
                const ref = liveRefs[name];
                if (ref && state.sliders[name] && !state.sliders[name].override) showSliderValue(ref, live[name]);
            });
        }

        function connectWS() {
            ws = new WebSocket("ws://" + window.location.hostname + ":8001");
            ws.onopen = () => setStatus(true);
            ws.onmessage = (event) => {
                const msg = JSON.parse(event.data);
                if (msg.live) { updateLive(msg.live); return; }      // never touches the controls
                if (isInitialized) return;
                state = normaliseLoaded(msg);
                isInitialized = true;
                syncFixed();
                render();
            };
            ws.onclose = () => { isInitialized = false; setStatus(false); setTimeout(connectWS, 1000); };
        }
        buildInputs();
        buildRelays();
        connectWS();
        function sendState() {
            if (ws && ws.readyState === WebSocket.OPEN && isInitialized) ws.send(JSON.stringify(state));
        }

        function render() {
            liveRefs = {};
            const gs = $('grid-sliders'), gb = $('grid-buttons');
            gs.innerHTML = ''; gb.innerHTML = '';
            const sNames = Object.keys(state.sliders), bNames = Object.keys(state.buttons);
            sNames.forEach(name => gs.appendChild(sliderTile(name, state.sliders[name])));
            bNames.forEach(name => gb.appendChild(buttonTile(name, state.buttons[name])));
            $('group-sliders').style.display = sNames.length ? 'block' : 'none';
            $('group-buttons').style.display = bNames.length ? 'block' : 'none';
            $('empty').style.display = (sNames.length || bNames.length) ? 'none' : 'block';
            $('count-sliders').innerText = sNames.length;
            $('count-buttons').innerText = bNames.length;
        }

        function sliderTile(name, d) {
            const min = d.min, max = d.max, val = d.value;
            const dec = d.integer ? 0 : 2;
            const vmSig = !!d.from_vm;                 // pre-filled VM sensor: LIVE or OVERRIDE
            const tile = el(`
                <div class="tile">
                    <div class="tile-head"><span class="tile-name">${name}</span><button class="gear" title="Edit">⚙</button></div>
                    <div class="topic">${d.topic} · ${d.key}</div>
                    ${vmSig ? `<div class="ovr-row"><label class="switch"><input type="checkbox" class="ovr" ${d.override ? 'checked' : ''}><span></span></label><span class="ovr-label"></span></div>` : ''}
                    <div class="big-val ${d.noise ? 'noisy' : ''}">${parseFloat(val).toFixed(dec)}</div>
                    <div class="slider-wrap">
                        <input type="range" min="${min}" max="${max}" step="${d.integer ? 1 : 'any'}" value="${val}" style="--pct:${pct(val,min,max)}%">
                        <div class="bounds"><span>${min}</span><span>${max}</span></div>
                    </div>
                    <div class="noise-row">
                        <label class="switch"><input type="checkbox" class="nz" ${d.noise ? 'checked' : ''}><span></span></label>
                        <span>White noise</span>
                        <span class="sd ${d.noise ? 'on' : ''}">σ <input type="number" step="0.1" value="${d.std_dev}"></span>
                    </div>
                </div>`);
            const range = tile.querySelector('input[type=range]');
            const big = tile.querySelector('.big-val');
            const ref = { range, big, min, max, dec };

            function applyMode() {                      // LIVE (follows the VM) or OVERRIDE (slider decides)
                if (!vmSig) return;
                const ov = state.sliders[name].override;
                range.disabled = !ov;
                tile.classList.toggle('live', !ov);
                tile.classList.toggle('overridden', ov);
                tile.querySelector('.ovr-label').innerText = ov ? 'Override' : 'Live (VM)';
            }
            if (vmSig) {
                liveRefs[name] = ref;
                if (!d.override && liveValues[name] !== undefined) showSliderValue(ref, liveValues[name]);
                tile.querySelector('.ovr').addEventListener('change', (e) => {
                    const dd = state.sliders[name];
                    dd.override = e.target.checked;
                    // start the override from the value the VM is producing right now
                    if (dd.override && liveValues[name] !== undefined) {
                        dd.value = liveValues[name];
                        showSliderValue(ref, dd.value);
                    }
                    applyMode();
                    sendState();
                });
                applyMode();
            }

            range.addEventListener('input', () => {
                const v = parseFloat(range.value);
                state.sliders[name].value = v;
                big.innerText = v.toFixed(dec);
                range.style.setProperty('--pct', pct(v, min, max) + '%');
                sendState();
            });
            const sw = tile.querySelector('.nz');
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

        function buttonTile(name, d) {
            const tile = el(`
                <div class="tile pad">
                    <div class="tile-head"><span class="tile-name">${name}</span><button class="gear" title="Edit">⚙</button></div>
                    <div class="topic">${d.topic} · ${d.key}</div>
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

        $('comp-type').addEventListener('change', (e) => {
            $('range-inputs').style.display = e.target.value === 'slider' ? 'flex' : 'none';
        });
        function addComponent() {
            const type = $('comp-type').value;
            const raw = $('comp-name').value;
            if (!raw.trim()) return alert("Please provide a name!");
            const name = clean(raw);
            if (state.sliders[name] || state.buttons[name]) return alert("Name already exists!");
            const topic = cleanTopic($('comp-topic').value, '/emulator/' + name);
            const key = cleanKey($('comp-key').value) || name;

            if (type === 'slider') {
                const min = parseFloat($('comp-min').value), max = parseFloat($('comp-max').value);
                if (isNaN(min) || isNaN(max) || min >= max) return alert("Min must be lower than max!");
                state.sliders[name] = { topic, key, value: (min + max) / 2, min, max, noise: false, std_dev: 1.0, integer: false };
            } else {
                state.buttons[name] = { topic, key, value: false, integer: false };
            }
            sendState(); render();
            $('comp-name').value = ''; $('comp-topic').value = ''; $('comp-key').value = '';
        }
        $('add-btn').addEventListener('click', addComponent);
        $('comp-name').addEventListener('keydown', (e) => { if (e.key === 'Enter') addComponent(); });

        function openModal(type, name) {
            editing = { type, name };
            const d = (type === 'slider' ? state.sliders : state.buttons)[name];
            $('modal-title').innerText = 'Edit ' + type;
            $('m-name').value = name;
            $('m-topic').value = d.topic;
            $('m-key').value = d.key;
            $('m-range').style.display = type === 'slider' ? 'flex' : 'none';
            if (type === 'slider') { $('m-min').value = d.min; $('m-max').value = d.max; }
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
            data.topic = cleanTopic($('m-topic').value, '/emulator/' + newName);
            data.key = cleanKey($('m-key').value) || newName;
            if (type === 'slider') {
                const min = parseFloat($('m-min').value), max = parseFloat($('m-max').value);
                if (isNaN(min) || isNaN(max) || min >= max) return alert("Min must be lower than max!");
                data.min = min; data.max = max;
                data.value = Math.max(min, Math.min(max, data.value));
            }
            if (newName !== oldName) {
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
# 5. MAIN LOOP
# ==========================================
async def main():
    threading.Thread(target=start_http_server, daemon=True).start()
    await websockets.serve(ui_ws_handler, "0.0.0.0", 8001)

    loop = asyncio.get_running_loop()
    await loop.create_datagram_endpoint(lambda: vm.CppUdpListener(), local_addr=UDP_FROM_CPP)
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    vehicle = vm.VehicleModel()
    sdc = vm.ShutdownSystem()
    dt = 1.0 / SIM_HZ

    async with FoxgloveServer("0.0.0.0", 8765, "VM Emulator") as fg_server:
        print("🦊 Foxglove streaming live at ws://localhost:8765")
        chans = {}

        async def pub(topic, payload, ts):
            ch = chans.get(topic)
            if ch is None:
                ch = chans[topic] = DynamicChannel(fg_server, topic)
            await ch.send(ts, payload)

        next_tick = time.perf_counter()
        tick = 0

        while True:
            state = UI_STATE
            inp = state["inputs"]
            rel = state["relays"]
            now_ns = time.time_ns()
            act = dict(vm.ACT)                       # exactly what the C++ code last commanded

            # ---------- 1. the Vehicle Model (same steps as vm_server.py) ----------
            sdc.update(dt,
                       dash=rel["dash"], vcu=act["sdc_vcu_relay"],
                       master_panel=rel["master_panel"], bms=rel["bms"],
                       brake_bar=vehicle.brake_pressure_bar,
                       motor_power_w=vehicle.motor_power_w)
            torque_cmd = act["motor_torque"] if (sdc.closed or not vm.SDC_CUTS_MOTOR) else 0.0
            vehicle.step(torque_cmd,
                         float(inp["steering"]["value"]),
                         float(inp["brake"]["value"]),
                         dt)

            real = vm.real_sensor_values(vehicle, inp, sdc)
            groups = {"vm": {}, "other": {}}
            for name, cfg in vm.ADC_SENSORS.items():
                groups[cfg["group"]][name] = vm.sample_sensor(cfg, real[name], vm.noise_lsb(name))
            for name, cfg in vm.DIGITAL_SENSORS.items():
                groups[cfg["group"]][name] = 0       # buttons: set by the button components below
            topics = {vm.sensor_topic(g): p for g, p in groups.items()}

            can_in = vm.build_can_in(vehicle, sdc)
            if can_in:
                topics.setdefault("/CAN/MCU_IN", {}).update(can_in)

            # ---------- 2. live values for the browser (BEFORE the overrides) ----------
            live = {}
            if tick % BROWSER_DIV == 0 and CLIENTS:
                for name, d in state["sliders"].items():
                    if d.get("from_vm"):
                        v = get_path(topics.get(d["topic"], {}), d["key"])
                        if v is not None:
                            live[name] = v

            # ---------- 3. overrides and custom components ----------
            for d in state["sliders"].values():
                if d.get("from_vm") and not d.get("override"):
                    continue                          # pre-filled VM sensor, not overridden: keep the VM value
                val = float(d["value"])
                if d.get("noise"):
                    val += random.gauss(0.0, float(d.get("std_dev", 1.0)))
                    val = clamp(val, d["min"], d["max"])
                if d.get("integer"):
                    val = int(round(val))
                set_path(topics.setdefault(d["topic"], {}), d["key"], val)
            for d in state["buttons"].values():
                val = bool(d["value"])
                set_path(topics.setdefault(d["topic"], {}), d["key"], int(val) if d.get("integer") else val)

            # ---------- 4. Foxglove ----------
            for topic, payload in topics.items():
                await pub(topic, payload, now_ns)
            if tick % vm.PUBLISH_DIV == 0:
                fwd = {"/actuators": act}
                if vm.CAN_OUT:
                    fwd["/CAN/MCU_OUT"] = vm.CAN_OUT
                for topic, payload in fwd.items():
                    await pub(topic, payload, now_ns)
                for topic in [t for t in chans if t not in topics and t not in fwd]:
                    await chans.pop(topic).close()    # topic no longer used

            # ---------- 5. UDP to C++ : same tree as the real VM ----------
            tree = {}
            for topic, payload in topics.items():
                set_in_tree(tree, topic, payload)
            udp_sock.sendto(json.dumps(tree).encode("utf8"), UDP_TO_CPP)

            # ---------- 6. browser ----------
            if live:
                await broadcast_to_browsers(json.dumps({"live": live}))

            # ---------- 7. pacing ----------
            tick += 1
            next_tick += dt
            delay = next_tick - time.perf_counter()
            if delay > 0:
                await asyncio.sleep(delay)
            else:
                if delay < -0.25:
                    next_tick = time.perf_counter()
                await asyncio.sleep(0)


if __name__ == "__main__":
    try:
        print("Starting VM emulator...")
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nShutting down.")