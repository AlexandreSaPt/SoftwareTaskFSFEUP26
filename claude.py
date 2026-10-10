import asyncio
import json
import math
import time
import threading
import random
import os
import socket
from http.server import HTTPServer, BaseHTTPRequestHandler
import websockets
from foxglove_websocket.server import FoxgloveServer

# ==========================================
# 0. PARAMETERS (edit here)
# ==========================================
SIM_HZ = 200                 # physics + wheel-speed sampling rate
PUBLISH_DIV = 4              # slow sensors / custom controls: SIM_HZ/4 = 50 Hz
BROWSER_DIV = 10             # car animation data to the browser: 20 Hz

# ---- vehicle ----
MASS = 250.0                 # kg (car + driver)
WHEEL_RADIUS = 0.25          # m
WHEELBASE = 1.55             # m
TRACK = 1.20                 # m
MAX_STEER = 0.35             # rad at steering = +/-1  (positive = LEFT)
T_DRIVE_MAX = 600.0          # Nm at the wheels: limit applied to the torque coming from C++
T_BRAKE_MAX = 1000.0         # Nm at the wheels at brake pedal = 100 %
BRAKE_P_MAX_BAR = 80.0       # bar of brake line pressure at brake pedal = 100 %
DRIVETRAIN_EFF = 0.90
P_MAX = 80_000.0             # W
MU = 1.4
G = 9.81
RHO = 1.225
CDA = 1.1
C_ROLL = 0.015
C_DRIVETRAIN = 2.0           # N per (m/s)
WHEEL_TEETH = 4              # sensor pulses per wheel revolution

# ---- C++ -> Python torque interface ----
TORQUE_KEY = "drive_torque_nm"   # key in the JSON sent by C++ to UDP port 9001
TORQUE_TIMEOUT = 0.5             # s without a packet -> torque falls back to 0 (watchdog)

# ---- ADC ----
ADC_BITS = 16
ADC_MAX = 2 ** ADC_BITS - 1      # 65535
ADC_VREF = 3.3                   # V

# Each sensor: real quantity range  ->  sensor output voltage range (inside 0..3.3 V)
# "div" = publish every div-th simulation tick.
SENSOR_CFG = {
    "wheel_speed":          dict(label="Wheel speed",          unit="level",
                                 real_min=0.0, real_max=1.0,
                                 v_min=0.0,  v_max=ADC_VREF, div=1),
    "accelerator_position": dict(label="Accelerator position", unit="%",
                                 real_min=0.0, real_max=100.0,
                                 v_min=0.33, v_max=2.97, div=PUBLISH_DIV),
    "brake_pedal_position": dict(label="Brake pedal position", unit="%",
                                 real_min=0.0, real_max=100.0,
                                 v_min=0.33, v_max=2.97, div=PUBLISH_DIV),
    "steering_angle":       dict(label="Steering angle",       unit="deg",
                                 real_min=-math.degrees(MAX_STEER), real_max=math.degrees(MAX_STEER),
                                 v_min=0.33, v_max=2.97, div=PUBLISH_DIV),
    "brake_pressure":       dict(label="Brake pressure",       unit="bar",
                                 real_min=0.0, real_max=BRAKE_P_MAX_BAR,
                                 v_min=0.33, v_max=2.97, div=PUBLISH_DIV),
}

FIXED_INPUTS = {             # name: (min, max)   -- set by the user only
    "throttle": (0.0, 100.0),
    "steering": (-1.0, 1.0),
    "brake":    (0.0, 100.0),
}
NOISE_MAX = 500.0            # slider max, std-dev in ADC counts (LSB)
NOISE_DEFAULT = 5.0
FIXED_NOISE = {name: (0.0, NOISE_MAX) for name in SENSOR_CFG}

WHEELS = {                   # position relative to rear axle centre: (x forward, y left)
    "fl": (WHEELBASE,  TRACK / 2),
    "fr": (WHEELBASE, -TRACK / 2),
    "rl": (0.0,        TRACK / 2),
    "rr": (0.0,       -TRACK / 2),
}


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


# ==========================================
# 0.1 VEHICLE MODEL
# ==========================================
class VehicleModel:
    """Longitudinal point-mass + kinematic bicycle model."""

    def __init__(self):
        self.v = 0.0
        self.ax = 0.0
        self.ay = 0.0
        self.steer_angle = 0.0
        self.yaw_rate = 0.0
        self.yaw = 0.0
        self.x = 0.0
        self.y = 0.0
        self.t_drive = 0.0
        self.t_brake = 0.0
        self.t_net = 0.0
        self.brake_pressure_bar = 0.0
        self.wheel_speed = {k: 0.0 for k in WHEELS}
        self.wheel_angle = {k: 0.0 for k in WHEELS}   # rad, accumulated rotation

    def step(self, drive_torque, steering, brake, dt):
        """drive_torque: Nm at the wheels (comes from the C++ code)."""
        drive_torque = clamp(drive_torque, 0.0, T_DRIVE_MAX)
        brake = clamp(brake, 0.0, 100.0) / 100.0
        steering = clamp(steering, -1.0, 1.0)
        v = self.v

        # ---- torques at the wheels ----
        f_drive = DRIVETRAIN_EFF * drive_torque / WHEEL_RADIUS
        f_drive = min(f_drive, DRIVETRAIN_EFF * P_MAX / max(v, 0.5))   # power limit
        self.t_drive = f_drive * WHEEL_RADIUS
        self.t_brake = brake * T_BRAKE_MAX
        self.brake_pressure_bar = brake * BRAKE_P_MAX_BAR
        self.t_net = self.t_drive - self.t_brake

        f_wheel = self.t_net / WHEEL_RADIUS
        f_max = MU * MASS * G
        f_wheel = clamp(f_wheel, -f_max, f_max)

        # ---- resistances ----
        f_roll = C_ROLL * MASS * G
        if v > 0.0:
            f_res = f_roll + C_DRIVETRAIN * v + 0.5 * RHO * CDA * v * v
            a = (f_wheel - f_res) / MASS
        else:
            a = (f_wheel - f_roll) / MASS if f_wheel > f_roll else 0.0

        v_new = max(0.0, v + a * dt)
        self.ax = (v_new - v) / dt
        self.v = v_new

        # ---- steering / lateral ----
        self.steer_angle = steering * MAX_STEER
        self.yaw_rate = self.v * math.tan(self.steer_angle) / WHEELBASE
        self.ay = self.v * self.yaw_rate
        self.yaw += self.yaw_rate * dt
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt

        # ---- wheel speeds ----
        for k, (wx, wy) in WHEELS.items():
            s = math.hypot(self.v - self.yaw_rate * wy, self.yaw_rate * wx)
            self.wheel_speed[k] = s
            self.wheel_angle[k] += s / WHEEL_RADIUS * dt

    def wheel_levels(self):
        """Ideal rectangle wave (0 / 1) per wheel: 50 % duty, WHEEL_TEETH pulses / rev."""
        return {
            k: 1.0 if (a * WHEEL_TEETH / (2 * math.pi)) % 1.0 < 0.5 else 0.0
            for k, a in self.wheel_angle.items()
        }

    def visual(self):
        """Only what the web app needs to draw the car."""
        return {
            "steering_angle_deg": math.degrees(self.steer_angle),
            "wheel_speed_mps": dict(self.wheel_speed),
        }


# ==========================================
# 0.2 SENSORS: real value -> ADC raw value
# ==========================================
def real_to_adc(real, cfg, sigma_lsb=0.0):
    """Map a real physical value to a 16-bit ADC raw value.

    1. real value  -> sensor output voltage (linear over cfg's ranges)
    2. voltage     -> ideal ADC counts (3.3 V = 65535)
    3. add white noise (gaussian, std-dev in LSB)
    4. round + saturate to 0 .. 65535
    """
    span = cfg["real_max"] - cfg["real_min"]
    volts = cfg["v_min"] + (real - cfg["real_min"]) / span * (cfg["v_max"] - cfg["v_min"])
    volts = clamp(volts, 0.0, ADC_VREF)
    counts = volts / ADC_VREF * ADC_MAX
    if sigma_lsb > 0.0:
        counts += random.gauss(0.0, sigma_lsb)
    return int(clamp(round(counts), 0, ADC_MAX))


def real_sensor_values(vehicle, inputs):
    """The TRUE physical value measured by every sensor (before ADC / noise)."""
    return {
        "wheel_speed":          vehicle.wheel_levels(),                          # dict per wheel
        "accelerator_position": clamp(float(inputs["throttle"]["value"]), 0.0, 100.0),
        "brake_pedal_position": clamp(float(inputs["brake"]["value"]), 0.0, 100.0),
        "steering_angle":       math.degrees(vehicle.steer_angle),
        "brake_pressure":       vehicle.brake_pressure_bar,
    }


def sample_sensor(name, real, sigma):
    cfg = SENSOR_CFG[name]
    if isinstance(real, dict):                      # one independent sensor per wheel
        return {k: real_to_adc(v, cfg, sigma) for k, v in real.items()}
    return real_to_adc(real, cfg, sigma)


# ==========================================
# 1. THE FRONTEND UI
# ==========================================
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
        .btn-ghost.small { padding: 4px 12px; font-size: 11px; letter-spacing: 1px; text-transform: uppercase; color: var(--muted); }
        .btn-danger { background: transparent; color: var(--danger); border: 1px solid var(--danger); padding: 10px 16px; font-size: 13px; }
        .btn-danger:hover { background: var(--danger); color: #fff; }

        .group { margin-bottom: 30px; }
        .grid-sliders { display: grid; grid-template-columns: repeat(auto-fill, minmax(300px, 1fr)); gap: 16px; }
        .grid-buttons { display: grid; grid-template-columns: repeat(auto-fill, minmax(150px, 1fr)); gap: 16px; }
        .empty { text-align: center; color: var(--muted); padding: 40px 10px; border: 1px dashed var(--border); border-radius: var(--radius); margin-bottom: 30px; }

        .tile { position: relative; background: var(--surface); border: 1px solid var(--border); border-radius: var(--radius); padding: 16px 18px; transition: border-color .15s; }
        .tile:hover { border-color: #34436a; }
        .tile.fixed { border-top: 2px solid var(--accent); }
        .tile-head { display: flex; justify-content: space-between; align-items: center; margin-bottom: 6px; min-height: 28px; }
        .tile-name { font-family: 'Rajdhani',sans-serif; font-weight: 600; font-size: 1.1em; letter-spacing: 1px; text-transform: uppercase; color: var(--muted); overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
        .gear { background: transparent; color: var(--muted); font-size: 16px; padding: 2px 7px; }
        .gear:hover { color: var(--accent); }

        .big-val { font-family: 'Rajdhani',sans-serif; font-weight: 700; font-size: 3em; line-height: 1; color: var(--text); font-variant-numeric: tabular-nums; }
        .big-val small { font-size: 0.4em; color: var(--muted); margin-left: 6px; font-weight: 600; }
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
        .pad .tile-head { margin-bottom: 10px; }

        /* ---------- car ---------- */
        .car-box { background: var(--surface); border: 1px solid var(--border); border-radius: var(--radius); padding: 14px; display: flex; justify-content: center; }
        .car-box svg { width: 100%; max-width: 230px; height: auto; }
        .car-body { fill: var(--surface-2); stroke: var(--border); stroke-width: 2; }
        .car-nose { fill: var(--accent); }
        .car-wheel { fill: var(--text); stroke: var(--accent); stroke-width: 2; }
        .car-tread { stroke: var(--bg); stroke-width: 18; stroke-dasharray: 4 8; fill: none; }

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
                <input type="text" id="comp-name" placeholder="e.g. Battery_temp"/>
            </div>
            <div class="row" id="range-inputs">
                <div class="field"><label>Min</label><input type="number" id="comp-min" value="0"/></div>
                <div class="field"><label>Max</label><input type="number" id="comp-max" value="100"/></div>
            </div>
            <button class="btn-primary" id="add-btn">+ Add</button>
        </aside>

        <main>
            <!-- fixed vehicle inputs -->
            <section class="group">
                <h2 class="section-title">Vehicle inputs <button class="btn-ghost small" id="zero-all">Zero all</button></h2>
                <div class="grid-sliders" id="grid-inputs"></div>
            </section>

            <!-- fixed sensor noise (std-dev, ADC counts) -->
            <section class="group">
                <h2 class="section-title">Sensor noise (σ, ADC counts)</h2>
                <div class="grid-sliders" id="grid-noise"></div>
            </section>

            <!-- car -->
            <section class="group">
                <div class="car-box">
                    <svg viewBox="0 0 220 300" xmlns="http://www.w3.org/2000/svg">
                        <rect class="car-body" x="72" y="30" width="76" height="240" rx="30"/>
                        <rect class="car-nose" x="95" y="30" width="30" height="8" rx="4"/>
                        <g id="wheel-fl"><rect class="car-wheel" x="44"  y="55"  width="22" height="46" rx="4"/><line class="car-tread" id="tread-fl" x1="55"  y1="57"  x2="55"  y2="99"/></g>
                        <g id="wheel-fr"><rect class="car-wheel" x="154" y="55"  width="22" height="46" rx="4"/><line class="car-tread" id="tread-fr" x1="165" y1="57"  x2="165" y2="99"/></g>
                        <g id="wheel-rl"><rect class="car-wheel" x="44"  y="205" width="22" height="46" rx="4"/><line class="car-tread" id="tread-rl" x1="55"  y1="207" x2="55"  y2="249"/></g>
                        <g id="wheel-rr"><rect class="car-wheel" x="154" y="205" width="22" height="46" rx="4"/><line class="car-tread" id="tread-rr" x1="165" y1="207" x2="165" y2="249"/></g>
                    </svg>
                </div>
            </section>

            <!-- custom components -->
            <div id="empty" class="empty">No custom components yet. Create a slider or button in the sidebar.</div>
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
        const INPUT_DEFS = [
            { key: 'throttle', label: 'Throttle',           unit: '%', min: 0,  max: 100 },
            { key: 'steering', label: 'Steering',           unit: '',  min: -1, max: 1   },
            { key: 'brake',    label: 'Brake pedal travel', unit: '%', min: 0,  max: 100 },
        ];
        const NOISE_DEFS = [
            { key: 'wheel_speed',          label: 'Wheel speed',          unit: 'LSB', min: 0, max: __NOISE_MAX__ },
            { key: 'accelerator_position', label: 'Accelerator position', unit: 'LSB', min: 0, max: __NOISE_MAX__ },
            { key: 'brake_pedal_position', label: 'Brake pedal position', unit: 'LSB', min: 0, max: __NOISE_MAX__ },
            { key: 'steering_angle',       label: 'Steering angle',       unit: 'LSB', min: 0, max: __NOISE_MAX__ },
            { key: 'brake_pressure',       label: 'Brake pressure',       unit: 'LSB', min: 0, max: __NOISE_MAX__ },
        ];
        const DEFS = { inputs: INPUT_DEFS, noise: NOISE_DEFS };
        const WHEEL_CENTERS = { fl: [55, 78], fr: [165, 78] };
        const TREAD_PERIOD = 12;   // px (dash 4 + gap 8)
        const PX_PER_M = 14;       // visual tread speed

        let state = { inputs: {}, noise: {}, sliders: {}, buttons: {} };
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
        function normaliseLoaded(loaded) {
            const out = { inputs: {}, noise: {}, sliders: loaded.sliders || {}, buttons: loaded.buttons || {} };
            ['inputs', 'noise'].forEach(group => {
                DEFS[group].forEach(f => {
                    const src = loaded[group] && loaded[group][f.key];
                    const v = src ? parseFloat(src.value) : 0;
                    out[group][f.key] = { value: isNaN(v) ? 0 : v, min: f.min, max: f.max };
                });
            });
            return out;
        }
        function connectWS() {
            ws = new WebSocket("ws://" + window.location.hostname + ":8001");
            ws.onopen = () => setStatus(true);
            ws.onmessage = (event) => {
                const msg = JSON.parse(event.data);
                if (msg.telemetry) { updateCar(msg.telemetry); return; }   // never touches the controls
                if (isInitialized) return;
                state = normaliseLoaded(msg);
                isInitialized = true;
                syncFixed();
                render();
            };
            ws.onclose = () => { isInitialized = false; setStatus(false); setTimeout(connectWS, 1000); };
        }
        function sendState() {
            if (ws && ws.readyState === WebSocket.OPEN && isInitialized) ws.send(JSON.stringify(state));
        }

        function el(html) { const t = document.createElement('template'); t.innerHTML = html.trim(); return t.content.firstChild; }

        // ---------- fixed sliders (vehicle inputs + sensor noise) ----------
        // Built once. They only send to the server when the USER moves them.
        function buildFixed(group, gridId, dec, withZero) {
            const grid = $(gridId);
            DEFS[group].forEach(f => {
                const tile = el(`
                    <div class="tile fixed" id="${group}-${f.key}">
                        <div class="tile-head"><span class="tile-name">${f.label}</span>${withZero ? '<button class="gear" title="Zero">⟲</button>' : ''}</div>
                        <div class="big-val"><span class="v">0</span><small>${f.unit}</small></div>
                        <div class="slider-wrap">
                            <input type="range" min="${f.min}" max="${f.max}" step="any" value="0">
                            <div class="bounds"><span>${f.min}</span><span>${f.max}</span></div>
                        </div>
                    </div>`);
                tile.querySelector('input[type=range]').addEventListener('input', (e) => {
                    if (!isInitialized) return;
                    setFixed(group, f.key, parseFloat(e.target.value), dec);
                    sendState();
                });
                if (withZero) tile.querySelector('.gear').addEventListener('click', () => {
                    if (!isInitialized) return;
                    setFixed(group, f.key, 0, dec);
                    sendState();
                });
                grid.appendChild(tile);
            });
        }
        function setFixed(group, key, v, dec) {
            const f = DEFS[group].find(x => x.key === key);
            v = Math.max(f.min, Math.min(f.max, v));
            state[group][key].value = v;
            const tile = $(group + '-' + key);
            tile.querySelector('.v').innerText = v.toFixed(dec);
            const range = tile.querySelector('input[type=range]');
            range.value = v;
            range.style.setProperty('--pct', pct(v, f.min, f.max) + '%');
        }
        function syncFixed() {
            INPUT_DEFS.forEach(f => setFixed('inputs', f.key, state.inputs[f.key].value, 2));
            NOISE_DEFS.forEach(f => setFixed('noise', f.key, state.noise[f.key].value, 1));
        }
        $('zero-all').addEventListener('click', () => {
            if (!isInitialized) return;
            INPUT_DEFS.forEach(f => setFixed('inputs', f.key, 0, 2));
            sendState();
        });

        // ---------- car (steering + wheels turning) ----------
        const wheelSpeed = { fl: 0, fr: 0, rl: 0, rr: 0 };
        const treadOffset = { fl: 0, fr: 0, rl: 0, rr: 0 };
        function updateCar(t) {
            const w = t.wheel_speed_mps || {};
            Object.keys(wheelSpeed).forEach(k => { wheelSpeed[k] = w[k] || 0; });
            const deg = t.steering_angle_deg || 0;   // positive = left = counter-clockwise from above
            ['fl', 'fr'].forEach(k => {
                const [cx, cy] = WHEEL_CENTERS[k];
                $('wheel-' + k).setAttribute('transform', 'rotate(' + (-deg) + ' ' + cx + ' ' + cy + ')');
            });
        }
        let lastFrame = performance.now();
        function animate(now) {
            const dt = Math.min(0.1, (now - lastFrame) / 1000);
            lastFrame = now;
            Object.keys(wheelSpeed).forEach(k => {
                treadOffset[k] = (treadOffset[k] - wheelSpeed[k] * PX_PER_M * dt) % TREAD_PERIOD;
                $('tread-' + k).setAttribute('stroke-dashoffset', treadOffset[k]);
            });
            requestAnimationFrame(animate);
        }

        // ---------- rendering of CUSTOM components (structure only) ----------
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

        // ---------- boot ----------
        buildFixed('inputs', 'grid-inputs', 2, true);
        buildFixed('noise', 'grid-noise', 1, false);
        connectWS();
        requestAnimationFrame(animate);
    </script>
</body>
</html>
""".replace("__NOISE_MAX__", str(int(NOISE_MAX)))

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


def _normalise_group(raw, spec, default):
    raw = raw if isinstance(raw, dict) else {}
    out = {}
    for name, (lo, hi) in spec.items():
        try:
            v = float(raw.get(name, {}).get("value", default))
        except (TypeError, ValueError, AttributeError):
            v = default
        if not math.isfinite(v):
            v = default
        out[name] = {"value": clamp(v, lo, hi), "min": lo, "max": hi}
    return out


def normalise_state(s):
    """Make sure the UI state always has sliders, buttons, the 3 fixed inputs and the 5 noise sliders."""
    if not isinstance(s, dict):
        s = {}
    s.setdefault("sliders", {})
    s.setdefault("buttons", {})
    s["inputs"] = _normalise_group(s.get("inputs"), FIXED_INPUTS, 0.0)
    s["noise"] = _normalise_group(s.get("noise"), FIXED_NOISE, NOISE_DEFAULT)
    return s


UI_STATE = normalise_state({})

if os.path.exists(CONFIG_FILE):
    try:
        with open(CONFIG_FILE, "r") as f:
            UI_STATE = normalise_state(json.load(f))
        for btn in UI_STATE["buttons"].values():
            btn["value"] = False                      # no stuck buttons
        for name in FIXED_INPUTS:
            UI_STATE["inputs"][name]["value"] = 0.0   # car starts with pedals released
        print(f"💾 Loaded existing UI layout from {CONFIG_FILE}")
    except Exception as e:
        print(f"⚠️ Failed to load {CONFIG_FILE}: {e}")

CLIENTS = set()   # connected browsers (for the car animation)


async def ui_ws_handler(websocket, *args, **kwargs):
    global UI_STATE

    await websocket.send(json.dumps(UI_STATE))
    CLIENTS.add(websocket)

    try:
        async for message in websocket:
            try:
                UI_STATE = normalise_state(json.loads(message))
                with open(CONFIG_FILE, "w") as f:
                    json.dump(UI_STATE, f, indent=4)
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


# ---- data coming from C++ (UDP 9001) ----
CPP_STATE = {}            # last full packet, forwarded to Foxglove as /cpp_telemetry
TORQUE_REQUEST = 0.0      # Nm at the wheels, set ONLY by the C++ code
TORQUE_STAMP = 0.0        # time.monotonic() of the last torque update


class CppUdpListener(asyncio.DatagramProtocol):
    def __init__(self):
        self.first_message = True

    def datagram_received(self, data, addr):
        global CPP_STATE, TORQUE_REQUEST, TORQUE_STAMP
        try:
            msg = json.loads(data.decode('utf-8'))
        except (json.JSONDecodeError, UnicodeDecodeError):
            return
        if not isinstance(msg, dict):
            return

        CPP_STATE = msg

        # The ONLY thing C++ can control is the drive torque.
        # Throttle / steering / brake are exclusively set from the web UI.
        t = msg.get(TORQUE_KEY)
        if isinstance(t, (int, float)) and not isinstance(t, bool) and math.isfinite(t):
            TORQUE_REQUEST = clamp(float(t), 0.0, T_DRIVE_MAX)
            TORQUE_STAMP = time.monotonic()

        if self.first_message:
            print(f"✅ Success! Received data from C++: {msg}")
            self.first_message = False


def obj_schema(props):
    return {"type": "object", "properties": props}


async def main():
    threading.Thread(target=start_http_server, daemon=True).start()
    await websockets.serve(ui_ws_handler, "0.0.0.0", 8001)

    loop = asyncio.get_running_loop()
    await loop.create_datagram_endpoint(
        lambda: CppUdpListener(),
        local_addr=('127.0.0.1', 9001)
    )

    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    vehicle = VehicleModel()

    async with FoxgloveServer("0.0.0.0", 8765, "Dynamic UI Server") as fg_server:
        print("🦊 Foxglove streaming live at ws://localhost:8765")

        # ---- one Foxglove topic per real sensor (raw ADC counts, 0..65535) ----
        num = {"type": "number"}
        sensor_chans = {}
        for name in SENSOR_CFG:
            if name == "wheel_speed":
                schema = obj_schema({k: num for k in WHEELS})
            else:
                schema = obj_schema({"raw": num})
            sensor_chans[name] = await fg_server.add_channel({
                "topic": f"/sensors/{name}",
                "encoding": "json",
                "schemaName": "ADC_" + name,
                "schema": json.dumps(schema)
            })

        # ---- C++ telemetry channel ----
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

        # ---- custom UI channel (dynamic) ----
        chan_id = None
        current_sliders = set()
        current_buttons = set()
        custom = {"sliders": {}, "buttons": {}}

        latest_adc = {name: ({k: 0 for k in WHEELS} if name == "wheel_speed" else 0)
                      for name in SENSOR_CFG}

        dt = 1.0 / SIM_HZ
        next_tick = time.perf_counter()
        tick = 0

        while True:
            tick += 1
            inp = UI_STATE["inputs"]

            # ---------- 1. torque from C++ (with watchdog) ----------
            if time.monotonic() - TORQUE_STAMP > TORQUE_TIMEOUT:
                torque = 0.0
            else:
                torque = TORQUE_REQUEST

            # ---------- 2. vehicle model ----------
            vehicle.step(
                torque,
                float(inp["steering"]["value"]),
                float(inp["brake"]["value"]),
                dt,
            )

            # ---------- 3. sensors: real value -> ADC raw (+ white noise) ----------
            real = real_sensor_values(vehicle, inp)
            now_ns = time.time_ns()
            for name, cfg in SENSOR_CFG.items():
                if tick % cfg["div"] != 0:
                    continue
                sigma = float(UI_STATE["noise"][name]["value"])
                adc = sample_sensor(name, real[name], sigma)
                latest_adc[name] = adc
                payload = adc if name == "wheel_speed" else {"raw": adc}
                await fg_server.send_message(
                    sensor_chans[name], now_ns, json.dumps(payload).encode("utf8"))

            # ---------- 4. custom controls + C++ telemetry (50 Hz) ----------
            if tick % PUBLISH_DIV == 0:
                new_sliders = set(UI_STATE.get("sliders", {}).keys())
                new_buttons = set(UI_STATE.get("buttons", {}).keys())

                if new_sliders != current_sliders or new_buttons != current_buttons:
                    if chan_id is not None:
                        await fg_server.remove_channel(chan_id)
                    schema = {
                        "type": "object",
                        "properties": {
                            "sliders": {"type": "object", "properties": {n: {"type": "number"} for n in new_sliders}},
                            "buttons": {"type": "object", "properties": {n: {"type": "boolean"} for n in new_buttons}}
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

                custom = {"sliders": {}, "buttons": {}}
                for name, data in UI_STATE.get("sliders", {}).items():
                    val = float(data.get("value", 0.0))
                    if data.get("noise"):
                        val += random.gauss(0, float(data.get("std_dev", 1.0)))
                        val = max(float(data.get("min", val)), min(float(data.get("max", val)), val))
                    custom["sliders"][name] = val
                for name, data in UI_STATE.get("buttons", {}).items():
                    custom["buttons"][name] = bool(data.get("value", False))

                if chan_id is not None and (current_sliders or current_buttons):
                    await fg_server.send_message(
                        chan_id, now_ns, json.dumps(custom).encode("utf8"))

                if CPP_STATE:
                    await fg_server.send_message(
                        cpp_chan_id, now_ns, json.dumps(CPP_STATE).encode("utf8"))

            # ---------- 5. UDP to C++ : sensors (ADC raw) + custom controls ----------
            udp_payload = {
                "sensors": latest_adc,
                "sliders": custom["sliders"],
                "buttons": custom["buttons"],
            }
            udp_sock.sendto(json.dumps(udp_payload).encode("utf8"), ("127.0.0.1", 9000))

            # ---------- 6. car animation data to the browser ----------
            if tick % BROWSER_DIV == 0 and CLIENTS:
                await broadcast_to_browsers(json.dumps({"telemetry": vehicle.visual()}))

            # ---------- 7. fixed-rate pacing ----------
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
        print("Starting Foxglove Dynamic UI Backend...")
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nShutting down.")