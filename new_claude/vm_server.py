#!/usr/bin/env python3
"""
vm_server.py  -  Vehicle Model (VM) + Shutdown-circuit model, streamed to Foxglove and to the C++ code.

Foxglove topics (JSON). One topic per group, the signals are fields inside it, so Foxglove
shows each group as a single expandable entry:
  /sensors/vm      vehicle-model sensors, ADC raw 0..65535:
                   wheel_speed{fl,fr,rl,rr}, accelerator_position, brake_pedal_position,
                   steering_angle, brake_pressure, accel_x, accel_y
  /sensors/other   ATS_btn, R2D_btn (digital 0/1) and sdc_after_<dash|vcu|master_panel|bms|bspd> (ADC raw)
  /actuators       what the C++ code commanded: motor_torque, TS_activated_led, R2D_LED,
                   sdc_vcu_relay, brake_light  (echo of what the VM received)
  /CAN/MCU_OUT     free-form data written by the C++ code
  /CAN/MCU_IN      free-form data sent to the C++ code (placeholder, see build_can_in())

UDP to C++   127.0.0.1:9000   (200 Hz)  -> same tree as the topics, e.g.
        {"sensors": {"vm": {"brake_pressure": 123, "wheel_speed": {"fl": 1, ...}}, "other": {...}},
         "CAN": {"MCU_IN": {...}}}
UDP from C++ 127.0.0.1:9001   ->
        {"actuators": {"motor_torque": 120.0, "sdc_vcu_relay": true, "TS_activated_led": false,
                       "R2D_LED": false, "brake_light": false},
         "CAN": {"MCU_OUT": {...anything...}}}
        Keys can be sent separately; the VM keeps the last value of each one.

Run:   python3 vm_server.py                                  -> start the VM  (UI on http://localhost:8000)
       python3 vm_server.py --export-signals vm_signals.json -> write the signal list for the emulator
"""
import asyncio
import json
import math
import os
import random
import socket
import sys
import threading
import time
from http.server import HTTPServer, BaseHTTPRequestHandler

import websockets
from foxglove_websocket.server import FoxgloveServer

# ==========================================
# 0. PARAMETERS (edit here)
# ==========================================
SIM_HZ = 200                 # physics + sensor sampling rate (all sensors)
PUBLISH_DIV = 4              # actuators / CAN to Foxglove: SIM_HZ/4 = 50 Hz

UDP_TO_CPP = ("127.0.0.1", 9000)
UDP_FROM_CPP = ("127.0.0.1", 9001)

# ---- sensor noise (white, gaussian). NOT adjustable in the UI. ----
SENSOR_NOISE_LSB_DEFAULT = 5.0   # std-dev in ADC counts (LSB) applied to every ADC sensor
SENSOR_NOISE_LSB = {             # per-sensor override, e.g. {"brake_pressure": 20.0, "sdc_after_bms": 8.0}
}

# ---- vehicle ----
MASS = 250.0                 # kg (car + driver)
WHEEL_RADIUS = 0.25          # m
WHEELBASE = 1.55             # m
TRACK = 1.20                 # m
MAX_STEER = 0.35             # rad at steering = +/-1  (positive = LEFT)
MOTOR_TORQUE_MAX = 600.0     # Nm at the wheels: maximum torque the motor can give
T_BRAKE_MAX = 1000.0         # Nm at the wheels at brake pedal = 100 %
BRAKE_P_MAX_BAR = 80.0       # bar of brake line pressure at brake pedal = 100 %
DRIVETRAIN_EFF = 0.90
P_MAX = 80_000.0             # W, motor power limit
MU = 1.4
G = 9.81
RHO = 1.225
CDA = 1.1
C_ROLL = 0.015
C_DRIVETRAIN = 2.0           # N per (m/s)
WHEEL_TEETH = 4              # wheel-speed sensor pulses per wheel revolution
ACCEL_RANGE_G = 4.0          # accelerometer full scale: +/- this many g (positive x = forward, y = left)

# ---- shutdown circuit ----
VBAT = 24.0                  # V
SDC_ORDER = ["dash", "vcu", "master_panel", "bms", "bspd"]    # order along the line
UI_RELAYS = ["dash", "master_panel", "bms"]                   # latching buttons in the UI
SDC_CUTS_MOTOR = True        # physics: open shutdown circuit -> motor has no torque
BSPD_BRAKE_BAR = 30.0        # brake pressure above this = "hard braking"
BSPD_POWER_W = 5_000.0       # motor power above this while hard braking = fault
BSPD_RESET_S = 10.0          # fault must be absent this long before the relay closes again
# measurement connection problems (reads 0 V + noise whatever the real voltage is)
SDC_DROPOUT_PROB = 0.05      # probability of a dropout starting ...
SDC_DROPOUT_CHECK_S = 1.0    # ... at each trial, one trial per channel every this many seconds
SDC_DROPOUT_DURATION_S = 0.5 # dropout duration (mean)
SDC_DROPOUT_JITTER_S = 0.1   # dropout duration noise (std-dev)

# ---- actuators: value of each one until the C++ code writes it for the first time ----
# (power-on state of the outputs; the VM has no timeout / safety logic, the MCU owns that)
ACT_INITIAL = {"motor_torque": 0.0, "TS_activated_led": False, "R2D_LED": False,
               "sdc_vcu_relay": False, "brake_light": False}   # sdc_vcu_relay: True = closed

# ---- ADC ----
ADC_BITS = 16
ADC_MAX = 2 ** ADC_BITS - 1      # 65535
ADC_VREF = 3.3                   # V

FIXED_INPUTS = {                 # name: (min, max)   -- set by the user only (web UI)
    "throttle": (0.0, 100.0),
    "steering": (-1.0, 1.0),
    "brake":    (0.0, 100.0),
}

WHEELS = {                       # position relative to rear axle centre: (x forward, y left)
    "fl": (WHEELBASE,  TRACK / 2),
    "fr": (WHEELBASE, -TRACK / 2),
    "rl": (0.0,        TRACK / 2),
    "rr": (0.0,       -TRACK / 2),
}


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


# ==========================================
# 0.1 SENSOR TABLE
# ==========================================
def _adc(group, unit, rmin, rmax, vmin, vmax, default_real, channels=None):
    return dict(group=group, unit=unit, real_min=rmin, real_max=rmax,
                v_min=vmin, v_max=vmax, default_real=default_real, channels=channels)


_ST = math.degrees(MAX_STEER)
_AR = ACCEL_RANGE_G * G
ADC_SENSORS = {
    # ---- group "vm": measurements that come from the vehicle dynamics model ----
    "wheel_speed":          _adc("vm", "level", 0.0, 1.0,   0.0,  ADC_VREF, 0.0, list(WHEELS)),
    "accelerator_position": _adc("vm", "%",     0.0, 100.0, 0.33, 2.97, 0.0),
    "brake_pedal_position": _adc("vm", "%",     0.0, 100.0, 0.33, 2.97, 0.0),
    "steering_angle":       _adc("vm", "deg",   -_ST, _ST,  0.33, 2.97, 0.0),
    "brake_pressure":       _adc("vm", "bar",   0.0, BRAKE_P_MAX_BAR, 0.33, 2.97, 0.0),
    "accel_x":              _adc("vm", "m/s2",  -_AR, _AR,  0.33, 2.97, 0.0),
    "accel_y":              _adc("vm", "m/s2",  -_AR, _AR,  0.33, 2.97, 0.0),
}
# ---- group "other": shutdown circuit voltages (VBAT -> voltage divider -> 3.3 V) ----
for _n in SDC_ORDER:
    ADC_SENSORS[f"sdc_after_{_n}"] = _adc("other", "V", 0.0, VBAT, 0.0, ADC_VREF, VBAT)

DIGITAL_SENSORS = {              # buttons: digital input, 0 / 1, no ADC and no noise
    "ATS_btn": dict(group="other"),
    "R2D_btn": dict(group="other"),
}
ACTUATORS = {                    # name: (type, unit)
    "motor_torque":     ("number", "Nm"),
    "TS_activated_led": ("boolean", ""),
    "R2D_LED":          ("boolean", ""),
    "sdc_vcu_relay":    ("boolean", ""),
    "brake_light":      ("boolean", ""),
}


def noise_lsb(name):
    return float(SENSOR_NOISE_LSB.get(name, SENSOR_NOISE_LSB_DEFAULT))


def sensor_topic(group):
    return f"/sensors/{group}"


# ==========================================
# 0.2 VEHICLE MODEL
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
        self.motor_torque = 0.0       # Nm at the wheels, what the motor really delivers
        self.motor_power_w = 0.0      # motor torque * wheel angular velocity (braking NOT included)
        self.t_brake = 0.0
        self.brake_pressure_bar = 0.0
        self.wheel_speed = {k: 0.0 for k in WHEELS}
        self.wheel_angle = {k: 0.0 for k in WHEELS}

    def step(self, motor_torque_cmd, steering, brake, dt):
        motor_torque_cmd = clamp(motor_torque_cmd, 0.0, MOTOR_TORQUE_MAX)
        brake = clamp(brake, 0.0, 100.0) / 100.0
        steering = clamp(steering, -1.0, 1.0)
        v = self.v
        omega = v / WHEEL_RADIUS

        # ---- torques ----
        self.motor_torque = min(motor_torque_cmd, P_MAX / max(omega, 0.5 / WHEEL_RADIUS))
        self.t_brake = brake * T_BRAKE_MAX
        self.brake_pressure_bar = brake * BRAKE_P_MAX_BAR
        t_net = DRIVETRAIN_EFF * self.motor_torque - self.t_brake      # summed torque at the wheels

        f_wheel = t_net / WHEEL_RADIUS
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
        self.motor_power_w = self.motor_torque * (self.v / WHEEL_RADIUS)

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


# ==========================================
# 0.3 SHUTDOWN CIRCUIT
# ==========================================
class ShutdownSystem:
    """dash -> vcu -> master_panel -> bms -> bspd, relays/buttons in series."""

    def __init__(self):
        self.t = 0.0                                  # simulation time
        self.bspd_closed = True
        self.bspd_fault = False
        self.last_fault_t = -1e9
        self.relays = {n: True for n in SDC_ORDER}
        self._drop_until = {n: 0.0 for n in SDC_ORDER}
        self._next_trial = {n: random.uniform(0.0, SDC_DROPOUT_CHECK_S) for n in SDC_ORDER}

    def update(self, dt, dash, vcu, master_panel, bms, brake_bar, motor_power_w):
        self.t += dt
        # BSPD: hard braking while the motors push more than 5 kW -> open for at least 10 s
        self.bspd_fault = brake_bar > BSPD_BRAKE_BAR and motor_power_w > BSPD_POWER_W
        if self.bspd_fault:
            self.bspd_closed = False
            self.last_fault_t = self.t
        elif not self.bspd_closed and self.t - self.last_fault_t >= BSPD_RESET_S:
            self.bspd_closed = True
        self.relays = {"dash": dash, "vcu": vcu, "master_panel": master_panel,
                       "bms": bms, "bspd": self.bspd_closed}

    @property
    def closed(self):
        return all(self.relays.values())

    def true_voltages(self):
        """Voltage after each element: VBAT until the first open relay, 0 V from there on."""
        v, out = VBAT, {}
        for n in SDC_ORDER:
            if not self.relays[n]:
                v = 0.0
            out[n] = v
        return out

    def measured_voltages(self):
        """True voltages, but each channel can randomly read 0 V (bad connection)."""
        true = self.true_voltages()
        out = {}
        for n in SDC_ORDER:
            if self.t >= self._drop_until[n] and self.t >= self._next_trial[n]:
                self._next_trial[n] = self.t + SDC_DROPOUT_CHECK_S
                if random.random() < SDC_DROPOUT_PROB:
                    self._drop_until[n] = self.t + max(
                        0.0, random.gauss(SDC_DROPOUT_DURATION_S, SDC_DROPOUT_JITTER_S))
            out[n] = 0.0 if self.t < self._drop_until[n] else true[n]
        return out


# ==========================================
# 0.4 SENSORS: real value -> ADC raw value
# ==========================================
def real_to_adc(real, cfg, sigma_lsb=0.0):
    """Map a real physical value to a 16-bit ADC raw value.

    1. real value -> sensor output voltage (linear, cfg ranges)
    2. voltage    -> ideal ADC counts (3.3 V = 65535)
    3. + white noise (gaussian, std-dev in LSB)
    4. round and saturate to 0 .. 65535
    """
    span = cfg["real_max"] - cfg["real_min"]
    volts = cfg["v_min"] + (real - cfg["real_min"]) / span * (cfg["v_max"] - cfg["v_min"])
    volts = clamp(volts, 0.0, ADC_VREF)
    counts = volts / ADC_VREF * ADC_MAX
    if sigma_lsb > 0.0:
        counts += random.gauss(0.0, sigma_lsb)
    return int(clamp(round(counts), 0, ADC_MAX))


def real_sensor_values(vehicle, inputs, sdc):
    """TRUE physical value seen by every ADC sensor (before ADC and noise)."""
    out = {
        "wheel_speed":          vehicle.wheel_levels(),
        "accelerator_position": clamp(float(inputs["throttle"]["value"]), 0.0, 100.0),
        "brake_pedal_position": clamp(float(inputs["brake"]["value"]), 0.0, 100.0),
        "steering_angle":       math.degrees(vehicle.steer_angle),
        "brake_pressure":       vehicle.brake_pressure_bar,
        "accel_x":              vehicle.ax,
        "accel_y":              vehicle.ay,
    }
    for n, v in sdc.measured_voltages().items():
        out[f"sdc_after_{n}"] = v
    return out


def sample_sensor(cfg, real, sigma):
    if cfg["channels"]:                              # one independent sensor per channel
        return {k: real_to_adc(real[k], cfg, sigma) for k in cfg["channels"]}
    return real_to_adc(real, cfg, sigma)


def build_can_in(vehicle, sdc):
    """Data sent to the C++ code on /CAN/MCU_IN. Empty for now: add your own frames here."""
    return {}


# ==========================================
# 0.5 SIGNAL LIST (JSON for the emulator)
# ==========================================
SIGNALS_VERSION = 2


def signals_description():
    signals = []
    for name, c in ADC_SENSORS.items():
        signals.append({
            "name": name,
            "group": c["group"],
            "topic": sensor_topic(c["group"]),
            "kind": "adc",
            "keys": [f"{name}.{k}" for k in c["channels"]] if c["channels"] else [name],
            "raw_min": 0,
            "raw_max": ADC_MAX,
            "default_raw": real_to_adc(c["default_real"], c, 0.0),
            "unit": c["unit"],
            "real_min": c["real_min"],
            "real_max": c["real_max"],
            "v_min": c["v_min"],
            "v_max": c["v_max"],
            "rate_hz": SIM_HZ,
            "noise_lsb": noise_lsb(name),
        })
    for name, c in DIGITAL_SENSORS.items():
        signals.append({
            "name": name,
            "group": c["group"],
            "topic": sensor_topic(c["group"]),
            "kind": "digital",
            "keys": [name],
            "raw_min": 0,
            "raw_max": 1,
            "default_raw": 0,
            "rate_hz": SIM_HZ,
        })
    return {
        "version": SIGNALS_VERSION,
        "description": "Everything the VM sends. 'keys' are the field paths ('.' = nested object) "
                       "inside the Foxglove topic; the UDP packet to C++ has the same tree.",
        "adc": {"bits": ADC_BITS, "max": ADC_MAX, "vref_v": ADC_VREF},
        "udp_to_cpp": f"{UDP_TO_CPP[0]}:{UDP_TO_CPP[1]}",
        "udp_from_cpp": f"{UDP_FROM_CPP[0]}:{UDP_FROM_CPP[1]}",
        "signals": signals,
        "actuators_received_from_cpp": {
            "topic": "/actuators",
            "fields": [
                {"name": n, "type": t, "unit": u,
                 **({"min": 0.0, "max": MOTOR_TORQUE_MAX} if n == "motor_torque" else {})}
                for n, (t, u) in ACTUATORS.items()
            ],
        },
        "can": {"mcu_out": "/CAN/MCU_OUT", "mcu_in": "/CAN/MCU_IN"},
    }


# ==========================================
# 1. FOXGLOVE HELPERS
# ==========================================
def schema_of(v):
    if isinstance(v, bool):
        return {"type": "boolean"}
    if isinstance(v, (int, float)):
        return {"type": "number"}
    if isinstance(v, str):
        return {"type": "string"}
    if isinstance(v, dict):
        return {"type": "object", "properties": {k: schema_of(x) for k, x in v.items()}}
    if isinstance(v, list):
        return {"type": "array", "items": schema_of(v[0]) if v else {}}
    return {}


class DynamicChannel:
    """JSON channel whose schema follows the payload (re-created if the keys/types change)."""

    def __init__(self, server, topic):
        self.server = server
        self.topic = topic
        self.chan_id = None
        self.sig = None

    async def send(self, ts_ns, payload):
        schema = {"type": "object", "properties": {k: schema_of(v) for k, v in payload.items()}}
        sig = json.dumps(schema, sort_keys=True)
        if sig != self.sig:
            if self.chan_id is not None:
                await self.server.remove_channel(self.chan_id)
            self.chan_id = await self.server.add_channel({
                "topic": self.topic,
                "encoding": "json",
                "schemaName": self.topic.strip("/").replace("/", "_"),
                "schema": json.dumps(schema),
            })
            self.sig = sig
        await self.server.send_message(self.chan_id, ts_ns, json.dumps(payload).encode("utf8"))


def set_in_tree(tree, topic, payload):
    """'/sensors/vm' + {...} -> tree['sensors']['vm'] = {...}"""
    parts = [p for p in topic.split("/") if p]
    node = tree
    for p in parts[:-1]:
        node = node.setdefault(p, {})
    node.setdefault(parts[-1], {}).update(payload)


# ==========================================
# 2. WEB UI
# ==========================================
HTML_PAGE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>FS FEUP | Vehicle Model</title>
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

        main { max-width: 1100px; margin: auto; padding: 24px 28px 60px; }
        @media (max-width: 860px) { main { padding: 16px; } }
        .section-title { font-family: 'Rajdhani',sans-serif; font-size: 1.05em; letter-spacing: 2px; text-transform: uppercase; color: var(--accent); margin: 0 0 14px; display: flex; align-items: center; justify-content: space-between; }
        button { font-family: inherit; cursor: pointer; border: none; border-radius: 6px; transition: all .15s; }
        .btn-ghost { background: transparent; color: var(--muted); border: 1px solid var(--border); padding: 4px 12px; font-size: 11px; letter-spacing: 1px; text-transform: uppercase; }
        .btn-ghost:hover { background: var(--surface-2); }

        .group { margin-bottom: 30px; }
        .grid-sliders { display: grid; grid-template-columns: repeat(auto-fill, minmax(300px, 1fr)); gap: 16px; }
        .grid-buttons { display: grid; grid-template-columns: repeat(auto-fill, minmax(150px, 1fr)); gap: 16px; }

        .tile { position: relative; background: var(--surface); border: 1px solid var(--border); border-radius: var(--radius); padding: 16px 18px; transition: border-color .15s; }
        .tile:hover { border-color: #34436a; }
        .tile.fixed { border-top: 2px solid var(--accent); }
        .tile-head { display: flex; justify-content: space-between; align-items: center; margin-bottom: 6px; min-height: 28px; }
        .tile-name { font-family: 'Rajdhani',sans-serif; font-weight: 600; font-size: 1.1em; letter-spacing: 1px; text-transform: uppercase; color: var(--muted); overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
        .gear { background: transparent; color: var(--muted); font-size: 16px; padding: 2px 7px; }
        .gear:hover { color: var(--accent); }

        .big-val { font-family: 'Rajdhani',sans-serif; font-weight: 700; font-size: 3em; line-height: 1; color: var(--text); font-variant-numeric: tabular-nums; }
        .big-val small { font-size: 0.4em; color: var(--muted); margin-left: 6px; font-weight: 600; }
        .relay-state.closed { color: var(--ok); }
        .relay-state.open { color: var(--danger); }
        .slider-wrap { margin-top: 14px; }
        input[type=range] { -webkit-appearance: none; appearance: none; width: 100%; height: 8px; border-radius: 4px; outline: none; cursor: pointer;
            background: linear-gradient(to right, var(--accent) var(--pct,50%), var(--surface-2) var(--pct,50%)); }
        input[type=range]::-webkit-slider-thumb { -webkit-appearance: none; width: 20px; height: 20px; border-radius: 50%; background: var(--text); border: 3px solid var(--accent); box-shadow: 0 0 10px rgba(255,106,19,0.5); }
        input[type=range]::-moz-range-thumb { width: 14px; height: 14px; border-radius: 50%; background: var(--text); border: 3px solid var(--accent); }
        .bounds { display: flex; justify-content: space-between; font-size: 0.72em; color: var(--muted); margin-top: 6px; }
        .switch { position: relative; width: 44px; height: 24px; flex-shrink: 0; }
        .switch input { opacity: 0; width: 0; height: 0; }
        .switch span { position: absolute; inset: 0; background: var(--surface-2); border: 1px solid var(--danger); border-radius: 999px; cursor: pointer; transition: .2s; }
        .switch span::before { content: ''; position: absolute; width: 18px; height: 18px; left: 2px; top: 2px; background: var(--muted); border-radius: 50%; transition: .2s; }
        .switch input:checked + span { background: rgba(61,220,132,0.25); border-color: var(--ok); }
        .switch input:checked + span::before { transform: translateX(20px); background: var(--ok); }

        .tile.pad { padding: 14px; text-align: center; }
        .pad-btn { width: 100%; aspect-ratio: 1; border-radius: 50%; background: var(--surface-2); border: 2px solid var(--accent); color: var(--accent); font-family: 'Rajdhani',sans-serif; font-weight: 700; letter-spacing: 2px; font-size: 0.95em; user-select: none; -webkit-user-select: none; touch-action: none; max-width: 120px; }
        .pad-btn:hover { background: rgba(255,106,19,0.12); }
        .pad-btn.held { background: var(--accent); color: #0B0B0B; box-shadow: 0 0 28px rgba(255,106,19,0.6); transform: scale(0.96); }
        .pad .tile-head { margin-bottom: 10px; }
    </style>
</head>
<body>
    <div class="topbar">
        <div class="brand">FS <span>FEUP</span><small>VEHICLE MODEL</small></div>
        <div class="status" id="status"><span class="dot"></span><span id="status-text">Disconnected</span></div>
    </div>

    <main>
        <section class="group">
            <h2 class="section-title">Vehicle inputs <button class="btn-ghost" id="zero-all">Zero all</button></h2>
            <div class="grid-sliders" id="grid-inputs"></div>
        </section>

        <section class="group">
            <h2 class="section-title">Shutdown line (latching)</h2>
            <div class="grid-sliders" id="grid-relays"></div>
        </section>

        <section class="group">
            <h2 class="section-title">Buttons (hold)</h2>
            <div class="grid-buttons" id="grid-buttons"></div>
        </section>
    </main>

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
        const BUTTON_DEFS = [
            { key: 'ATS_btn', label: 'ATS' },
            { key: 'R2D_btn', label: 'R2D' },
        ];

        let state = { inputs: {}, relays: {}, buttons: {} };
        let ws;
        let isInitialized = false;

        const $ = (id) => document.getElementById(id);
        const pct = (v, min, max) => max === min ? 0 : ((v - min) / (max - min)) * 100;
        function el(html) { const t = document.createElement('template'); t.innerHTML = html.trim(); return t.content.firstChild; }

        function setStatus(online) {
            $('status').classList.toggle('online', online);
            $('status-text').innerText = online ? 'Connected' : 'Reconnecting...';
        }
        function normaliseLoaded(loaded) {
            const out = { inputs: {}, relays: {}, buttons: {} };
            INPUT_DEFS.forEach(f => {
                const src = loaded.inputs && loaded.inputs[f.key];
                const v = src ? parseFloat(src.value) : 0;
                out.inputs[f.key] = { value: isNaN(v) ? 0 : v, min: f.min, max: f.max };
            });
            RELAY_DEFS.forEach(r => { out.relays[r.key] = !(loaded.relays && loaded.relays[r.key] === false); });
            BUTTON_DEFS.forEach(b => { out.buttons[b.key] = !!(loaded.buttons && loaded.buttons[b.key]); });
            return out;
        }
        function connectWS() {
            ws = new WebSocket("ws://" + window.location.hostname + ":8001");
            ws.onopen = () => setStatus(true);
            ws.onmessage = (event) => {
                if (isInitialized) return;
                state = normaliseLoaded(JSON.parse(event.data));
                isInitialized = true;
                syncAll();
            };
            ws.onclose = () => { isInitialized = false; setStatus(false); setTimeout(connectWS, 1000); };
        }
        function sendState() {
            if (ws && ws.readyState === WebSocket.OPEN && isInitialized) ws.send(JSON.stringify(state));
        }

        // ---------- vehicle inputs: only send when the USER moves them ----------
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

        // ---------- latching relays ----------
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

        // ---------- momentary buttons ----------
        function buildButtons() {
            BUTTON_DEFS.forEach(b => {
                const tile = el(`
                    <div class="tile pad">
                        <div class="tile-head"><span class="tile-name">${b.label}</span></div>
                        <button class="pad-btn">HOLD</button>
                    </div>`);
                const pad = tile.querySelector('.pad-btn');
                const set = (pressed) => {
                    pad.classList.toggle('held', pressed);
                    if (isInitialized && state.buttons[b.key] !== pressed) {
                        state.buttons[b.key] = pressed;
                        sendState();
                    }
                };
                pad.addEventListener('pointerdown', (e) => { pad.setPointerCapture(e.pointerId); set(true); });
                ['pointerup', 'pointercancel', 'lostpointercapture'].forEach(ev => pad.addEventListener(ev, () => set(false)));
                $('grid-buttons').appendChild(tile);
            });
        }

        function syncAll() {
            INPUT_DEFS.forEach(f => setInput(f.key, state.inputs[f.key].value));
            RELAY_DEFS.forEach(r => setRelay(r.key, state.relays[r.key]));
        }

        buildInputs();
        buildRelays();
        buildButtons();
        connectWS();
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
# 3. UI STATE + WEBSOCKET
# ==========================================
def normalise_state(s):
    if not isinstance(s, dict):
        s = {}
    inputs = s.get("inputs") if isinstance(s.get("inputs"), dict) else {}
    relays = s.get("relays") if isinstance(s.get("relays"), dict) else {}
    buttons = s.get("buttons") if isinstance(s.get("buttons"), dict) else {}
    out_inputs = {}
    for name, (lo, hi) in FIXED_INPUTS.items():
        try:
            v = float(inputs.get(name, {}).get("value", 0.0))
        except (TypeError, ValueError, AttributeError):
            v = 0.0
        if not math.isfinite(v):
            v = 0.0
        out_inputs[name] = {"value": clamp(v, lo, hi), "min": lo, "max": hi}
    return {
        "inputs":  out_inputs,
        "relays":  {k: relays.get(k, True) is not False for k in UI_RELAYS},      # True = closed
        "buttons": {k: bool(buttons.get(k, False)) for k in DIGITAL_SENSORS},     # True = pressed
    }


UI_STATE = normalise_state({})            # pedals released, all relays closed, buttons released


async def ui_ws_handler(websocket, *args, **kwargs):
    global UI_STATE
    await websocket.send(json.dumps(UI_STATE))
    try:
        async for message in websocket:
            try:
                UI_STATE = normalise_state(json.loads(message))
            except json.JSONDecodeError:
                pass
    except websockets.exceptions.ConnectionClosed:
        pass


# ==========================================
# 4. DATA FROM C++ (UDP)
# ==========================================
ACT = dict(ACT_INITIAL)       # last value of each actuator written by C++
CAN_OUT = {}                  # last /CAN/MCU_OUT payload


def _unwrap(v):
    return v.get("value") if isinstance(v, dict) else v


def _is_num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


class CppUdpListener(asyncio.DatagramProtocol):
    def __init__(self):
        self.first_message = True

    def datagram_received(self, data, addr):
        global CAN_OUT
        try:
            msg = json.loads(data.decode('utf-8'))
        except (json.JSONDecodeError, UnicodeDecodeError):
            return
        if not isinstance(msg, dict):
            return

        acts = msg.get("actuators")
        if isinstance(acts, dict):
            for name in ACTUATORS:
                if name not in acts:
                    continue
                v = _unwrap(acts[name])
                if name == "motor_torque":
                    if _is_num(v):
                        ACT[name] = clamp(float(v), 0.0, MOTOR_TORQUE_MAX)   # the motor cannot give more
                elif isinstance(v, (bool, int, float)):
                    ACT[name] = bool(v)

        can = msg.get("CAN")
        if isinstance(can, dict) and isinstance(can.get("MCU_OUT"), dict):
            CAN_OUT = can["MCU_OUT"]

        if self.first_message:
            print(f"✅ Received first packet from C++: {msg}")
            self.first_message = False


# ==========================================
# 5. MAIN LOOP
# ==========================================
async def main():
    threading.Thread(target=start_http_server, daemon=True).start()
    await websockets.serve(ui_ws_handler, "0.0.0.0", 8001)

    loop = asyncio.get_running_loop()
    await loop.create_datagram_endpoint(lambda: CppUdpListener(), local_addr=UDP_FROM_CPP)
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    vehicle = VehicleModel()
    sdc = ShutdownSystem()
    dt = 1.0 / SIM_HZ

    async with FoxgloveServer("0.0.0.0", 8765, "Vehicle Model") as fg_server:
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
            inp = UI_STATE["inputs"]
            rel = UI_STATE["relays"]
            btn = UI_STATE["buttons"]
            now_ns = time.time_ns()
            act = dict(ACT)                  # exactly what the C++ code last commanded

            # ---------- 1. shutdown circuit (BSPD uses last step's brake pressure / motor power) ----------
            sdc.update(dt,
                       dash=rel["dash"], vcu=act["sdc_vcu_relay"],
                       master_panel=rel["master_panel"], bms=rel["bms"],
                       brake_bar=vehicle.brake_pressure_bar,
                       motor_power_w=vehicle.motor_power_w)

            # ---------- 2. vehicle model ----------
            torque_cmd = act["motor_torque"] if (sdc.closed or not SDC_CUTS_MOTOR) else 0.0
            vehicle.step(torque_cmd,
                         float(inp["steering"]["value"]),
                         float(inp["brake"]["value"]),
                         dt)

            # ---------- 3. sensors (real value -> ADC raw + white noise), one message per group ----------
            real = real_sensor_values(vehicle, inp, sdc)
            groups = {"vm": {}, "other": {}}
            for name, cfg in ADC_SENSORS.items():
                groups[cfg["group"]][name] = sample_sensor(cfg, real[name], noise_lsb(name))
            for name, cfg in DIGITAL_SENSORS.items():
                groups[cfg["group"]][name] = 1 if btn[name] else 0

            tree = {}
            for group, payload in groups.items():
                topic = sensor_topic(group)
                await pub(topic, payload, now_ns)
                set_in_tree(tree, topic, payload)

            # ---------- 4. actuators (echo) + CAN to Foxglove (50 Hz) ----------
            can_in = build_can_in(vehicle, sdc)
            if tick % PUBLISH_DIV == 0:
                await pub("/actuators", act, now_ns)
                if CAN_OUT:
                    await pub("/CAN/MCU_OUT", CAN_OUT, now_ns)
                if can_in:
                    await pub("/CAN/MCU_IN", can_in, now_ns)

            # ---------- 5. UDP to C++ (same tree as the topics) ----------
            if can_in:
                set_in_tree(tree, "/CAN/MCU_IN", can_in)
            udp_sock.sendto(json.dumps(tree).encode("utf8"), UDP_TO_CPP)

            # ---------- 6. fixed-rate pacing ----------
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
    if len(sys.argv) >= 3 and sys.argv[1] == "--export-signals":
        with open(sys.argv[2], "w") as f:
            json.dump(signals_description(), f, indent=2)
        print(f"Wrote {sys.argv[2]}")
        sys.exit(0)
    try:
        print("Starting Vehicle Model backend...")
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nShutting down.")
