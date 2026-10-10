#pragma once
// ============================================================================
// recruits.hpp  -  the MCU logic. setup() runs once, loop() runs once per VM packet (every 5 ms).
//
// What this example does (every actuator is used):
//   sdc_vcu_relay     closed while the pedals are healthy; opened on a pedal fault
//   TS_activated_led  ON after pressing ATS (with the shutdown circuit closed)
//   R2D_LED           ON after pressing R2D while holding the brake (TS must be on)
//   brake_light       ON while the brake pressure is above a threshold (with hysteresis)
//   motor_torque      accelerator pedal -> torque, or CRUISE CONTROL (PI on the wheel-speed sensors)
//
// How to drive in the VM web page:
//   1. hold ATS                       -> TS LED on
//   2. brake > ~10 %, press R2D       -> R2D LED on (ready to drive)
//   3. accelerator up                 -> the car moves
//   4. press R2D again while moving   -> cruise control ON at the current speed
//      (accelerator more than cruise  -> you override it,  brake or press R2D again -> cruise OFF)
//   Open the Master panel / BMS / Dash relay in the UI: the TS drops after the debounce time.
//   Press accelerator > 25 % and brake together: pedal fault, torque 0, VCU relay opens.
// ============================================================================
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <iostream>
#include <string>
#include <nlohmann/json.hpp>

#include "vm_io.hpp"

using json = nlohmann::json;

// ============================================================================
// Constants you can tune
// ============================================================================
constexpr double PI = 3.14159265358979323846;

// ---- wheel speed sensor (rectangle wave, 4 pulses per wheel revolution) ----
constexpr double WHEEL_RADIUS_M     = 0.25;
constexpr int    WSS_TEETH          = 4;
constexpr double WSS_PULSE_DIST_M   = 2.0 * PI * WHEEL_RADIUS_M / WSS_TEETH;   // metres per pulse
constexpr int    WSS_HIGH_RAW       = 40000;    // Schmitt trigger: rising above this = pulse high
constexpr int    WSS_LOW_RAW        = 25000;    //                  falling below this = pulse low
constexpr size_t WSS_EDGES_KEPT     = 4;        // speed = average over the last 3 pulse periods
constexpr double WSS_TIMEOUT_S      = 0.5;      // no pulse for this long -> speed 0 (about 0.8 m/s)
constexpr double SPEED_FILTER_TAU_S = 0.15;     // low-pass on the vehicle speed

// ---- driver / pedals ----
constexpr double TORQUE_REQUEST_MAX = 600.0;    // Nm, accelerator 100 % (the VM clamps to 0..600 anyway)
constexpr double BRAKE_ON_BAR       = 5.0;      // "brake pressed": R2D, cruise cancel, plausibility
constexpr double BRAKE_LIGHT_ON_BAR = 3.0;      // brake light turns on above this ...
constexpr double BRAKE_LIGHT_OFF_BAR= 1.5;      // ... and off below this (hysteresis)
constexpr double THROTTLE_DEADZONE_PCT = 1.0;   // below this the accelerator counts as released (ADC noise)
constexpr double APPS_IMPLAUS_PCT   = 25.0;     // accelerator above this together with the brake = fault
constexpr double APPS_RELEASE_PCT   = 5.0;      // fault is released when the accelerator is below this
constexpr int    PEDAL_RAW_MIN      = ADC_MAX * 5 / 100;    // raw outside 5 %..95 % of the ADC range
constexpr int    PEDAL_RAW_MAX      = ADC_MAX * 95 / 100;   //   = wiring fault (short / open circuit)

// ---- shutdown circuit measurements ----
// The VM randomly makes a channel read 0 V for about 0.5 s (bad connection), so a low reading only
// counts as "open" after it has been low for SDC_DEBOUNCE_S. A real open lasts much longer.
constexpr int    SDC_HIGH_RAW       = ADC_MAX / 2;          // above = ~12 V = closed
constexpr double SDC_DEBOUNCE_S     = 1.0;

// ---- cruise control ----
constexpr double CRUISE_MIN_SPEED_MPS = 3.0;    // can only be engaged above this speed
constexpr double CRUISE_KP = 60.0;              // Nm per (m/s) of speed error
constexpr double CRUISE_KI = 15.0;              // Nm per (m/s) per second

// ============================================================================
// Helpers
// ============================================================================

// Measures the speed of ONE wheel from its rectangle wave.
//   update() is called once per VM packet with the raw ADC value.
//   Rising edges are timestamped; speed = distance per pulse / time between pulses.
class WheelSpeedSensor {
public:
    void update(int raw) {
        t_ += VM_SAMPLE_PERIOD_S;
        // Schmitt trigger (two thresholds) so that noise cannot create false edges
        if (!high_ && raw > WSS_HIGH_RAW) {
            high_ = true;
            if (!edges_.empty() && t_ - edges_.back() > WSS_TIMEOUT_S) edges_.clear();   // restart from standstill
            edges_.push_back(t_);
            if (edges_.size() > WSS_EDGES_KEPT) edges_.pop_front();
        } else if (high_ && raw < WSS_LOW_RAW) {
            high_ = false;
        }
    }
    double speed_mps() const {
        if (edges_.size() < 2) return 0.0;
        double since_last = t_ - edges_.back();
        if (since_last > WSS_TIMEOUT_S) return 0.0;
        double period = (edges_.back() - edges_.front()) / double(edges_.size() - 1);
        period = std::max(period, since_last);          // when pulses stop, the speed falls smoothly
        return WSS_PULSE_DIST_M / period;
    }
private:
    double t_ = 0.0;
    bool high_ = false;
    std::deque<double> edges_;
};

// PI controller with output limits and anti-windup
struct PiController {
    double kp, ki, out_min, out_max;
    double integral = 0.0;
    void reset(double initial_output) { integral = std::clamp(initial_output, out_min, out_max); }
    double update(double error, double dt) {
        double u_raw = kp * error + integral;
        double u = std::clamp(u_raw, out_min, out_max);
        // only keep integrating if that does not push further into saturation
        if (u_raw == u || (u_raw > out_max && error < 0) || (u_raw < out_min && error > 0))
            integral += ki * error * dt;
        return u;
    }
};

// true for ONE sample when the signal goes from 0 to 1 (button press)
struct RisingEdge {
    bool prev = false;
    bool update(bool now) { bool r = now && !prev; prev = now; return r; }
};

// Confirms a low signal only after it has been low for a while
struct LowDebounce {
    static constexpr int N = int(SDC_DEBOUNCE_S / VM_SAMPLE_PERIOD_S);
    int low_count = 0;
    bool update(bool low) {                    // returns true = confirmed low
        low_count = low ? std::min(low_count + 1, N) : 0;
        return low_count >= N;
    }
};

enum class State { TS_OFF, TS_ON, READY_TO_DRIVE };
inline const char* state_name(State s) {
    switch (s) {
        case State::TS_OFF:         return "TS_OFF";
        case State::TS_ON:          return "TS_ON";
        case State::READY_TO_DRIVE: return "READY_TO_DRIVE";
    }
    return "?";
}

// ============================================================================
// Normal Variable Declarations (the MCU state)
// ============================================================================
inline std::array<WheelSpeedSensor, 4> wss;
inline double speed_mps = 0.0;                          // filtered vehicle speed from the 4 wheel sensors

inline std::array<LowDebounce, 5> sdc_low;              // dash, vcu, master_panel, bms, bspd
inline const std::array<const char*, 5> SDC_NAMES = {"dash", "vcu", "master_panel", "bms", "bspd"};

inline State state = State::TS_OFF;
inline RisingEdge ats_edge, r2d_edge;
inline bool pedal_fault = false;
inline bool brake_light_on = false;

inline bool cruise_active = false;
inline double cruise_set_speed = 0.0;
inline PiController cruise_pi{CRUISE_KP, CRUISE_KI, 0.0, TORQUE_REQUEST_MAX};

inline void setup() {
    // ==========================================
    // SETUP CODE
    // ==========================================
    std::cout << "Setup complete." << std::endl;
}

// ============================================================================
// loop(): called once per VM packet (every 5 ms)
//   s         all the sensors (raw ADC counts, buttons 0/1)
//   can_in    data received on /CAN/MCU_IN  (empty object if nothing is sent)
//   act       the actuators: write the values you want. They keep their value until you change them.
//   can_out   data to publish on /CAN/MCU_OUT
// ============================================================================
inline void loop(const Sensors& s, const json& can_in, Actuators& act, json& can_out) {
    const double dt = VM_SAMPLE_PERIOD_S;

    // ------------------------------------------------------------------
    // 1. ADC raw -> physical values
    // ------------------------------------------------------------------
    const double throttle_pct = std::clamp(accelerator_pct(s.accelerator_position), 0.0, 100.0);
    const double brake_bar    = brake_pressure_bar(s.brake_pressure);

    // ------------------------------------------------------------------
    // 2. Vehicle speed from the wheel speed sensors (WSS)
    // ------------------------------------------------------------------
    double sum = 0.0;
    for (int i = 0; i < 4; ++i) {
        wss[i].update(s.wheel_speed[i]);
        sum += wss[i].speed_mps();
    }
    const double raw_speed = sum / 4.0;
    speed_mps += dt / (SPEED_FILTER_TAU_S + dt) * (raw_speed - speed_mps);

    // ------------------------------------------------------------------
    // 3. Shutdown circuit: which element opened?
    //    The line is dash -> vcu -> master_panel -> bms -> bspd. After the first open element
    //    everything reads 0 V, so the FIRST channel that is (confirmed) low is the one that opened.
    // ------------------------------------------------------------------
    const std::array<int, 5> sdc_raw = {s.sdc_after_dash, s.sdc_after_vcu, s.sdc_after_master_panel,
                                        s.sdc_after_bms, s.sdc_after_bspd};
    int first_open = -1;
    for (int i = 0; i < 5; ++i) {
        bool open = sdc_low[i].update(sdc_raw[i] < SDC_HIGH_RAW);
        if (open && first_open < 0) first_open = i;
    }
    const bool sdc_ok = (first_open < 0);

    // ------------------------------------------------------------------
    // 4. Pedal checks (this is what drives the sdc_vcu_relay actuator)
    // ------------------------------------------------------------------
    const bool range_fault = s.accelerator_position < PEDAL_RAW_MIN || s.accelerator_position > PEDAL_RAW_MAX ||
                             s.brake_pedal_position < PEDAL_RAW_MIN || s.brake_pedal_position > PEDAL_RAW_MAX;
    const bool implausible = throttle_pct > APPS_IMPLAUS_PCT && brake_bar > BRAKE_ON_BAR;
    if (implausible) pedal_fault = true;                                         // latched ...
    else if (pedal_fault && throttle_pct < APPS_RELEASE_PCT) pedal_fault = false;// ... until the accelerator is released
    const bool fault = pedal_fault || range_fault;

    // ------------------------------------------------------------------
    // 5. Drive sequence: TS_OFF -> TS_ON -> READY_TO_DRIVE
    // ------------------------------------------------------------------
    const bool ats_press = ats_edge.update(s.ATS_btn);
    const bool r2d_press = r2d_edge.update(s.R2D_btn);
    const State prev_state = state;

    if (!sdc_ok || fault) {
        state = State::TS_OFF;                                   // shutdown circuit open or fault: TS off
    } else if (state == State::TS_OFF) {
        if (ats_press) state = State::TS_ON;
    } else if (state == State::TS_ON) {
        if (r2d_press && brake_bar > BRAKE_ON_BAR) state = State::READY_TO_DRIVE;   // R2D needs the brake
    }

    // ------------------------------------------------------------------
    // 6. Cruise control
    // ------------------------------------------------------------------
    // Optional: a set speed [m/s] sent on CAN /CAN/MCU_IN (key "cruise_set_speed").
    // The VM leaves MCU_IN empty; in vm_emulator.py add a slider with topic /CAN/MCU_IN and key cruise_set_speed.
    const bool can_has_set_speed = can_in.contains("cruise_set_speed") && can_in["cruise_set_speed"].is_number();
    if (can_has_set_speed) cruise_set_speed = can_in["cruise_set_speed"].get<double>();

    if (state != State::READY_TO_DRIVE || fault || brake_bar > BRAKE_ON_BAR) {
        cruise_active = false;                                   // brake, fault or TS off cancels the cruise
    } else if (r2d_press && prev_state == State::READY_TO_DRIVE) {   // R2D pressed again = toggle
        if (cruise_active) {
            cruise_active = false;
        } else if (speed_mps > CRUISE_MIN_SPEED_MPS) {
            cruise_active = true;
            if (!can_has_set_speed) cruise_set_speed = speed_mps;    // hold the speed we have now
            cruise_pi.reset(act.motor_torque);                       // start from the current torque: no jerk
        }
    }

    // ------------------------------------------------------------------
    // 7. Motor torque request
    // ------------------------------------------------------------------
    double torque = 0.0;
    if (state == State::READY_TO_DRIVE && !fault) {
        if (throttle_pct > THROTTLE_DEADZONE_PCT)
            torque = throttle_pct / 100.0 * TORQUE_REQUEST_MAX;          // driver
        if (cruise_active) {
            double cruise_torque = cruise_pi.update(cruise_set_speed - speed_mps, dt);
            torque = std::max(torque, cruise_torque);                    // more accelerator = override cruise
        }
    }

    // ------------------------------------------------------------------
    // 8. Write the actuators
    // ------------------------------------------------------------------
    act.motor_torque     = std::clamp(torque, 0.0, TORQUE_REQUEST_MAX);
    act.TS_activated_led = (state != State::TS_OFF);
    act.R2D_LED          = (state == State::READY_TO_DRIVE);
    act.sdc_vcu_relay    = !fault;                                   // true = closed

    if (brake_bar > BRAKE_LIGHT_ON_BAR) brake_light_on = true;
    else if (brake_bar < BRAKE_LIGHT_OFF_BAR) brake_light_on = false;
    act.brake_light = brake_light_on;

    // ==========================================
    // PUBLISH DATA FROM C++ BACK TO PYTHON (CAN)
    // ==========================================
    // can_out is a normal JSON object. Anything you put in it appears in Foxglove on /CAN/MCU_OUT
    // (numbers, booleans, strings and nested objects all work). Keys you add later show up automatically.
    can_out["state"]              = state_name(state);
    can_out["speed_mps"]          = speed_mps;                       // from the wheel speed sensors
    can_out["throttle_pct"]       = throttle_pct;
    can_out["brake_bar"]          = brake_bar;
    can_out["steering_deg"]       = steering_deg(s.steering_angle);
    can_out["accel_x_mps2"]       = accel_mps2(s.accel_x);
    can_out["accel_y_mps2"]       = accel_mps2(s.accel_y);
    can_out["sdc_ok"]             = sdc_ok;
    can_out["sdc_first_open"]     = sdc_ok ? "none" : SDC_NAMES[first_open];
    can_out["pedal_fault"]        = fault;
    can_out["cruise"]["active"]   = cruise_active;                   // nested object: cruise.active
    can_out["cruise"]["set_mps"]  = cruise_set_speed;
}
