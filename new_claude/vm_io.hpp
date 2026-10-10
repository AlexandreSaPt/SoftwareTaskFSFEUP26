#pragma once
// ============================================================================
// vm_io.hpp  -  everything that talks to the Vehicle Model (VM).
//   * Sensors / Actuators structs
//   * ADC raw  ->  physical value conversions (inverse of the VM sensor table)
//   * JSON packet  ->  Sensors,   Actuators + CAN  ->  JSON packet
// You normally do NOT need to edit this file. Write your code in recruits.hpp.
//
// Packet from the VM (UDP 9000, 200 Hz), same tree as the Foxglove topics:
//   {"sensors": {"vm":    {"wheel_speed": {"fl":..,"fr":..,"rl":..,"rr":..}, "accelerator_position":..,
//                          "brake_pedal_position":.., "steering_angle":.., "brake_pressure":..,
//                          "accel_x":.., "accel_y":..},
//                 "other": {"ATS_btn":0/1, "R2D_btn":0/1, "sdc_after_dash":.., "sdc_after_vcu":..,
//                           "sdc_after_master_panel":.., "sdc_after_bms":.., "sdc_after_bspd":..}},
//    "CAN": {"MCU_IN": {...}}}                       <- "CAN" only exists if something is on MCU_IN
// Packet to the VM (UDP 9001):
//   {"actuators": {"motor_torque":.., "TS_activated_led":.., "R2D_LED":.., "sdc_vcu_relay":.., "brake_light":..},
//    "CAN": {"MCU_OUT": {...}}}
// ============================================================================
#include <array>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

// ---------------------------------------------------------------------------
// ADC (16 bit, 3.3 V) and sampling
// ---------------------------------------------------------------------------
constexpr int    ADC_MAX = 65535;
constexpr double ADC_VREF = 3.3;                       // V
constexpr double VM_SAMPLE_PERIOD_S = 1.0 / 200.0;     // the VM sends one packet every 5 ms (SIM_HZ = 200)

// ---------------------------------------------------------------------------
// ADC raw -> physical value. These ranges must match the VM sensor table.
//   volts = raw * 3.3 / 65535 ;  real = linear map of volts over the sensor's voltage range
// Noise can push a value slightly outside the nominal range (they are NOT clamped here).
// ---------------------------------------------------------------------------
inline double lin_map(double x, double x0, double x1, double y0, double y1) {
    return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
}
inline double adc_volts(int raw) { return raw * ADC_VREF / ADC_MAX; }

constexpr double SENSOR_V_MIN = 0.33;                  // sensor output at the low end of its range [V]
constexpr double SENSOR_V_MAX = 2.97;                  // sensor output at the high end of its range [V]

inline double accelerator_pct(int raw)     { return lin_map(adc_volts(raw), SENSOR_V_MIN, SENSOR_V_MAX, 0.0, 100.0); }   // %
inline double brake_pedal_pct(int raw)     { return lin_map(adc_volts(raw), SENSOR_V_MIN, SENSOR_V_MAX, 0.0, 100.0); }   // %
inline double brake_pressure_bar(int raw)  { return lin_map(adc_volts(raw), SENSOR_V_MIN, SENSOR_V_MAX, 0.0, 80.0); }    // bar
inline double steering_deg(int raw)        { return lin_map(adc_volts(raw), SENSOR_V_MIN, SENSOR_V_MAX, -20.05, 20.05); } // deg, + = left
inline double accel_mps2(int raw)          { return lin_map(adc_volts(raw), SENSOR_V_MIN, SENSOR_V_MAX, -39.24, 39.24); } // m/s^2 (+/-4 g)
inline double sdc_volts(int raw)           { return adc_volts(raw) / ADC_VREF * 24.0; }                                  // V (VBAT = 24 V)

// ---------------------------------------------------------------------------
// Sensors: raw ADC counts (0..65535), buttons are 0/1
// ---------------------------------------------------------------------------
enum Wheel { FL = 0, FR = 1, RL = 2, RR = 3 };

struct Sensors {
    // /sensors/vm
    std::array<int, 4> wheel_speed{};     // raw value of the rectangle wave, index with FL/FR/RL/RR
    int accelerator_position = 0;
    int brake_pedal_position = 0;
    int steering_angle = 0;
    int brake_pressure = 0;
    int accel_x = 0;
    int accel_y = 0;
    // /sensors/other
    int sdc_after_dash = 0;
    int sdc_after_vcu = 0;
    int sdc_after_master_panel = 0;
    int sdc_after_bms = 0;
    int sdc_after_bspd = 0;
    bool ATS_btn = false;
    bool R2D_btn = false;
};

// Missing fields keep their default value (0), so a partial packet never crashes the code.
inline Sensors parse_sensors(const json& j) {
    using P = json::json_pointer;
    auto geti = [&](const char* path) { return j.value(P(path), 0); };

    Sensors s;
    s.wheel_speed[FL] = geti("/sensors/vm/wheel_speed/fl");
    s.wheel_speed[FR] = geti("/sensors/vm/wheel_speed/fr");
    s.wheel_speed[RL] = geti("/sensors/vm/wheel_speed/rl");
    s.wheel_speed[RR] = geti("/sensors/vm/wheel_speed/rr");
    s.accelerator_position = geti("/sensors/vm/accelerator_position");
    s.brake_pedal_position = geti("/sensors/vm/brake_pedal_position");
    s.steering_angle       = geti("/sensors/vm/steering_angle");
    s.brake_pressure       = geti("/sensors/vm/brake_pressure");
    s.accel_x              = geti("/sensors/vm/accel_x");
    s.accel_y              = geti("/sensors/vm/accel_y");

    s.sdc_after_dash         = geti("/sensors/other/sdc_after_dash");
    s.sdc_after_vcu          = geti("/sensors/other/sdc_after_vcu");
    s.sdc_after_master_panel = geti("/sensors/other/sdc_after_master_panel");
    s.sdc_after_bms          = geti("/sensors/other/sdc_after_bms");
    s.sdc_after_bspd         = geti("/sensors/other/sdc_after_bspd");
    s.ATS_btn = geti("/sensors/other/ATS_btn") != 0;
    s.R2D_btn = geti("/sensors/other/R2D_btn") != 0;
    return s;
}

// ---------------------------------------------------------------------------
// Actuators: what the C++ code (the MCU) commands
// ---------------------------------------------------------------------------
struct Actuators {
    double motor_torque = 0.0;       // Nm at the wheels. The VM clamps it to 0..600 Nm
    bool TS_activated_led = false;
    bool R2D_LED = false;
    bool sdc_vcu_relay = false;      // true = relay closed (VCU element of the shutdown line closed)
    bool brake_light = false;
};

// Actuators + CAN MCU_OUT -> packet for the VM. An empty can_out is not sent.
inline json build_output(const Actuators& a, const json& can_out) {
    json out;
    out["actuators"] = {
        {"motor_torque",     a.motor_torque},
        {"TS_activated_led", a.TS_activated_led},
        {"R2D_LED",          a.R2D_LED},
        {"sdc_vcu_relay",    a.sdc_vcu_relay},
        {"brake_light",      a.brake_light},
    };
    if (!can_out.empty()) out["CAN"]["MCU_OUT"] = can_out;
    return out;
}
