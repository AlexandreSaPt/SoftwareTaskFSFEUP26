#include <cstdint>

class McuAdcData {
private:
    // Raw uint16_t measurements
    uint16_t brake_pressure = 0;
    uint16_t asms = 0;
    uint16_t ebs_sensor_1 = 0;
    uint16_t ebs_sensor_2 = 0;
    uint16_t ats = 0;
    uint16_t sdc_dash = 0;
    uint16_t sdc_vcu = 0;
    uint16_t sdc_master_panel = 0;
    uint16_t sdc_bms = 0;
    uint16_t sdc_bspd = 0;

    // is_old boolean flags (initialized to true, meaning no fresh data yet)
    bool brake_pressure_is_old = true;
    bool asms_is_old = true;
    bool ebs_sensor_1_is_old = true;
    bool ebs_sensor_2_is_old = true;
    bool ats_is_old = true;
    bool sdc_dash_is_old = true;
    bool sdc_vcu_is_old = true;
    bool sdc_master_panel_is_old = true;
    bool sdc_bms_is_old = true;
    bool sdc_bspd_is_old = true;

public:
    // ==========================================
    // SETTERS (Populates data & marks as fresh)
    // ==========================================
    void set_brake_pressure(uint16_t val) { brake_pressure = val; brake_pressure_is_old = false; }
    void set_asms(uint16_t val)           { asms = val; asms_is_old = false; }
    void set_ebs_sensor_1(uint16_t val)   { ebs_sensor_1 = val; ebs_sensor_1_is_old = false; }
    void set_ebs_sensor_2(uint16_t val)   { ebs_sensor_2 = val; ebs_sensor_2_is_old = false; }
    void set_ats(uint16_t val)            { ats = val; ats_is_old = false; }
    void set_sdc_dash(uint16_t val)       { sdc_dash = val; sdc_dash_is_old = false; }
    void set_sdc_vcu(uint16_t val)        { sdc_vcu = val; sdc_vcu_is_old = false; }
    void set_sdc_master_panel(uint16_t val){ sdc_master_panel = val; sdc_master_panel_is_old = false; }
    void set_sdc_bms(uint16_t val)        { sdc_bms = val; sdc_bms_is_old = false; }
    void set_sdc_bspd(uint16_t val)       { sdc_bspd = val; sdc_bspd_is_old = false; }

    // ==========================================
    // GETTERS (Returns data & marks as old)
    // ==========================================
    uint16_t get_brake_pressure() { brake_pressure_is_old = true; return brake_pressure; }
    uint16_t get_asms()           { asms_is_old = true; return asms; }
    uint16_t get_ebs_sensor_1()   { ebs_sensor_1_is_old = true; return ebs_sensor_1; }
    uint16_t get_ebs_sensor_2()   { ebs_sensor_2_is_old = true; return ebs_sensor_2; }
    uint16_t get_ats()            { ats_is_old = true; return ats; }
    uint16_t get_sdc_dash()       { sdc_dash_is_old = true; return sdc_dash; }
    uint16_t get_sdc_vcu()        { sdc_vcu_is_old = true; return sdc_vcu; }
    uint16_t get_sdc_master_panel(){ sdc_master_panel_is_old = true; return sdc_master_panel; }
    uint16_t get_sdc_bms()        { sdc_bms_is_old = true; return sdc_bms; }
    uint16_t get_sdc_bspd()       { sdc_bspd_is_old = true; return sdc_bspd; }

    // ==========================================
    // IS_OLD GETTERS (Just checks the flag without altering it)
    // ==========================================
    bool get_brake_pressure_is_old() const { return brake_pressure_is_old; }
    bool get_asms_is_old() const           { return asms_is_old; }
    bool get_ebs_sensor_1_is_old() const   { return ebs_sensor_1_is_old; }
    bool get_ebs_sensor_2_is_old() const   { return ebs_sensor_2_is_old; }
    bool get_ats_is_old() const            { return ats_is_old; }
    bool get_sdc_dash_is_old() const       { return sdc_dash_is_old; }
    bool get_sdc_vcu_is_old() const        { return sdc_vcu_is_old; }
    bool get_sdc_master_panel_is_old() const { return sdc_master_panel_is_old; }
    bool get_sdc_bms_is_old() const        { return sdc_bms_is_old; }
    bool get_sdc_bspd_is_old() const       { return sdc_bspd_is_old; }
};