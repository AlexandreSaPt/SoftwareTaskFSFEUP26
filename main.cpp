#include "mcu_signals.pb.h"
#include "mcap_playback.hpp"

int main() {
    McuAdcData my_vehicle_data;
    McapPlayback playback;

    if (!playback.open("test_run.mcap")) {
        return 1;
    }

    std::cout << "Starting simulation loop...\n";

    // This loop runs continuously until the file runs out of messages
    while (playback.get_next_msg(my_vehicle_data)) {
        
        // At this exact moment, only ONE signal was updated. 
        // We can use the 'is_old' flags to figure out which one it was.

        if (!my_vehicle_data.get_brake_pressure_is_old()) {
            // The get() function reads it, and instantly flips 'is_old' back to true
            std::cout << "[NEW] Brake Pressure ADC: " 
                      << my_vehicle_data.get_brake_pressure() << "\n";
        }
        
        if (!my_vehicle_data.get_asms_is_old()) {
            std::cout << "[NEW] ASMS ADC: " 
                      << my_vehicle_data.get_asms() << "\n";
        }

        // You can run your state machine logic here!
        // check_state_machine(my_vehicle_data);
    }

    std::cout << "End of MCAP file reached.\n";
    return 0;
}