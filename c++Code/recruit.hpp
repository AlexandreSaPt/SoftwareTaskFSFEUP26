#pragma once

#include <iostream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

// Normal Variable Declarations


inline void setup() {
    // ==========================================
    // SETUP CODE
    // ==========================================
    std::cout << "Setup complete." << std::endl;
}

inline void loop(json data, json& data_out) {
    // ==========================================
    // EXECUTE CODE BASED ON BUTTONS
    // ==========================================
    // Let's assume you created a button named "Fire_Laser" in your Web UI
    if (data["buttons"].contains("Fire_Laser")) {
        bool is_pressed = data["buttons"]["Fire_Laser"];
        
        // Trigger event ONLY on the initial press (Rising Edge)
        if (is_pressed) {
            std::cout << "Being pressed" << std::endl;
            // TODO: Call your C++ hardware execution code here
        }
    }

    // ==========================================
    // EXECUTE CODE BASED ON SLIDERS
    // ==========================================
    // Let's assume you created a slider named "Throttle"
    if (data["sliders"].contains("Throttle")) {
        double throttle_val = data["sliders"]["Throttle"];
        
        std::cout << "Engine Throttle set to: " << throttle_val << std::endl;
        
    }


    // ==========================================
    // PUBLISH DATA FROM C++ BACK TO PYTHON
    // ==========================================
    
    // Create a JSON object with your C++ variables
    data_out["rpm"] = 1;
    data_out["temperature_c"] = 85.5 + (2 / 1000.0);
    data_out["system_status"] = 1; // 1 = OK
}
