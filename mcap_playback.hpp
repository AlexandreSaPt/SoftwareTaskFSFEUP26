#include <iostream>
#include <string>
#include <memory>
#include <mcap/reader.hpp>
#include "mcu_signals.pb.h" 
#include "McuAdcData.hpp"
// (Assume McuAdcData from the previous step is included here)

class McapPlayback {
private:
    mcap::McapReader reader;
    
    // We use a pointer to safely store the view after opening the file
    std::unique_ptr<mcap::MessageView> message_view;
    
    // These are our "bookmarks" to track our position in the file
    mcap::MessageView::Iterator current_msg;
    mcap::MessageView::Iterator end_msg;

public:
    // 1. Initialize the file and set up our bookmarks
    bool open(const std::string& filepath) {
        auto status = reader.open(filepath);
        if (!status.ok()) {
            std::cerr << "Failed to open MCAP file.\n";
            return false;
        }

        // Generate the view and set our start and end points
        message_view = std::make_unique<mcap::MessageView>(reader.readMessages());
        current_msg = message_view->begin();
        end_msg = message_view->end();
        
        return true;
    }

    // 2. The Step Function (Reads ONE relevant message and pauses)
    // Returns true if data was updated. Returns false if we hit the end of the file.
    bool get_next_msg(McuAdcData& sensor_data) {
        
        // We use a while loop just in case we encounter a message topic
        // that we don't care about. It will skip it and read the next one.
        while (current_msg != end_msg) {
            
            // Grab the topic and raw bytes from our current bookmark
            const std::string& topic = (*current_msg).channel->topic;
            auto msg_data = (*current_msg).message.data;
            auto msg_size = (*current_msg).message.dataSize;

            // ADVANCE THE BOOKMARK for the next time this function is called!
            ++current_msg;

            // Unpack the Protobuf bytes
            mcu::RawAdcSensor sensor_msg;
            if (!sensor_msg.ParseFromArray(msg_data, msg_size)) {
                continue; // Failed to parse, try the next message
            }

            uint16_t raw_val = static_cast<uint16_t>(sensor_msg.adc_value());
            bool topic_matched = true;

            // Route the data using your if statements
            if (topic == "rawMCU/brake_pressure")      { sensor_data.set_brake_pressure(raw_val); } 
            else if (topic == "rawMCU/asms")           { sensor_data.set_asms(raw_val); }
            else if (topic == "rawMCU/ebs_sensor_1")   { sensor_data.set_ebs_sensor_1(raw_val); }
            else if (topic == "rawMCU/ebs_sensor_2")   { sensor_data.set_ebs_sensor_2(raw_val); }
            else if (topic == "rawMCU/ats")            { sensor_data.set_ats(raw_val); }
            else if (topic == "rawMCU/SDC/dash")       { sensor_data.set_sdc_dash(raw_val); }
            else if (topic == "rawMCU/SDC/vcu")        { sensor_data.set_sdc_vcu(raw_val); }
            else if (topic == "rawMCU/SDC/master_panel"){ sensor_data.set_sdc_master_panel(raw_val); }
            else if (topic == "rawMCU/SDC/bms")        { sensor_data.set_sdc_bms(raw_val); }
            else if (topic == "rawMCU/SDC/bspd")       { sensor_data.set_sdc_bspd(raw_val); }
            else {
                topic_matched = false; // It was a topic we don't care about
            }

            // If we successfully updated our object, PAUSE the function and return to main!
            if (topic_matched) {
                return true;
            }
        }

        // If we break out of the while loop, we reached the end of the file
        return false;
    }
};