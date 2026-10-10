// ============================================================================
// main.cpp  -  the "MCU": talks to the Vehicle Model (VM) over UDP.
//
//   VM  --(UDP :9000, 200 Hz)-->  this program   sensors (+ CAN MCU_IN)
//   VM  <--(UDP :9001)--------    this program   actuators (+ CAN MCU_OUT)
//
// Every packet received is one 5 ms "tick": recruits.hpp::loop() is called ONCE PER PACKET, in order,
// so nothing is skipped (the wheel-speed square wave needs every sample).
// After the packets that were waiting have been processed, the actuators are sent back.
//
// Build:  g++ -std=c++17 -O2 main.cpp -o mcu          (needs nlohmann-json: libnlohmann-json3-dev)
// Run:    start vm_server.py (or vm_emulator.py), then ./mcu
// ============================================================================
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <nlohmann/json.hpp>

#include "vm_io.hpp"
#include "recruits.hpp"

int main() {
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) { perror("socket"); return 1; }
    int one = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    // Listen for sensor packets from the VM (port 9000)
    sockaddr_in listen_addr{};
    listen_addr.sin_family = AF_INET;
    listen_addr.sin_port = htons(9000);
    listen_addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(sockfd, (const sockaddr*)&listen_addr, sizeof(listen_addr)) < 0) { perror("bind"); return 1; }

    // Send actuators / CAN to the VM (port 9001)
    sockaddr_in vm_addr{};
    vm_addr.sin_family = AF_INET;
    vm_addr.sin_port = htons(9001);
    vm_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);   // 127.0.0.1

    std::cout << "MCU running. Listening on 9000, sending to 9001..." << std::endl;

    setup();                      // recruits.hpp

    Actuators act;                // actuator values: they keep their last value until you change them
    json can_out = json::object();// CAN MCU_OUT: whatever you write here is sent to /CAN/MCU_OUT

    std::vector<char> buf(65536);
    auto last_print = std::chrono::steady_clock::now();
    bool waiting_printed = false;
    unsigned long packets = 0;

    while (true) {
        // Sleep until a packet arrives (max 200 ms so we notice if the VM is not running)
        pollfd pfd{sockfd, POLLIN, 0};
        int r = poll(&pfd, 1, 200);
        if (r < 0) {
            if (errno == EINTR) continue;
            perror("poll");
            break;
        }
        if (r == 0) {
            if (!waiting_printed) { std::cout << "Waiting for the VM on UDP 9000..." << std::endl; waiting_printed = true; }
            continue;
        }
        waiting_printed = false;

        // Read EVERY waiting packet, oldest first, and run loop() for each one
        bool processed_any = false;
        while (true) {
            ssize_t n = recvfrom(sockfd, buf.data(), buf.size(), MSG_DONTWAIT, nullptr, nullptr);
            if (n < 0) break;                       // queue empty

            try {
                json packet = json::parse(buf.begin(), buf.begin() + n);
                Sensors sensors = parse_sensors(packet);

                // CAN MCU_IN: only present in the packet if something is being sent on it
                json can_in = json::object();
                if (packet.contains("CAN") && packet["CAN"].contains("MCU_IN"))
                    can_in = packet["CAN"]["MCU_IN"];

                loop(sensors, can_in, act, can_out);   // <<< your code (recruits.hpp)
                processed_any = true;
                ++packets;
            } catch (const std::exception& e) {
                std::cerr << "Packet error: " << e.what() << '\n';
            }
        }

        // Send the actuators (and CAN) back to the VM
        if (processed_any) {
            std::string payload = build_output(act, can_out).dump();
            sendto(sockfd, payload.c_str(), payload.size(), 0,
                   (const sockaddr*)&vm_addr, sizeof(vm_addr));
        }

        // Status line once per second
        auto now = std::chrono::steady_clock::now();
        if (now - last_print > std::chrono::seconds(1)) {
            last_print = now;
            std::cout << "packets=" << packets
                      << "  torque=" << act.motor_torque << " Nm"
                      << "  TS=" << act.TS_activated_led
                      << "  R2D=" << act.R2D_LED
                      << "  relay=" << act.sdc_vcu_relay
                      << "  brake_light=" << act.brake_light << std::endl;
        }
    }

    close(sockfd);
    return 0;
}
