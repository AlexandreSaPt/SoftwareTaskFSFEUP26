#include <iostream>
#include <string>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <unistd.h>
#include <nlohmann/json.hpp>

#include "recruit.hpp"

using json = nlohmann::json;

int main() {
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    
    // Socket to listen for incoming data from Python (Port 9000)
    sockaddr_in servaddr{};
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(9000);
    servaddr.sin_addr.s_addr = INADDR_ANY;
    bind(sockfd, (const sockaddr *)&servaddr, sizeof(servaddr));

    // Socket to send data back to Python (Port 9001)
    sockaddr_in python_addr{};
    python_addr.sin_family = AF_INET;
    python_addr.sin_port = htons(9001);
    python_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK); // 127.0.0.1

    std::cout << "C++ Engine running. Listening on 9000, sending to 9001..." << std::endl;

    setup(); // Call the setup function from recruit.hpp

    while (true) {
        int bytes_waiting;
        ioctl(sockfd, FIONREAD, &bytes_waiting);
        
        json data_out = json::object(); // Prepare an empty JSON object to send back

        if (bytes_waiting > 0) {
            std::string buffer(bytes_waiting + 1, '\0');
            int n = recvfrom(sockfd, &buffer[0], bytes_waiting, 0, nullptr, nullptr);
            buffer[n] = '\0';
            
            try {
                json data_in = json::parse(buffer);

                loop(data_in, data_out); // Recruits stuff

            } catch (const std::exception& e) {
                std::cerr << "JSON Parse error: " << e.what() << '\n';
            }

            // Convert to string
            std::string payload_str = data_out.dump();
            std::cout << "Sending telemetry: " << payload_str << std::endl;

            // Send over UDP to port 9001
            sendto(sockfd, payload_str.c_str(), payload_str.size(), 0, 
                (const sockaddr*)&python_addr, sizeof(python_addr));

        }
        
        // Run loop at ~100Hz
        usleep(10000); 
    }
    
    close(sockfd);
    return 0;
}