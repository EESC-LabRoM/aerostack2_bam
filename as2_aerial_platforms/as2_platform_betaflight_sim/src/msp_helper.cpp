#include "as2_platform_betaflight_sim/msp_helper.hpp"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <chrono>
#include <thread>

namespace betaflight_sim {

MSPHelper::MSPHelper(const std::string& connection_string, int baud_rate)
    : connection_string_(connection_string), baud_rate_(baud_rate), connected_(false),
      is_udp_connection_(false), serial_fd_(-1), udp_socket_(-1) {
    
    // Check if this is a UDP connection
    if (connection_string_.find("udp:") == 0) {
        is_udp_connection_ = true;
    }
}

MSPHelper::~MSPHelper() {
    disconnect();
}

bool MSPHelper::connect() {
    if (connected_) {
        return true;
    }
    
    if (is_udp_connection_) {
        // Parse UDP connection string: udp:hostname:port
        size_t colon1 = connection_string_.find(':', 4);  // Skip "udp:"
        if (colon1 == std::string::npos) {
            std::cerr << "Invalid UDP connection string: " << connection_string_ << std::endl;
            return false;
        }
        
        std::string hostname = connection_string_.substr(4, colon1 - 4);
        int port = std::stoi(connection_string_.substr(colon1 + 1));
        
        // Create UDP socket
        udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_socket_ < 0) {
            std::cerr << "Failed to create UDP socket" << std::endl;
            return false;
        }
        
        // Setup address
        memset(&udp_addr_, 0, sizeof(udp_addr_));
        udp_addr_.sin_family = AF_INET;
        udp_addr_.sin_port = htons(port);
        
        if (hostname == "localhost" || hostname == "127.0.0.1") {
            udp_addr_.sin_addr.s_addr = inet_addr("127.0.0.1");
        } else {
            udp_addr_.sin_addr.s_addr = inet_addr(hostname.c_str());
        }
        
        std::cout << "Connected to Betaflight SITL via UDP: " << hostname << ":" << port << std::endl;
    } else {
        // Serial connection
        serial_fd_ = open(connection_string_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ < 0) {
            std::cerr << "Failed to open serial port: " << connection_string_ << std::endl;
            return false;
        }
        
        // Configure serial port
        struct termios options;
        tcgetattr(serial_fd_, &options);
        
        // Set baud rate
        switch (baud_rate_) {
            case 9600: cfsetispeed(&options, B9600); cfsetospeed(&options, B9600); break;
            case 19200: cfsetispeed(&options, B19200); cfsetospeed(&options, B19200); break;
            case 38400: cfsetispeed(&options, B38400); cfsetospeed(&options, B38400); break;
            case 57600: cfsetispeed(&options, B57600); cfsetospeed(&options, B57600); break;
            case 115200: cfsetispeed(&options, B115200); cfsetospeed(&options, B115200); break;
            default:
                std::cerr << "Unsupported baud rate: " << baud_rate_ << std::endl;
                close(serial_fd_);
                serial_fd_ = -1;
                return false;
        }
        
        // 8N1, no flow control
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_cflag &= ~CRTSCTS;
        
        // Raw input/output
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_oflag &= ~OPOST;
        
        // Set timeouts
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 10;  // 1 second timeout
        
        // Apply settings
        if (tcsetattr(serial_fd_, TCSANOW, &options) < 0) {
            std::cerr << "Failed to configure serial port" << std::endl;
            close(serial_fd_);
            serial_fd_ = -1;
            return false;
        }
        
        // Flush buffers
        tcflush(serial_fd_, TCIOFLUSH);
        
        std::cout << "Connected to Betaflight via serial: " << connection_string_ 
                  << " at " << baud_rate_ << " baud" << std::endl;
    }
    
    connected_ = true;
    return true;
}

void MSPHelper::disconnect() {
    if (!connected_) {
        return;
    }
    
    if (is_udp_connection_ && udp_socket_ >= 0) {
        close(udp_socket_);
        udp_socket_ = -1;
    } else if (serial_fd_ >= 0) {
        close(serial_fd_);
        serial_fd_ = -1;
    }
    
    connected_ = false;
    std::cout << "Disconnected from Betaflight" << std::endl;
}

bool MSPHelper::isConnected() const {
    return connected_;
}

uint8_t MSPHelper::calculateChecksum(uint8_t command_id, const std::vector<uint8_t>& payload) {
    uint8_t checksum = 0;
    uint8_t length = static_cast<uint8_t>(payload.size());
    
    // Include length and command in checksum
    checksum ^= length;
    checksum ^= command_id;
    
    // Include payload in checksum
    for (uint8_t byte : payload) {
        checksum ^= byte;
    }
    
    return checksum;
}

std::vector<uint8_t> MSPHelper::buildMSPPacket(uint8_t command_id, const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> packet;
    
    // MSP header: $M<
    packet.push_back('$');
    packet.push_back('M');
    packet.push_back('<');
    
    // Length
    packet.push_back(static_cast<uint8_t>(payload.size()));
    
    // Command ID
    packet.push_back(command_id);
    
    // Payload
    packet.insert(packet.end(), payload.begin(), payload.end());
    
    // Checksum
    uint8_t checksum = calculateChecksum(command_id, payload);
    packet.push_back(checksum);
    
    return packet;
}

int MSPHelper::writeData(const std::vector<uint8_t>& data) {
    if (!connected_) {
        return -1;
    }
    
    if (is_udp_connection_) {
        return sendto(udp_socket_, data.data(), data.size(), 0, 
                     (struct sockaddr*)&udp_addr_, sizeof(udp_addr_));
    } else {
        return write(serial_fd_, data.data(), data.size());
    }
}

int MSPHelper::readData(std::vector<uint8_t>& data, size_t max_length) {
    if (!connected_) {
        return -1;
    }
    
    data.resize(max_length);
    
    if (is_udp_connection_) {
        socklen_t addr_len = sizeof(udp_addr_);
        int bytes_read = recvfrom(udp_socket_, data.data(), max_length, 0,
                                 (struct sockaddr*)&udp_addr_, &addr_len);
        if (bytes_read > 0) {
            data.resize(bytes_read);
        }
        return bytes_read;
    } else {
        int bytes_read = read(serial_fd_, data.data(), max_length);
        if (bytes_read > 0) {
            data.resize(bytes_read);
        }
        return bytes_read;
    }
}

bool MSPHelper::sendMSPCommand(uint8_t command_id, const std::vector<uint16_t>& data) {
    if (!connected_) {
        return false;
    }
    
    // Convert uint16_t data to uint8_t payload (little endian)
    std::vector<uint8_t> payload;
    for (uint16_t value : data) {
        payload.push_back(value & 0xFF);         // Low byte
        payload.push_back((value >> 8) & 0xFF);  // High byte
    }
    
    // Build and send MSP packet
    std::vector<uint8_t> packet = buildMSPPacket(command_id, payload);
    int bytes_written = writeData(packet);
    
    return bytes_written == static_cast<int>(packet.size());
}

bool MSPHelper::sendMSPRequest(uint8_t command_id) {
    return sendMSPCommand(command_id, {});
}

bool MSPHelper::readMSPResponse(uint8_t expected_command, std::vector<uint8_t>& response_data) {
    if (!connected_) {
        return false;
    }
    
    // Try to read MSP response with timeout
    auto start_time = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::milliseconds(1000);  // 1 second timeout
    
    std::vector<uint8_t> buffer;
    size_t header_index = 0;
    const char* header = "$M>";
    
    while (std::chrono::steady_clock::now() - start_time < timeout) {
        std::vector<uint8_t> chunk;
        int bytes_read = readData(chunk, 256);
        
        if (bytes_read <= 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        
        buffer.insert(buffer.end(), chunk.begin(), chunk.end());
        
        // Look for MSP header
        while (buffer.size() >= 3 && header_index < 3) {
            if (buffer[header_index] == header[header_index]) {
                header_index++;
            } else {
                header_index = 0;
                buffer.erase(buffer.begin());
            }
        }
        
        if (header_index == 3 && buffer.size() >= 6) {
            // We have the header, check if we have length and command
            uint8_t length = buffer[3];
            uint8_t command = buffer[4];
            
            if (command == expected_command && buffer.size() >= static_cast<size_t>(6 + length)) {
                // Extract payload
                response_data.clear();
                response_data.insert(response_data.end(), 
                                   buffer.begin() + 5, 
                                   buffer.begin() + 5 + length);
                
                // Verify checksum
                uint8_t received_checksum = buffer[5 + length];
                uint8_t calculated_checksum = calculateChecksum(command, response_data);
                
                if (received_checksum == calculated_checksum) {
                    return true;
                } else {
                    std::cerr << "MSP checksum mismatch" << std::endl;
                    return false;
                }
            }
        }
    }
    
    std::cerr << "MSP response timeout" << std::endl;
    return false;
}

bool MSPHelper::sendRCCommand(uint16_t roll, uint16_t pitch, uint16_t yaw, uint16_t throttle,
                              uint16_t aux1, uint16_t aux2, uint16_t aux3, uint16_t aux4) {
    
    // Clamp values to valid RC range
    auto clamp = [](uint16_t value) -> uint16_t {
        return std::max(RC_MIN, std::min(RC_MAX, value));
    };
    
    std::vector<uint16_t> rc_data = {
        clamp(roll),     // Channel 0: Roll
        clamp(pitch),    // Channel 1: Pitch  
        clamp(throttle), // Channel 2: Throttle
        clamp(yaw),      // Channel 3: Yaw
        clamp(aux1),     // Channel 4: AUX1
        clamp(aux2),     // Channel 5: AUX2 (servo/bomb release)
        clamp(aux3),     // Channel 6: AUX3 (mode switching)
        clamp(aux4)      // Channel 7: AUX4
    };
    
    return sendMSPCommand(MSP_SET_RAW_RC, rc_data);
}

bool MSPHelper::readRCValues(std::vector<uint16_t>& rc_values) {
    if (!sendMSPRequest(MSP_RC)) {
        return false;
    }
    
    std::vector<uint8_t> response;
    if (!readMSPResponse(MSP_RC, response)) {
        return false;
    }
    
    // Parse RC values (16-bit little endian)
    rc_values.clear();
    for (size_t i = 0; i + 1 < response.size(); i += 2) {
        uint16_t value = response[i] | (response[i + 1] << 8);
        rc_values.push_back(value);
    }
    
    return true;
}

bool MSPHelper::readAltitude(float& altitude) {
    if (!sendMSPRequest(MSP_ALTITUDE)) {
        return false;
    }
    
    std::vector<uint8_t> response;
    if (!readMSPResponse(MSP_ALTITUDE, response)) {
        return false;
    }
    
    if (response.size() >= 6) {
        // Parse altitude (32-bit signed integer in cm)
        int32_t altitude_cm = response[0] | (response[1] << 8) | 
                             (response[2] << 16) | (response[3] << 24);
        altitude = altitude_cm / 100.0f;  // Convert cm to meters
        return true;
    }
    
    return false;
}

bool MSPHelper::readAnalog(float& voltage, float& current, uint16_t& rssi) {
    if (!sendMSPRequest(MSP_ANALOG)) {
        return false;
    }
    
    std::vector<uint8_t> response;
    if (!readMSPResponse(MSP_ANALOG, response)) {
        return false;
    }
    
    if (response.size() >= 7) {
        // Parse analog values
        voltage = (response[0] | (response[1] << 8)) / 10.0f;  // Voltage in 0.1V units
        current = response[2] | (response[3] << 8);            // Current in mA
        rssi = response[4] | (response[5] << 8);               // RSSI
        return true;
    }
    
    return false;
}

} // namespace betaflight_sim
