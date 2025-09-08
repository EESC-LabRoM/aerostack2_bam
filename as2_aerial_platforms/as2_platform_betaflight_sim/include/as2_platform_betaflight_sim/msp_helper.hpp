#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <memory>
#include <sys/socket.h>
#include <netinet/in.h>

namespace betaflight_sim {

/**
 * @brief MSP (Multiwii Serial Protocol) Helper for Betaflight communication
 * Based on the Python implementation from:
 * https://medium.com/illumination/fpv-autonomous-operation-with-betaflight-and-raspberry-pi-0caeb4b3ca69
 */
class MSPHelper {
public:
    // MSP Command IDs (from Betaflight MSP protocol)
    static constexpr uint8_t MSP_RC = 105;
    static constexpr uint8_t MSP_ALTITUDE = 109;
    static constexpr uint8_t MSP_ANALOG = 110;
    static constexpr uint8_t MSP_SET_RAW_RC = 200;
    
    // RC Channel indices
    static constexpr int RC_ROLL = 0;
    static constexpr int RC_PITCH = 1;
    static constexpr int RC_THROTTLE = 2;
    static constexpr int RC_YAW = 3;
    static constexpr int RC_AUX1 = 4;
    static constexpr int RC_AUX2 = 5;  // Used for servo/bomb release
    static constexpr int RC_AUX3 = 6;
    static constexpr int RC_AUX4 = 7;
    
    // RC Value ranges
    static constexpr uint16_t RC_MIN = 1000;
    static constexpr uint16_t RC_MAX = 2000;
    static constexpr uint16_t RC_NEUTRAL = 1500;
    
    /**
     * @brief Constructor
     * @param connection_string Either serial port (/dev/ttyUSB0) or UDP (udp:localhost:5761)
     * @param baud_rate Baud rate for serial connection (default: 115200)
     */
    MSPHelper(const std::string& connection_string, int baud_rate = 115200);
    
    /**
     * @brief Destructor
     */
    ~MSPHelper();
    
    /**
     * @brief Connect to Betaflight
     * @return true if connected successfully
     */
    bool connect();
    
    /**
     * @brief Disconnect from Betaflight
     */
    void disconnect();
    
    /**
     * @brief Check if connected
     * @return true if connected
     */
    bool isConnected() const;
    
    /**
     * @brief Send RC command to Betaflight (MSP_SET_RAW_RC)
     * @param roll Roll channel value (1000-2000)
     * @param pitch Pitch channel value (1000-2000)  
     * @param yaw Yaw channel value (1000-2000)
     * @param throttle Throttle channel value (1000-2000)
     * @param aux1 AUX1 channel value (1000-2000)
     * @param aux2 AUX2 channel value (1000-2000) - used for servo
     * @param aux3 AUX3 channel value (1000-2000)
     * @param aux4 AUX4 channel value (1000-2000)
     * @return true if command sent successfully
     */
    bool sendRCCommand(uint16_t roll, uint16_t pitch, uint16_t yaw, uint16_t throttle,
                       uint16_t aux1 = RC_NEUTRAL, uint16_t aux2 = RC_NEUTRAL,
                       uint16_t aux3 = RC_NEUTRAL, uint16_t aux4 = RC_NEUTRAL);
    
    /**
     * @brief Read RC values from Betaflight (MSP_RC)
     * @param rc_values Output vector of 8 RC channel values
     * @return true if read successfully
     */
    bool readRCValues(std::vector<uint16_t>& rc_values);
    
    /**
     * @brief Read altitude from Betaflight (MSP_ALTITUDE)
     * @param altitude Output altitude in meters
     * @return true if read successfully
     */
    bool readAltitude(float& altitude);
    
    /**
     * @brief Read analog values from Betaflight (MSP_ANALOG)
     * @param voltage Output battery voltage
     * @param current Output current consumption  
     * @param rssi Output RSSI value
     * @return true if read successfully
     */
    bool readAnalog(float& voltage, float& current, uint16_t& rssi);

private:
    std::string connection_string_;
    int baud_rate_;
    bool connected_;
    
    // Connection type (serial or UDP)
    bool is_udp_connection_;
    
    // Serial connection
    int serial_fd_;
    
    // UDP connection  
    int udp_socket_;
    struct sockaddr_in udp_addr_;
    
    /**
     * @brief Calculate MSP checksum
     * @param command_id MSP command ID
     * @param payload Payload data
     * @return calculated checksum
     */
    uint8_t calculateChecksum(uint8_t command_id, const std::vector<uint8_t>& payload);
    
    /**
     * @brief Build MSP packet
     * @param command_id MSP command ID
     * @param payload Payload data
     * @return complete MSP packet
     */
    std::vector<uint8_t> buildMSPPacket(uint8_t command_id, const std::vector<uint8_t>& payload);
    
    /**
     * @brief Send MSP command
     * @param command_id MSP command ID
     * @param data Data to send
     * @return true if sent successfully
     */
    bool sendMSPCommand(uint8_t command_id, const std::vector<uint16_t>& data);
    
    /**
     * @brief Send MSP request (no data)
     * @param command_id MSP command ID
     * @return true if sent successfully
     */
    bool sendMSPRequest(uint8_t command_id);
    
    /**
     * @brief Read MSP response
     * @param expected_command Expected command ID
     * @param response_data Output response data
     * @return true if read successfully
     */
    bool readMSPResponse(uint8_t expected_command, std::vector<uint8_t>& response_data);
    
    /**
     * @brief Write data to connection (serial or UDP)
     * @param data Data to write
     * @return number of bytes written
     */
    int writeData(const std::vector<uint8_t>& data);
    
    /**
     * @brief Read data from connection (serial or UDP)
     * @param data Output data buffer
     * @param max_length Maximum bytes to read
     * @return number of bytes read
     */
    int readData(std::vector<uint8_t>& data, size_t max_length);
};

} // namespace betaflight_sim
