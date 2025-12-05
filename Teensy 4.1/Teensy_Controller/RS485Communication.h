#ifndef RS485_COMMUNICATION_H
#define RS485_COMMUNICATION_H

#include <Arduino.h>
#include <CRC16.h>
#include "SystemConfig.h"

// ============================================================
// RS485 COMMUNICATION MODULE
// This module handles all RS485 protocol communication
// MESSAGE FORMAT AND PROTOCOL ARE LOCKED - DO NOT MODIFY
// ============================================================

class RS485Communication {
private:
    CRC16 crc;
    const int dePin;
    HardwareSerial& serialPort;
    const uint32_t baudRate;
    
    // Internal helper functions
    bool receiveResponse(uint8_t *buffer, size_t length, unsigned long timeout);
    bool parseResponse(uint8_t *response, uint8_t jointID, 
                       int32_t &gearboxCount, int32_t &motorCount, int32_t &motorRotations);

public:
    RS485Communication(int dePinNum, HardwareSerial& serial, uint32_t baud);
    
    void begin();
    bool pollJoint(uint8_t jointID, int32_t &gearboxCount, int32_t &motorCount, int32_t &motorRotations);  // Now returns bool
};

#endif // RS485_COMMUNICATION_H
