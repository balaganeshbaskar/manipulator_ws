#include "RS485Communication.h"

RS485Communication::RS485Communication(int dePinNum, HardwareSerial& serial, uint32_t baud)
    : dePin(dePinNum), serialPort(serial), baudRate(baud) {
}

void RS485Communication::begin() {
    serialPort.begin(baudRate);
    pinMode(dePin, OUTPUT);
    digitalWrite(dePin, LOW);
}

bool RS485Communication::pollJoint(uint8_t jointID, int32_t &gearboxCount, int32_t &motorCount, int32_t &motorRotations) {
    // Set to receive mode
    digitalWrite(dePin, LOW);
    delayMicroseconds(100);
    
    // Build message: STX, ID, CMD, CRC_H, CRC_L, ETX
    uint8_t msg[6] = {0x02, jointID, 0x06, 0, 0, 0x03};
    
    // Calculate CRC over STX, ID, CMD
    crc.restart();
    crc.add(msg[0]);
    crc.add(msg[1]);
    crc.add(msg[2]);
    uint16_t c = crc.calc();
    msg[3] = c >> 8;
    msg[4] = c & 0xFF;
    
    // Clear any pending data
    while(serialPort.available()) serialPort.read();
    
    // Transmit
    digitalWrite(dePin, HIGH);
    delayMicroseconds(100);
    serialPort.write(msg, 6);
    serialPort.flush();
    delayMicroseconds(100);
    digitalWrite(dePin, LOW);
    
    // Receive response
    uint8_t response[13];
    if(receiveResponse(response, 13, 200)) {
        return parseResponse(response, jointID, gearboxCount, motorCount, motorRotations);
    }
    
    return false;
}

bool RS485Communication::receiveResponse(uint8_t *r, size_t len, unsigned long to) {
    unsigned long s = millis();
    size_t i = 0;
    bool stx = false;
    
    while(millis() - s < to && i < len) {
        if(serialPort.available()) {
            uint8_t b = serialPort.read();
            if(!stx && b == 0x02) {
                stx = true;
                r[i++] = b;
            } else if(stx) {
                r[i++] = b;
            }
        }
    }
    return (i == len && r[len-1] == 0x03);
}


bool RS485Communication::parseResponse(uint8_t *response, uint8_t jointID, 
                                       int32_t &gearboxCount, int32_t &motorCount, int32_t &motorRotations) {
    // Response format from YOUR ORIGINAL CODE:
    // STX ID DATA... CRC_H CRC_L ETX
    // response[0] = STX (0x02)
    // response[1] = ID
    // response[2..9] = data bytes
    // response[10] = CRC_H
    // response[11] = CRC_L
    // response[12] = ETX (0x03)
    
    // Verify CRC (over bytes 0-9, exactly as your original code)
    crc.restart();
    for(int i = 0; i <= 9; i++) crc.add(response[i]);
    uint16_t calcCRC = crc.calc();
    uint16_t recvCRC = ((uint16_t)response[10] << 8) | response[11];
    if(calcCRC != recvCRC) return false;
    
    // Extract data exactly as your original code:
    // Motor count (2 bytes)
    motorCount = ((uint16_t)response[2] << 8) | response[3];
    
    // Motor rotations (2 bytes, signed)
    motorRotations = ((int16_t)response[4] << 8) | response[5];
    
    // Gearbox count (2 bytes)
    gearboxCount = ((uint16_t)response[6] << 8) | response[7];
    
    return true;
}
