// Arduino Nano RS485 Master Polling Node
// Polls joint module with robust STX detection

#include <SoftwareSerial.h>
#include <CRC16.h>

// RS485 pins on Nano Master
#define RS485_TX_PIN 8  // Digital pin 3 → MAX485 DI
#define RS485_RX_PIN 9  // Digital pin 4 → MAX485 RO
#define RS485_DE_PIN 10  // Digital pin 2 → MAX485 DE & RE (tied together)

SoftwareSerial rs485(RS485_RX_PIN, RS485_TX_PIN);
CRC16 crc;

const uint8_t JOINT_ID = 1;  // Joint ID to poll

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  
  rs485.begin(57600);
  pinMode(RS485_DE_PIN, OUTPUT);
  digitalWrite(RS485_DE_PIN, LOW);  // Start in receive mode

  Serial.println(F("\n===== Arduino Nano RS485 Master ====="));
  Serial.println(F("Polling Joint Module...\n"));
  delay(1000);
}

void loop() {
  // Build 6-BYTE poll message: [STX][ID][CODE][CRC_H][CRC_L][ETX]
  uint8_t pollMsg[6];  // Changed to 6
  pollMsg[0] = 0x02;
  pollMsg[1] = JOINT_ID;
  pollMsg[2] = 0x06;  // CODE_LIVE_DATA

  crc.restart();
  crc.add(pollMsg[0]);
  crc.add(pollMsg[1]);
  crc.add(pollMsg[2]);  // Include CODE
  uint16_t cval = crc.getCRC();

  pollMsg[3] = (cval >> 8) & 0xFF;
  pollMsg[4] = cval & 0xFF;
  pollMsg[5] = 0x03;  // ETX at position 5

  // Clear buffer
  while (rs485.available()) rs485.read();

  // Send
  digitalWrite(RS485_DE_PIN, HIGH);
  delay(2);
  rs485.write(pollMsg, 6);  // Send 6 bytes
  rs485.flush();
  delay(2);
  digitalWrite(RS485_DE_PIN, LOW);

  Serial.print(F("TX → "));
  for (int i = 0; i < 6; i++) {
    if (pollMsg[i] < 0x10) Serial.print(F("0"));
    Serial.print(pollMsg[i], HEX);
    Serial.print(F(" "));
  }
  Serial.println();

  // 4. Receive response with robust STX detection (13 bytes expected)
  uint8_t response[13];
  bool received = receiveResponse(response, 13, 200); // 200ms timeout

  if (received) {
    Serial.print(F("RX ← "));
    for (int i = 0; i < 13; i++) {
      if (response[i] < 0x10) Serial.print(F("0"));
      Serial.print(response[i], HEX);
      Serial.print(F(" "));
    }
    Serial.println();

    // 5. Validate frame structure
    if (response[0] == 0x02 && response[12] == 0x03 && response[1] == JOINT_ID) {
      // 6. Verify CRC (bytes 0-9)
      crc.restart();
      for (int i = 0; i <= 9; i++) {
        crc.add(response[i]);
      }
      uint16_t calcCRC = crc.getCRC();
      uint16_t recvCRC = ((uint16_t)response[10] << 8) | response[11];

      if (calcCRC == recvCRC) {
        Serial.println(F("✓ Valid response - CRC OK"));

        // 7. Parse data
        uint16_t motorCount = ((uint16_t)response[2] << 8) | response[3];
        int16_t motorRotations = ((int16_t)response[4] << 8) | response[5];
        uint16_t gearboxCount = ((uint16_t)response[6] << 8) | response[7];
        uint8_t switches = response[8];
        uint8_t status = response[9];

        Serial.print(F("  Motor: "));
        Serial.print(motorCount);
        Serial.print(F(" | Rot: "));
        Serial.print(motorRotations);
        Serial.print(F(" | Gearbox: "));
        Serial.print(gearboxCount);
        Serial.print(F(" | L1: "));
        Serial.print((switches & 0x01) ? F("ON") : F("off"));
        Serial.print(F(" L2: "));
        Serial.print((switches & 0x02) ? F("ON") : F("off"));
        Serial.print(F(" | Status: "));
        Serial.println((status & 0x01) ? F("READY") : F("ERROR"));
      } else {
        Serial.print(F("✗ CRC Mismatch (calc: 0x"));
        Serial.print(calcCRC, HEX);
        Serial.print(F(" recv: 0x"));
        Serial.print(recvCRC, HEX);
        Serial.println(F(")"));
      }
    } else {
      Serial.println(F("✗ Frame error or wrong Joint ID"));
    }
  } else {
    Serial.println(F("✗ Timeout - No valid response"));
  }

  Serial.println();
  delay(1000);  // Poll every 1 second
}

// ============================================================
// ROBUST BYTE-BY-BYTE RECEIVE WITH STX DETECTION
// ============================================================
bool receiveResponse(uint8_t *response, size_t length, unsigned long timeoutMs) {
  unsigned long startTime = millis();
  size_t index = 0;
  bool gotSTX = false;

  // Read byte-by-byte, looking for STX first
  while (millis() - startTime < timeoutMs && index < length) {
    if (rs485.available()) {
      uint8_t byte = rs485.read();

      if (!gotSTX) {
        // Looking for STX (0x02)
        if (byte == 0x02) {
          gotSTX = true;
          response[index++] = byte;
        }
        // else: discard garbage bytes before STX
      } else {
        // Already got STX, collect remaining bytes
        response[index++] = byte;
      }
    }
  }

  // Validate we got a complete frame with correct start and end markers
  if (index == length && gotSTX && response[length-1] == 0x03) {
    return true; // Valid frame received
  } else {
    return false; // Timeout or invalid frame
  }
}
