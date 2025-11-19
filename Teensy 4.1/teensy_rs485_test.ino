#include <CRC16.h>

#define RS485_SERIAL Serial1  // TX1 = pin 1, RX1 = pin 0
const int RS485_DE_PIN = 2;   // DE/RE control

const uint32_t RS485_BAUD = 57600;  // Match your joint module
const uint8_t JOINT_ID = 1;

CRC16 crc;

// Joint data structure
struct JointData {
  uint16_t motorCount;
  int16_t motorRotations;
  uint16_t gearboxCount;
  bool limitSwitch1;
  bool limitSwitch2;
  uint8_t statusByte;
  bool dataValid;
  unsigned long lastUpdate;
};

JointData joint1;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  RS485_SERIAL.begin(RS485_BAUD);
  pinMode(RS485_DE_PIN, OUTPUT);
  digitalWrite(RS485_DE_PIN, LOW);

  Serial.println(F("\n╔════════════════════════════════════════╗"));
  Serial.println(F("║  Teensy 4.1 RS485 Master - Final      ║"));
  Serial.println(F("╚════════════════════════════════════════╝"));
  Serial.print(F("Baud: "));
  Serial.print(RS485_BAUD);
  Serial.println(F(" | Polling: 20Hz (50ms)\n"));
  
  joint1.dataValid = false;
  delay(1000);
}

void loop() {
  static unsigned long lastPoll = 0;
  
  // 50ms polling (20 Hz)
  unsigned long now = millis();
  if (now - lastPoll < 50) {
    delay(50 - (now - lastPoll));
  }
  
  pollJoint(JOINT_ID);
  
  lastPoll = millis();
}

void pollJoint(uint8_t jointID) {
  // Force RX mode first
  digitalWrite(RS485_DE_PIN, LOW);
  delayMicroseconds(100);
  
  // Build poll message
  uint8_t pollMsg[6];
  pollMsg[0] = 0x02;
  pollMsg[1] = jointID;
  pollMsg[2] = 0x06;  // CODE_LIVE_DATA

  crc.restart();
  crc.add(pollMsg[0]);
  crc.add(pollMsg[1]);
  crc.add(pollMsg[2]);
  uint16_t calcCRC = crc.calc();

  pollMsg[3] = (calcCRC >> 8) & 0xFF;
  pollMsg[4] = calcCRC & 0xFF;
  pollMsg[5] = 0x03;

  // Clear RX buffer
  while (RS485_SERIAL.available()) RS485_SERIAL.read();

  // Switch to TX
  digitalWrite(RS485_DE_PIN, HIGH);
  delayMicroseconds(100);
  
  RS485_SERIAL.write(pollMsg, 6);
  RS485_SERIAL.flush();
  
  delayMicroseconds(100);
  
  // Switch to RX immediately
  digitalWrite(RS485_DE_PIN, LOW);

  // Receive response
  uint8_t response[13];
  if (receiveResponse(response, 13, 200)) {
    parseAndDisplay(response);
  } else {
    Serial.println(F("✗ Timeout\n"));
    joint1.dataValid = false;
  }
}

bool receiveResponse(uint8_t *response, size_t length, unsigned long timeoutMs) {
  unsigned long startTime = millis();
  size_t index = 0;
  bool gotSTX = false;

  while (millis() - startTime < timeoutMs && index < length) {
    if (RS485_SERIAL.available()) {
      uint8_t byte = RS485_SERIAL.read();

      if (!gotSTX) {
        if (byte == 0x02) {
          gotSTX = true;
          response[index++] = byte;
        }
      } else {
        response[index++] = byte;
      }
    }
  }

  return (index == length && gotSTX && response[length-1] == 0x03);
}

void parseAndDisplay(uint8_t *r) {
  // Validate frame
  if (r[0] != 0x02 || r[12] != 0x03 || r[1] != JOINT_ID) {
    Serial.println(F("✗ Frame error\n"));
    joint1.dataValid = false;
    return;
  }

  // Verify CRC
  crc.restart();
  for (int i = 0; i <= 9; i++) {
    crc.add(r[i]);
  }
  uint16_t calcCRC = crc.calc();
  uint16_t recvCRC = ((uint16_t)r[10] << 8) | r[11];

  if (calcCRC != recvCRC) {
    Serial.print(F("✗ CRC Mismatch (calc: 0x"));
    Serial.print(calcCRC, HEX);
    Serial.print(F(" recv: 0x"));
    Serial.print(recvCRC, HEX);
    Serial.println(F(")\n"));
    joint1.dataValid = false;
    return;
  }

  // Parse data
  joint1.motorCount = ((uint16_t)r[2] << 8) | r[3];
  joint1.motorRotations = ((int16_t)r[4] << 8) | r[5];
  joint1.gearboxCount = ((uint16_t)r[6] << 8) | r[7];
  joint1.limitSwitch1 = (r[8] & 0x01) != 0;
  joint1.limitSwitch2 = (r[8] & 0x02) != 0;
  joint1.statusByte = r[9];
  joint1.dataValid = true;
  joint1.lastUpdate = millis();

  // Calculate angles
  float motorDeg = (joint1.motorCount / 4096.0) * 360.0;
  float gearboxDeg = (joint1.gearboxCount / 4096.0) * 360.0;
  
  // Calculate slippage
  int32_t motorTotalCounts = (joint1.motorRotations * 4096L) + joint1.motorCount;
  int32_t expectedGearboxCounts = motorTotalCounts / 50;
  int32_t slippageCounts = joint1.gearboxCount - (expectedGearboxCounts % 4096);
  float slippageDeg = (slippageCounts / 4096.0) * 360.0;

  // Display formatted data
  Serial.print(F("✓ J"));
  Serial.print(JOINT_ID);
  Serial.print(F(" | Motor: "));
  Serial.print(joint1.motorCount);
  Serial.print(F(" ("));
  Serial.print(motorDeg, 2);
  Serial.print(F("°)"));
  
  Serial.print(F(" | Rot: "));
  Serial.print(joint1.motorRotations);
  
  Serial.print(F(" | Gearbox: "));
  Serial.print(joint1.gearboxCount);
  Serial.print(F(" ("));
  Serial.print(gearboxDeg, 2);
  Serial.print(F("°)"));
  
  Serial.print(F(" | L1: "));
  Serial.print(joint1.limitSwitch1 ? F("ON") : F("off"));
  Serial.print(F(" L2: "));
  Serial.print(joint1.limitSwitch2 ? F("ON") : F("off"));
  
  Serial.print(F(" | Slip: "));
  Serial.print(slippageCounts);
  Serial.print(F(" ("));
  Serial.print(slippageDeg, 2);
  Serial.print(F("°)"));
  
  Serial.print(F(" | "));
  Serial.println((joint1.statusByte & 0x01) ? F("READY") : F("ERROR"));
  
  Serial.println();
}
