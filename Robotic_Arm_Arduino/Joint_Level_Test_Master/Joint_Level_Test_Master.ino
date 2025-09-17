#include <SoftwareSerial.h>
#include <CRC16.h>

// Minimal debug flag
const uint8_t debugFlag = 0;

const uint8_t RS485_TX_PIN = 8;
const uint8_t RS485_RX_PIN = 9;
const uint8_t RS485_DE_PIN = 10;

SoftwareSerial rs485(RS485_RX_PIN, RS485_TX_PIN);

uint8_t TARGET_JOINT_ID = 1;  // Changed from const to allow modification
const unsigned long POLL_INTERVAL = 100;  // Faster polling for testing

// CRC calculator
CRC16 crc;

// Only live data command now
#define CODE_LIVE_DATA 0x06

bool manualMode = false;

void setup() 
{
  Serial.begin(115200);
  rs485.begin(38400);
  pinMode(RS485_DE_PIN, OUTPUT);
  digitalWrite(RS485_DE_PIN, LOW);

  Serial.println(F("=== Simplified Joint Tester ==="));
  Serial.print(F("Auto-polling live data from Joint "));  // Fixed F() concatenation
  Serial.println(TARGET_JOINT_ID);
  Serial.println(F("Press 'm' for manual mode, 'h' for help"));
  Serial.println(F("======================================"));
}

void loop() 
{
  // Check for user input
  if (Serial.available()) {
    char input = Serial.read();
    processUserInput(input);
  }

  // Auto polling - only live data now
  if (!manualMode) {
    static unsigned long lastPoll = 0;
    if (millis() - lastPoll > POLL_INTERVAL) 
    {
      sendPoll();
      receiveResponse();
      lastPoll = millis();
    }
  }
}

void processUserInput(char input) {
  switch(input) {
    case 't': 
    case 'T':
      sendPoll();
      receiveResponse();
      break;
      
    case 'm': 
    case 'M':
      manualMode = !manualMode; 
      Serial.print(F("Mode: "));
      Serial.println(manualMode ? F("MANUAL - Press 't' to test") : F("AUTO - Continuous polling"));
      break;
      
    case 'h': 
    case 'H':
      printHelp();
      break;
      
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
      TARGET_JOINT_ID = input - '0';
      Serial.print(F("Target Joint ID changed to: "));
      Serial.println(TARGET_JOINT_ID);
      break;
      
    default: 
      Serial.println(F("Unknown command. Press 'h' for help."));
      break;
  }
}

void printHelp() {
  Serial.println(F("\n=== HELP MENU ==="));
  Serial.println(F("t - Send single test poll"));
  Serial.println(F("m - Toggle Manual/Auto mode"));
  Serial.println(F("1-5 - Change target joint ID"));
  Serial.println(F("h - Show this help"));
  Serial.println(F("=================="));
}

void sendPoll() 
{
  uint8_t msg[6];
  msg[0] = 0x02;  // STX
  msg[1] = TARGET_JOINT_ID;
  msg[2] = CODE_LIVE_DATA;
  
  crc.restart();
  crc.add(msg[0]);
  crc.add(msg[1]);
  crc.add(msg[2]);
  uint16_t calcCRC = crc.getCRC();
  
  msg[3] = (calcCRC >> 8) & 0xFF;
  msg[4] = calcCRC & 0xFF;
  msg[5] = 0x03;  // ETX

  digitalWrite(RS485_DE_PIN, HIGH);
  delay(1);
  rs485.write(msg, 6);
  rs485.flush();
  delay(1);
  digitalWrite(RS485_DE_PIN, LOW);

  if (debugFlag) {
    Serial.print(F("Poll sent to Joint "));
    Serial.println(TARGET_JOINT_ID);
  }
}

void receiveResponse() 
{
  uint8_t response[13];
  uint8_t index = 0;
  unsigned long timeout = millis() + 200;  // Shorter timeout
  bool gotSTX = false;

  // Simple receive
  while (millis() < timeout && index < 13) 
  {
    if (rs485.available()) 
    {
      uint8_t byte = rs485.read();
      
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

  if (index == 13) {
    // CRC check
    crc.restart();
    for (int i = 0; i < 10; i++) {
      crc.add(response[i]);
    }
    uint16_t calcCRC = crc.getCRC();
    uint16_t recvCRC = ((uint16_t)response[10] << 8) | response[11];

    if (calcCRC == recvCRC && response[12] == 0x03) {
      parseResponse(response);
    } else {
      Serial.println(F("❌ CRC Error or Invalid ETX"));
      printRawData(response, index);
    }
  } else if (index > 0) {
    Serial.print(F("❌ Incomplete packet ("));
    Serial.print(index);
    Serial.println(F(" bytes)"));
    printRawData(response, index);
  } else {
    Serial.println(F("❌ Timeout - No response"));
  }
}

void parseResponse(uint8_t* r) {
  // Verify this is from correct joint and is live data
  if (r[1] != TARGET_JOINT_ID) {
    Serial.print(F("⚠️  Response from wrong joint: "));
    Serial.println(r[1]);
    return;
  }
  
  if (r[2] != CODE_LIVE_DATA) {
    Serial.print(F("⚠️  Unexpected command response: 0x"));
    Serial.println(r[2], HEX);
    return;
  }

  // Parse the simplified live data format
  uint16_t motorAngle = (r[3] << 8) | r[4];
  uint16_t gearboxAngle = (r[5] << 8) | r[6];
  uint8_t limitSwitches = r[7];
  uint8_t systemStatus = r[8];

  // Display data in clean format
  Serial.print(F("Joint "));
  Serial.print(r[1]);
  Serial.print(F(" | Motor: "));
  Serial.print(motorAngle / 10.0, 1);
  Serial.print(F("° | Gearbox: "));
  Serial.print(gearboxAngle / 10.0, 1);
  Serial.print(F("° | L1: "));
  Serial.print((limitSwitches & 0x01) ? F("ON") : F("off"));
  Serial.print(F(" | L2: "));
  Serial.print((limitSwitches & 0x02) ? F("ON") : F("off"));
  Serial.print(F(" | Status: "));
  Serial.print((systemStatus & 0x01) ? F("READY") : F("NOT_READY"));

  // Calculate and display slippage (since it's no longer sent)
  float expectedGearbox = motorAngle / 50.0; // Assuming 50:1 gear ratio
  float actualGearbox = gearboxAngle / 10.0;
  float slippage = actualGearbox - expectedGearbox;
  Serial.print(F(" | Slippage: "));
  Serial.print(slippage, 2);
  Serial.println(F("°"));

  // Alert on limit switch activation
  if (limitSwitches & 0x03) {
    Serial.println(F("🚨 LIMIT SWITCH ACTIVE!"));
  }

  // Alert on system not ready
  if (!(systemStatus & 0x01)) {
    Serial.println(F("⚠️  SYSTEM NOT READY"));
  }
}

void printRawData(uint8_t* data, uint8_t length) {
  Serial.print(F("Raw data ("));
  Serial.print(length);
  Serial.print(F(" bytes): "));
  for (int i = 0; i < length; i++) {
    if (data[i] < 0x10) Serial.print(F("0"));
    Serial.print(data[i], HEX);
    Serial.print(F(" "));
  }
  Serial.println();
}
