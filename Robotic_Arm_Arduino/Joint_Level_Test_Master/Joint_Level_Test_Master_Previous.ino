#include <SoftwareSerial.h>
#include <CRC16.h>

// Minimal debug flag
const uint8_t debugFlag = 0;

const uint8_t RS485_TX_PIN = 8;
const uint8_t RS485_RX_PIN = 9;
const uint8_t RS485_DE_PIN = 10;

SoftwareSerial rs485(RS485_RX_PIN, RS485_TX_PIN);

const uint8_t TARGET_JOINT_ID = 1;
const unsigned long POLL_INTERVAL = 1000;

// CRC calculator
CRC16 crc;

// Command codes
#define CODE_STARTUP_STATUS   0x05
#define CODE_LIVE_DATA        0x06
#define CODE_SELF_DIAGNOSTICS 0x08
#define CODE_ERROR_REPORT     0x09
#define CODE_ALARM_RESET      0x0A
#define CODE_SENSOR_RESET     0x0B
#define CODE_EMERGENCY        0x0C

// Current command
uint8_t currentCommand = CODE_LIVE_DATA;
bool manualMode = false;

void setup() 
{
  Serial.begin(115200);
  rs485.begin(38400);
  pinMode(RS485_DE_PIN, OUTPUT);
  digitalWrite(RS485_DE_PIN, LOW);

  Serial.println(F("=== Joint Module Tester ==="));
  printMenu();
}

void loop() 
{
  // Check for user input
  if (Serial.available()) {
    char input = Serial.read();
    processUserInput(input);
  }

  // Auto polling
  if (!manualMode) {
    static unsigned long lastPoll = 0;
    if (millis() - lastPoll > POLL_INTERVAL) 
    {
      sendPoll(currentCommand);
      receiveResponse();
      lastPoll = millis();
    }
  }
}

void printMenu() {
  Serial.println(F("\n=== TEST MENU ==="));
  Serial.println(F("1-Startup 2-Live 3-Diag 4-Error"));
  Serial.println(F("5-AlarmRst 6-SensorRst 7-Emergency"));
  Serial.println(F("m-Manual h-Help t-Test"));
  Serial.print(F("Mode: "));
  Serial.println(manualMode ? F("MANUAL") : F("AUTO"));
}

void processUserInput(char input) {
  switch(input) {
    case '1': currentCommand = CODE_STARTUP_STATUS; executeTest(); break;
    case '2': currentCommand = CODE_LIVE_DATA; executeTest(); break;
    case '3': currentCommand = CODE_SELF_DIAGNOSTICS; executeTest(); break;
    case '4': currentCommand = CODE_ERROR_REPORT; executeTest(); break;
    case '5': currentCommand = CODE_ALARM_RESET; executeTest(); break;
    case '6': currentCommand = CODE_SENSOR_RESET; executeTest(); break;
    case '7': currentCommand = CODE_EMERGENCY; executeTest(); break;
    case 'm': manualMode = !manualMode; 
              Serial.println(manualMode ? F("MANUAL") : F("AUTO")); break;
    case 't': if (manualMode) executeTest(); break;
    case 'h': printMenu(); break;
    default: Serial.println(F("Invalid")); break;
  }
}

void executeTest() {
  if (manualMode) {
    sendPoll(currentCommand);
    receiveResponse();
  }
}

void sendPoll(uint8_t command) 
{
  uint8_t msg[6];
  msg[0] = 0x02;
  msg[1] = TARGET_JOINT_ID;
  msg[2] = command;
  
  crc.restart();
  crc.add(msg[0]);
  crc.add(msg[1]);
  crc.add(msg[2]);
  uint16_t calcCRC = crc.getCRC();
  
  msg[3] = (calcCRC >> 8) & 0xFF;
  msg[4] = calcCRC & 0xFF;
  msg[5] = 0x03;

  digitalWrite(RS485_DE_PIN, HIGH);
  delay(1);
  rs485.write(msg, 6);
  rs485.flush();
  delay(1);
  digitalWrite(RS485_DE_PIN, LOW);

  // Print command name
  Serial.print(F("Sent: "));
  printCmd(command);
  Serial.println();
}

void printCmd(uint8_t cmd) {
  switch(cmd) {
    case CODE_STARTUP_STATUS: Serial.print(F("Startup")); break;
    case CODE_LIVE_DATA: Serial.print(F("Live")); break;
    case CODE_SELF_DIAGNOSTICS: Serial.print(F("Diag")); break;
    case CODE_ERROR_REPORT: Serial.print(F("Error")); break;
    case CODE_ALARM_RESET: Serial.print(F("AlarmRst")); break;
    case CODE_SENSOR_RESET: Serial.print(F("SensorRst")); break;
    case CODE_EMERGENCY: Serial.print(F("Emergency")); break;
  }
}

void receiveResponse() 
{
  uint8_t response[13];
  uint8_t index = 0;
  unsigned long timeout = millis() + 500;
  bool gotSTX = false;

  // Simple receive - no state machine to save memory
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
    // Quick CRC check
    crc.restart();
    for (int i = 0; i < 10; i++) {
      crc.add(response[i]);
    }
    uint16_t calcCRC = crc.getCRC();
    uint16_t recvCRC = ((uint16_t)response[10] << 8) | response[11];

    if (calcCRC == recvCRC && response[12] == 0x03) {
      parseResponse(response);
    } else {
      Serial.println(F("CRC Error"));
    }
  } else {
    Serial.println(F("Timeout"));
  }
}

void parseResponse(uint8_t* r) {
  uint8_t cmd = r[2];
  
  Serial.print(F("Response: "));
  printCmd(cmd);
  Serial.println();
  
  switch(cmd) {
    case CODE_STARTUP_STATUS:
      Serial.print(F("Ready:")); Serial.println((r[3] & 0x01) ? F("Y") : F("N"));
      Serial.print(F("Motor:")); Serial.println(((r[4] << 8) | r[5]) / 10.0);
      Serial.print(F("Gearbox:")); Serial.println(((r[6] << 8) | r[7]) / 10.0);
      break;
      
    case CODE_LIVE_DATA:
      {
        uint16_t motor = (r[3] << 8) | r[4];
        uint16_t gearbox = (r[5] << 8) | r[6];
        int16_t slip = (r[7] << 8) | r[8];
        uint8_t flags = r[9];
        
        Serial.print(F("M:")); Serial.print(motor/10.0);
        Serial.print(F(" G:")); Serial.print(gearbox/10.0);
        Serial.print(F(" S:")); Serial.print(slip/10.0);
        Serial.print(F(" L1:")); Serial.print((flags & 0x01) ? 1 : 0);
        Serial.print(F(" L2:")); Serial.print((flags & 0x02) ? 1 : 0);
        Serial.print(F(" EMG:")); Serial.println((flags & 0x04) ? F("***YES***") : F("No"));
      }
      break;
      
    case CODE_SELF_DIAGNOSTICS:
      Serial.print(F("Motor:")); Serial.print(r[3] ? F("FAULT") : F("OK"));
      Serial.print(F(" Gearbox:")); Serial.print(r[4] ? F("FAULT") : F("OK"));
      Serial.print(F(" L1:")); Serial.print(r[5] ? F("FAULT") : F("OK"));
      Serial.print(F(" L2:")); Serial.print(r[6] ? F("FAULT") : F("OK"));
      Serial.print(F(" Slip:")); Serial.println(r[7] ? F("FAULT") : F("OK"));
      break;
      
    case CODE_ERROR_REPORT:
      {
        uint16_t err1 = (r[3] << 8) | r[4];
        uint16_t err2 = (r[5] << 8) | r[6];
        Serial.print(F("Err1:0x")); Serial.print(err1, HEX);
        Serial.print(F(" Uptime:")); Serial.println(err2);
      }
      break;
      
    case CODE_ALARM_RESET:
      Serial.print(F("ACK:")); Serial.print(r[3] ? F("OK") : F("FAIL"));
      Serial.print(F(" PrevEmg:0x")); Serial.println(r[4], HEX);
      break;
      
    case CODE_SENSOR_RESET:
      Serial.print(F("ACK:")); Serial.println(r[3] ? F("OK") : F("FAIL"));
      break;
      
    case CODE_EMERGENCY:
      {
        uint8_t type = r[3];
        uint16_t motor = (r[4] << 8) | r[5];
        Serial.print(F("EmgType:0x")); Serial.print(type, HEX);
        Serial.print(F(" Motor:")); Serial.println(motor/10.0);
      }
      break;
      
    default:
      Serial.println(F("Unknown"));
      break;
  }
}
