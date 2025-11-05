#include <Wire.h>
#include <TCA9548.h>          // RobTillaart's TCA9548 library
#include <Adafruit_AS5600.h>  // Adafruit's AS5600 library
#include <SoftwareSerial.h>
#include <CRC16.h>            // RobTillaart's CRC library
#include <avr/wdt.h>          // For watchdog timer

// Debug flag (1 = enable debug prints, 0 = disable)
const uint8_t debugFlag = 1;

// Define constants for modularity
const uint8_t JOINT_ID = 1;  // Change this for each joint (1-5)
const uint8_t MUX_ADDR = 0x70;  // Default I2C address for TCA9548
const uint8_t ENCODER_CHANNEL_MOTOR = 2;  // SD2/SC2 for motor encoder
const uint8_t ENCODER_CHANNEL_GEARBOX = 7;  // SD7/SC7 for gearbox encoder
const int LIMIT_SWITCH_PIN1 = 2;  // Pin for switch 1
const int LIMIT_SWITCH_PIN2 = 3;  // Pin for switch 2
const uint8_t RS485_TX_PIN = 8;
const uint8_t RS485_RX_PIN = 9;
const uint8_t RS485_DE_PIN = 10;  // DE/RE tied together

// Communication timeout
unsigned long lastValidCommand = 0;
const unsigned long COMMAND_TIMEOUT = 5000; // 5 seconds

// RS485 SoftwareSerial instance
SoftwareSerial rs485(RS485_RX_PIN, RS485_TX_PIN);

// I2C Multiplexer instance
TCA9548 mux(MUX_ADDR);

// AS5600 Encoder instances
Adafruit_AS5600 encoderMotor;
Adafruit_AS5600 encoderGearbox;

// CRC calculator
CRC16 crc;

// Variables for data (use int16 for angles in tenths of degrees for precision)
int16_t motorAngle = 0;
int16_t gearboxAngle = 0;
bool limitSwitch1 = false;
bool limitSwitch2 = false;

// Message buffer (13 bytes: [STX][ID][CODE][Data(7)][CRC(2)][ETX])
uint8_t message[13];

// Code for live data command
#define CODE_LIVE_DATA 0x06

// Debounce arrays
uint32_t lastChangeTimes[2] = {0, 0};
bool lastStates[2] = {LOW, LOW};

// System status
bool systemReady = false;

void printLiveData();
void handleRS485Command();
void sendLiveData();
void readSensors();
void runCRCTest();

void setup() {
  wdt_disable(); // Disable watchdog during setup
  Serial.begin(115200);

  runCRCTest();

  
  Serial.println("Joint " + String(JOINT_ID) + " initializing...");

  Wire.begin();
  lastValidCommand = millis();

  // Initialize hardware
  if (!mux.begin()) {
    Serial.println("Mux not found. Check connections.");
    while (1);
  }

  mux.selectChannel(ENCODER_CHANNEL_MOTOR);
  if (!encoderMotor.begin()) {
    Serial.println("Motor encoder not found.");
    while (1);
  }
  
  mux.selectChannel(ENCODER_CHANNEL_GEARBOX);
  if (!encoderGearbox.begin()) {
    Serial.println("Gearbox encoder not found.");
    while (1);
  }

  // Initialize limit switches
  pinMode(LIMIT_SWITCH_PIN1, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_PIN2, INPUT_PULLUP);

  // Initialize RS485
  rs485.begin(38400);
  pinMode(RS485_DE_PIN, OUTPUT);
  digitalWrite(RS485_DE_PIN, LOW);

  // Read initial sensor values
  readSensors();
  systemReady = true;

  wdt_enable(WDTO_8S); // Enable 8-second watchdog
  Serial.println("Initialization complete.");
}

void loop() 
{
  wdt_reset(); // Reset watchdog timer

  // Handle incoming RS485 commands
  if (rs485.available()) {
    handleRS485Command();
  }

  // Update sensor readings every 50ms
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 50) {
    readSensors();
    lastUpdate = millis();
    printLiveData();
  }

  // Simple communication timeout check
  if (millis() - lastValidCommand > COMMAND_TIMEOUT) {
    // Could set a flag here if needed, but no complex emergency handling
    if (debugFlag) {
      Serial.println("Communication timeout");
    }
    lastValidCommand = millis(); // Reset to prevent spam
  }
}

void runCRCTest() 
{
    Serial.println("\n--- Starting CRC-16 Test ---");

    // Test Case 1: The original poll message
    uint8_t test1[] = {0x02, 0x01, 0x06};
    crc.restart();
    crc.add(test1, sizeof(test1));
    uint16_t result1 = crc.getCRC();
    Serial.print("Input: 02 01 06       -> CRC: 0x");
    if (result1 < 0x1000) Serial.print("0");
    if (result1 < 0x100) Serial.print("0");
    if (result1 < 0x10) Serial.print("0");
    Serial.println(result1, HEX);

    // Test Case 2: Simple ascending numbers
    uint8_t test2[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    crc.restart();
    crc.add(test2, sizeof(test2));
    uint16_t result2 = crc.getCRC();
    Serial.print("Input: 01 02 03 04 05 -> CRC: 0x");
    if (result2 < 0x1000) Serial.print("0");
    if (result2 < 0x100) Serial.print("0");
    if (result2 < 0x10) Serial.print("0");
    Serial.println(result2, HEX);

    // Test Case 3: A longer message with mixed values
    uint8_t test3[] = {0xAA, 0x55, 0x00, 0xFF, 0x12, 0x34, 0x56, 0x78};
    crc.restart();
    crc.add(test3, sizeof(test3));
    uint16_t result3 = crc.getCRC();
    Serial.print("Input: AA 55 ... 78 -> CRC: 0x");
    if (result3 < 0x1000) Serial.print("0");
    if (result3 < 0x100) Serial.print("0");
    if (result3 < 0x10) Serial.print("0");
    Serial.println(result3, HEX);

    // Test Case 4: A single byte
    uint8_t test4[] = {0x42};
    crc.restart();
    crc.add(test4, sizeof(test4));
    uint16_t result4 = crc.getCRC();
    Serial.print("Input: 42             -> CRC: 0x");
    if (result4 < 0x1000) Serial.print("0");
    if (result4 < 0x100) Serial.print("0");
    if (result4 < 0x10) Serial.print("0");
    Serial.println(result4, HEX);

    // Test Case 5: All zeros
    uint8_t test5[] = {0x00, 0x00, 0x00};
    crc.restart();
    crc.add(test5, sizeof(test5));
    uint16_t result5 = crc.getCRC();
    Serial.print("Input: 00 00 00       -> CRC: 0x");
    if (result5 < 0x1000) Serial.print("0");
    if (result5 < 0x100) Serial.print("0");
    if (result5 < 0x10) Serial.print("0");
    Serial.println(result5, HEX);

    Serial.println("--- CRC-16 Test Complete ---\n");
}


void handleRS485Command() {

  if (debugFlag) {
      Serial.println("Inside handleRS485Command!");
    }

  uint8_t buf[6];  // Poll message: [STX][JOINT_ID][CODE][CRC(2)][ETX]
  rs485.readBytes(buf, 6);

  if (debugFlag) {
      // *** PRINT RECEIVED DATA ***
      Serial.print("RX Raw: ");
      for (int i = 0; i < 6; i++) {
        if (buf[i] < 0x10) Serial.print("0");
        Serial.print(buf[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }

  // Validate message
  if (buf[0] == 0x02 && buf[1] == JOINT_ID) {  // Check STX and Joint ID
    // Verify CRC
    crc.restart();
    crc.add(buf[0]);
    crc.add(buf[1]);
    crc.add(buf[2]);
    uint16_t calculatedCRC = crc.getCRC();
    uint16_t receivedCRC = ((uint16_t)buf[3] << 8) | buf[4];

    if (debugFlag) {

      // *** PRINT CRC COMPARISON ***
      Serial.print("Calc CRC: 0x");
      if (calculatedCRC < 0x1000) Serial.print("0");
      if (calculatedCRC < 0x100) Serial.print("0");
      if (calculatedCRC < 0x10) Serial.print("0");
      Serial.print(calculatedCRC, HEX);
      
      Serial.print(" | Recv CRC: 0x");
      if (receivedCRC < 0x1000) Serial.print("0");
      if (receivedCRC < 0x100) Serial.print("0");
      if (receivedCRC < 0x10) Serial.print("0");
      Serial.println(receivedCRC, HEX);

    }

    if (calculatedCRC == receivedCRC) {
      if (debugFlag) {
      Serial.println("✓✓✓ CRC MATCH - Sending response!");
      }
      lastValidCommand = millis(); // Update last valid command time
      
      // Only respond to live data requests (ignore all other commands)
      if (buf[2] == CODE_LIVE_DATA || buf[2] != CODE_LIVE_DATA) {
        // Always send live data regardless of command
        sendLiveData();
      }
    } else if (debugFlag) {
      Serial.println("CRC mismatch");
    }
  }

}


void readSensors() {
  // Read encoders
  mux.selectChannel(ENCODER_CHANNEL_MOTOR);
  motorAngle = (int16_t)(encoderMotor.getAngle() * 10.0);

  mux.selectChannel(ENCODER_CHANNEL_GEARBOX);
  gearboxAngle = (int16_t)(encoderGearbox.getAngle() * 10.0);

  // Read limit switches with debouncing
  limitSwitch1 = debounceRead(LIMIT_SWITCH_PIN1);
  limitSwitch2 = debounceRead(LIMIT_SWITCH_PIN2);
}

bool debounceRead(uint8_t pin) {
  int index = (pin == LIMIT_SWITCH_PIN1) ? 0 : 1;
  bool currentState = digitalRead(pin);
  
  if (currentState != lastStates[index] && (millis() - lastChangeTimes[index]) > 20) {
    lastChangeTimes[index] = millis();
    lastStates[index] = currentState;
  }
  return lastStates[index];
}

void sendLiveData() {
  message[0] = 0x02;  // STX
  message[1] = JOINT_ID;
  message[2] = CODE_LIVE_DATA;

  // Motor angle (bytes 3-4)
  message[3] = (motorAngle >> 8) & 0xFF;
  message[4] = motorAngle & 0xFF;

  // Gearbox angle (bytes 5-6)
  message[5] = (gearboxAngle >> 8) & 0xFF;
  message[6] = gearboxAngle & 0xFF;

  // Limit switches (byte 7)
  message[7] = (limitSwitch1 ? 0x01 : 0x00) | (limitSwitch2 ? 0x02 : 0x00);

  // System ready flag (byte 8)
  message[8] = systemReady ? 0x01 : 0x00;

  // Reserved byte
  message[9] = 0x00;

  // Calculate CRC
  crc.restart();
  for (int i = 0; i <= 9; i++) {
    crc.add(message[i]);
  }
  uint16_t calcCRC = crc.getCRC();

  message[10] = (calcCRC >> 8) & 0xFF;
  message[11] = calcCRC & 0xFF;
  message[12] = 0x03;  // ETX

  // Send message
  digitalWrite(RS485_DE_PIN, HIGH);
  delay(1);
  rs485.write(message, 13);
  rs485.flush();
  delay(1);
  digitalWrite(RS485_DE_PIN, LOW);

  if (debugFlag) {
    Serial.println("Live data sent");
  }
}

void printLiveData() {
  // Calculate slippage (assuming 50:1 gear ratio)
  float expectedGearbox = (motorAngle / 10.0) / 50.0;
  float actualGearbox = gearboxAngle / 10.0;
  float slippage = actualGearbox - expectedGearbox;

  // Print timestamp
  Serial.print("[");
  Serial.print(millis() / 1000);
  Serial.print("s] Joint ");
  Serial.print(JOINT_ID);
  
  // Print encoder data
  Serial.print(" | Motor: ");
  Serial.print(motorAngle / 10.0, 1);
  Serial.print("° | Gearbox: ");
  Serial.print(actualGearbox, 1);
  Serial.print("°");
  
  // Print limit switches
  Serial.print(" | L1: ");
  Serial.print(limitSwitch1 ? "ON" : "off");
  Serial.print(" | L2: ");
  Serial.print(limitSwitch2 ? "ON" : "off");
  
  // Print system status
  Serial.print(" | Status: ");
  Serial.print(systemReady ? "READY" : "NOT_READY");
  
  // Print calculated slippage
  Serial.print(" | Slippage: ");
  Serial.print(slippage, 2);
  Serial.print("°");

  // Alert indicators
  if (limitSwitch1 || limitSwitch2) {
    Serial.print(" 🚨 LIMIT!");
  }
  
  if (!systemReady) {
    Serial.print(" ⚠️ NOT_READY");
  }

  Serial.println();
}
