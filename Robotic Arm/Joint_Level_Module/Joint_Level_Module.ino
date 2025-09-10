#include <Wire.h>
#include <TCA9548.h>          // RobTillaart's TCA9548 library
#include <Adafruit_AS5600.h>  // Adafruit's AS5600 library
#include <SoftwareSerial.h>
#include <CRC16.h>            // RobTillaart's CRC library

// Debug flag (1 = enable debug prints, 0 = disable)
const uint8_t debugFlag = 0;

// Define constants for modularity
const uint8_t JOINT_ID = 1;  // Change this for each joint (1-5)
const uint8_t MUX_ADDR = 0x70;  // Default I2C address for TCA9548
const uint8_t ENCODER_CHANNEL_MOTOR = 2;  // SD2/SC2 for motor encoder
const uint8_t ENCODER_CHANNEL_GEARBOX = 7;  // SD7/SC7 for gearbox encoder
const int LIMIT_SWITCH_PIN1 = 2;  // Interrupt-capable pin for switch 1
const int LIMIT_SWITCH_PIN2 = 3;  // Interrupt-capable pin for switch 2
const uint8_t RS485_TX_PIN = 8;
const uint8_t RS485_RX_PIN = 9;
const uint8_t RS485_DE_PIN = 10;  // DE/RE tied together

// Gearbox ratio for slippage calculation
const float GEAR_RATIO = 50.0;

// Threshold for slippage detection (in degrees)
const float SLIPPAGE_THRESHOLD = 0.5;

// Bus idle threshold (ms)
const unsigned long BUS_IDLE_THRESHOLD = 20;  // ms of silence before transmit

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
int16_t slippage = 0;
bool limitSwitch1 = false;
bool limitSwitch2 = false;
bool emergencyFlag = false;

// Message buffer (12 bytes: [STX][ID][CMD][Motor(2)][Gearbox(2)][Slippage(2)][Switches(1)][CRC(2)][ETX])
uint8_t message[12];

// Function prototypes
void readEncoders();
void calculateSlippage();
void checkLimitSwitches();
void sendData();
void handleEmergency();
void printMonitoringData();
void isrLimit1();
void isrLimit2();
bool isBusIdle(unsigned long threshold);
bool debounceRead();

void setup() {
  Serial.begin(115200);
  Serial.println("Joint " + String(JOINT_ID) + " initializing...");

  Wire.begin();

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

  pinMode(LIMIT_SWITCH_PIN1, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_PIN2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_PIN1), isrLimit1, RISING);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_PIN2), isrLimit2, RISING);

  rs485.begin(9600);
  pinMode(RS485_DE_PIN, OUTPUT);
  digitalWrite(RS485_DE_PIN, LOW);

  readEncoders();
  calculateSlippage();
  checkLimitSwitches();

  Serial.println("Initialization complete.");
}

void loop() {
  if (rs485.available()) {
    if (debugFlag) Serial.println("RS485 Available...");
    uint8_t buf[5];  // Poll message: [STX][JOINT_ID][CMD][CRC(2)]
    rs485.readBytes(buf, 5);
    if (debugFlag) {
      for (int i = 0; i < 5; i++) {
        Serial.print(buf[i], HEX);
        Serial.print(" : ");
        Serial.println(" (HEX)");
      }
    }

    if (buf[0] == 0x02 && buf[1] == JOINT_ID && buf[2] == 0x01)
    {  // Poll CMD = 0x01
      Serial.println("ID Match! Sending data...");
      crc.restart();
      crc.add(buf[0]);
      crc.add(buf[1]);
      crc.add(buf[2]);
      uint16_t calculatedCRC = crc.getCRC();
      if (debugFlag) {
        Serial.print("calculatedCRC: ");
        Serial.println(calculatedCRC);
      }

      uint16_t receivedCRC = ((uint16_t)buf[3] << 8) | buf[4];
      if (debugFlag) {
        Serial.print("receivedCRC: ");
        Serial.println(receivedCRC);
      }

      if (receivedCRC == calculatedCRC) 
      {
        sendData();
      } else {
        Serial.println("CRC mismatch on poll");
      }
    }
    else if (debugFlag)
    {
      Serial.println(".");
    }
  }

  // Periodic safety check and monitoring (every 50ms)
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 50) 
  {
    readEncoders();
    calculateSlippage();
    checkLimitSwitches();
    printMonitoringData();  // Added for continuous monitoring
    lastCheck = millis();
  }
}

// Read encoders and convert to int16 (tenths of degrees)
void readEncoders() {
  mux.selectChannel(ENCODER_CHANNEL_MOTOR);
  motorAngle = (int16_t)(encoderMotor.getAngle() * 10.0);

  mux.selectChannel(ENCODER_CHANNEL_GEARBOX);
  gearboxAngle = (int16_t)(encoderGearbox.getAngle() * 10.0);
}

// Calculate slippage (in tenths of degrees)
void calculateSlippage() {
  int16_t expectedGearbox = motorAngle / GEAR_RATIO;
  slippage = gearboxAngle - expectedGearbox;
}


// Check limit switches (adjusted for NC: triggered when HIGH)
void checkLimitSwitches() 
{
  bool debounced_limit1 = debounceRead(LIMIT_SWITCH_PIN1);
  bool debounced_limit2 = debounceRead(LIMIT_SWITCH_PIN2);

  // Update ISR flags to follow debounced physical state
  limitSwitch1 = debounced_limit1;
  limitSwitch2 = debounced_limit2;

  if (debounced_limit1 || debounced_limit2)
  {
    if (debugFlag) Serial.println("Limit Switch Triggered!");
    if (!emergencyFlag) {         // Call emergency handler only on state change
      emergencyFlag = true;
      handleEmergency();
    }
  }
  else
  {
    emergencyFlag = false;
  }
}



bool debounceRead(uint8_t pin) {
  static uint32_t lastChange = 0;
  static bool lastState = LOW;
  bool currentState = digitalRead(pin);
  if (currentState != lastState && (millis() - lastChange) > 20) {
    lastChange = millis();
    lastState = currentState;
  }
  return lastState;
}

// Print monitoring data in single line
void printMonitoringData() {
  Serial.print("Motor Encoder: ");
  Serial.print(motorAngle / 10.0);
  Serial.print(" Degree ||| Gearbox Encoder: ");
  Serial.print(gearboxAngle / 10.0);
  Serial.print(" Degree ||| Limit Switch 1: ");
  Serial.print(limitSwitch1 ? 1 : 0);
  Serial.print(" ||| Limit Switch 2: ");
  Serial.print(limitSwitch2 ? 1 : 0);
  Serial.print(" ||| Slippage: ");
  Serial.println(slippage / 10.0);
}

// Send data packet
void sendData() {

  if (debugFlag) Serial.println("1 Inside Send data");

  message[0] = 0x02;  // STX (kept as in your original)
  message[1] = JOINT_ID;
  message[2] = 0x01;  // Response CMD
  message[3] = (motorAngle >> 8) & 0xFF;
  message[4] = motorAngle & 0xFF;
  message[5] = (gearboxAngle >> 8) & 0xFF;
  message[6] = gearboxAngle & 0xFF;
  message[7] = (slippage >> 8) & 0xFF;
  message[8] = slippage & 0xFF;
  message[9] = (limitSwitch1 ? 0x01 : 0x00) | (limitSwitch2 ? 0x02 : 0x00);
  crc.restart();

  if (debugFlag) Serial.println("2 After message");

  for (int i = 0; i < 10; i++) 
  {
    crc.add(message[i]);
  }

  if (debugFlag) Serial.println("3 CRC Added");

  uint16_t calcCRC = crc.getCRC();
  message[10] = (calcCRC >> 8) & 0xFF;
  message[11] = calcCRC & 0xFF;

  if (debugFlag) {
    Serial.println("DATA as sent through RS485:");
    for (int i = 0; i < 12; i++) 
    {
      Serial.print(message[i], HEX);
      Serial.print(" : ");
      Serial.println(" (HEX)");
    }
  }

  // Toggle to transmit mode
  digitalWrite(RS485_DE_PIN, HIGH);
  delay(1);
  rs485.write(message, 12);
  rs485.flush();
  delay(1);
  digitalWrite(RS485_DE_PIN, LOW);
  Serial.println("DATA Sent.");
}

// Handle emergency (transmit 5 times for reliability)
void handleEmergency() {
  
  message[0] = 0x02;
  message[1] = JOINT_ID;
  message[2] = 0x02;  // Emergency CMD

  // Hard-coded "1" for motor, gearbox, slippage (16-bit values)
  message[3] = 0x00;  // Motor MSB
  message[4] = 0x01;  // Motor LSB
  message[5] = 0x00;  // Gearbox MSB
  message[6] = 0x01;  // Gearbox LSB
  message[7] = 0x00;  // Slippage MSB
  message[8] = 0x01;  // Slippage LSB

  // Impossible case: both limit switches ON = 0x03
  message[9] = 0x03;

  // CRC over first 10 bytes
  crc.restart();
  for (int i = 0; i < 10; i++) {
    crc.add(message[i]);
  }
  uint16_t calcCRC = crc.getCRC();
  message[10] = (calcCRC >> 8) & 0xFF;
  message[11] = calcCRC & 0xFF;

  if (debugFlag) {
    Serial.print("Emergency packet: ");
    for (int i = 0; i < 12; i++) {
      Serial.print(message[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }

  // Transmit 5 times with bus idle check and retries
  for (int attempt = 0; attempt < 5; attempt++) {
    int retries = 0;
    while (retries < 3 && !isBusIdle(BUS_IDLE_THRESHOLD)) {
      retries++;
      delay(20);  // Exponential backoff
    }
    if (retries < 3) {
      digitalWrite(RS485_DE_PIN, HIGH);
      delay(1);
      rs485.write(message, 12);
      rs485.flush();
      digitalWrite(RS485_DE_PIN, LOW);
      delay(1);  // Longer gap so master can parse
    } else if (debugFlag) {
      Serial.println("Bus busy, emergency skipped on attempt " + String(attempt));
    }
  }

  Serial.println("Emergency triggered! Sent 5 times.");
  emergencyFlag = false;  // Reset
}

// Bus idle check
bool isBusIdle(unsigned long threshold) {
  unsigned long start = millis();
  while (millis() - start < threshold) {
    if (rs485.available()) {
      return false;  // Data detected, bus busy
    }
  }
  return true;  // No data, bus idle
}

// ISRs for limits
void isrLimit1() {
  limitSwitch1 = true;
  emergencyFlag = true;
}

void isrLimit2() {
  limitSwitch2 = true;
  emergencyFlag = true;
}
