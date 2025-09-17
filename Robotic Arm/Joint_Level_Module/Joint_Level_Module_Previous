#include <Wire.h>
#include <TCA9548.h>          // RobTillaart's TCA9548 library
#include <Adafruit_AS5600.h>  // Adafruit's AS5600 library
#include <SoftwareSerial.h>
#include <CRC16.h>            // RobTillaart's CRC library
#include <avr/wdt.h>  // For watchdog timer

// Debug flag (1 = enable debug prints, 0 = disable)
const uint8_t debugFlag = 0;

// Add to global variables
unsigned long lastValidCommand = 0;
const unsigned long COMMAND_TIMEOUT = 5000; // 5 seconds

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

// Message buffer (13 bytes: [STX][ID][CODE][Data(7)][CRC(2)][ETX])
uint8_t message[13];

// Codes for commands
#define CODE_STARTUP_STATUS   0x05
#define CODE_LIVE_DATA        0x06
#define CODE_SELF_DIAGNOSTICS 0x08
#define CODE_ERROR_REPORT     0x09
#define CODE_ALARM_RESET      0x0A
#define CODE_SENSOR_RESET     0x0B
#define CODE_EMERGENCY        0x0C
// Add other codes as needed

// Arrays for debounce states for each limit switch pin
uint32_t lastChangeTimes[2] = {0, 0};
bool lastStates[2] = {LOW, LOW};


// Define diagnostics buffer size (adjust as needed)
#define DIAG_BUFFER_SIZE 10
// Buffer arrays
int16_t diagMotorAngle[DIAG_BUFFER_SIZE];
int16_t diagGearboxAngle[DIAG_BUFFER_SIZE];
int16_t diagSlippage[DIAG_BUFFER_SIZE];
bool diagLimitSwitch1[DIAG_BUFFER_SIZE];
bool diagLimitSwitch2[DIAG_BUFFER_SIZE];
uint8_t diagIndex = 0;
bool diagnosticsActive = false;
unsigned long lastSampleTime = 0;
const unsigned long DIAG_SAMPLE_INTERVAL = 10; // ms between samples

// Diagnostics status bytes for each sensor/status
uint8_t motorEncoderStatus = 0x00;  // 0=OK, 1=Fault
uint8_t gearboxEncoderStatus = 0x00;
uint8_t limitSwitch1Status = 0x00;
uint8_t limitSwitch2Status = 0x00;
uint8_t slippageStatus = 0x00;


// Global variables for startup status tracking
bool systemInitialized = false;
bool encodersInitialized = false;
bool limitSwitchesInitialized = false;
bool communicationInitialized = false;
bool calibrationComplete = false;

// Startup status flags
uint8_t startupStatusFlags = 0x00;


// Emergency broadcasting variables
bool emergencyActive = false;
bool emergencyFlagActive = false;
bool emergencyAcknowledged = false;

bool motorEncoderEmergency = false;
bool gearboxEncoderEmergency = false;
bool limitSwitch1Emergency = false;
bool limitSwitch2Emergency = false;
bool slippageEmergency = false;
bool mechanicalBindingEmergency = false;
bool systemFaultEmergency = false;


uint8_t emergencyType = 0x00;
unsigned long emergencyTimestamp = 0;

// Emergency type definitions
#define EMERGENCY_LIMIT_SWITCH_1    0x01
#define EMERGENCY_LIMIT_SWITCH_2    0x02  
#define EMERGENCY_BOTH_LIMITS       0x03
#define EMERGENCY_MECHANICAL_BIND   0x04
#define EMERGENCY_SENSOR_FAILURE    0x08
#define EMERGENCY_ENCODER_FAULT     0x10
#define EMERGENCY_COMMUNICATION     0x20
#define EMERGENCY_SLIPPAGE_CRITICAL 0x40
#define EMERGENCY_SYSTEM_FAULT      0x80


// Global error tracking variables
uint16_t currentErrorCode1 = 0x0000;
uint16_t currentErrorCode2 = 0x0000;

// Error code definitions
#define ERROR_ENCODER_MOTOR_FAIL    0x0001
#define ERROR_ENCODER_GEARBOX_FAIL  0x0002
#define ERROR_LIMIT_SWITCH_1_FAIL   0x0004
#define ERROR_LIMIT_SWITCH_2_FAIL   0x0008
#define ERROR_SLIPPAGE_HIGH         0x0010
#define ERROR_COMMUNICATION_TIMEOUT 0x0020
#define ERROR_INITIALIZATION_FAIL   0x0040
#define ERROR_ENCODER_OUT_OF_RANGE  0x0080


// Function prototypes
void readEncoders();
void calculateSlippage();
void checkLimitSwitches();

void processCommand(uint8_t code);

void sendStartupStatus();
void sendLiveData();
// void sendProcessedData();
void sendSelfDiagnostics();

void sendAlarmReset();
void sendSensorReset();
void sendEmergency();

void printMonitoringData();

// void isrLimit1();
// void isrLimit2();

bool isBusIdle(unsigned long threshold);
bool debounceRead(uint8_t pin);

void performStartupChecks() ;
void calculateStartupFlags();
void sendStartupStatus();

void detectErrors();
void sendErrorReport();

void triggerEmergency(uint8_t emergencyTypeFlag);
void checkEmergencyConditions();
void startSelfDiagnostics();
void handleSelfDiagnostics();

void clearAllEmergencyFlags();

void attemptSystemRecovery();

void setup() 
{
  wdt_disable(); // Disable watchdog during setup
  Serial.begin(115200);
  Serial.println("Joint " + String(JOINT_ID) + " initializing...");

  Wire.begin();
  lastValidCommand = millis(); // Initialize to current time

  clearAllEmergencyFlags(); // Clearing or Setting all emergency flags to zero
  // Perform comprehensive startup checks
  performStartupChecks();

  // attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_PIN1), isrLimit1, RISING);
  // attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_PIN2), isrLimit2, RISING);

  rs485.begin(38400);
  pinMode(RS485_DE_PIN, OUTPUT);
  digitalWrite(RS485_DE_PIN, LOW);

  readEncoders();
  calculateSlippage();
  checkLimitSwitches();

  wdt_enable(WDTO_8S); // Enable 8-second watchdog

  Serial.println("Initialization complete.");
}



void loop() 
{

  wdt_reset(); // Reset watchdog timer

  // Check for communication timeout - improved logic
  static bool communicationEmergencyTriggered = false;
  if (millis() - lastValidCommand > COMMAND_TIMEOUT) 
  {
    if (!emergencyFlagActive && systemInitialized && !communicationEmergencyTriggered) 
    {
      triggerEmergency(EMERGENCY_COMMUNICATION);
      communicationEmergencyTriggered = true;  // Prevent spam
    }
  } else {
    communicationEmergencyTriggered = false;  // Reset flag when communication resumes
  }

  
  if (rs485.available()) 
  {
    if (debugFlag) Serial.println("RS485 Available...");
    uint8_t buf[6];  // Poll message: [STX][JOINT_ID][CODE][CRC(2)][ETX or omitted]

    rs485.readBytes(buf, 6);

    if (debugFlag) 
    {
      for (int i = 0; i < 6; i++) 
      {
        Serial.print(buf[i], HEX);
        Serial.print(" : ");
        Serial.println(" (HEX)");
      }
    }

    if (buf[0] == 0x02 && buf[1] == JOINT_ID) {  // Validate STX and ID
      crc.restart();
      crc.add(buf[0]);
      crc.add(buf[1]);
      crc.add(buf[2]);  // Code
      uint16_t calculatedCRC = crc.getCRC();
      uint16_t receivedCRC = ((uint16_t)buf[3] << 8) | buf[4];

      if (debugFlag) {
        Serial.print("calculatedCRC: ");
        Serial.println(calculatedCRC);
        Serial.print("receivedCRC: ");
        Serial.println(receivedCRC);
      }

      if (calculatedCRC == receivedCRC) 
      {
        if (debugFlag) Serial.println("CRC OK, processing command");
        processCommand(buf[2]);  // Dispatch based on code byte
      } 
      else 
      {
        Serial.println("CRC mismatch on poll");
      }
    } 
    else if (debugFlag) 
    {
      Serial.println(".");
    }
  }

  handleSelfDiagnostics();

  // Periodic safety check and monitoring (every 50ms)
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 50) 
  {
    readEncoders();
    calculateSlippage();
    checkLimitSwitches();

    // Check for emergency conditions
    checkEmergencyConditions();

    printMonitoringData();  // Added for continuous monitoring
    lastCheck = millis();
  }


}



void processCommand(uint8_t code) 
{

   lastValidCommand = millis(); // Update timestamp on valid command

  switch (code) 
  {
    case CODE_STARTUP_STATUS:
      sendStartupStatus();
      break;

    case CODE_LIVE_DATA:
      sendLiveData();
      break;

    case CODE_SELF_DIAGNOSTICS:
      startSelfDiagnostics();
      break;

    case CODE_ERROR_REPORT:
      sendErrorReport();
      break;

    case CODE_ALARM_RESET:
      sendAlarmReset();
      break;

    case CODE_SENSOR_RESET:
      sendSensorReset();
      break;

    case CODE_EMERGENCY:
      sendEmergency();
      break;

    default:
      // Unknown command: fallback to sending live data
      sendLiveData();
      break;
  }
}

void sendStartupStatus() 
{
  // Calculate startup status flags byte
  calculateStartupFlags();
  
  message[0] = 0x02;  // STX
  message[1] = JOINT_ID;
  message[2] = CODE_STARTUP_STATUS;

  // Byte 3: Overall startup status flags
  message[3] = startupStatusFlags;
  
  // Bytes 4-5: Current motor encoder position (for position verification)
  message[4] = (motorAngle >> 8) & 0xFF;
  message[5] = motorAngle & 0xFF;
  
  // Bytes 6-7: Current gearbox encoder position (for position verification)
  message[6] = (gearboxAngle >> 8) & 0xFF;
  message[7] = gearboxAngle & 0xFF;
  
  // Byte 8: Current limit switch states
  message[8] = (limitSwitch1 ? 0x01 : 0x00) | (limitSwitch2 ? 0x02 : 0x00);
  
  // Byte 9: Reserved for additional status or error codes
  message[9] = 0x00;

  // Calculate CRC and send message
  crc.restart();
  for (int i = 0; i <= 9; i++) {
    crc.add(message[i]);
  }
  uint16_t calcCRC = crc.getCRC();

  message[10] = (calcCRC >> 8) & 0xFF;
  message[11] = calcCRC & 0xFF;
  message[12] = 0x03;  // ETX

  // Send via RS485
  digitalWrite(RS485_DE_PIN, HIGH);
  delay(1);
  rs485.write(message, 13);
  rs485.flush();
  delay(1);
  digitalWrite(RS485_DE_PIN, LOW);

  if (debugFlag) {
    Serial.println("Startup status sent:");
    Serial.print("Status Flags: 0x"); Serial.println(startupStatusFlags, HEX);
    Serial.print("Motor Angle: "); Serial.println(motorAngle / 10.0);
    Serial.print("Gearbox Angle: "); Serial.println(gearboxAngle / 10.0);
    Serial.print("Limit Switches: "); Serial.println(message[8], BIN);
  }
}


void calculateStartupFlags() 
{
  startupStatusFlags = 0x00;
  
  // Bit 0: Overall system ready (1 = ready, 0 = not ready)
  if (systemInitialized && encodersInitialized && limitSwitchesInitialized && communicationInitialized) {
    startupStatusFlags |= 0x01;
  }
  
  // Bit 1: Encoders initialized successfully
  if (encodersInitialized) {
    startupStatusFlags |= 0x02;
  }
  
  // Bit 2: Limit switches initialized
  if (limitSwitchesInitialized) {
    startupStatusFlags |= 0x04;
  }
  
  // Bit 3: Communication established
  if (communicationInitialized) {
    startupStatusFlags |= 0x08;
  }
  
  // Bit 4: Calibration/homing complete
  if (calibrationComplete) {
    startupStatusFlags |= 0x10;
  }
  
  // Bit 5: Critical fault detected (1 = fault exists)
  // Check for any critical initialization failures
  if (!encodersInitialized || !limitSwitchesInitialized) {
    startupStatusFlags |= 0x20;
  }
  
  // Bits 6-7: Reserved for future use
}


void performStartupChecks() 
{
  if (!mux.begin()) 
  {
    Serial.println("Mux not found. Check connections.");
    while (1);
  }

  mux.selectChannel(ENCODER_CHANNEL_MOTOR);
  if (!encoderMotor.begin()) {
    Serial.println("Motor encoder not found.");
    encodersInitialized = false;  // Explicitly set to false
    while (1);
  }
  
  mux.selectChannel(ENCODER_CHANNEL_GEARBOX);
  if (!encoderGearbox.begin()) {
    Serial.println("Gearbox encoder not found.");
    encodersInitialized = false;  // Explicitly set to false
    while (1);
  }

  // ADD THIS LINE - was missing!
  encodersInitialized = true;
  if (debugFlag) Serial.println("Encoders initialized successfully");

  // Check limit switch initialization
  pinMode(LIMIT_SWITCH_PIN1, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_PIN2, INPUT_PULLUP);

  // Simple connectivity test - read initial states
  digitalRead(LIMIT_SWITCH_PIN1);
  digitalRead(LIMIT_SWITCH_PIN2);
  limitSwitchesInitialized = true;
  if (debugFlag) Serial.println("Limit switches initialized");
  
  // Communication is initialized if we reach this point
  communicationInitialized = true;
  if (debugFlag) Serial.println("Communication initialized");
  
  // Perform basic calibration - read initial encoder positions
  if (encodersInitialized) {
    readEncoders();
    calculateSlippage();
    calibrationComplete = true;
    if (debugFlag) Serial.println("Basic calibration complete");
  }
  
  // Set overall system status
  systemInitialized = (encodersInitialized && limitSwitchesInitialized && communicationInitialized);
  
  if (debugFlag) {
    Serial.print("System initialization complete. Status: ");
    Serial.println(systemInitialized ? "READY" : "FAULT");
  }
}



void sendLiveData() 
{
  message[0] = 0x02;  // STX
  message[1] = JOINT_ID;
  message[2] = CODE_LIVE_DATA;

  message[3] = (motorAngle >> 8) & 0xFF;
  message[4] = motorAngle & 0xFF;

  message[5] = (gearboxAngle >> 8) & 0xFF;
  message[6] = gearboxAngle & 0xFF;

  message[7] = (slippage >> 8) & 0xFF;
  message[8] = slippage & 0xFF;

  // ENHANCED Byte 9: Status flags
  message[9] = 0x00;
  
  // Bits 0-1: Limit switch states (existing)
  message[9] |= (limitSwitch1 ? 0x01 : 0x00);
  message[9] |= (limitSwitch2 ? 0x02 : 0x00);
  
  // DEBUG: Print flag status
  Serial.print("DEBUG: emergencyFlagActive = ");
  Serial.println(emergencyFlagActive ? "true" : "false");
  
  // Bit 2: Emergency state flag (NEW!)
  message[9] |= (emergencyFlagActive ? 0x04 : 0x00);
  
  // Bit 3: System initialized flag (BONUS - useful info)
  message[9] |= (systemInitialized ? 0x08 : 0x00);
  
  // Bit 4: Diagnostics active flag (BONUS)
  message[9] |= (diagnosticsActive ? 0x10 : 0x00);

  
  
  // Bits 5-7: Reserved for future use
  // message[9] |= (futureFlag ? 0x20 : 0x00);  // Bit 5
  // message[9] |= (futureFlag ? 0x40 : 0x00);  // Bit 6  
  // message[9] |= (futureFlag ? 0x80 : 0x00);  // Bit 7

  crc.restart();
  for (int i = 0; i <= 9; i++) {
    crc.add(message[i]);
  }
  uint16_t calcCRC = crc.getCRC();

  message[10] = (calcCRC >> 8) & 0xFF;
  message[11] = calcCRC & 0xFF;

  message[12] = 0x03;  // ETX

  digitalWrite(RS485_DE_PIN, HIGH);
  delay(1);
  rs485.write(message, 13);
  rs485.flush();
  delay(1);
  digitalWrite(RS485_DE_PIN, LOW);

  if (debugFlag) {
    Serial.println("Live data sent:");
    for (int i = 0; i < 13; i++) {
      Serial.print(message[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}


void clearAllEmergencyFlags() 
{
  emergencyFlagActive = false;  // This will make Bit 2 = 0 in live data
  emergencyActive = false;
  emergencyAcknowledged = true;
  
  // Clear all specific emergency flags
  motorEncoderEmergency = false;
  gearboxEncoderEmergency = false;
  limitSwitch1Emergency = false;
  limitSwitch2Emergency = false;
  slippageEmergency = false;
  mechanicalBindingEmergency = false;
  systemFaultEmergency = false;
  
  emergencyFlag = false;
  emergencyType = 0x00;
  
  if (debugFlag) 
  {
    Serial.println("All emergency flags cleared - Live data emergency bit will be 0");
  }
}


/////////////////////////

// Call this to start diagnostic data collection (e.g., on receiving the diag command)
void startSelfDiagnostics() 
{
  diagIndex = 0;
  diagnosticsActive = true;
  motorEncoderStatus = 0x00;
  gearboxEncoderStatus = 0x00;
  limitSwitch1Status = 0x00;
  limitSwitch2Status = 0x00;
  slippageStatus = 0x00;
  lastSampleTime = millis();
  if (debugFlag) Serial.println("Self diagnostics started. Collecting data...");
}

// Call this continuously from loop to collect and process diagnostic data
void handleSelfDiagnostics() 
{
  if (!diagnosticsActive) return;
  
  unsigned long now = millis();
  if (now - lastSampleTime >= DIAG_SAMPLE_INTERVAL) {
    readEncoders();
    calculateSlippage();
    checkLimitSwitches();
    
    if (diagIndex < DIAG_BUFFER_SIZE) {
      diagMotorAngle[diagIndex] = motorAngle;
      diagGearboxAngle[diagIndex] = gearboxAngle;
      diagSlippage[diagIndex] = slippage;
      diagLimitSwitch1[diagIndex] = limitSwitch1;
      diagLimitSwitch2[diagIndex] = limitSwitch2;
      diagIndex++;
    } else {
      // ADD THIS: Buffer overflow protection
      diagnosticsActive = false;
      Serial.println("Diagnostics buffer full - stopping collection");
      sendSelfDiagnostics();
      return;
    }
    lastSampleTime = now;
  }

  // Detect if motion stopped: simple logic checking jitter in last 10 samples
  if (diagIndex > 10) {
    bool motionStopped = true;
    for (int i = diagIndex - 10; i < diagIndex - 1; i++) {
      if (abs(diagMotorAngle[i + 1] - diagMotorAngle[i]) > 50) {  // 5 degrees jitter allowed
        motionStopped = false;
        break;
      }
    }
    if (motionStopped) {
      diagnosticsActive = false;
      if (debugFlag) Serial.println("Motion stopped, processing diagnostics...");

      // Check motor encoder validity
      for (int i = 0; i < diagIndex; i++) {
        if (diagMotorAngle[i] < 0 || diagMotorAngle[i] > 40960) {
          motorEncoderStatus = 1;  // Fault
          break;
        }
      }

      // Check gearbox encoder validity
      for (int i = 0; i < diagIndex; i++) {
        if (diagGearboxAngle[i] < 0 || diagGearboxAngle[i] > 40960) {
          gearboxEncoderStatus = 1;  // Fault
          break;
        }
      }

      // Check slippage threshold exceeded anywhere
      for (int i = 0; i < diagIndex; i++) {
        if (abs(diagSlippage[i]) > SLIPPAGE_THRESHOLD * 10) {
          slippageStatus = 1;  // Fault
          break;
        }
      }

      // Check limit switches triggered at least once during motion (otherwise fault)
      bool ls1Good = false;
      bool ls2Good = false;
      for (int i = 0; i < diagIndex; i++) {
        if (diagLimitSwitch1[i]) ls1Good = true;
        if (diagLimitSwitch2[i]) ls2Good = true;
      }
      limitSwitch1Status = ls1Good ? 0 : 1;
      limitSwitch2Status = ls2Good ? 0 : 1;

      sendSelfDiagnostics();
    }
  }
}

//////////////////////////////

void sendSelfDiagnostics() 
{
  message[0] = 0x02;  // STX
  message[1] = JOINT_ID;
  message[2] = CODE_SELF_DIAGNOSTICS;

  // Use separate bytes for each sensor/status (0=OK, 1=Fault)
  message[3] = motorEncoderStatus;
  message[4] = gearboxEncoderStatus;
  message[5] = limitSwitch1Status;
  message[6] = limitSwitch2Status;
  message[7] = slippageStatus;

  // Fill remaining bytes with zeros
  message[8] = 0x00;
  message[9] = 0x00;

  crc.restart();
  for (int i = 0; i <= 9; i++) 
  {
    crc.add(message[i]);
  }
  uint16_t calcCRC = crc.getCRC();

  message[10] = (calcCRC >> 8) & 0xFF;
  message[11] = calcCRC & 0xFF;
  message[12] = 0x03;  // ETX

  digitalWrite(RS485_DE_PIN, HIGH);
  delay(1);
  rs485.write(message, 13);
  rs485.flush();
  delay(1);
  digitalWrite(RS485_DE_PIN, LOW);

  if (debugFlag) 
  {
    Serial.println("Self diagnostics sent:");
    Serial.print("Motor Encoder Status: "); Serial.println(message[3]);
    Serial.print("Gearbox Encoder Status: "); Serial.println(message[4]);
    Serial.print("Limit Switch 1 Status: "); Serial.println(message[5]);
    Serial.print("Limit Switch 2 Status: "); Serial.println(message[6]);
    Serial.print("Slippage Status: "); Serial.println(message[7]);
    for (int i = 0; i < 13; i++) 
    {
      Serial.print(message[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}

void attemptSystemRecovery() 
{
  Serial.println("Attempting system recovery...");
  
  // Re-initialize critical systems
  if (!encodersInitialized) {
    mux.selectChannel(ENCODER_CHANNEL_MOTOR);
    if (encoderMotor.begin()) {
      mux.selectChannel(ENCODER_CHANNEL_GEARBOX);
      if (encoderGearbox.begin()) {
        encodersInitialized = true;
        Serial.println("Encoders recovered");
      }
    }
  }
  
  // Update system status
  systemInitialized = (encodersInitialized && limitSwitchesInitialized && communicationInitialized);
}

void detectErrors() 
{
  currentErrorCode1 = 0x0000;
  currentErrorCode2 = 0x0000;
  
  // Check encoder initialization
  if (!encodersInitialized) {
    currentErrorCode1 |= ERROR_ENCODER_MOTOR_FAIL | ERROR_ENCODER_GEARBOX_FAIL;
    attemptSystemRecovery(); // ADD THIS
  }
  // Check encoder range
  if (motorAngle < -1000 || motorAngle > 50000) {
    currentErrorCode1 |= ERROR_ENCODER_MOTOR_FAIL;
  }
  
  if (gearboxAngle < -1000 || gearboxAngle > 50000) {
    currentErrorCode1 |= ERROR_ENCODER_GEARBOX_FAIL;
  }
  
  // Check excessive slippage
  if (abs(slippage) > (SLIPPAGE_THRESHOLD * 20)) {  // 2x threshold = error level
    currentErrorCode1 |= ERROR_SLIPPAGE_HIGH;
  }
  
  // Check initialization failures
  if (!systemInitialized) {
    currentErrorCode1 |= ERROR_INITIALIZATION_FAIL;
  }
  
  // Add timestamp or counter to errorCode2 if needed
  currentErrorCode2 = (uint16_t)(millis() / 1000);  // Time since startup in seconds
}

void sendErrorReport() {
  // Detect current errors before sending report
  detectErrors();
  
  message[0] = 0x02;  // STX
  message[1] = JOINT_ID;
  message[2] = CODE_ERROR_REPORT;

  // Error report format: Error Code 1 + Error Code 2 + Reserved
  message[3] = (currentErrorCode1 >> 8) & 0xFF;
  message[4] = currentErrorCode1 & 0xFF;
  message[5] = (currentErrorCode2 >> 8) & 0xFF;
  message[6] = currentErrorCode2 & 0xFF;
  message[7] = 0x00;  // Reserved
  message[8] = 0x00;  // Reserved
  message[9] = 0x00;  // Reserved

  crc.restart();
  for (int i = 0; i <= 9; i++) {
    crc.add(message[i]);
  }
  uint16_t calcCRC = crc.getCRC();

  message[10] = (calcCRC >> 8) & 0xFF;
  message[11] = calcCRC & 0xFF;
  message[12] = 0x03;  // ETX

  digitalWrite(RS485_DE_PIN, HIGH);
  delay(1);
  rs485.write(message, 13);
  rs485.flush();
  delay(1);
  digitalWrite(RS485_DE_PIN, LOW);

  if (debugFlag) {
    Serial.println("Error report sent:");
    Serial.print("Error Code 1: 0x"); Serial.println(currentErrorCode1, HEX);
    Serial.print("Error Code 2: 0x"); Serial.println(currentErrorCode2, HEX);
    for (int i = 0; i < 13; i++) {
      Serial.print(message[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}

void sendAlarmReset() 
{

// Capture emergency type BEFORE clearing
  uint8_t previousEmergencyType = emergencyType;

  // Clear ALL emergency flags FIRST
  clearAllEmergencyFlags();  // <-- CALL IT HERE

  message[0] = 0x02;  // STX
  message[1] = JOINT_ID;
  message[2] = CODE_ALARM_RESET;

  // Alarm reset format: ACK Flag + Reserved
  message[3] = 0x01;  // ACK Flag - 0x01 = Acknowledged
  message[4] = previousEmergencyType; 
  message[5] = 0x00;  // Reserved
  message[6] = 0x00;  // Reserved
  message[7] = 0x00;  // Reserved
  message[8] = 0x00;  // Reserved
  message[9] = 0x00;  // Reserved

  crc.restart();
  for (int i = 0; i <= 9; i++) {
    crc.add(message[i]);
  }
  uint16_t calcCRC = crc.getCRC();

  message[10] = (calcCRC >> 8) & 0xFF;
  message[11] = calcCRC & 0xFF;
  message[12] = 0x03;  // ETX

  digitalWrite(RS485_DE_PIN, HIGH);
  delay(1);
  rs485.write(message, 13);
  rs485.flush();
  delay(1);
  digitalWrite(RS485_DE_PIN, LOW);

  if (debugFlag) {
    Serial.println("Alarm reset completed - All emergency flags cleared!");
    Serial.print("Previously active emergency type: 0x");
    Serial.println(previousEmergencyType, HEX);
  }
}

void sendSensorReset() {
  message[0] = 0x02;  // STX
  message[1] = JOINT_ID;
  message[2] = CODE_SENSOR_RESET;

  // Sensor reset format: ACK Flag + Reserved
  message[3] = 0x01;  // ACK Flag - 0x01 = Acknowledged
  message[4] = 0x00;  // Reserved
  message[5] = 0x00;  // Reserved
  message[6] = 0x00;  // Reserved
  message[7] = 0x00;  // Reserved
  message[8] = 0x00;  // Reserved
  message[9] = 0x00;  // Reserved

  crc.restart();
  for (int i = 0; i <= 9; i++) {
    crc.add(message[i]);
  }
  uint16_t calcCRC = crc.getCRC();

  message[10] = (calcCRC >> 8) & 0xFF;
  message[11] = calcCRC & 0xFF;
  message[12] = 0x03;  // ETX

  digitalWrite(RS485_DE_PIN, HIGH);
  delay(1);
  rs485.write(message, 13);
  rs485.flush();
  delay(1);
  digitalWrite(RS485_DE_PIN, LOW);

  if (debugFlag) {
    Serial.println("Sensor reset sent:");
    for (int i = 0; i < 13; i++) {
      Serial.print(message[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}

void sendEmergency() {
  message[0] = 0x02;  // STX
  message[1] = JOINT_ID;
  message[2] = CODE_EMERGENCY;

  // Byte 3: Emergency type flags (what triggered the emergency)
  message[3] = emergencyType;
  
  // Bytes 4-5: Motor encoder snapshot at time of emergency
  message[4] = (motorAngle >> 8) & 0xFF;
  message[5] = motorAngle & 0xFF;
  
  // Bytes 6-7: Gearbox encoder snapshot at time of emergency
  message[6] = (gearboxAngle >> 8) & 0xFF;
  message[7] = gearboxAngle & 0xFF;
  
  // Bytes 8-9: Slippage value at time of emergency
  message[8] = (slippage >> 8) & 0xFF;
  message[9] = slippage & 0xFF;

  // Calculate CRC and send
  crc.restart();
  for (int i = 0; i <= 9; i++) {
    crc.add(message[i]);
  }
  uint16_t calcCRC = crc.getCRC();
  message[10] = (calcCRC >> 8) & 0xFF;
  message[11] = calcCRC & 0xFF;
  message[12] = 0x03;  // ETX

  // Simple single transmission
  digitalWrite(RS485_DE_PIN, HIGH);
  delay(1);
  rs485.write(message, 13);
  rs485.flush();
  delay(1);
  digitalWrite(RS485_DE_PIN, LOW);
}


// Trigger emergency based on different conditions
void triggerEmergency(uint8_t emergencyTypeFlag) 
{
  emergencyActive = true;
  emergencyFlagActive = true;
  emergencyType = emergencyTypeFlag;
  emergencyTimestamp = millis();
  
  // Fix the logic - use == for combined flags
  if (emergencyTypeFlag == EMERGENCY_LIMIT_SWITCH_1) {
    limitSwitch1Emergency = true;
    Serial.println("Limit Switch 1 Emg SET");
  }
  else if (emergencyTypeFlag == EMERGENCY_LIMIT_SWITCH_2) {
    limitSwitch2Emergency = true;
    Serial.println("Limit Switch 2 Emg SET");
  }
  else if (emergencyTypeFlag == EMERGENCY_BOTH_LIMITS) {
    limitSwitch1Emergency = true;
    limitSwitch2Emergency = true;
    mechanicalBindingEmergency = true;
  }
  else if (emergencyTypeFlag == EMERGENCY_ENCODER_FAULT) {
    motorEncoderEmergency = true;
    gearboxEncoderEmergency = true;
  }
  else if (emergencyTypeFlag == EMERGENCY_SLIPPAGE_CRITICAL) {
    slippageEmergency = true;
  }
  // ADD THIS CASE
  else if (emergencyTypeFlag == EMERGENCY_COMMUNICATION) {
    systemFaultEmergency = true;
  }
  
  // Capture current sensor state at time of emergency
  readEncoders();
  calculateSlippage();
  
  // Immediate emergency broadcast
  sendEmergency();
  
  if (debugFlag) {
    Serial.print("Emergency triggered and FLAGS SET: Type 0x");
    Serial.print(emergencyType, HEX);
    Serial.print(" at time: ");
    Serial.println(emergencyTimestamp);
  }
}


// Updated limit switch ISRs with emergency broadcasting
// void isrLimit1() 
// {
//   if (!emergencyFlagActive && !limitSwitch1Emergency) 
//   {  // CHANGED THIS LINE
//     triggerEmergency(EMERGENCY_LIMIT_SWITCH_1);
//   }
// }

// void isrLimit2() 
// {
//   if (!emergencyFlagActive && !limitSwitch2Emergency) 
//   {  // CHANGED THIS LINE
//     triggerEmergency(EMERGENCY_LIMIT_SWITCH_2);
//   }
// }


// Check for other emergency conditions (call this in main loop)
void checkEmergencyConditions() {
  static unsigned long lastEmergencyCheck = 0;
  
  if (millis() - lastEmergencyCheck > 20) {
    
    // Only check for new emergencies if not already in emergency state
    if (!emergencyFlagActive) {  // CHANGED THIS LINE
      
      // // Check for excessive slippage (critical level)
      // if (abs(slippage) > (SLIPPAGE_THRESHOLD * 30)) { // This is the original line, x 1000 for testing purpose only 
      // if (abs(slippage) > (SLIPPAGE_THRESHOLD * 1000)) {
      //   triggerEmergency(EMERGENCY_SLIPPAGE_CRITICAL);
      // }

      // Check for encoder faults
      if (motorAngle < -1000 || motorAngle > 50000 || gearboxAngle < -1000 || gearboxAngle > 50000) {
        triggerEmergency(EMERGENCY_ENCODER_FAULT);
      }
      // Check for both limit switches triggered
      else if (limitSwitch1 && limitSwitch2) {
        triggerEmergency(EMERGENCY_BOTH_LIMITS);
      }
    }
    
    lastEmergencyCheck = millis();
  }
}

void readEncoders() 
{
  mux.selectChannel(ENCODER_CHANNEL_MOTOR);
  motorAngle = (int16_t)(encoderMotor.getAngle() * 10.0);

  mux.selectChannel(ENCODER_CHANNEL_GEARBOX);
  gearboxAngle = (int16_t)(encoderGearbox.getAngle() * 10.0);
}

void calculateSlippage() 
{
  int16_t expectedGearbox = motorAngle / GEAR_RATIO;
  slippage = gearboxAngle - expectedGearbox;
}

void checkLimitSwitches() 
{
  bool debounced_limit1 = debounceRead(LIMIT_SWITCH_PIN1);
  bool debounced_limit2 = debounceRead(LIMIT_SWITCH_PIN2);

  limitSwitch1 = debounced_limit1;
  limitSwitch2 = debounced_limit2;

  // Trigger emergency WHILE switch is pressed (not just on edge)
  if (debounced_limit1) {  // As long as limit switch 1 is pressed
    triggerEmergency(EMERGENCY_LIMIT_SWITCH_1);
  }
  
  if (debounced_limit2) {  // As long as limit switch 2 is pressed  
    triggerEmergency(EMERGENCY_LIMIT_SWITCH_2);
  }
  
  // Both switches pressed = mechanical binding emergency
  if (debounced_limit1 && debounced_limit2) {
    triggerEmergency(EMERGENCY_BOTH_LIMITS);
  }
}


bool debounceRead(uint8_t pin) 
{
  int index = (pin == LIMIT_SWITCH_PIN1) ? 0 : 1; // Map pin to index

  bool currentState = digitalRead(pin);
  if (currentState != lastStates[index] && (millis() - lastChangeTimes[index]) > 20) {
    lastChangeTimes[index] = millis();
    lastStates[index] = currentState;
  }
  return lastStates[index];
}

void printMonitoringData() 
{
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

