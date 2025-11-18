#include <Wire.h>
#include <TCA9548.h>          // RobTillaart's TCA9548 library
#include <Adafruit_AS5600.h>  // Adafruit's AS5600 library
#include <SoftwareSerial.h>
#include <CRC16.h>            // RobTillaart's CRC library
#include <avr/wdt.h>          // For watchdog timer

// ============================================================
// CONFIGURATION
// ============================================================
const uint8_t debugFlag = 1;  // 1 = enable debug prints, 0 = disable

// Joint and hardware configuration
const uint8_t JOINT_ID = 1;              // Change this for each joint (1-5)
const uint8_t MUX_ADDR = 0x70;           // I2C address for TCA9548
const uint8_t ENCODER_CHANNEL_MOTOR = 2; // SD2/SC2 for motor encoder
const uint8_t ENCODER_CHANNEL_GEARBOX = 7; // SD7/SC7 for gearbox encoder
const int LIMIT_SWITCH_PIN1 = 2;
const int LIMIT_SWITCH_PIN2 = 3;
const uint8_t RS485_TX_PIN = 8;
const uint8_t RS485_RX_PIN = 9;
const uint8_t RS485_DE_PIN = 10;

// Gear ratio for slippage calculation
const float GEAR_RATIO = 50.0;

// Communication timeout
unsigned long lastValidCommand = 0;
const unsigned long COMMAND_TIMEOUT = 5000; // 5 seconds

// ============================================================
// HARDWARE INSTANCES
// ============================================================
SoftwareSerial rs485(RS485_RX_PIN, RS485_TX_PIN);
TCA9548 mux(MUX_ADDR);
Adafruit_AS5600 encoderMotor;
Adafruit_AS5600 encoderGearbox;
CRC16 crc;

// ============================================================
// DATA STORAGE (RAW COUNTS)
// ============================================================
uint16_t motorCount = 0;           // 0-4095 raw count
int16_t motorRotations = 0;        // Multi-turn tracking
uint16_t gearboxCount = 0;         // 0-4095 raw count
bool limitSwitch1 = false;
bool limitSwitch2 = false;

// Previous motor count for rotation tracking
uint16_t prevMotorCount = 0;

// ============================================================
// ERROR FLAGS (Status Byte)
// ============================================================
struct ErrorFlags {
  bool systemReady = false;           // Bit 0
  bool muxError = false;              // Bit 1
  bool motorEncoderError = false;     // Bit 2
  bool gearboxEncoderError = false;   // Bit 3
  bool motorMagnetError = false;      // Bit 4
  bool gearboxMagnetError = false;    // Bit 5
  bool commTimeout = false;           // Bit 6
  // Bit 7: Reserved
} errors;

// ============================================================
// MESSAGE BUFFER (13 bytes)
// ============================================================
uint8_t message[13];

// Message format:
// [0]    STX (0x02)
// [1]    ID (1-5)
// [2-3]  motor_count (uint16, 0-4095)
// [4-5]  motor_rotations (int16, signed)
// [6-7]  gearbox_count (uint16, 0-4095)
// [8]    switches (bit 0: L1, bit 1: L2)
// [9]    status (8 error flags)
// [10-11] CRC-16
// [12]   ETX (0x03)

// ============================================================
// DEBOUNCE
// ============================================================
uint32_t lastChangeTimes[2] = {0, 0};
bool lastStates[2] = {LOW, LOW};

// ============================================================
// FUNCTION DECLARATIONS
// ============================================================
void initHardware();
void readSensors();
void checkEncoderHealth();
void handleRS485Command();
void sendLiveData();
void printLiveData();
uint8_t buildStatusByte();
bool debounceRead(uint8_t pin);
uint16_t readRawCount(Adafruit_AS5600 &encoder);
void runCRCTest();

// ============================================================
// SETUP
// ============================================================
void setup() {
  wdt_disable(); // Disable watchdog during setup
  Serial.begin(115200);
  
  if (debugFlag) {
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║  Joint Module - Raw Count Protocol    ║");
    Serial.println("╚════════════════════════════════════════╝");
    Serial.print("Joint ID: ");
    Serial.println(JOINT_ID);
    runCRCTest();
  }
  
  Wire.begin();
  lastValidCommand = millis();

  // Initialize limit switches (always works)
  pinMode(LIMIT_SWITCH_PIN1, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_PIN2, INPUT_PULLUP);

  // Initialize RS485 FIRST (so we can report errors)
  rs485.begin(9600);
  pinMode(RS485_DE_PIN, OUTPUT);
  digitalWrite(RS485_DE_PIN, LOW);
  if (debugFlag) Serial.println("✓ RS485 initialized");

  // Initialize hardware with fault tolerance
  initHardware();

  // Read initial sensor values
  readSensors();
  checkEncoderHealth();

  // Update system ready status
  errors.systemReady = !errors.muxError && 
                       !errors.motorEncoderError && 
                       !errors.gearboxEncoderError;

  wdt_enable(WDTO_8S); // Enable 8-second watchdog
  
  if (debugFlag) {
    Serial.println("\n=== Initialization Complete ===");
    if (errors.systemReady) {
      Serial.println("Status: ✓ READY");
    } else {
      Serial.println("Status: ⚠ ERRORS DETECTED");
      printErrorStatus();
    }
  }
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  wdt_reset(); // Reset watchdog timer

  // Handle RS485 commands
  if (rs485.available()) {
    handleRS485Command();
  }

  // Update sensor readings every 50ms
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 50) {
    readSensors();
    checkEncoderHealth();
    lastUpdate = millis();
    
    if (debugFlag) {
      printLiveData();
    }
  }

  // Communication timeout check
  if (millis() - lastValidCommand > COMMAND_TIMEOUT) {
    errors.commTimeout = true;
    if (debugFlag) {
      Serial.println("⚠ Communication timeout");
    }
    lastValidCommand = millis(); // Reset to prevent spam
  } else {
    errors.commTimeout = false;
  }
}

// ============================================================
// HARDWARE INITIALIZATION
// ============================================================
void initHardware() {
  // Try to initialize multiplexer
  if (!mux.begin()) {
    Serial.println("❌ ERROR: Mux not found");
    errors.muxError = true;
  } else {
    if (debugFlag) Serial.println("✓ Mux initialized");
  }


  // Try to initialize gearbox encoder
  if (!errors.muxError) 
  {
    mux.selectChannel(ENCODER_CHANNEL_GEARBOX);
    delay(10);
    if (!encoderGearbox.begin()) 
    {
      Serial.println("❌ ERROR: Gearbox encoder not found");
      errors.gearboxEncoderError = true;
    } 
    else 
    {
      if (debugFlag) Serial.println("✓ Gearbox encoder initialized");
      // Check magnet immediately
      if (!encoderGearbox.isMagnetDetected()) 
      {
        Serial.println("⚠ WARNING: Gearbox encoder magnet not detected");
        errors.gearboxMagnetError = true;
      }
    }
  }

  // Try to initialize motor encoder
  if (!errors.muxError) 
  {
    mux.selectChannel(ENCODER_CHANNEL_MOTOR);
    delay(10);
    if (!encoderMotor.begin()) 
    {
      Serial.println("❌ ERROR: Motor encoder not found");
      errors.motorEncoderError = true;
    } 
    else 
    {
      if (debugFlag) Serial.println("✓ Motor encoder initialized");
      // Check magnet immediately
      if (!encoderMotor.isMagnetDetected()) 
      {
        Serial.println("⚠ WARNING: Motor encoder magnet not detected");
        errors.motorMagnetError = true;
      }
    }
  }
}

// ============================================================
// READ RAW ENCODER COUNTS
// ============================================================
uint16_t readRawCount(Adafruit_AS5600 &encoder) {
  // Read 12-bit raw angle register directly
  // Returns 0-4095
  return encoder.getRawAngle();  // Use getRawAngle() for 0-4095 counts
}

// ============================================================
// READ SENSORS
// ============================================================
void readSensors() {
  // Read motor encoder (with multi-turn tracking)
  if (!errors.motorEncoderError && !errors.muxError && !errors.motorMagnetError) {
    mux.selectChannel(ENCODER_CHANNEL_MOTOR);
    delayMicroseconds(500);
    uint16_t currentMotorCount = readRawCount(encoderMotor);
    
    // Detect rotation crossing (0 → 4095 or 4095 → 0)
    if (prevMotorCount > 3000 && currentMotorCount < 1000) {
      // Crossed 0 going forward
      motorRotations++;
      if (debugFlag) {
        Serial.print("Motor rotation forward: ");
        Serial.println(motorRotations);
      }
    } else if (prevMotorCount < 1000 && currentMotorCount > 3000) {
      // Crossed 0 going backward
      motorRotations--;
      if (debugFlag) {
        Serial.print("Motor rotation backward: ");
        Serial.println(motorRotations);
      }
    }
    
    motorCount = currentMotorCount;
    prevMotorCount = currentMotorCount;
  } else {
    motorCount = 0;  // Default on error
  }

  // Read gearbox encoder (no multi-turn needed)
  if (!errors.gearboxEncoderError && !errors.muxError && !errors.gearboxMagnetError) {
    mux.selectChannel(ENCODER_CHANNEL_GEARBOX);
    delayMicroseconds(500);
    gearboxCount = readRawCount(encoderGearbox);
  } else {
    gearboxCount = 0;  // Default on error
  }

  // Read limit switches
  limitSwitch1 = debounceRead(LIMIT_SWITCH_PIN1);
  limitSwitch2 = debounceRead(LIMIT_SWITCH_PIN2);
}

// ============================================================
// CHECK ENCODER HEALTH
// ============================================================
void checkEncoderHealth() {
  // Check motor encoder magnet
  if (!errors.motorEncoderError && !errors.muxError) {
    mux.selectChannel(ENCODER_CHANNEL_MOTOR);
    if (!encoderMotor.isMagnetDetected()) {
      if (!errors.motorMagnetError) {
        Serial.println("❌ Motor encoder magnet lost!");
        errors.motorMagnetError = true;
      }
    } else {
      if (errors.motorMagnetError) {
        Serial.println("✓ Motor encoder magnet restored");
        errors.motorMagnetError = false;
      }
    }
  }

  // Check gearbox encoder magnet
  if (!errors.gearboxEncoderError && !errors.muxError) {
    mux.selectChannel(ENCODER_CHANNEL_GEARBOX);
    if (!encoderGearbox.isMagnetDetected()) {
      if (!errors.gearboxMagnetError) {
        Serial.println("❌ Gearbox encoder magnet lost!");
        errors.gearboxMagnetError = true;
      }
    } else {
      if (errors.gearboxMagnetError) {
        Serial.println("✓ Gearbox encoder magnet restored");
        errors.gearboxMagnetError = false;
      }
    }
  }

  // Update system ready flag
  errors.systemReady = !errors.muxError && 
                       !errors.motorEncoderError && 
                       !errors.gearboxEncoderError &&
                       !errors.motorMagnetError &&
                       !errors.gearboxMagnetError;
}

// ============================================================
// BUILD STATUS BYTE
// ============================================================
uint8_t buildStatusByte() {
  uint8_t status = 0;
  if (errors.systemReady)           status |= 0x01;  // Bit 0
  if (errors.muxError)              status |= 0x02;  // Bit 1
  if (errors.motorEncoderError)     status |= 0x04;  // Bit 2
  if (errors.gearboxEncoderError)   status |= 0x08;  // Bit 3
  if (errors.motorMagnetError)      status |= 0x10;  // Bit 4
  if (errors.gearboxMagnetError)    status |= 0x20;  // Bit 5
  if (errors.commTimeout)           status |= 0x40;  // Bit 6
  // Bit 7: Reserved
  return status;
}

// ============================================================
// HANDLE RS485 COMMAND
// ============================================================
void handleRS485Command() {
  if (debugFlag) {
    Serial.println("RS485 command received");
  }

  // Simple poll message (no CODE field needed anymore)
  uint8_t buf[5];  // [STX][ID][CRC(2)][ETX]
  rs485.readBytes(buf, 5);

  if (debugFlag) {
    Serial.print("RX: ");
    for (int i = 0; i < 5; i++) {
      if (buf[i] < 0x10) Serial.print("0");
      Serial.print(buf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }

  // Validate message
  if (buf[0] == 0x02 && buf[1] == JOINT_ID && buf[4] == 0x03) {
    // Verify CRC
    crc.restart();
    crc.add(buf[0]);
    crc.add(buf[1]);
    uint16_t calculatedCRC = crc.getCRC();
    uint16_t receivedCRC = ((uint16_t)buf[2] << 8) | buf[3];

    if (debugFlag) {
      Serial.print("Calc CRC: 0x");
      Serial.print(calculatedCRC, HEX);
      Serial.print(" | Recv CRC: 0x");
      Serial.println(receivedCRC, HEX);
    }

    if (calculatedCRC == receivedCRC) {
      if (debugFlag) Serial.println("✓ CRC MATCH - Sending response");
      lastValidCommand = millis();
      sendLiveData();
    } else if (debugFlag) {
      Serial.println("✗ CRC mismatch");
    }
  }
}

// ============================================================
// SEND LIVE DATA (13 bytes)
// ============================================================
void sendLiveData() {
  message[0] = 0x02;  // STX
  message[1] = JOINT_ID;

  // Motor count (bytes 2-3, uint16)
  message[2] = (motorCount >> 8) & 0xFF;
  message[3] = motorCount & 0xFF;

  // Motor rotations (bytes 4-5, int16 signed)
  message[4] = (motorRotations >> 8) & 0xFF;
  message[5] = motorRotations & 0xFF;

  // Gearbox count (bytes 6-7, uint16)
  message[6] = (gearboxCount >> 8) & 0xFF;
  message[7] = gearboxCount & 0xFF;

  // Switches (byte 8)
  message[8] = (limitSwitch1 ? 0x01 : 0x00) | (limitSwitch2 ? 0x02 : 0x00);

  // Status byte (byte 9)
  message[9] = buildStatusByte();

  // Calculate CRC (bytes 0-9)
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
  delayMicroseconds(100);
  rs485.write(message, 13);
  rs485.flush();
  delayMicroseconds(100);
  digitalWrite(RS485_DE_PIN, LOW);

  if (debugFlag) {
    Serial.print("TX: ");
    for (int i = 0; i < 13; i++) {
      if (message[i] < 0x10) Serial.print("0");
      Serial.print(message[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}

// ============================================================
// PRINT LIVE DATA (HUMAN READABLE)
// ============================================================
void printLiveData() {
  // Convert to degrees for display
  float motorDegrees = (motorCount / 4096.0) * 360.0;
  float gearboxDegrees = (gearboxCount / 4096.0) * 360.0;
  
  // Calculate slippage (in counts)
  int32_t motorTotalCounts = (motorRotations * 4096L) + motorCount;
  int32_t expectedGearboxCounts = motorTotalCounts / 50;
  int32_t slippageCounts = gearboxCount - (expectedGearboxCounts % 4096);
  float slippageDegrees = (slippageCounts / 4096.0) * 360.0;

  // Print timestamp
  Serial.print("[");
  Serial.print(millis() / 1000);
  Serial.print("s] J");
  Serial.print(JOINT_ID);
  
  // Print encoder data (counts + degrees)
  Serial.print(" | Motor: ");
  Serial.print(motorCount);
  Serial.print(" (");
  Serial.print(motorDegrees, 2);
  Serial.print("°)");
  
  Serial.print(" | Rot: ");
  Serial.print(motorRotations);
  
  Serial.print(" | Gearbox: ");
  Serial.print(gearboxCount);
  Serial.print(" (");
  Serial.print(gearboxDegrees, 2);
  Serial.print("°)");
  
  // Print limit switches
  Serial.print(" | L1: ");
  Serial.print(limitSwitch1 ? "ON" : "off");
  Serial.print(" L2: ");
  Serial.print(limitSwitch2 ? "ON" : "off");
  
  // Print system status
  Serial.print(" | ");
  Serial.print(errors.systemReady ? "READY" : "ERROR");
  
  // Print slippage
  Serial.print(" | Slip: ");
  Serial.print(slippageCounts);
  Serial.print(" (");
  Serial.print(slippageDegrees, 2);
  Serial.print("°)");

  // Alert indicators
  if (limitSwitch1 || limitSwitch2) {
    Serial.print(" 🚨");
  }
  
  if (!errors.systemReady) {
    Serial.print(" ⚠");
    if (errors.muxError) Serial.print(" MUX");
    if (errors.motorEncoderError) Serial.print(" MOT_ENC");
    if (errors.gearboxEncoderError) Serial.print(" GB_ENC");
    if (errors.motorMagnetError) Serial.print(" MOT_MAG");
    if (errors.gearboxMagnetError) Serial.print(" GB_MAG");
  }

  Serial.println();
}

// ============================================================
// DEBOUNCE LIMIT SWITCH
// ============================================================
bool debounceRead(uint8_t pin) {
  int index = (pin == LIMIT_SWITCH_PIN1) ? 0 : 1;
  bool currentState = digitalRead(pin);
  
  if (currentState != lastStates[index] && (millis() - lastChangeTimes[index]) > 20) {
    lastChangeTimes[index] = millis();
    lastStates[index] = currentState;
  }
  return lastStates[index];
}

// ============================================================
// PRINT ERROR STATUS
// ============================================================
void printErrorStatus() {
  Serial.println("\n=== ERROR STATUS ===");
  Serial.print("Mux: "); Serial.println(errors.muxError ? "ERROR" : "OK");
  Serial.print("Motor Encoder: "); Serial.println(errors.motorEncoderError ? "ERROR" : "OK");
  Serial.print("Gearbox Encoder: "); Serial.println(errors.gearboxEncoderError ? "ERROR" : "OK");
  Serial.print("Motor Magnet: "); Serial.println(errors.motorMagnetError ? "ERROR" : "OK");
  Serial.print("Gearbox Magnet: "); Serial.println(errors.gearboxMagnetError ? "ERROR" : "OK");
  Serial.println("====================\n");
}

// ============================================================
// CRC TEST
// ============================================================
void runCRCTest() {
  Serial.println("\n--- CRC-16 Test ---");
  
  // Test new poll message format (no CODE)
  uint8_t test1[] = {0x02, 0x01};
  crc.restart();
  crc.add(test1, sizeof(test1));
  Serial.print("Poll msg (02 01) -> CRC: 0x");
  Serial.println(crc.getCRC(), HEX);
  
  Serial.println("--- Test Complete ---\n");
}
