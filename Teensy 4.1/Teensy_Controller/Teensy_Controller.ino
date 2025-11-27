#include "RoboticJoint.h"
#include <CRC16.h>

// ============================================================
// SYSTEM CONFIGURATION
// ============================================================
RoboticJoint joints[1] = {
  RoboticJoint(1, 3, 4, 5) // ID, Step, Dir, Ena
};
const int NUM_JOINTS = 1;
#define ACTIVE_JOINT 0

// RS485
#define RS485_SERIAL Serial1
const int RS485_DE_PIN = 2;
const uint32_t RS485_BAUD = 57600;
const uint8_t JOINT_ID = 1;
CRC16 crc;

// Test Manager Globals (Exact Copy)
struct TestMetric {
  uint16_t cycle_num;
  int8_t direction;
  float out_target, out_actual, out_error_deg;
  unsigned long out_time;
  uint8_t out_corrections;
  float ret_target, ret_actual, ret_error_deg;
  unsigned long ret_time;
  uint8_t ret_corrections;
};

const int MAX_TEST_CYCLES = 50;
TestMetric test_data[MAX_TEST_CYCLES];
int test_total_cycles = 0, test_current_cycle = 0;
bool test_active = false;
unsigned long test_dwell_timer = 0;
const unsigned long DWELL_TIME_MS = 1000;
int test_side_constraint = 1;
enum TestState { T_IDLE, T_WAIT_OUT, T_WAIT_HOME, T_MEASURING };
TestState tState = T_IDLE;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  
  RS485_SERIAL.begin(RS485_BAUD);
  pinMode(RS485_DE_PIN, OUTPUT);
  digitalWrite(RS485_DE_PIN, LOW);
  randomSeed(analogRead(14));

  Serial.println(F("\n╔═══════════════════════════════════════════════════╗"));
  Serial.println(F("║  PRECISION JOINT CONTROLLER - ULTRA HIGH RES      ║"));
  Serial.println(F("╚═══════════════════════════════════════════════════╝\n"));
  
  for(int i=0; i<NUM_JOINTS; i++) joints[i].begin();
  
  // Wait for encoder data
  Serial.print(F("Waiting for encoder..."));
  while (!joints[ACTIVE_JOINT].isDataValid()) {
    pollJoint();
    delay(10);
  }
  Serial.println(F(" OK"));
  
  // The first call to updateSensorData already sets targets correctly
  
  Serial.print(F("Initial: ")); Serial.print(joints[ACTIVE_JOINT].getCurrentAngle(), 4); Serial.println(F("°"));
  Serial.print(F("Target Set To: ")); Serial.print(joints[ACTIVE_JOINT].getTargetAngle(), 4); Serial.println(F("°"));
  Serial.println(F("Ready! Try: M 150"));
}

void loop() {
  static unsigned long lastPoll = 0;
  static unsigned long lastLog = 0;

  updateTestManager();
  processCommands();

  if (millis() - lastPoll >= 20) {
    pollJoint();
    if (joints[ACTIVE_JOINT].isDataValid()) {
      joints[ACTIVE_JOINT].update(); // Watchdog always runs
    }
    lastPoll = millis();
  }
  
  if (millis() - lastLog >= 100) {
    if (joints[ACTIVE_JOINT].getState() != HOLDING) {
      logData();
    }
    lastLog = millis();
  }
}

void processCommands() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n'); cmd.trim();
  if (cmd.length() == 0) return;
  
  char t = cmd.charAt(0);
  if (t == 'M') joints[ACTIVE_JOINT].moveTo(cmd.substring(2).toFloat());
  else if (t == 'R') joints[ACTIVE_JOINT].moveRelative(cmd.substring(2).toFloat());
  else if (t == 'S') joints[ACTIVE_JOINT].stop();
  else if (t == 'X') joints[ACTIVE_JOINT].relax();
  else if (t == 'Q') { pollJoint(); joints[ACTIVE_JOINT].printStatus(); }
  else if (t == 'T') joints[ACTIVE_JOINT].printStatus();
  else if (t == 'Z') {
    int firstSpace = cmd.indexOf(' ');
    int secondSpace = cmd.indexOf(' ', firstSpace + 1);
    int cycles = 5, side = 1;
    if (firstSpace > 0) {
      if (secondSpace > 0) {
        cycles = cmd.substring(firstSpace + 1, secondSpace).toInt();
        side = cmd.substring(secondSpace + 1).toInt();
      } else {
        cycles = cmd.substring(firstSpace + 1).toInt();
      }
    }
    startAccuracyTest(cycles, side);
  }
}

void pollJoint() {
  digitalWrite(RS485_DE_PIN, LOW); delayMicroseconds(100);
  uint8_t msg[6] = {0x02, JOINT_ID, 0x06, 0, 0, 0x03};
  crc.restart(); crc.add(msg[0]); crc.add(msg[1]); crc.add(msg[2]);
  uint16_t c = crc.calc(); msg[3]=c>>8; msg[4]=c&0xFF;
  while(RS485_SERIAL.available()) RS485_SERIAL.read();
  digitalWrite(RS485_DE_PIN, HIGH); delayMicroseconds(100);
  RS485_SERIAL.write(msg, 6); RS485_SERIAL.flush();
  delayMicroseconds(100); digitalWrite(RS485_DE_PIN, LOW);
  uint8_t r[13]; if(receiveResponse(r, 13, 200)) parseResponse(r);
}

bool receiveResponse(uint8_t *r, size_t len, unsigned long to) {
  unsigned long s=millis(); size_t i=0; bool stx=false;
  while(millis()-s<to && i<len) {
    if(RS485_SERIAL.available()) {
      uint8_t b=RS485_SERIAL.read();
      if(!stx && b==0x02) { stx=true; r[i++]=b; } else if(stx) r[i++]=b;
    }
  } return (i==len && r[len-1]==0x03);
}

void parseResponse(uint8_t *r) {
  crc.restart(); for(int i=0; i<=9; i++) crc.add(r[i]);
  if(crc.calc()!=(((uint16_t)r[10]<<8)|r[11])) return;
  uint16_t motCount=((uint16_t)r[2]<<8)|r[3];
  int16_t motRot=((int16_t)r[4]<<8)|r[5];
  uint16_t gbCount=((uint16_t)r[6]<<8)|r[7];
  joints[ACTIVE_JOINT].updateSensorData(gbCount, motCount, motRot);
}

void logData() {
  float gb = joints[ACTIVE_JOINT].getCurrentAngle();
  float err = joints[ACTIVE_JOINT].getError();
  
  Serial.print(F("[")); Serial.print((millis() - joints[ACTIVE_JOINT].getMoveStartTime()) / 1000.0, 1);
  Serial.print(F("s] T:")); Serial.print(joints[ACTIVE_JOINT].getTargetAngle(), 2);
  Serial.print(F("° Pos:")); Serial.print(gb, 3);
  Serial.print(F("° Err:")); Serial.print(err, 4);
  Serial.print(F("° V:")); Serial.print(joints[ACTIVE_JOINT].getVelocity(), 1);
  Serial.print(F(" ")); Serial.println(joints[ACTIVE_JOINT].getStateStr());
}

// TEST MANAGER (EXACT COPY)
void startAccuracyTest(int cycles, int side) {
  if (cycles > MAX_TEST_CYCLES) cycles = MAX_TEST_CYCLES;
  if (cycles < 1) cycles = 1;
  test_total_cycles = cycles; test_current_cycle = 0; test_active = true; test_side_constraint = side;
  Serial.println(F("\n══════════════════════════════════════════════════════════════════════"));
  Serial.print(F("⚠ STARTING BIDIRECTIONAL METROLOGY: ")); Serial.print(cycles); Serial.println(F(" CYCLES"));
  Serial.println(F("══════════════════════════════════════════════════════════════════════"));
  triggerRandomMove();
}

void triggerRandomMove() {
  long randAngle = random(15, 170);
  if (test_side_constraint == 0) { if (random(0, 2) == 1) randAngle *= -1; }
  else if (test_side_constraint == -1) { randAngle = -1 * abs(randAngle); }
  else { randAngle = abs(randAngle); }
  
  Serial.print(F("\n[CYCLE ")); Serial.print(test_current_cycle + 1);
  Serial.print(F("/")); Serial.print(test_total_cycles);
  Serial.print(F("] 1. Moving OUT to: ")); Serial.println(randAngle);
  
  test_data[test_current_cycle].cycle_num = test_current_cycle + 1;
  test_data[test_current_cycle].out_target = (float)randAngle;
  test_data[test_current_cycle].direction = (randAngle > joints[ACTIVE_JOINT].getCurrentAngle()) ? 1 : -1;
  
  tState = T_WAIT_OUT;
  joints[ACTIVE_JOINT].moveTo((float)randAngle);
}

void updateTestManager() {
  if (!test_active) return;
  if (joints[ACTIVE_JOINT].getState() != HOLDING) return;
  
  switch (tState) {
    case T_WAIT_OUT: {
      delay(100);
      TestMetric *d = &test_data[test_current_cycle];
      d->out_actual = joints[ACTIVE_JOINT].getCurrentAngle();
      float err = d->out_target - d->out_actual;
      if (err > 180) err -= 360; if (err < -180) err += 360;
      d->out_error_deg = err;
      d->out_time = millis() - joints[ACTIVE_JOINT].getMoveStartTime();
      d->out_corrections = joints[ACTIVE_JOINT].getCorrectionAttempts();
      
      Serial.print(F(">> OUT COMPLETE. Err: ")); Serial.print(d->out_error_deg, 4); Serial.println(F("°"));
      Serial.println(F(">> 2. Returning HOME (0.000°)..."));
      
      d->ret_target = 0.0000;
      tState = T_WAIT_HOME;
      joints[ACTIVE_JOINT].moveTo(0.000);
      break;
    }
    case T_WAIT_HOME: {
      delay(100);
      TestMetric *d = &test_data[test_current_cycle];
      d->ret_actual = joints[ACTIVE_JOINT].getCurrentAngle();
      float err = d->ret_target - d->ret_actual;
      if (err > 180) err -= 360; if (err < -180) err += 360;
      d->ret_error_deg = err;
      d->ret_time = millis() - joints[ACTIVE_JOINT].getMoveStartTime();
      d->ret_corrections = joints[ACTIVE_JOINT].getCorrectionAttempts();
      
      Serial.print(F(">> HOME COMPLETE. Err: ")); Serial.print(d->ret_error_deg, 4); Serial.println(F("°"));
      test_dwell_timer = millis(); tState = T_MEASURING;
      break;
    }
    case T_MEASURING:
      if (millis() - test_dwell_timer >= DWELL_TIME_MS) {
        test_current_cycle++;
        if (test_current_cycle >= test_total_cycles) { finishTest(); }
        else { triggerRandomMove(); }
      }
      break;
    case T_IDLE: break;
  }
}

void finishTest() {
  test_active = false; tState = T_IDLE;
  Serial.println(F("\n\n══════════════════════════════════════════════════════════════════════════════════════════════"));
  Serial.println(F("                               BIDIRECTIONAL METROLOGY REPORT"));
  Serial.println(F("══════════════════════════════════════════════════════════════════════════════════════════════"));
  Serial.println(F("Iter,Dir,Target_Out,Err_Out,Time_Out,Corr_Out,Target_Ret,Err_Ret,Time_Ret,Corr_Ret"));
  
  float sum_err = 0, max_err = 0;
  for (int i = 0; i < test_total_cycles; i++) {
    TestMetric *d = &test_data[i];
    Serial.print(d->cycle_num); Serial.print(F(","));
    Serial.print(d->direction > 0 ? "+" : "-"); Serial.print(F(","));
    Serial.print(d->out_target, 1); Serial.print(F(","));
    Serial.print(d->out_error_deg, 4); Serial.print(F(","));
    Serial.print(d->out_time); Serial.print(F(","));
    Serial.print(d->out_corrections); Serial.print(F(","));
    Serial.print(d->ret_target, 1); Serial.print(F(","));
    Serial.print(d->ret_error_deg, 4); Serial.print(F(","));
    Serial.print(d->ret_time); Serial.print(F(","));
    Serial.println(d->ret_corrections);
    
    float e1 = abs(d->out_error_deg), e2 = abs(d->ret_error_deg);
    sum_err += (e1 + e2);
    if (e1 > max_err) max_err = e1;
    if (e2 > max_err) max_err = e2;
  }
  Serial.println(F("══════════════════════════════════════════════════════════════════════════════════════════════"));
  Serial.print(F("TOTAL MOVES:      ")); Serial.println(test_total_cycles * 2);
  Serial.print(F("AVG ERROR (ABS):  ")); Serial.print(sum_err / (test_total_cycles * 2), 5); Serial.println(F("°"));
  Serial.print(F("MAX ERROR:        ")); Serial.print(max_err, 5); Serial.println(F("°"));
  Serial.println(F("══════════════════════════════════════════════════════════════════════════════════════════════"));
}
