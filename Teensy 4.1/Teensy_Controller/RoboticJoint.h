#ifndef ROBOTIC_JOINT_H
#define ROBOTIC_JOINT_H

#include <Arduino.h>
#include <TeensyTimerTool.h>

using namespace TeensyTimerTool;

// ============================================================
// CONSTANTS (EXACT COPY FROM ORIGINAL)
// ============================================================
#define GEAR_RATIO 50.0
#define MOTOR_DIRECTION_SIGN -1
#define MICROSTEPS 8
#define MOTOR_STEPS_PER_REV (200.0 * MICROSTEPS)
#define GEARBOX_STEPS_PER_DEGREE ((MOTOR_STEPS_PER_REV / 360.0) * GEAR_RATIO)

// #define MAX_VELOCITY 40.0
#define ACCELERATION 100.0
#define DECEL_SAFETY_FACTOR 1.4 // was 2
#define DEADZONE 0.050
#define PRECISION_THRESHOLD 0.01
#define HYBRID_SWITCH_THRESHOLD 2.0
#define MAX_ACCEPTABLE_ERROR 0.5
#define CORRECTION_SPEED 1.5
#define MIN_SPEED_FAR 8.0
#define MIN_SPEED_NEAR 4.0
#define MIN_SPEED_CLOSE 1.0
#define SETTLING_DURATION 150
#define MAX_CORRECTION_ATTEMPTS 10

// States (EXACT COPY)
enum ControlState {
  IDLE, ACCELERATING, CRUISING, DECELERATING, SETTLING, CORRECTING, HOLDING
};

// ============================================================
// STEP GENERATOR CLASS (EXACT COPY FROM ORIGINAL)
// ============================================================
class StepGenerator {
private:
  PeriodicTimer timer;
  uint8_t stepPin;
  volatile bool running;
  volatile uint32_t stepCount;
  
public:
  StepGenerator(uint8_t pin) : stepPin(pin), running(false), stepCount(0) {
    pinMode(stepPin, OUTPUT);
    digitalWrite(stepPin, LOW);
  }
  
  void setSpeed(float deg_per_sec) {
    if (abs(deg_per_sec) < 0.001) { stop(); return; }
    float steps_per_sec = abs(deg_per_sec) * GEARBOX_STEPS_PER_DEGREE;
    float interval_us = 1000000.0 / steps_per_sec;
    if (interval_us < 5.0) interval_us = 5.0;
    if (interval_us > 500000.0) { stop(); return; }
    if (!running) {
      running = true;
      timer.begin([this]() { this->pulseISR(); }, interval_us);
    } else {
      timer.setPeriod(interval_us);
    }
  }
  
  void stop() {
    running = false;
    timer.end();
    digitalWriteFast(stepPin, LOW);
  }
  
  uint32_t getSteps() { return stepCount; }
  void resetSteps() { stepCount = 0; }
  
private:
  void pulseISR() {
    if (!running) return;
    digitalWriteFast(stepPin, HIGH);
    delayMicroseconds(2);
    digitalWriteFast(stepPin, LOW);
    stepCount++;
  }
};

// ============================================================
// ROBOTIC JOINT CLASS
// ============================================================
class RoboticJoint {
  public:
    RoboticJoint(uint8_t id, uint8_t stepPin, uint8_t dirPin, uint8_t enaPin);
    
    void begin();
    void waitForEncoder(); // NEW: Encapsulates the encoder wait
    void update();
    void moveTo(float angle);
    void moveRelative(float delta);
    void stop();
    void relax();
    
    void updateSensorData(uint16_t gbCount, uint16_t motCount, int16_t motRot);
    
    // Getters
    uint8_t getId() { return _id; }
    float getCurrentAngle() { return countToAngle(current_position_count); }
    float getTargetAngle() { return countToAngle(target_count); }
    float getError() { return (position_error_counts / 4096.0) * 360.0; }
    float getVelocity() { return current_velocity; }
    ControlState getState() { return state; }

    const char* getStateStr();
    bool isDataValid() { return _dataValid; }
    bool isMoveActive() { return move_active; }

    uint8_t getCorrectionAttempts() { return correction_attempts; }
    uint32_t getSteps() { return stepGen->getSteps(); }
    unsigned long getMoveStartTime() { return move_start_time; }
    
    void printMoveSummary();
    void printStatus();

    void setMaxVelocity(float speed) {
        if (speed > 0.1 && speed <= 100.0) { // Safety limits
            _maxVelocity = speed;
        }
    }

  private:
    uint8_t _id, _stepPin, _dirPin, _enaPin;
    StepGenerator* stepGen;
    bool _dataValid = false;

    // ADD THIS LINE:
    ControlState state = HOLDING;
    
    // ALL VARIABLES FROM ORIGINAL (EXACT COPY)
    float target_angle_gb = 0.0;
    float target_angle_motor = 0.0;
    float current_angle_gb = 0.0;
    float current_angle_motor = 0.0;
    float target_count = 0.0;
    float current_position_count = 0.0;
    float position_error_counts = 0.0;
    float current_velocity = 0.0;
    float last_error_degrees = 0.0;
    bool direction_cw = true;
    bool move_active = false;
    unsigned long settling_start_time = 0;
    uint8_t correction_attempts = 0;
    unsigned long move_start_time = 0;
    unsigned long log_start_time = 0;

    float _maxVelocity = 40.0; // Default start speed
    
    // Helper Functions (EXACT COPY)
    float countToAngle(float c);
    float angleToCount(float a);
    float getMotorAngleTotal(uint16_t motCount, int16_t motRot);
    void setDirection(bool cw);
    
};

#endif

