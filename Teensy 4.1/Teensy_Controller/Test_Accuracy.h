// This is the return to 0 test for physical Plunger dial test

#ifndef TEST_ACCURACY_H
#define TEST_ACCURACY_H

#include "TestBase.h"

// Struct for Data Storage
struct AccMetric {
  uint16_t id;
  int8_t dir;
  float out_tgt, out_act, out_err;
  unsigned long out_time;
  float ret_tgt, ret_act, ret_err;
  unsigned long ret_time;
};

class AccuracyTest : public RoboticTest {
  public:
    AccuracyTest(int cycles, int side) : _totalCycles(cycles), _sideConstraint(side) {
      _testName = "Accuracy (Metrology)";
    }
    
    void start(RoboticJoint* joint) override {
      _joint = joint;
      _currentCycle = 0;
      _isFinished = false;
      
      // Limit Cycles to RAM constraints
      if (_totalCycles > 50) _totalCycles = 50;
      
      Serial.println(F("Iter,Dir,T_Out,E_Out,t_Out,T_Ret,E_Ret,t_Ret"));
      triggerOutward();
    }
    
    void update() override {
      if (_isFinished) return;
      
      // WAIT FOR STABILITY
      if (_joint->getState() != HOLDING) return;
      
      switch (_state) {
        case 1: // WAIT_OUT
          delay(100); // Mechanical Settle
          captureOutward();
          // Move Home
          _state = 2; 
          _moveStart = millis();
          _joint->moveTo(0.000);
          break;
          
        case 2: // WAIT_HOME
          delay(100);
          captureReturn();
          // Dwell then Next
          _dwellStart = millis();
          _state = 3;
          break;
          
        case 3: // DWELL
          if (millis() - _dwellStart > 3000) {
            _currentCycle++;
            if (_currentCycle >= _totalCycles) {
               finish();
            } else {
               triggerOutward();
            }
          }
          break;
      }
    }
    
    void stop() override {
      _isFinished = true;
    }
    
    bool isComplete() override {
      return _isFinished;
    }

  private:
    // Config
    int _totalCycles;
    int _sideConstraint;
    
    // State
    int _currentCycle;
    int _state = 0; // 0=Idle, 1=WaitOut, 2=WaitHome, 3=Dwell
    bool _isFinished = false;
    unsigned long _moveStart;
    unsigned long _dwellStart;
    
    // Data
    AccMetric _data[50]; // Buffer
    
    // Logic
    void triggerOutward() {
      long randAngle = random(15, 170); 
      if (_sideConstraint == 0) { if (random(0, 2) == 1) randAngle *= -1; } 
      else if (_sideConstraint == -1) { randAngle = -1 * abs(randAngle); }
      else { randAngle = abs(randAngle); }
      
      // Store Target info immediately
      _data[_currentCycle].id = _currentCycle + 1;
      _data[_currentCycle].out_tgt = (float)randAngle;
      float current = _joint->getCurrentAngle();
      _data[_currentCycle].dir = (randAngle > current) ? 1 : -1;
      
      _state = 1; // Wait for Outward Arrival
      _moveStart = millis();
      _joint->moveTo((float)randAngle);
    }
    
    void captureOutward() {
      AccMetric* d = &_data[_currentCycle];
      d->out_act = _joint->getCurrentAngle();
      d->out_err = normalizeError(d->out_tgt - d->out_act);
      d->out_time = millis() - _moveStart;
      // Print partial result to keep connection alive
      Serial.print(F(">> Cycle ")); Serial.print(_currentCycle+1);
      Serial.print(F(" OutErr: ")); Serial.println(d->out_err, 4);
    }
    
    void captureReturn() {
      AccMetric* d = &_data[_currentCycle];
      d->ret_tgt = 0.0000;
      d->ret_act = _joint->getCurrentAngle();
      d->ret_err = normalizeError(d->ret_tgt - d->ret_act);
      d->ret_time = millis() - _moveStart;
    }
    
    void finish() {
      _isFinished = true;
      // Print Full CSV Report
      Serial.println(F("\n=== REPORT ==="));
      for(int i=0; i<_totalCycles; i++) {
        AccMetric* d = &_data[i];
        Serial.print(d->id); Serial.print(",");
        Serial.print(d->dir > 0 ? "+" : "-"); Serial.print(",");
        Serial.print(d->out_tgt, 1); Serial.print(",");
        Serial.print(d->out_err, 4); Serial.print(",");
        Serial.print(d->out_time); Serial.print(",");
        Serial.print(d->ret_tgt, 1); Serial.print(",");
        Serial.print(d->ret_err, 4); Serial.print(",");
        Serial.println(d->ret_time);
      }
      Serial.println(F("=============="));
    }
    
    float normalizeError(float err) {
      if (err > 180) err -= 360; if (err < -180) err += 360; return err;
    }
};

#endif
