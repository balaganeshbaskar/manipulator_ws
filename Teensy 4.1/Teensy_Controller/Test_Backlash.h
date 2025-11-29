#ifndef TEST_BACKLASH_H
#define TEST_BACKLASH_H

#include "TestBase.h"

// Data Structure for this specific test
struct BacklashMetric {
    uint16_t cycle;
    float start_pos;    // Where we started (should be ~StartAngle)
    float peak_pos;     // How far we got
    float return_pos;   // Where we ended up (The critical value)
    float hysteresis;   // The Backlash (Return - Start)
    unsigned long duration;
};

class BacklashTest : public RoboticTest {
public:
    // Constructor takes the 3 Inputs
    BacklashTest(float startAngle, float moveAngle, int cycles) 
        : _startAngle(startAngle), _moveAngle(moveAngle), _totalCycles(cycles) {
        _testName = "Backlash Characterization";
    }

    void start(RoboticJoint* joint) override {
        _joint = joint;
        _currentCycle = 0;
        _isFinished = false;

        if (_totalCycles > 50) _totalCycles = 50; // Protect RAM

        Serial.println(F("Cycle,Start_Pos,Peak_Pos,Return_Pos,Backlash_Deg,Time_ms"));

        // Start the State Machine
        _state = 1; // Begin Pre-load Sequence
    }

    void update() override {
        if (_isFinished) return;

        // Wait for Joint to be stable (HOLDING) before next step
        // FIXED: Use HOLDING instead of J_HOLDING
        if (_joint->getState() != HOLDING) return;

        switch (_state) {
            case 1: // PRELOAD: RETRACT
                // Move back 5 degrees to ensure we approach from negative side
                // This guarantees we are pushing against the "Forward" face of the gear.
                _joint->moveTo(_startAngle - 5.0);
                _state = 2;
                break;

            case 2: // PRELOAD: APPROACH
                // Move TO the start angle (Forward approach)
                _joint->moveTo(_startAngle);
                _state = 3;
                break;

            case 3: // READY TO CYCLE
                // We are now at Start Angle, positively loaded.
                delay(200); // Short mechanical settle
                _currentBaseline = _joint->getCurrentAngle();
                _cycleStartTime = millis();

                // TRIGGER OUTWARD MOVE
                _joint->moveTo(_startAngle + _moveAngle);
                _state = 4;
                break;

            case 4: // OUTWARD ARRIVAL -> RAPID REVERSE
                // We just arrived at the target. 
                // NO DELAY. Reverse immediately to catch dynamic behavior.
                _peakPos = _joint->getCurrentAngle();

                // TRIGGER RETURN
                _joint->moveTo(_startAngle);
                _state = 5;
                break;

            case 5: // RETURN ARRIVAL -> MEASURE
                // We are back. Measure immediately.
                delay(250); // Wait for settle to get accurate reading
                captureData();

                // Next Step
                _currentCycle++;
                if (_currentCycle >= _totalCycles) {
                    finish();
                } else {
                    // RESET MECHANICS:
                    // We are currently on the "Negative" side of the tooth. 
                    // We must reload the "Positive" side for the next cycle to be valid.
                    _state = 1; // Loop back to Preload
                }
                break;
        }
    }

    void stop() override { _isFinished = true; }
    bool isComplete() override { return _isFinished; }

private:
    // Inputs
    float _startAngle;
    float _moveAngle;
    int _totalCycles;

    // State
    int _currentCycle = 0;
    int _state = 0; 
    bool _isFinished = false;

    // Runtime Data
    float _currentBaseline;
    float _peakPos;
    unsigned long _cycleStartTime;
    BacklashMetric _data[50]; // Data Buffer

    void captureData() {
        BacklashMetric* d = &_data[_currentCycle];
        d->cycle = _currentCycle + 1;
        d->start_pos = _currentBaseline;
        d->peak_pos = _peakPos;
        d->return_pos = _joint->getCurrentAngle();
        d->hysteresis = d->return_pos - d->start_pos; 
        d->duration = millis() - _cycleStartTime;

        // Stream data (CSV Format)
        Serial.print(d->cycle); Serial.print(",");
        Serial.print(d->start_pos, 4); Serial.print(",");
        Serial.print(d->peak_pos, 4); Serial.print(",");
        Serial.print(d->return_pos, 4); Serial.print(",");
        Serial.print(d->hysteresis, 4); Serial.print(",");
        Serial.println(d->duration);
    }

    void finish() {
        _isFinished = true;
        Serial.println(F("TEST COMPLETE."));

        // Calculate Average Backlash
        float total_backlash = 0;
        for(int i=0; i<_totalCycles; i++) {
            total_backlash += (_data[i].hysteresis); 
        }
        Serial.print(F("Avg Backlash Shift: ")); 
        Serial.print(total_backlash / _totalCycles, 5);
        Serial.println(F(" deg"));
    }
};

#endif


