#ifndef TEST_MANAGER_H
#define TEST_MANAGER_H

#include "TestBase.h"

class TestManager {
  public:
    TestManager() : _currentTest(nullptr), _active(false) {}
    
    // Run a specific test on a specific joint
    // USAGE: manager.runTest(new AccuracyTest(10, 1), &joints[0]);
    void runTest(RoboticTest* newTest, RoboticJoint* joint) {
      stopTest(); // Clear any existing test
      
      _currentTest = newTest;
      _active = true;
      
      Serial.print(F(">> MANAGER: Starting ")); 
      Serial.println(_currentTest->getName());
      
      _currentTest->start(joint);
    }
    
    void update() {
      if (!_active || _currentTest == nullptr) return;
      
      _currentTest->update();
      
      if (_currentTest->isComplete()) {
        Serial.println(F(">> MANAGER: Test Completed."));
        stopTest();
      }
    }
    
    void stopTest() {
      if (_currentTest != nullptr) {
        _currentTest->stop(); // Allow test to cleanup
        delete _currentTest;  // Free memory
        _currentTest = nullptr;
      }
      _active = false;
    }
    
    bool isActive() { return _active; }
    
  private:
    RoboticTest* _currentTest;
    bool _active;
};

#endif

