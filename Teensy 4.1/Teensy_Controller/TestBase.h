#ifndef TEST_BASE_H
#define TEST_BASE_H

#include "RoboticJoint.h"

// Abstract Base Class for ALL future tests
class RoboticTest {
  public:
    virtual ~RoboticTest() {}
    
    // Called once when the test starts
    virtual void start(RoboticJoint* joint) = 0;
    
    // Called every loop
    virtual void update() = 0;
    
    // Called when test is stopped manually
    virtual void stop() = 0;
    
    // Check if finished
    virtual bool isComplete() = 0;
    
    // Helpers
    const char* getName() { return _testName; }
    
  protected:
    const char* _testName = "Generic";
    RoboticJoint* _joint;
};

#endif

