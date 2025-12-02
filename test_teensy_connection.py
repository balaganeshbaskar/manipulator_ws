#!/usr/bin/env python3
import serial
import time

def test_teensy():
    print("=" * 50)
    print("  TEENSY-WSL COMMUNICATION TEST")
    print("=" * 50)
    
    try:
        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        print("✓ Connected to /dev/ttyACM0\n")
        time.sleep(2)
        
        ser.reset_input_buffer()
        
        # TEST 1: STATUS
        print("TEST 1: Sending <STATUS>")
        print("-" * 50)
        ser.write(b"<STATUS>\n")
        time.sleep(0.2)
        
        for _ in range(10):
            if ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                print(f"  {line}")
            time.sleep(0.1)
        
        # TEST 2: Joint command
        print("\nTEST 2: Sending joint positions")
        print("-" * 50)
        test_cmd = "<0.785,-0.523,1.047,0.0,0.0>"
        print(f"  Command: {test_cmd}")
        ser.write((test_cmd + "\n").encode())
        time.sleep(0.2)
        
        feedback_count = 0
        start_time = time.time()
        
        while time.time() - start_time < 2:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('<') and line.endswith('>'):
                    if feedback_count < 5:
                        print(f"  Feedback {feedback_count+1}: {line}")
                    feedback_count += 1
                else:
                    print(f"  {line}")
            time.sleep(0.05)
        
        print(f"\n  Total feedback messages: {feedback_count}")
        
        print("\n" + "=" * 50)
        if feedback_count >= 5:
            print("✓ TEST PASSED - Communication working!")
        else:
            print("⚠ TEST INCOMPLETE - Limited feedback")
        print("=" * 50)
        
        ser.close()
        
    except Exception as e:
        print(f"\n✗ ERROR: {e}")

if __name__ == "__main__":
    test_teensy()
