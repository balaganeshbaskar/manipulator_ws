import serial
import time
import re
import csv
import statistics
import sys


# ================= CONFIGURATION =================
SERIAL_PORT = '/dev/ttyACM0'  
BAUD_RATE = 115200
TIMEOUT = 30
OUTPUT_FILE = 'trajectory_test_results.csv'
MAX_BUFFER_SIZE = 20  # Match Teensy's WAYPOINT_BUFFER_SIZE

# Same TEST_SEQUENCE as before...
TEST_SEQUENCE = [
    (2.0, 4.5), (4.0, 6.3), (6.0, 7.7), (8.0, 8.9), (10.0, 10.0),
    (12.0, 11.0), (14.0, 11.8), (16.0, 12.6), (18.0, 13.4), (20.0, 14.1),
    (22.0, 14.8), (24.0, 15.0), (26.0, 15.0), (28.0, 15.0), (30.0, 15.0),
    (32.0, 15.0), (34.0, 15.0), (36.0, 15.0), (38.0, 15.0), (40.0, 15.0),
    (42.0, 15.0), (44.0, 15.0), (46.0, 15.0), (48.0, 15.0), (50.0, 15.0),
    (52.0, 15.0), (54.0, 15.0), (56.0, 15.0), (58.0, 15.0), (60.0, 15.0),
    (62.0, 15.0), (64.0, 15.0), (66.0, 15.0), (68.0, 15.0), (70.0, 14.1),
    (72.0, 13.4), (74.0, 12.6), (76.0, 11.8), (78.0, 11.0), (80.0, 10.0),
    (82.0, 8.9), (84.0, 7.7), (86.0, 6.3), (88.0, 4.5), (90.0, 2.0)
]


def wait_for_response(ser, expected_response, timeout=2.0):
    """Wait for a specific response from Teensy"""
    start_time = time.time()
    
    while (time.time() - start_time) < timeout:
        if ser.in_waiting:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"   [TEENSY]: {line}")
                    if expected_response in line:
                        return True
            except Exception as e:
                print(f"   [Read Error]: {e}")
        time.sleep(0.01)
    
    return False


def read_all_pending(ser):
    """Read all pending data from serial"""
    messages = []
    while ser.in_waiting:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                messages.append(line)
                print(f"   [TEENSY]: {line}")
        except:
            pass
    return messages


def send_trajectory_with_flow_control(ser, trajectory):
    """Send trajectory with smart flow control"""
    
    # Step 1: Reset buffer
    print("   Step 1: Resetting buffer...")
    ser.write(b"@RESET\n")
    if not wait_for_response(ser, "BUFFER_CLEARED", timeout=2.0):
        print("   ERROR: No BUFFER_CLEARED response")
        return False
    
    # Step 2: Send waypoints with adaptive pacing
    print(f"   Step 2: Sending {len(trajectory)} waypoints...")
    
    buffered_count = 0
    execution_started = False
    
    for i, (target_j1, vel_j1) in enumerate(trajectory):
        # Determine motion type
        if i == 0:
            motion_type = 0  # START
        elif i == len(trajectory) - 1:
            motion_type = 2  # STOP
        else:
            motion_type = 1  # VIA
        
        # Format waypoint command
        cmd = f"W {target_j1:.4f},0.0,0.0,0.0,0.0,{vel_j1:.2f},5.0,5.0,5.0,5.0,{motion_type}\n"
        
        # ✅ FLOW CONTROL: Wait if buffer is getting full
        if buffered_count >= (MAX_BUFFER_SIZE - 2):
            print(f"   [Flow Control] Buffer near full ({buffered_count}/{MAX_BUFFER_SIZE}), pausing 100ms...")
            time.sleep(0.1)
            # Read any messages to update buffer count
            messages = read_all_pending(ser)
            # Assume some waypoints were consumed during execution
            if execution_started:
                buffered_count = max(0, buffered_count - 2)
        
        print(f"   [{i+1}/{len(trajectory)}] Sending: J1={target_j1:.1f}° @ {vel_j1:.1f}°/s (Buffer: {buffered_count}/{MAX_BUFFER_SIZE})")
        ser.write(cmd.encode())
        ser.flush()
        
        # Wait for response with timeout
        start_wait = time.time()
        response_received = False
        
        while (time.time() - start_wait) < 1.0:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"   [TEENSY]: {line}")
                    
                    if "BUFFERED" in line:
                        buffered_count += 1
                        response_received = True
                        break
                    elif "BUFFER_FULL" in line:
                        print(f"   WARNING: Buffer full at waypoint {i+1}, waiting...")
                        time.sleep(0.2)
                        # Retry sending this waypoint
                        ser.write(cmd.encode())
                        ser.flush()
                        continue
                    elif "TRAJECTORY_START" in line or "EXECUTING" in line:
                        execution_started = True
                        buffered_count = max(0, buffered_count - 1)  # First waypoint consumed
            time.sleep(0.01)
        
        if not response_received:
            print(f"   ERROR: No response for waypoint {i+1}")
            return False
        
        # Small delay between waypoints (adjust based on your system)
        time.sleep(0.02)
    
    print("   All waypoints sent successfully!")
    return True


def monitor_execution(ser, timeout=30.0):
    """Monitor trajectory execution"""
    start_time = time.time()
    execution_started = False
    joint_states = []
    
    print(f"   Monitoring trajectory execution (Timeout: {timeout}s)...")
    
    while (time.time() - start_time) < timeout:
        if ser.in_waiting:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                
                print(f"   [TEENSY]: {line}")
                
                if "TRAJECTORY_START" in line or "EXECUTING" in line:
                    execution_started = True
                
                if "COMPLETE" in line:
                    print("   -> Trajectory COMPLETE!")
                    return True, joint_states
                
                # Parse joint state
                if line.startswith("@JOINT_STATE"):
                    parts = line.replace("@JOINT_STATE ", "").split(",")
                    if len(parts) >= 10:
                        state = {
                            'timestamp': time.time() - start_time,
                            'positions': [float(parts[i]) for i in range(5)],
                            'velocities': [float(parts[i]) for i in range(5, 10)]
                        }
                        joint_states.append(state)
                        
            except Exception as e:
                print(f"   [Read Error]: {e}")
        else:
            time.sleep(0.01)
    
    print("\n   !!! TIMEOUT waiting for completion !!!")
    return False, joint_states


def run_test():
    ser = None
    print(f"\n{'='*60}")
    print(f"CONTINUOUS TRAJECTORY TEST (Flow Control Enabled)")
    print(f"{'='*60}")
    print(f"Target Port: {SERIAL_PORT}")
    print(f"Waypoints: {len(TEST_SEQUENCE)}")
    print(f"Buffer Size: {MAX_BUFFER_SIZE}")
    
    try:
        # 1. Connect FIRST
        print("\n1. Connecting to Teensy...", end='', flush=True)
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1.0)
        print(" ✓ Connected")
        
        print("2. Waiting for initialization (2s)...")
        time.sleep(2)
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # 3. Set mode
        print("3. Setting MANUAL mode...")
        ser.write(b"@MANUAL\n")
        time.sleep(0.3)
        read_all_pending(ser)

        # ✅ 4. NOW home the joint (AFTER serial is connected)
        print("\n4. Homing J1 to 0°...")
        home_joint(ser, joint_num=1, target=0.0, velocity=2.0)
        time.sleep(1.0)  # Wait for homing to complete

        # 5. Send trajectory
        print(f"\n5. Sending trajectory with flow control...")
        if not send_trajectory_with_flow_control(ser, TEST_SEQUENCE):
            print("✗ Failed to send trajectory")
            return

        # 6. Monitor execution
        print("\n6. Monitoring execution...")
        success, joint_states = monitor_execution(ser, timeout=TIMEOUT)
        
        if not success:
            print("✗ Execution failed or timed out")
            return
        
        print(f"\n✓ Trajectory completed!")
        
        # Analysis
        if joint_states:
            j1_positions = [state['positions'][0] for state in joint_states]
            print(f"\n{'='*60}")
            print("RESULTS")
            print(f"{'='*60}")
            print(f"Samples: {len(joint_states)}")
            print(f"J1 Range: {min(j1_positions):.2f}° to {max(j1_positions):.2f}°")
            print(f"Target: {TEST_SEQUENCE[-1][0]:.2f}°")
            print(f"Final: {j1_positions[-1]:.2f}°")
            print(f"Error: {abs(j1_positions[-1] - TEST_SEQUENCE[-1][0]):.4f}°")
            
            # Save CSV
            try:
                with open(OUTPUT_FILE, 'w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(['timestamp', 'j1_pos', 'j1_vel'])
                    for state in joint_states:
                        writer.writerow([state['timestamp'], state['positions'][0], state['velocities'][0]])
                print(f"✓ Saved: {OUTPUT_FILE}")
            except Exception as e:
                print(f"✗ CSV Error: {e}")

    except serial.SerialException as e:
        print(f"\n✗ Serial Error: {e}")
    except KeyboardInterrupt:
        print("\n✗ Interrupted")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if ser and ser.is_open:
            ser.write(b"@RESET\n")
            time.sleep(0.2)
            ser.close()
            print("\n✓ Serial closed")


def home_joint(ser, joint_num=1, target=0.0, velocity=2.0, timeout=60.0):
    """Home a joint - monitors actual position until target reached"""
    print(f"Homing J{joint_num} to {target:.1f}° @ {velocity:.1f}°/s...")
    
    cmd = f"@HOME J{joint_num} {target:.2f} {velocity:.2f}\n"
    ser.write(cmd.encode())
    ser.flush()
    
    start_time = time.time()
    homing_sent = False
    settled_start_time = None
    tolerance = 0.7  # Within 0.15° is considered "arrived"
    settle_duration = 0.5  # Must stay within tolerance for 0.5s
    
    print("   Waiting for homing to complete...")
    
    while (time.time() - start_time) < timeout:
        if ser.in_waiting:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"   [TEENSY]: {line}")
                    
                    # Check for homing command acknowledgment
                    if "HOMING:" in line:
                        homing_sent = True
                        print("   ✓ Homing started, monitoring position...")
                    
                    # Parse position feedback: [28.0s] Tgt:0.00 Cur:2.90 Err:-2.90 Vel:0.0
                    match = re.search(r"Tgt:([-\d\.]+).*Cur:([-\d\.]+).*Err:([-\d\.]+).*Vel:([-\d\.]+)", line)
                    if match and homing_sent:
                        current_pos = float(match.group(2))
                        error = float(match.group(3))
                        vel = float(match.group(4))
                        
                        # Check if we're within tolerance
                        if abs(error) <= tolerance:
                            if settled_start_time is None:
                                settled_start_time = time.time()
                                print(f"   → Within tolerance (error: {error:.3f}°), waiting for settle...")
                            elif (time.time() - settled_start_time) >= settle_duration:
                                print(f"   ✓ Homing COMPLETE! Final position: {current_pos:.2f}°, Error: {error:.3f}°")
                                return True
                        else:
                            # Still moving or error increased
                            if settled_start_time is not None:
                                print(f"   → Error increased to {error:.3f}°, continuing...")
                            settled_start_time = None
                            
            except Exception as e:
                print(f"   [Parse Error]: {e}")
        
        time.sleep(0.01)
    
    print(f"   ✗ Homing TIMEOUT after {timeout}s")
    return False



if __name__ == "__main__":
    run_test()
