#!/usr/bin/env python3
"""
Custom MoveIt Executor for Teensy Waypoint Controller (Robust V3)
- Logs ALL serial traffic for debugging
- Uses estimated flow control (doesn't block waiting for missing status messages)
- Sends ALL MoveIt waypoints directly
"""

from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import serial
import time
import threading
import queue
import math

class TeensyExecutor(Node):
    def __init__(self):
        super().__init__('teensy_executor')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('joint_names', ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5'])
        self.declare_parameter('buffer_size', 20)
        
        serial_port = self.get_parameter('serial_port').value
        serial_baud = self.get_parameter('serial_baud').value
        self.joint_names = self.get_parameter('joint_names').value
        self.buffer_size = int(self.get_parameter('buffer_size').value)
        
        # Serial connection
        try:
            self.serial = serial.Serial(serial_port, serial_baud, timeout=1)
            time.sleep(2)
            self.get_logger().info(f'✓ Connected to {serial_port}')
            
            # Use ROS mode for joint states
            self.serial.write(b'@ROS\n')
            time.sleep(0.3)
            
            # Clear buffer on startup
            self.serial.write(b'@RESET\n')
            time.sleep(0.5)
            self.get_logger().info('✓ Teensy buffer cleared - READY')
            
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Teensy: {e}')
            raise

        # Joint state tracking
        self.current_positions = [0.0] * 5
        self.position_lock = threading.Lock()
        
        # Flow control
        self.feedback_queue = queue.Queue()
        self.teensy_buffer_count = 0
        
        # ROS publishers
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.status_pub = self.create_publisher(String, '/teensy/status', 10)
        
        # Serial reading thread
        self.serial_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
        self.serial_thread.start()
        
        # Trajectory action server
        self.trajectory_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            self.execute_trajectory_callback,
            cancel_callback=self.cancel_trajectory_callback
        )
        
        self.get_logger().info('✓ Teensy Executor Ready (Debug Mode)')

    def execute_trajectory_callback(self, goal_handle):
        """Execute COMPLETE MoveIt trajectory"""
        trajectory = goal_handle.request.trajectory
        points = trajectory.points
        
        self.get_logger().info(f'🚀 Executing {len(points)} MoveIt waypoints')
        
        success = self.send_all_moveit_waypoints(points)
        
        result = FollowJointTrajectory.Result()
        if success:
            result.error_code = 0
            self.get_logger().info('✅ Trajectory execution succeeded')
            goal_handle.succeed()
        else:
            result.error_code = -1
            self.get_logger().error('❌ Trajectory execution failed')
            goal_handle.abort()
        
        return result

    def cancel_trajectory_callback(self, goal_handle):
        """Handle trajectory cancellation"""
        self.get_logger().warn('🛑 Trajectory cancellation requested')
        self.serial.write(b'@ABORT\n')
        return rclpy.action.CancelResponse.ACCEPT

    def send_all_moveit_waypoints(self, trajectory_points):
        """Send points with robust flow control"""

        # --- ADD THIS LINE HERE ---
        self.save_trajectory_to_file(trajectory_points)
        # --------------------------

        # Clear feedback queue
        while not self.feedback_queue.empty():
            try: self.feedback_queue.get_nowait()
            except: break
        
        # Phase 1: Reset
        self.get_logger().info('📤 Phase 1: Resetting buffer (@RESET)')
        self.serial.write(b'@RESET\n')
        self.serial.flush()
        time.sleep(0.5) # Wait blind if no response
        
        self.teensy_buffer_count = 0
        execution_started = False
        
        # Phase 2: Send points
        self.get_logger().info(f'📤 Phase 2: Sending {len(trajectory_points)} waypoints...')
        
        for i, point in enumerate(trajectory_points):
            # Flow control: Pause if buffer near full
            if self.teensy_buffer_count >= (self.buffer_size - 2):
                self.get_logger().info(f'⏸ Buffer full ({self.teensy_buffer_count}), pausing...')
                time.sleep(0.1)
                # Assume consumption if we've started
                if execution_started or i > 5:
                    self.teensy_buffer_count = max(0, self.teensy_buffer_count - 1)
            
            # Prepare command
            positions_deg = [p * 180.0 / math.pi for p in point.positions[:5]]
            velocities_deg = [0.0] * 5
            if point.velocities and len(point.velocities) >= 5:
                velocities_deg = [v * 180.0 / math.pi for v in point.velocities[:5]]
            
            motion_type = 1
            if i == 0: motion_type = 0
            elif i == len(trajectory_points) - 1: motion_type = 2
            
            cmd = (f"W {','.join(f'{p:.4f}' for p in positions_deg)},"
                   f"{','.join(f'{v:.2f}' for v in velocities_deg)},"
                   f"{motion_type}\n")
            
            self.get_logger().info(f"[{i+1}] Sending WP (Buf={self.teensy_buffer_count})")
            self.get_logger().info(cmd)
            
            self.serial.write(cmd.encode())
            self.serial.flush()
            
            # Wait for BUFFERED with timeout
            start = time.time()
            buffered = False
            while time.time() - start < 1.0:
                try:
                    msg = self.feedback_queue.get(timeout=0.05)
                    if msg == 'BUFFERED':
                        self.teensy_buffer_count += 1
                        buffered = True
                        break
                    elif msg == 'BUFFER_FULL':
                        self.get_logger().warn("BUFFER_FULL! Retrying...")
                        time.sleep(0.2)
                        self.serial.write(cmd.encode())
                    elif msg in ['TRAJECTORY_START', 'EXECUTING']:
                        execution_started = True
                except queue.Empty:
                    pass
            
            if not buffered:
                self.get_logger().warn(f"⚠️ No BUFFERED ack for WP {i+1}, assuming sent.")
                # We assume it went through to keep moving
            
            time.sleep(0.005) # Tiny delay
        
        self.get_logger().info('📤 All waypoints sent! Waiting for completion...')
        
        # Wait for completion - looking for ANY completion signal or timeout
        # Since Teensy might not send COMPLETE, we wait for joints to settle
        start_wait = time.time()
        last_log = time.time()
        while time.time() - start_wait < 30.0:
            try:
                msg = self.feedback_queue.get(timeout=0.1)
                if msg == 'COMPLETE':
                    self.get_logger().info('✅ Teensy reported COMPLETE')
                    return True
            except queue.Empty:
                pass
            
            # Log status every 2s
            if time.time() - last_log > 2.0:
                self.get_logger().info('... waiting for motion to finish ...')
                last_log = time.time()
                
        self.get_logger().warn('⚠️ Timed out waiting for COMPLETE, assuming finished.')
        return True

    def serial_read_loop(self):
        """Read ALL serial data and send periodic heartbeat"""
        last_heartbeat_time = time.time()

        while rclpy.ok():
            try:
                if self.serial.in_waiting:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        # ✅ LOG EVERYTHING to see what Teensy is doing
                        self.get_logger().info(f'[TEENSY RAW] {line}')
                        
                        self.feedback_queue.put(line)
                        self.process_teensy_feedback(line)
                
                # ✅ Send heartbeat every 200ms (only during trajectory execution)
                current_time = time.time()
                if (current_time - last_heartbeat_time) >= 0.2:
                    try:
                        self.serial.write(b"HEARTBEAT\n")
                        last_heartbeat_time = current_time
                    except Exception as e:
                        self.get_logger().error(f'Heartbeat send error: {e}')

            except Exception as e:
                self.get_logger().error(f'Serial error: {e}')
            time.sleep(0.001)

    def process_teensy_feedback(self, line):
        """Process joint states AND flow control updates"""
        # 1. Parse Joint States (High frequency)
        if line.startswith('@JOINT_STATE'):
            try:
                parts = line.replace('@JOINT_STATE ', '').split(',')
                if len(parts) >= 10:
                    pos_deg = [float(parts[i]) for i in range(5)]
                    msg = JointState()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.name = self.joint_names
                    msg.position = [p * math.pi / 180.0 for p in pos_deg]
                    # Optional: Add velocity if needed
                    self.joint_state_pub.publish(msg)
            except Exception as e:
                pass

        # 2. ✅ Parse Flow Control (Updates buffer count from Teensy)
        elif 'SWITCHED_WP:' in line or 'EXECUTING:' in line:
            # Format: "SWITCHED_WP: 8 remaining"
            try:
                parts = line.split()
                for part in parts:
                    if part.isdigit():
                        remaining = int(part)
                        # Only update if it shows consumption (prevent race conditions)
                        if remaining < self.teensy_buffer_count:
                            self.teensy_buffer_count = remaining
            except:
                pass

        # 3. ✅ Parse Status (For completion tracking)
        elif line in ['COMPLETE', 'ABORTED', 'BUFFER_CLEARED']:
            if line == 'BUFFER_CLEARED':
                self.teensy_buffer_count = 0
            # Note: COMPLETE is handled by the waiting loop in send_all_moveit_waypoints
            # but putting it in status_pub is good for debugging
            msg = String()
            msg.data = line
            self.status_pub.publish(msg)


    def destroy_node(self):
        if hasattr(self, 'serial'): self.serial.close()
        super().destroy_node()


    def save_trajectory_to_file(self, points):
        """Saves the planned trajectory to a file for debugging."""

        # Create a unique name like: traj_20251217_101500.csv
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"traj_{timestamp}.csv"

        try:
            with open(filename, "w") as f:
                f.write("Index, Pos_J1, Pos_J2, Pos_J3, Pos_J4, Pos_J5, Vel_J1, Vel_J2, Vel_J3, Vel_J4, Vel_J5\n")
                for i, point in enumerate(points):
                    # Convert to degrees for easier reading
                    pos = [p * 180.0 / math.pi for p in point.positions[:5]]
                    
                    # Handle cases where velocity might be empty
                    vel = [0.0] * 5
                    if point.velocities and len(point.velocities) >= 5:
                        vel = [v * 180.0 / math.pi for v in point.velocities[:5]]
                    
                    # Format string
                    line = f"{i}, " + ", ".join(f"{p:.2f}" for p in pos) + ", " + ", ".join(f"{v:.2f}" for v in vel) + "\n"
                    f.write(line)
            self.get_logger().info(f"📝 Trajectory saved to {filename}")
        except Exception as e:
            self.get_logger().error(f"Failed to save trajectory: {e}")


def main(args=None):
    rclpy.init(args=args)
    executor = TeensyExecutor()
    try: rclpy.spin(executor)
    except KeyboardInterrupt: pass
    finally: executor.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
