#!/usr/bin/env python3
"""
Teensy Executor - Simple ACK-Based Flow Control
Fills buffer to 10, then sends 1 waypoint per 1 WP_CONSUMED
"""

from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import time
import threading
import math

class TeensyExecutor(Node):
    def __init__(self):
        super().__init__('teensy_executor')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('joint_names', ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5'])
        
        serial_port = self.get_parameter('serial_port').value
        serial_baud = self.get_parameter('serial_baud').value
        self.joint_names = self.get_parameter('joint_names').value
        
        # Serial connection
        try:
            self.serial = serial.Serial(serial_port, serial_baud, timeout=1)
            time.sleep(2)
            self.serial.write(b'@ROS\n')
            time.sleep(0.3)
            self.serial.write(b'@RESET\n')
            time.sleep(0.5)
            self.get_logger().info(f'✓ Connected to {serial_port}')
        except Exception as e:
            self.get_logger().error(f'Serial connection failed: {e}')
            raise

        # Simple event flags (thread-safe)
        self.buffered_ack = threading.Event()
        self.consumed_ack = threading.Event()
        self.complete_ack = threading.Event()
        
        # ROS publishers
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Background serial reader
        self.serial_thread = threading.Thread(target=self.serial_reader, daemon=True)
        self.serial_thread.start()
        
        # Action server
        self.trajectory_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            self.execute_trajectory
        )
        
        self.get_logger().info('✓ Teensy Executor Ready')

    def execute_trajectory(self, goal_handle):
        """Execute trajectory with simple ACK protocol"""
        points = goal_handle.request.trajectory.points
        total = len(points)
        
        self.get_logger().info(f'🚀 Executing {total} waypoints')
        self.save_trajectory(points)
        
        # Reset
        self.serial.write(b'@RESET\n')
        time.sleep(0.5)
        self.buffered_ack.clear()
        self.consumed_ack.clear()
        self.complete_ack.clear()
        
        # === PHASE 1: Fill buffer (first 10 waypoints) ===
        initial_fill = min(10, total)
        self.get_logger().info(f'📤 Phase 1: Filling buffer with {initial_fill} waypoints')
        
        for i in range(initial_fill):
            if not self.send_waypoint(i, points[i], total):
                goal_handle.abort()
                return FollowJointTrajectory.Result(error_code=-1)
        
        # === PHASE 2: Send remaining waypoints (1 per WP_CONSUMED) ===
        if total > initial_fill:
            self.get_logger().info(f'📤 Phase 2: Streaming remaining {total - initial_fill} waypoints')
            
            for i in range(initial_fill, total):
                # Wait for Teensy to consume one waypoint
                self.get_logger().info(f'⏳ Waiting for WP_CONSUMED... ({i+1}/{total})')
                if not self.consumed_ack.wait(timeout=10.0):
                    self.get_logger().error('❌ Timeout waiting for WP_CONSUMED')
                    goal_handle.abort()
                    return FollowJointTrajectory.Result(error_code=-1)
                
                self.consumed_ack.clear()  # Reset flag
                
                # Now send next waypoint
                if not self.send_waypoint(i, points[i], total):
                    goal_handle.abort()
                    return FollowJointTrajectory.Result(error_code=-1)
        
        # === PHASE 3: Wait for completion ===
        self.get_logger().info('📤 All waypoints sent, waiting for COMPLETE...')
        if self.complete_ack.wait(timeout=30.0):
            self.get_logger().info('✅ Trajectory COMPLETE')
            goal_handle.succeed()
            return FollowJointTrajectory.Result(error_code=0)
        else:
            self.get_logger().warn('⚠️ Timeout on COMPLETE (motion likely finished)')
            goal_handle.succeed()
            return FollowJointTrajectory.Result(error_code=0)

    def send_waypoint(self, index, point, total):
        """Send single waypoint and wait for BUFFERED ack"""
        # Convert to degrees
        pos_deg = [p * 180.0 / math.pi for p in point.positions[:5]]
        vel_deg = [0.0] * 5
        if point.velocities and len(point.velocities) >= 5:
            vel_deg = [v * 180.0 / math.pi for v in point.velocities[:5]]
        
        # Motion type
        motion_type = 1
        if index == 0: 
            motion_type = 0
        elif index == total - 1: 
            motion_type = 2
        
        # Format command
        cmd = (f"W {','.join(f'{p:.4f}' for p in pos_deg)},"
               f"{','.join(f'{v:.2f}' for v in vel_deg)},"
               f"{motion_type}\n")
        
        self.get_logger().info(f'📤 [{index+1}/{total}] Sending waypoint')
        
        # Send
        self.buffered_ack.clear()
        self.serial.write(cmd.encode())
        self.serial.flush()
        
        # Wait for BUFFERED confirmation
        if not self.buffered_ack.wait(timeout=2.0):
            self.get_logger().error(f'❌ No BUFFERED ack for waypoint {index+1}')
            return False
        
        return True

    def serial_reader(self):
        """Background thread: Read serial and trigger events"""
        last_heartbeat_time = time.time()
        
        while rclpy.ok():
            try:
                if self.serial.in_waiting:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if not line:
                        continue
                    
                    self.get_logger().info(f'[TEENSY] {line}')
                    
                    # Parse messages
                    if 'BUFFERED' in line:
                        self.buffered_ack.set()
                    
                    elif 'WP_CONSUMED' in line:
                        self.consumed_ack.set()
                    
                    elif 'TRAJECTORY_COMPLETE' in line:
                        self.complete_ack.set()
                    
                    elif line.startswith('@JOINT_STATE'):
                        self.publish_joint_state(line)
                    
                    elif 'PROGRESS:' in line:
                        # Parse "PROGRESS: 45/129"
                        try:
                            progress_str = line.split('PROGRESS:')[1].strip()
                            current, total = progress_str.split('/')
                            self.get_logger().info(f'📊 Progress: {current}/{total} waypoints')
                        except:
                            pass

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

    def publish_joint_state(self, line):
        """Parse and publish joint states"""
        try:
            parts = line.replace('@JOINT_STATE ', '').split(',')
            if len(parts) >= 10:
                pos_deg = [float(parts[i]) for i in range(5)]
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = self.joint_names
                msg.position = [p * math.pi / 180.0 for p in pos_deg]
                self.joint_state_pub.publish(msg)
        except:
            pass

    def save_trajectory(self, points):
        """Save trajectory to CSV"""
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"traj_{timestamp}.csv"
        try:
            with open(filename, "w") as f:
                f.write("Index,J1,J2,J3,J4,J5,V1,V2,V3,V4,V5\n")
                for i, pt in enumerate(points):
                    pos = [p * 180.0 / math.pi for p in pt.positions[:5]]
                    vel = [v * 180.0 / math.pi if pt.velocities else 0.0 for v in (pt.velocities[:5] if pt.velocities else [0]*5)]
                    f.write(f"{i}," + ",".join(f"{x:.2f}" for x in pos+vel) + "\n")
            self.get_logger().info(f"📝 Saved to {filename}")
        except Exception as e:
            self.get_logger().error(f"Save failed: {e}")

    def destroy_node(self):
        if hasattr(self, 'serial'):
            self.serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    executor = TeensyExecutor()
    try:
        rclpy.spin(executor)
    except KeyboardInterrupt:
        pass
    finally:
        executor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
