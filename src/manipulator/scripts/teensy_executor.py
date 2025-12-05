#!/usr/bin/env python3
"""
Custom MoveIt Executor for Teensy Waypoint Controller
5-Waypoint Queue with Flow Control and Trajectory Lifecycle Management
"""
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import serial
import time
import threading
import queue


class TeensyExecutor(Node):
    def __init__(self):
        super().__init__('teensy_executor')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('joint_names', ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5'])
        self.declare_parameter('planning_group', 'arm_group')
        self.declare_parameter('max_waypoints', 50) # ↓ From 50 → 10 total            
        self.declare_parameter('min_waypoint_distance', 0.05)  # ↑ From 0.05 → 0.3 rad (~17°)
        self.declare_parameter('buffer_size', 5)
        self.declare_parameter('min_buffer_to_start', 3)
        
        serial_port = self.get_parameter('serial_port').value
        serial_baud = self.get_parameter('serial_baud').value
        self.joint_names = self.get_parameter('joint_names').value
        self.planning_group = self.get_parameter('planning_group').value
        self.max_waypoints = self.get_parameter('max_waypoints').value
        self.min_distance = self.get_parameter('min_waypoint_distance').value
        self.buffer_size = self.get_parameter('buffer_size').value
        self.min_buffer_to_start = self.get_parameter('min_buffer_to_start').value
        
        # Serial connection
        try:
            self.serial = serial.Serial(serial_port, serial_baud, timeout=1)
            time.sleep(2)
            self.get_logger().info(f'✓ Connected to {serial_port}')
            
            # Switch to ROS2 mode
            self.serial.write(b'@ROS2\n')
            time.sleep(0.5)
            self.get_logger().info('✓ Teensy in ROS2 waypoint mode')
            
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Teensy: {e}')
            raise
        
        # MoveIt action client
        self.moveit_client = ActionClient(self, MoveGroup, '/move_action')
        
        # Joint state tracking
        self.current_positions = [0.0] * 5
        self.position_lock = threading.Lock()
        
        # Flow control
        self.feedback_queue = queue.Queue()
        self.teensy_buffer_count = 0
        self.teensy_executing = False
        self.trajectory_active = False  # Track if trajectory is in progress
        
        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.status_pub = self.create_publisher(String, '/teensy/status', 10)
        
        # Subscribers
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        
        # Serial reading thread
        self.serial_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
        self.serial_thread.start()
        
        # Trajectory action server
        self.trajectory_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            self.execute_trajectory_callback,
            cancel_callback=self.cancel_trajectory_callback  # Handle cancellation
        )
        
        self.get_logger().info('✓ Teensy Executor Ready (5-WP Queue with Lifecycle)')

    def execute_trajectory_callback(self, goal_handle):
        """Handle trajectory execution requests from MoveIt"""
        self.get_logger().info('📥 Received trajectory execution request')
        
        # If previous trajectory is active, abort it first
        if self.trajectory_active:
            self.get_logger().warn('⚠ Aborting previous trajectory...')
            self.abort_trajectory()
            time.sleep(0.5)  # Wait for abort to complete
        
        trajectory = goal_handle.request.trajectory
        
        # Extract waypoints
        waypoints = []
        for point in trajectory.points:
            waypoints.append(list(point.positions))
        
        self.get_logger().info(f'Trajectory has {len(waypoints)} waypoints')
        
        # Filter and assign types
        filtered = self.filter_waypoints_from_points(waypoints)
        self.get_logger().info(f'✓ Filtered to {len(filtered)} waypoints')
        
        waypoints_with_types = self.assign_motion_types(filtered)
        
        # Mark trajectory as active
        self.trajectory_active = True
        
        # Execute with flow control
        success = self.execute_waypoints_flow_control(waypoints_with_types)
        
        # Mark trajectory as inactive
        self.trajectory_active = False
        
        if success:
            goal_handle.succeed()
            result = FollowJointTrajectory.Result()
            result.error_code = 0
            self.get_logger().info('✅ Trajectory execution succeeded')
            return result
        else:
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = -1
            self.get_logger().error('❌ Trajectory execution failed')
            return result
    
    def cancel_trajectory_callback(self, goal_handle):
        """Handle trajectory cancellation requests"""
        self.get_logger().warn('🛑 Trajectory cancellation requested')
        self.abort_trajectory()
        return rclpy.action.CancelResponse.ACCEPT
    
    def abort_trajectory(self):
        """
        Abort current trajectory and clear Teensy buffer
        Sends ABORT command (motion_type = 3)
        """
        self.get_logger().warn('⚠ Aborting trajectory...')
        
        # Send ABORT command with current positions to stop smoothly
        with self.position_lock:
            positions_deg = [p * 180.0 / 3.14159 for p in self.current_positions]
        
        # Motion type 3 = ABORT
        cmd = f"W {','.join(f'{p:.4f}' for p in positions_deg)},3\n"
        
        try:
            self.serial.write(cmd.encode())
            self.serial.flush()
            self.get_logger().info('→ ABORT command sent')
            
            # Wait for acknowledgment
            if self.wait_for_feedback('ABORTED', timeout=2.0):
                self.get_logger().info('✓ Trajectory aborted and buffer cleared')
            else:
                self.get_logger().warn('⚠ No ABORTED acknowledgment')
                
        except Exception as e:
            self.get_logger().error(f'Abort command failed: {e}')
        
        # Reset state
        self.teensy_executing = False
        self.teensy_buffer_count = 0
        self.trajectory_active = False
        
        # Clear feedback queue
        while not self.feedback_queue.empty():
            try:
                self.feedback_queue.get_nowait()
            except queue.Empty:
                break
    
    def serial_read_loop(self):
        """Background thread to read Teensy feedback"""
        while rclpy.ok():
            try:
                if self.serial.in_waiting:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    
                    # Put all feedback in queue
                    self.feedback_queue.put(line)
                    
                    # Also process for joint states and logging
                    self.process_teensy_feedback(line)
                    
            except Exception as e:
                self.get_logger().error(f'Serial read error: {e}')
            time.sleep(0.005)  # 200Hz polling
    
    def process_teensy_feedback(self, line):
        """Process feedback from Teensy"""
        if line.startswith('<') and line.endswith('>'):
            # Joint state feedback
            try:
                positions_str = line[1:-1]
                positions = [float(x) for x in positions_str.split(',')]
                
                if len(positions) == 5:
                    with self.position_lock:
                        self.current_positions = positions
                    
                    # Publish to ROS2
                    msg = JointState()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.name = self.joint_names
                    msg.position = positions
                    self.joint_state_pub.publish(msg)
                    
            except Exception as e:
                self.get_logger().debug(f'Failed to parse joint states: {e}')
        
        elif line in ['BUFFERED', 'BUFFER_FULL', 'BUFFER_CLEARED', 'EXECUTING', 
                      'REACHED_WP1', 'REACHED_WP2', 'REACHED_WP3', 'REACHED_WP4', 
                      'REACHED_WP5', 'COMPLETE', 'STOPPED', 'ABORTED']:
            # Status messages
            msg = String()
            msg.data = line
            self.status_pub.publish(msg)
            
            # Update internal state
            if line == 'BUFFERED':
                self.teensy_buffer_count += 1
            elif line == 'BUFFER_CLEARED':
                self.teensy_buffer_count = 0
            elif line == 'EXECUTING':
                self.teensy_executing = True
            elif line.startswith('REACHED_WP'):
                self.teensy_buffer_count = max(0, self.teensy_buffer_count - 1)
            elif line in ['COMPLETE', 'ABORTED', 'STOPPED']:
                self.teensy_executing = False
                if line == 'ABORTED':
                    self.teensy_buffer_count = 0
        
        elif line.startswith('ERROR:'):
            self.get_logger().error(f'Teensy error: {line}')
    
    def joint_state_callback(self, msg):
        """Update current positions from joint states"""
        with self.position_lock:
            if len(msg.position) >= 5:
                self.current_positions = list(msg.position[:5])
    
    def filter_waypoints_from_points(self, waypoints):
        """Filter and normalize waypoints"""
        if len(waypoints) == 0:
            return []
        
        # Calculate total trajectory distance
        total_distance = 0.0
        for i in range(1, len(waypoints)):
            dist = sum(abs(waypoints[i][j] - waypoints[i-1][j]) for j in range(5))
            total_distance += dist
        
        # Adaptive filtering based on total distance
        if total_distance < 0.5:  # Small move
            self.min_distance = 0.05
        else:  # Large move
            self.min_distance = 0.25
        
        # Normalize all angles to prevent 360° jumps
        normalized = []
        for wp in waypoints:
            norm_wp = []
            for angle in wp:
                # Normalize to [-π, π]
                while angle > 3.14159:
                    angle -= 6.28318
                while angle < -3.14159:
                    angle += 6.28318
                norm_wp.append(angle)
            normalized.append(norm_wp)
        
        # Filter based on minimum distance
        filtered = [normalized[0]]
        
        for wp in normalized[1:]:
            distance = sum(abs(p - l) for p, l in zip(wp, filtered[-1]))
            
            if distance > self.min_distance:
                filtered.append(wp)
            
            if len(filtered) >= self.max_waypoints:
                break
        
        # Always include final waypoint
        if filtered[-1] != normalized[-1]:
            filtered.append(normalized[-1])
        
        return filtered
    
    def assign_motion_types(self, waypoints):
        """
        Assign START/VIA/STOP motion types to waypoints
        
        Motion Types:
        0 = START - First waypoint, clears buffer, begins new trajectory
        1 = VIA   - Intermediate waypoint, blends motion
        2 = STOP  - Final waypoint, decelerates to stop
        3 = ABORT - Emergency stop, clears buffer (handled separately)
        
        Returns list of (positions, motion_type) tuples
        """
        if len(waypoints) == 0:
            return []
        
        result = []
        for i, wp in enumerate(waypoints):
            if i == 0:
                motion_type = 0  # START - clears old buffer
            elif i == len(waypoints) - 1:
                motion_type = 2  # STOP
            else:
                motion_type = 1  # VIA
            
            result.append((wp, motion_type))
        
        return result
    
    def execute_waypoints_flow_control(self, waypoints_with_types):
        """
        Execute waypoints with 5-waypoint queue and flow control
        
        Protocol:
        1. First waypoint has START flag (motion_type=0) - clears Teensy buffer
        2. Send first min_buffer_to_start waypoints
        3. Wait for EXECUTING
        4. For each REACHED_WPx, send next waypoint
        5. Wait for COMPLETE
        
        Returns True if successful
        """
        self.get_logger().info(f'🎬 Executing {len(waypoints_with_types)} waypoints with flow control...')
        
        # Clear feedback queue
        while not self.feedback_queue.empty():
            try:
                self.feedback_queue.get_nowait()
            except queue.Empty:
                break
        
        # Reset state
        self.teensy_buffer_count = 0
        self.teensy_executing = False
        
        total_waypoints = len(waypoints_with_types)
        next_wp_to_send = 0
        completed_waypoints = 0
        
        # Phase 1: Initial buffer fill
        initial_fill = min(self.min_buffer_to_start, total_waypoints)
        self.get_logger().info(f'📤 Phase 1: Filling buffer with {initial_fill} waypoints...')
        
        # First waypoint MUST be START (motion_type=0) to clear old buffer
        positions, motion_type = waypoints_with_types[0]
        if motion_type != 0:
            self.get_logger().warn('⚠ First waypoint should be START, fixing...')
            waypoints_with_types[0] = (positions, 0)
        
        for i in range(initial_fill):
            positions, motion_type = waypoints_with_types[i]
            positions_deg = [p * 180.0 / 3.14159 for p in positions]
            
            success = self.send_waypoint_with_ack(positions_deg, motion_type, timeout=5.0)
            if not success:
                self.get_logger().error(f'❌ Failed to buffer waypoint {i+1}')
                self.abort_trajectory()
                return False
            
            type_name = ['START', 'VIA', 'STOP', 'ABORT'][motion_type]
            if motion_type == 0:
                type_name += ' (buffer cleared)'
            
            self.get_logger().info(f'  → Buffered WP {i+1}/{total_waypoints} ({type_name})')
            next_wp_to_send += 1
            
            time.sleep(0.05)  # 50ms between sends
        
        # Wait for EXECUTING
        self.get_logger().info('⏳ Waiting for EXECUTING...')
        if not self.wait_for_feedback('EXECUTING', timeout=3.0):
            self.get_logger().error('❌ Teensy did not start executing!')
            self.abort_trajectory()
            return False
        
        self.get_logger().info('✓ Motion started!')
        
        # Phase 2: Flow-controlled execution
        while completed_waypoints < total_waypoints:
            # Wait for waypoint completion or COMPLETE
            feedback = self.wait_for_feedback_any(
                ['REACHED_WP1', 'REACHED_WP2', 'REACHED_WP3', 'REACHED_WP4', 
                 'REACHED_WP5', 'COMPLETE', 'ABORTED'],
                timeout=20.0
            )
            
            if feedback is None:
                self.get_logger().error(f'❌ Timeout waiting for waypoint completion!')
                self.abort_trajectory()
                return False
            
            if feedback == 'ABORTED':
                self.get_logger().warn('⚠ Trajectory was aborted')
                return False
            
            if feedback == 'COMPLETE':
                self.get_logger().info('✅ Trajectory complete!')
                return True
            
            if feedback.startswith('REACHED_WP'):
                completed_waypoints += 1
                self.get_logger().info(f'✓ Completed waypoint {completed_waypoints}/{total_waypoints}')
                
                # Send next waypoint if available
                if next_wp_to_send < total_waypoints and self.teensy_buffer_count < self.buffer_size:
                    positions, motion_type = waypoints_with_types[next_wp_to_send]
                    positions_deg = [p * 180.0 / 3.14159 for p in positions]
                    
                    success = self.send_waypoint_with_ack(positions_deg, motion_type, timeout=3.0)
                    if success:
                        type_name = ['START', 'VIA', 'STOP', 'ABORT'][motion_type]
                        self.get_logger().info(f'  → Sent WP {next_wp_to_send+1}/{total_waypoints} ({type_name})')
                        next_wp_to_send += 1
                    else:
                        self.get_logger().error(f'❌ Failed to send waypoint {next_wp_to_send+1}')
                        self.abort_trajectory()
                        return False
        
        # Wait for final COMPLETE
        if self.wait_for_feedback('COMPLETE', timeout=5.0):
            self.get_logger().info('🎯 Motion complete!')
            return True
        else:
            self.get_logger().warn('⚠ Did not receive COMPLETE, but all waypoints reached')
            return True
    
    def send_waypoint_with_ack(self, positions_deg, motion_type, timeout=3.0):
        """
        Send waypoint and wait for acknowledgment
        
        Args:
            positions_deg: Joint positions in degrees
            motion_type: 0=START, 1=VIA, 2=STOP, 3=ABORT
            timeout: Seconds to wait for acknowledgment
        
        Returns True if successfully buffered
        """
        cmd = f"W {','.join(f'{p:.4f}' for p in positions_deg)},{motion_type}\n"
        
        try:
            # Send command
            self.serial.write(cmd.encode())
            self.serial.flush()
            
            # For START command, also expect BUFFER_CLEARED
            if motion_type == 0:
                if not self.wait_for_feedback('BUFFER_CLEARED', timeout=1.0):
                    self.get_logger().warn('⚠ No BUFFER_CLEARED after START')
            
            # Wait for BUFFERED or BUFFER_FULL
            start = time.time()
            while time.time() - start < timeout:
                try:
                    response = self.feedback_queue.get(timeout=0.1)
                    
                    if response == 'BUFFERED':
                        return True
                    elif response == 'BUFFER_FULL':
                        self.get_logger().warn('Buffer full, waiting...')
                        time.sleep(0.5)
                        # Retry send
                        self.serial.write(cmd.encode())
                        self.serial.flush()
                        
                except queue.Empty:
                    continue
            
            self.get_logger().error(f'No BUFFERED acknowledgment within {timeout}s')
            return False
            
        except Exception as e:
            self.get_logger().error(f'Send error: {e}')
            return False
    
    def wait_for_feedback(self, expected, timeout=10.0):
        """Wait for specific feedback message"""
        start = time.time()
        while time.time() - start < timeout:
            try:
                response = self.feedback_queue.get(timeout=0.1)
                if response == expected:
                    return True
            except queue.Empty:
                continue
        
        return False
    
    def wait_for_feedback_any(self, expected_list, timeout=10.0):
        """Wait for any of the expected feedback messages"""
        start = time.time()
        while time.time() - start < timeout:
            try:
                response = self.feedback_queue.get(timeout=0.1)
                if response in expected_list:
                    return response
            except queue.Empty:
                continue
        
        return None
    
    def emergency_stop(self):
        """Send emergency stop (same as abort)"""
        self.get_logger().warn('🚨 EMERGENCY STOP')
        self.abort_trajectory()
    
    def destroy_node(self):
        """Cleanup"""
        if hasattr(self, 'serial') and self.serial.is_open:
            self.serial.write(b'@MANUAL\n')
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
