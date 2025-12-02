import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import serial
import threading
import time


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('simulate', False)
        
        self.simulate = self.get_parameter('simulate').value
        self.port_name = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baud_rate').value
        
        # Subscribe to commands from TopicBasedSystem
        self.cmd_sub = self.create_subscription(
            JointState,
            '/hardware/motor_commands',
            self.command_callback,
            10
        )
        
        # Publish states to TopicBasedSystem
        self.state_pub = self.create_publisher(
            JointState,
            '/hardware/motor_states',
            10
        )
        
        # Diagnostics
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 1)

        # Storage
        self.latest_commands = [0.0] * 5
        self.current_positions = [0.0] * 5
        self.current_velocities = [0.0] * 5
        self.serial_conn = None
        self.lock = threading.Lock()
        
        # Initialization flag
        self.initialized = False
        
        # Start Connection
        if not self.simulate:
            self.connect_serial()
            if self.serial_conn and self.serial_conn.is_open:
                self.get_initial_position()
        else:
            self.initialized = True
            self.get_logger().info("Simulation mode: Starting at zero position")
            
        # Timers
        self.create_timer(0.02, self.control_loop)    # 50Hz
        self.create_timer(1.0, self.diagnostic_loop)  # 1Hz
        
        self.get_logger().info(f"Bridge Started. Mode: {'SIMULATION' if self.simulate else 'REAL HARDWARE'}")

    def connect_serial(self):
        """Connect to Teensy"""
        try:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
            self.serial_conn = serial.Serial(self.port_name, self.baud, timeout=0.1)
            time.sleep(2)  # Wait for Teensy to reset
            self.get_logger().info(f'Connected to {self.port_name}')
        except Exception as e:
            self.get_logger().error(f'Serial Connection Failed: {e}')

    def get_initial_position(self):
        """Read initial positions from Teensy"""
        if not self.serial_conn or not self.serial_conn.is_open:
            self.get_logger().error("Cannot initialize: Serial not connected")
            return
        
        self.get_logger().info("Reading initial joint positions from Teensy...")
        
        # Clear buffer
        self.serial_conn.reset_input_buffer()
        
        # Send STATUS request
        self.serial_conn.write(b"<STATUS>\n")
        
        max_attempts = 100
        for attempt in range(max_attempts):
            if self.serial_conn.in_waiting:
                raw_line = self.serial_conn.readline()
                try:
                    line = raw_line.decode('utf-8', errors='ignore').strip()
                    self.get_logger().debug(f"Received: {line}")
                    
                    if line.startswith('<') and line.endswith('>'):
                        content = line[1:-1]
                        parts = content.split(',')
                        if len(parts) == 5:
                            with self.lock:
                                self.current_positions = [float(p) for p in parts]
                                self.latest_commands = list(self.current_positions)
                            self.initialized = True
                            self.get_logger().info(f"✓ Initial positions [rad]: [{', '.join([f'{x:.3f}' for x in self.current_positions])}]")
                            return
                except Exception as e:
                    self.get_logger().debug(f"Parse error: {e}")
            
            time.sleep(0.02)
        
        self.get_logger().error("❌ Failed to read initial position from Teensy!")
        self.get_logger().warn("   Robot will NOT move until position is known.")

    def command_callback(self, msg):
        """Receive joint commands from TopicBasedSystem"""
        if not self.initialized:
            self.get_logger().warn("Ignoring command: Robot not initialized!", throttle_duration_sec=1.0)
            return
        
        with self.lock:
            if len(msg.position) >= 5:
                # Safety check
                for i in range(5):
                    delta = abs(msg.position[i] - self.current_positions[i])
                    if delta > 1.5:  # ~86 degrees
                        self.get_logger().error(f"Joint {i+1}: Dangerous jump of {delta:.2f} rad! IGNORING.")
                        return
                
                self.latest_commands = list(msg.position)[:5]
                self.get_logger().info(f"CMD: [{', '.join([f'{x:.3f}' for x in self.latest_commands])}]")

    def control_loop(self):
        """Main control loop"""
        
        # If not initialized, keep trying
        if not self.initialized and not self.simulate:
            if self.serial_conn and self.serial_conn.is_open:
                self.get_initial_position()
            return
        
        # --- SIMULATION MODE ---
        if self.simulate:
            with self.lock:
                for i in range(5):
                    delta = self.latest_commands[i] - self.current_positions[i]
                    max_step = 0.1
                    
                    if abs(delta) > 0.001:
                        step = max(min(delta, max_step), -max_step)
                        self.current_positions[i] += step
                        self.current_velocities[i] = step / 0.02
                    else:
                        self.current_positions[i] = self.latest_commands[i]
                        self.current_velocities[i] = 0.0
                        
            self.publish_state()
            return

        # --- REAL HARDWARE MODE ---
        if not self.serial_conn or not self.serial_conn.is_open:
            return

        try:
            # Send commands to Teensy (in radians)
            with self.lock:
                cmd_str = ",".join([f"{x:.4f}" for x in self.latest_commands])
            
            packet = f"<{cmd_str}>\n".encode('utf-8')
            self.serial_conn.write(packet)
            
            # Read feedback from Teensy
            if self.serial_conn.in_waiting:
                raw_line = self.serial_conn.readline()
                try:
                    line = raw_line.decode('utf-8', errors='ignore').strip()
                    
                    # Log ALL messages from Teensy (not just position data)
                    if not (line.startswith('<') and line.endswith('>')):
                        self.get_logger().info(f"[Teensy]: {line}")
                    
                    if line.startswith('<') and line.endswith('>'):
                        content = line[1:-1]
                        parts = content.split(',')
                        if len(parts) == 5:
                            with self.lock:
                                new_positions = [float(p) for p in parts]
                                # Calculate velocities
                                for i in range(5):
                                    self.current_velocities[i] = (new_positions[i] - self.current_positions[i]) / 0.02
                                self.current_positions = new_positions
                except UnicodeDecodeError:
                    pass

                            
            self.publish_state()
            
        except Exception as e:
            self.get_logger().warn(f'Serial IO Error: {e}')
            self.initialized = False
            self.connect_serial()

    def publish_state(self):
        """Publish current joint states"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
        
        with self.lock:
            msg.position = list(self.current_positions)
            msg.velocity = list(self.current_velocities)
            
        self.state_pub.publish(msg)

    def diagnostic_loop(self):
        """Report health status"""
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        stat = DiagnosticStatus()
        stat.name = "Serial Bridge"
        stat.hardware_id = "Manipulator Hardware"
        
        if self.simulate:
            stat.level = DiagnosticStatus.OK
            stat.message = "Simulation Mode"
        elif self.serial_conn and self.serial_conn.is_open and self.initialized:
            stat.level = DiagnosticStatus.OK
            stat.message = "Connected & Initialized"
        elif self.serial_conn and self.serial_conn.is_open:
            stat.level = DiagnosticStatus.WARN
            stat.message = "Connected but NOT initialized"
        else:
            stat.level = DiagnosticStatus.ERROR
            stat.message = "Disconnected"
            
        arr.status.append(stat)
        self.diag_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
