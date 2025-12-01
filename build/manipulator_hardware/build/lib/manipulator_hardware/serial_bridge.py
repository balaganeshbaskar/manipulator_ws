import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import serial
import threading
import time

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0') # Default for Teensy
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('simulate', False)
        
        self.simulate = self.get_parameter('simulate').value
        self.port_name = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baud_rate').value
        
        # 1. Subscribe to Commands (from ROS 2 Control)
        # TopicBasedSystem sends commands here
        self.cmd_sub = self.create_subscription(
            Float64MultiArray, 
            '/hardware/motor_commands', 
            self.command_callback, 
            10
        )
        
        # 2. Publish States (to ROS 2 Control)
        # TopicBasedSystem reads states from here
        self.state_pub = self.create_publisher(
            JointState, 
            '/hardware/motor_states', 
            10
        )
        
        # 3. Diagnostics (Health Reporting)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 1)

        # Storage
        self.latest_commands = [0.0] * 5
        self.current_positions = [0.0] * 5
        self.serial_conn = None
        self.lock = threading.Lock()
        
        # Start Connection
        if not self.simulate:
            self.connect_serial()
            
        # Timers
        self.create_timer(0.02, self.control_loop)    # 50Hz Control Loop
        self.create_timer(1.0, self.diagnostic_loop)  # 1Hz Health Check
        
        self.get_logger().info(f"Bridge Started. Mode: {'SIMULATION' if self.simulate else 'REAL HARDWARE'}")

    def connect_serial(self):
        try:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
            self.serial_conn = serial.Serial(self.port_name, self.baud, timeout=0.01)
            self.get_logger().info(f'Connected to {self.port_name}')
        except Exception as e:
            self.get_logger().error(f'Serial Connection Failed: {e}')

    def command_callback(self, msg):
        """Store latest command from MoveIt/ROS2Control"""
        with self.lock:
            if len(msg.data) >= 5:
                self.latest_commands = list(msg.data)[:5]

    def control_loop(self):
        """Main 50Hz Loop: Write to Serial -> Read from Serial -> Publish State"""
        
        # --- SIMULATION MODE ---
        if self.simulate:
            with self.lock:
                # Perfectly mimic motors (Instant move)
                self.current_positions = list(self.latest_commands)
            self.publish_state()
            return

        # --- REAL HARDWARE MODE ---
        if not self.serial_conn or not self.serial_conn.is_open:
            return

        try:
            # 1. SEND COMMANDS
            # Format: <J1,J2,J3,J4,J5> e.g., <90.0,0.0,0.0,0.0,0.0>
            with self.lock:
                cmd_str = ",".join([f"{x:.3f}" for x in self.latest_commands])
            
            packet = f"<{cmd_str}>\n".encode('utf-8')
            self.serial_conn.write(packet)
            
            # 2. READ FEEDBACK
            # Expects: <P1,P2,P3,P4,P5>
            if self.serial_conn.in_waiting:
                raw_line = self.serial_conn.readline()
                try:
                    line = raw_line.decode('utf-8').strip()
                    if line.startswith('<') and line.endswith('>'):
                        content = line[1:-1]
                        parts = content.split(',')
                        if len(parts) == 5:
                            with self.lock:
                                self.current_positions = [float(p) for p in parts]
                except UnicodeDecodeError:
                    pass # Ignore partial/garbage bytes
                            
            self.publish_state()
            
        except Exception as e:
            self.get_logger().warn(f'Serial IO Error: {e}')
            self.connect_serial() # Auto-reconnect

    def publish_state(self):
        """Package current positions into JointState and publish"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
        with self.lock:
            msg.position = self.current_positions
        self.state_pub.publish(msg)

    def diagnostic_loop(self):
        """Report health status"""
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        stat = DiagnosticStatus()
        stat.name = "Teensy Bridge"
        stat.hardware_id = "SerialPort"
        
        if self.simulate:
            stat.level = DiagnosticStatus.OK
            stat.message = "Simulation Mode"
        elif self.serial_conn and self.serial_conn.is_open:
            stat.level = DiagnosticStatus.OK
            stat.message = "Connected"
        else:
            stat.level = DiagnosticStatus.ERROR
            stat.message = "Disconnected / Error"
            
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
