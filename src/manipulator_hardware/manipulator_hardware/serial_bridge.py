import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import serial
import threading


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
        
        # Subscribe to commands from TopicBasedSystem (JointState type)
        self.cmd_sub = self.create_subscription(
            JointState,
            '/hardware/motor_commands',
            self.command_callback,
            10
        )
        
        # Publish states to TopicBasedSystem (JointState type)
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
        
        # Start Connection
        if not self.simulate:
            self.connect_serial()
            
        # Timers
        self.create_timer(0.02, self.control_loop)    # 50Hz
        self.create_timer(1.0, self.diagnostic_loop)  # 1Hz
        
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
        """Receive joint commands from TopicBasedSystem (as JointState)"""
        with self.lock:
            if len(msg.position) >= 5:
                self.latest_commands = list(msg.position)[:5]
                self.get_logger().info(f"Received CMD: [{', '.join([f'{x:.3f}' for x in self.latest_commands])}]")

    def control_loop(self):
        """Main control loop"""
        
        # --- SIMULATION MODE ---
        if self.simulate:
            with self.lock:
                # Smoothly approach commanded positions
                for i in range(5):
                    delta = self.latest_commands[i] - self.current_positions[i]
                    max_step = 0.1  # radians per 20ms cycle (~5 rad/s max velocity)
                    
                    if abs(delta) > 0.001:  # Still moving
                        step = max(min(delta, max_step), -max_step)
                        self.current_positions[i] += step
                        self.current_velocities[i] = step / 0.02  # rad/s
                    else:  # Reached target
                        self.current_positions[i] = self.latest_commands[i]
                        self.current_velocities[i] = 0.0
                        
            self.publish_state()
            return

        # --- REAL HARDWARE MODE ---
        if not self.serial_conn or not self.serial_conn.is_open:
            return

        try:
            # Send commands to Teensy
            with self.lock:
                cmd_str = ",".join([f"{x:.3f}" for x in self.latest_commands])
            
            packet = f"<{cmd_str}>\n".encode('utf-8')
            self.serial_conn.write(packet)
            
            # Read feedback from Teensy
            if self.serial_conn.in_waiting:
                raw_line = self.serial_conn.readline()
                try:
                    line = raw_line.decode('utf-8').strip()
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
            self.connect_serial()

    def publish_state(self):
        """Publish current joint states to TopicBasedSystem"""
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
        elif self.serial_conn and self.serial_conn.is_open:
            stat.level = DiagnosticStatus.OK
            stat.message = "Connected"
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
