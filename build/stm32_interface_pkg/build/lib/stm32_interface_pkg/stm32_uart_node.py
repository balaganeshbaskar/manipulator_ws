import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String  # We'll customize this later for real joint msgs

class STM32UARTNode(Node):
    def __init__(self):
        super().__init__('stm32_uart_node')

        # Replace with your actual serial port and baudrate
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)

        self.pub = self.create_publisher(String, '/stm32_rx', 10)
        self.sub = self.create_subscription(String, '/stm32_tx', self.send_to_stm32, 10)

        self.timer = self.create_timer(0.05, self.read_from_stm32)  # 20Hz polling

        self.get_logger().info('STM32 UART Node started')

    def read_from_stm32(self):
        if self.serial_port.in_waiting:
            line = self.serial_port.readline().decode('utf-8').strip()
            if line:
                msg = String()
                msg.data = line
                self.pub.publish(msg)
                self.get_logger().info(f'<< RX: {line}')

    def send_to_stm32(self, msg: String):
        command = msg.data.strip() + '\n'
        self.serial_port.write(command.encode('utf-8'))
        self.get_logger().info(f'>> TX: {command.strip()}')

def main(args=None):
    rclpy.init(args=args)
    node = STM32UARTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down UART node')
    finally:
        node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()
