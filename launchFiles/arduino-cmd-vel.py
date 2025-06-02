import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class CmdVelToSerial(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_serial')
        
        # Set your serial port and baudrate here
        self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10
        )
        self.subscription

    def listener_callback(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z
        data = f"{linear},{angular}\t"
        
        try:
            self.serial_port.write(data.encode())
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToSerial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
