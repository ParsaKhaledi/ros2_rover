import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import sys
import time

# =========================
# Configuration parameters
# =========================
DEFAULT_PORT = '/dev/ttyUSB0'
DEFAULT_BAUDRATE = 9600
DEFAULT_TIMEOUT = 1.0  # in seconds
DATA_TIMEOUT = 2.0  # time to wait before sending zero velocity
QUEUE_SIZE = 10
DEFAULT_TOPIC = '/cmd_vel'

# ========================
# Node class declaration
# ========================
class CmdVelToSerial(Node):
    def __init__(self, port, baudrate, timeout):
        super().__init__('cmd_vel_to_serial')
        self.serial_port = self.open_serial(port, baudrate, timeout)
        self.last_data_time = time.time()
        self.timer = self.create_timer(0.5, self.check_timeout)
        self.latest_data = "0.0,0.0\t"  # Initialize with zero velocity
        self.create_subscription(Twist, DEFAULT_TOPIC, self.listener_callback, QUEUE_SIZE)

    def open_serial(self, port, baudrate, timeout):
        try:
            return serial.Serial(port, baudrate, timeout=timeout)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            sys.exit(1)

    def listener_callback(self, msg: Twist):
        """ Stores the latest subscribed data instead of writing to serial directly """
        self.last_data_time = time.time()
        self.latest_data = f"{msg.linear.x},{msg.angular.z}\t"

    def write_to_serial(self, data):
        """ Writes the passed data to the serial port """
        try:
            self.serial_port.write(data.encode())
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write failed: {e}")

    def check_timeout(self):
        """ Sends zero velocity if no data is received for a certain timeout """
        if time.time() - self.last_data_time > DATA_TIMEOUT:
            self.latest_data = "0.0,0.0\t"

    def get_latest_data(self):
        """ Returns the latest received data """
        return self.latest_data

    def terminate(self):
        """ Closes the serial connection and destroys the node """
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.destroy_node()

# ========================
# Main function
# ========================
def main(args=None):
    rclpy.init(args=args)

    # Command-line argument handling
    try:
        port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
        baudrate = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_BAUDRATE
        timeout = float(sys.argv[3]) if len(sys.argv) > 3 else DEFAULT_TIMEOUT
    except ValueError:
        print("Invalid argument types. Usage: script.py [port] [baudrate] [timeout]")
        sys.exit(1)

    if timeout <= 0:
        print("Timeout must be positive.")
        sys.exit(1)

    node = CmdVelToSerial(port, baudrate, timeout)
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)  # Process ROS 2 callbacks
            latest_data = node.get_latest_data()  # Retrieve latest received data
            node.write_to_serial(latest_data)  # Write latest data to serial
    except KeyboardInterrupt:
        pass
    finally:
        node.terminate()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
