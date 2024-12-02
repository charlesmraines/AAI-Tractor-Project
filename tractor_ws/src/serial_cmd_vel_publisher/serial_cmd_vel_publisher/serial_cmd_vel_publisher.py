import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String  # For publishing IMU heading as a string
import serial
import threading

class CmdVelSerialPublisher(Node):
    def __init__(self):
        super().__init__('serial_cmd_vel_publisher')

        # Initialize the serial connection (modify with your correct serial port)
        self.serial_port = '/dev/ttyACM0'  # Adjust for your serial device
        self.baud_rate = 9600
        self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)

        # Mutex for serial communication to prevent collisions
        self.serial_lock = threading.Lock()

        # Subscriber to the cmd_vel topic
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publisher for IMU heading
        self.imu_publisher = self.create_publisher(String, '/imu/heading', 10)

        # Timer to periodically read IMU data
        self.timer = self.create_timer(0.1, self.read_imu_data)  # 10 Hz frequency

    def cmd_vel_callback(self, msg: Twist):
        # Get linear and angular velocities
        linear_vel = msg.linear.x  # Adjust for other dimensions if needed
        angular_vel = msg.angular.z

        # Format the data as a string "linear_vel angular_vel"
        data = f"{linear_vel} {angular_vel}\n"

        # Use lock to ensure thread-safe serial writes
        with self.serial_lock:
            self.ser.write(data.encode('utf-8'))
        self.get_logger().info(f"Sent cmd_vel data: {data.strip()}")

    def read_imu_data(self):
        # Use lock to ensure thread-safe serial reads
        with self.serial_lock:
            if self.ser.in_waiting > 0:
                raw_data = self.ser.readline().decode('utf-8').strip()
        
        self.get_logger().info(f"Raw IMU data: {raw_data}")

        try:
            # Parse the IMU heading data (assuming it's a float in degrees)
            imu_heading = float(raw_data)
            imu_heading_str = f"{imu_heading:.2f}"  # Format as a string with 2 decimal places

            # Publish the IMU heading as a string
            imu_msg = String()
            imu_msg.data = imu_heading_str
            self.imu_publisher.publish(imu_msg)

            self.get_logger().info(f"Published IMU heading: {imu_heading_str} degrees")
        except ValueError:
            self.get_logger().error(f"Invalid IMU data received: {raw_data}")


def main(args=None):
    rclpy.init(args=args)

    node = CmdVelSerialPublisher()

    try:
        rclpy.spin(node)
    finally:
        node.ser.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
