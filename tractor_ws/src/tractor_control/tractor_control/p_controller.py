import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import math
import numpy as np
import pandas as pd
import time

# Create a DataFrame to store the log of location and heading (for visualization)
log_data = {"time": [], "x": [], "y": [], "heading": []}
df = pd.DataFrame(log_data)

log_x, log_y, log_heading = 0.0, 0.0, 0
start_time = time.time()

def lla_to_ecef(lat, lon, alt):
    # WGS84 constants
    a = 6378137.0  # Equatorial radius in meters
    e2 = 6.69437999014e-3  # Square of eccentricity

    lat_rad = np.radians(lat)
    lon_rad = np.radians(lon)

    N = a / np.sqrt(1 - e2 * (np.sin(lat_rad) ** 2))

    x = (N + alt) * np.cos(lat_rad) * np.cos(lon_rad)
    y = (N + alt) * np.cos(lat_rad) * np.sin(lon_rad)
    z = (N * (1 - e2) + alt) * np.sin(lat_rad)

    return np.array([x, y, z])

class SimpleController(Node):
    def __init__(self):
        super().__init__('simple_controller')

        # Subscriber for GPS
        self.gps_sub = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.imu_pub = self.create_subscription(String, 'imu/heading', self.gps_callback, 10)
        self.twist = Twist()

        # Publisher for velocity command (Twist)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Inition Position
        self.initial_ecef = None
        self.initial_lla = None
        self.initial_enu = None

        # Target Goal
        self.goal = [5, 5]

        self.current_pos = None
        self.current_heading = 0.0

    def gps_callback(self, msg):
        if self.initial_enu is None:
            self.initial_lla = [msg.latitude, msg.longitude, 0]
            self.initial_ecef = lla_to_ecef(self.initial_lla[0], self.initial_lla[1], 0)
            self.initial_enu = self.ecef_to_enu(self.initial_ecef, self.initial_ecef)
        if self.current_pos is None:
            self.current_pos = self.ecef_to_enu(self.initial_ecef, self.initial_ecef)
        else:
            self.current_pos = self.ecef_to_enu(lla_to_ecef(msg.latitude, msg.longitude, 0), self.initial_ecef)
        self.control_loop()

    def imu_callback(self, msg):
        try:
            self.current_heading = float(msg.data)
            self.get_logger().info(f"IMU heading: {self.current_heading} degrees")
        except ValueError:
            self.get_logger().error(f"Invalid IMU heading received: {msg.data}")

    def control_loop(self):
        if self.current_pos is not None:
            x = self.current_pos[0]
            y = self.current_pos[1]
            current_time = time.time() - start_time

            # Log the data
            log_data = {"time": [current_time], "x": [x], "y": [y], "heading": [self.current_heading]}
            df = pd.concat([df, pd.DataFrame(log_data)], ignore_index=True)

            # Calculate distance to goal
            delta_pos = math.sqrt((self.goal[0] - x)**2 + (self.goal[1] - y)**2)
            desired_theta = math.degrees(math.atan2(self.goal[1] - y, self.goal[0] - x))
            heading_error = desired_theta - self.current_heading
            heading_error = (heading_error + 180) % 360 - 180  # Normalize to [-180, 180]

            if delta_pos > 1:
                # Proportional control for linear and angular velocity
                linear_vel = 0.5 * delta_pos  # Adjust this factor to tune the speed
                angular_vel = 0.05 * heading_error  # Adjust this factor to tune the turning rate

                # Clamp values
                linear_vel = max(min(linear_vel, 5.0), -5.0)
                angular_vel = max(min(angular_vel, 2.0), -2.0)
                # Create and publish the Twist message
                self.twist.linear.x = linear_vel
                self.twist.angular.z = angular_vel
                self.vel_pub.publish(self.twist)
            else:
                # If within threshold, stop
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.vel_pub.publish(self.twist)

    def ecef_to_enu(self, ecef, ref_ecef):
        # ENU transformation
        x, y, z = ecef
        ref_x, ref_y, ref_z = ref_ecef

        # Calculate differences
        dx = x - ref_x
        dy = y - ref_y
        dz = z - ref_z

        # Reference latitude and longitude in radians
        ref_lat_rad = np.radians(self.initial_lla[0])
        ref_lon_rad = np.radians(self.initial_lla[1])

        # Calculate ENU
        e = -np.sin(ref_lon_rad) * dx + np.cos(ref_lon_rad) * dy
        n = -np.sin(ref_lat_rad) * np.cos(ref_lon_rad) * dx - np.sin(ref_lat_rad) * np.sin(ref_lon_rad) * dy + np.cos(ref_lat_rad) * dz
        u = np.cos(ref_lat_rad) * np.cos(ref_lon_rad) * dx + np.cos(ref_lat_rad) * np.sin(ref_lon_rad) * dy + np.sin(ref_lat_rad) * dz

        return np.array([e, n, u])

def main(args=None):
    rclpy.init(args=args)
    controller = SimpleController()

    rclpy.spin(controller)
    df.to_csv("tractor_data_log.csv", index=False)
    print("Data saved to tractor_data_log.csv")
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
