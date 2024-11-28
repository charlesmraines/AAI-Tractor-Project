import rclpy
from rclpy.node import Node
import cv2
import depthai as dai
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

class DepthAIPublisherNode(Node):
    def __init__(self):
        super().__init__('depthai_publisher')

        # ROS publishers
        self.rgb_pub = self.create_publisher(Image, 'camera/rgb', 10)
        self.depth_pub = self.create_publisher(Image, 'camera/depth', 10)
        self.distance_pub = self.create_publisher(Float32, 'camera/distance', 10)

        # Bridge for OpenCV to ROS image conversion
        self.bridge = CvBridge()

        # Setup DepthAI pipeline
        self.pipeline = dai.Pipeline()

        # Define mono cameras for depth input
        mono_left = self.pipeline.create(dai.node.MonoCamera)
        mono_right = self.pipeline.create(dai.node.MonoCamera)
        mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

        # Define stereo depth node
        stereo = self.pipeline.create(dai.node.StereoDepth)
        stereo.setConfidenceThreshold(200)
        stereo.setLeftRightCheck(True)
        stereo.setDepthAlign(dai.CameraBoardSocket.RGB)

        # Link mono cameras to stereo depth node
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        # Output streams
        color_cam = self.pipeline.create(dai.node.ColorCamera)
        # color_cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        color_cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_400_P) # Set RGB camera resolution to 400p
        color_cam.setFps(30) # Optionally set a lower FPS if desired

        xout_rgb = self.pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        color_cam.video.link(xout_rgb.input)

        xout_depth = self.pipeline.create(dai.node.XLinkOut)
        xout_depth.setStreamName("depth")
        stereo.depth.link(xout_depth.input)

        # Start DepthAI device
        self.device = dai.Device(self.pipeline)
        self.rgb_queue = self.device.getOutputQueue(name="rgb", maxSize=8, blocking=False)
        self.depth_queue = self.device.getOutputQueue(name="depth", maxSize=8, blocking=False)

        # Timer to periodically publish frames
        self.timer = self.create_timer(0.1, self.publish_frames)

    # Calculate mean of a 3x3 region around the center without NumPy
    def calculate_mean_3x3(self, depth_frame, center_x, center_y):
        total_sum = 0
        count = 0

        # Iterate over the 3x3 region
        for i in range(center_y - 1, center_y + 2):  # Rows from center_y-1 to center_y+1
            for j in range(center_x - 1, center_x + 2):  # Columns from center_x-1 to center_x+1
                # Check bounds to ensure we don't access invalid indices
                if 0 <= i < len(depth_frame) and 0 <= j < len(depth_frame[0]):
                    total_sum += depth_frame[i][j]
                    count += 1

        # Calculate mean
        mean_value = total_sum / count if count > 0 else 0
        return mean_value

    def publish_frames(self):
        # Get RGB frame
        in_rgb = self.rgb_queue.get()
        rgb_frame = in_rgb.getCvFrame()

        # Get depth frame
        in_depth = self.depth_queue.get()
        depth_frame = in_depth.getFrame()

        # Calculate distance at center
        height, width = depth_frame.shape
        center_x, center_y = width // 2, height // 2
        # distance_mm = depth_frame[center_y, center_x] # Get the depth value at center coordinates
        # Calculate average depth in a 3x3 region around the center
        # region = depth_frame[center_y-1:center_y+2, center_x-1:center_x+2]
        distance_mm = self.calculate_mean_3x3(depth_frame, center_x, center_y)
        distance_m = distance_mm / 1000.0  # Convert to meters

        # Publish distance
        distance_msg = Float32()
        distance_msg.data = distance_m
        self.distance_pub.publish(distance_msg)
        self.get_logger().info(f"Distance to object: {distance_m:.2f} meters")

        # Convert RGB frame to ROS Image and publish
        rgb_msg = self.bridge.cv2_to_imgmsg(rgb_frame, encoding="bgr8")
        self.rgb_pub.publish(rgb_msg)

        # Convert depth frame to ROS Image and publish
        depth_msg = self.bridge.cv2_to_imgmsg(depth_frame, encoding="mono16")
        self.depth_pub.publish(depth_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DepthAIPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
