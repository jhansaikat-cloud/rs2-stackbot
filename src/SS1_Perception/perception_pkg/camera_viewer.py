import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraViewer(Node):
    def __init__(self):
        # Give this node a name
        super().__init__('camera_viewer')
        self.get_logger().info('Camera viewer node started!')

        # CvBridge converts ROS2 image messages into OpenCV images
        self.bridge = CvBridge()

        # Subscribe to the colour camera topic
        # When a new image arrives, call self.image_callback
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10  # queue size
        )

    def image_callback(self, msg):
        # Convert the ROS2 image message to an OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Display it in a window
        cv2.imshow('Camera Feed', frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = CameraViewer()
    rclpy.spin(node)       # keeps the node running
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
