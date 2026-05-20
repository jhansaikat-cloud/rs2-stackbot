import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class HSVTuner(Node):
    def __init__(self):
        super().__init__('hsv_tuner')
        self.bridge = CvBridge()
        self.frame = None

        # Subscribe to camera
        self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )

        # Create the tuner window with sliders
        cv2.namedWindow('HSV Tuner')
        cv2.createTrackbar('H min', 'HSV Tuner', 0,   179, lambda x: None)
        cv2.createTrackbar('H max', 'HSV Tuner', 179, 179, lambda x: None)
        cv2.createTrackbar('S min', 'HSV Tuner', 0,   255, lambda x: None)
        cv2.createTrackbar('S max', 'HSV Tuner', 255, 255, lambda x: None)
        cv2.createTrackbar('V min', 'HSV Tuner', 0,   255, lambda x: None)
        cv2.createTrackbar('V max', 'HSV Tuner', 255, 255, lambda x: None)

        # Run the display loop at 30fps using a ROS2 timer
        self.create_timer(0.033, self.display_loop)
        self.get_logger().info('HSV Tuner ready! Adjust sliders to find your colour range.')

    def image_callback(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def display_loop(self):
        if self.frame is None:
            return

        # Read current slider values
        h_min = cv2.getTrackbarPos('H min', 'HSV Tuner')
        h_max = cv2.getTrackbarPos('H max', 'HSV Tuner')
        s_min = cv2.getTrackbarPos('S min', 'HSV Tuner')
        s_max = cv2.getTrackbarPos('S max', 'HSV Tuner')
        v_min = cv2.getTrackbarPos('V min', 'HSV Tuner')
        v_max = cv2.getTrackbarPos('V max', 'HSV Tuner')

        # Convert to HSV and apply mask
        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv, lower, upper)

        # Show only the detected colour on a black background
        result = cv2.bitwise_and(self.frame, self.frame, mask=mask)

        # Print current values to terminal so you can copy them
        print(f'\rH:[{h_min}-{h_max}] S:[{s_min}-{s_max}] V:[{v_min}-{v_max}]', end='')

        # Show both windows side by side
        cv2.imshow('HSV Tuner', self.frame)
        cv2.imshow('Mask (white = detected)', mask)
        cv2.imshow('Result', result)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = HSVTuner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
