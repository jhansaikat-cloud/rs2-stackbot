import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# ─────────────────────────────────────────
# HSV colour ranges — update these once you
# run the tuner with your actual cubes
# ─────────────────────────────────────────
COLOURS = {
    'blue': {
        'lower1': np.array([103, 93,  50]),
        'upper1': np.array([120, 255, 170]),
        'display': (255, 0, 0)
    },
    'yellow': {
        'lower1': np.array([22,  132, 135]),
        'upper1': np.array([28,  255, 217]),
        'display': (0, 220, 220)
    },
    'green': {
        'lower1': np.array([30,  65,  91]),
        'upper1': np.array([72,  199, 255]),
        'display': (0, 200, 0)
    },
    'orange': {
        'lower1': np.array([6,   139, 154]),
        'upper1': np.array([20,  255, 255]),
        'display': (0, 140, 255)
    },
    'red': {
        'lower1': np.array([0,   139, 154]),   # we'll tune red properly when cubes arrive
        'upper1': np.array([5,   255, 255]),
        'lower2': np.array([170, 139, 154]),
        'upper2': np.array([179, 255, 255]),
        'display': (0, 0, 255)
    },
}

# Minimum blob size in pixels — filters out noise
MIN_AREA = 1000


class CubeDetector(Node):
    def __init__(self):
        super().__init__('cube_detector')
        self.bridge = CvBridge()
        self.frame = None

        self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )

        self.create_timer(0.033, self.detection_loop)
        self.get_logger().info('Cube detector started!')

    def image_callback(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def detect_colour(self, hsv, colour_name, colour_info):
        """Returns a list of detected blobs for a given colour."""
        # Create mask (handle red's double range)
        mask = cv2.inRange(hsv, colour_info['lower1'], colour_info['upper1'])
        if 'lower2' in colour_info:
            mask2 = cv2.inRange(hsv, colour_info['lower2'], colour_info['upper2'])
            mask = cv2.bitwise_or(mask, mask2)

        # Clean up noise with morphological operations
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)  # removes small noise
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # fills small holes

        # Find contours (outlines of blobs)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        detections = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < MIN_AREA:
                continue  # skip tiny blobs (noise)

            # Get bounding box
            x, y, w, h = cv2.boundingRect(contour)

            # Get centre pixel
            cx = x + w // 2
            cy = y + h // 2

            # Confidence score based on how square the detection is
            # (cubes should look roughly square from above)
            squareness = min(w, h) / max(w, h)

            detections.append({
                'colour': colour_name,
                'bbox': (x, y, w, h),
                'centre': (cx, cy),
                'area': area,
                'confidence': squareness
            })

        return detections

    def draw_detections(self, frame, detections):
        """Draw bounding boxes and labels on the frame."""
        for det in detections:
            colour = COLOURS[det['colour']]['display']
            x, y, w, h = det['bbox']
            cx, cy = det['centre']

            # Draw bounding box
            cv2.rectangle(frame, (x, y), (x + w, y + h), colour, 2)

            # Draw centre dot
            cv2.circle(frame, (cx, cy), 5, colour, -1)

            # Draw label with confidence
            label = f"{det['colour']} ({det['confidence']:.2f})"
            cv2.putText(
                frame, label,
                (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 2
            )

            # Draw pixel coordinates
            coord_label = f"px:({cx},{cy})"
            cv2.putText(
                frame, coord_label,
                (x, y + h + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1
            )

        return frame

    def detection_loop(self):
        if self.frame is None:
            return

        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        display = self.frame.copy()

        all_detections = []
        for colour_name, colour_info in COLOURS.items():
            detections = self.detect_colour(hsv, colour_name, colour_info)
            all_detections.extend(detections)

        # Draw everything on the frame
        display = self.draw_detections(display, all_detections)

        # Show detection count in top left corner
        count_label = f"Detected: {len(all_detections)} cube(s)"
        cv2.putText(
            display, count_label,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2
        )

        # Log to terminal
        if all_detections:
            for det in all_detections:
                self.get_logger().info(
                    f"{det['colour']} cube at pixel ({det['centre'][0]}, {det['centre'][1]}) "
                    f"| area: {det['area']:.0f} | confidence: {det['confidence']:.2f}"
                )

        cv2.imshow('Cube Detector', display)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = CubeDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
