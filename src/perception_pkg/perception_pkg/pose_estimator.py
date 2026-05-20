import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from std_msgs.msg import String
import message_filters
import cv2
import numpy as np
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs
import math

# Updated from /camera/camera/color/camera_info — serial 027322070904
FX = 917.625244140625
FY = 915.7077026367188
CX = 636.296875
CY = 351.4231872558594

DEPTH_WINDOW   = 20
MIN_AREA       = 2000
MIN_SQUARENESS = 0.45
BUFFER_SIZE    = 3

# How long (seconds) to keep publishing last known pose after cube disappears
POSE_TIMEOUT = 2.0

# Spatial clustering: detections within this distance (m) are the same cube
CLUSTER_DIST = 0.05  # 5cm

# Always process in this order so /object_labels is always red,yellow,blue
COLOUR_ORDER = ['red', 'yellow', 'blue']

COLOURS = {
    'red': {
        'ranges': [
            (np.array([0,   160, 150]), np.array([10,  255, 255])),
            (np.array([165, 160, 150]), np.array([180, 255, 255])),
        ],
        'bgr': (0, 0, 220)
    },
    'yellow': {
        'ranges': [(np.array([18, 80, 120]), np.array([33, 255, 255]))],
        'bgr': (0, 215, 255)
    },
    'blue': {
        'ranges': [(np.array([98, 180, 90]), np.array([108, 255, 200]))],
        'bgr': (220, 80, 0)
    },
}


class PoseEstimatorNode(Node):

    def __init__(self):
        super().__init__('pose_estimator')
        self.bridge = CvBridge()

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.colour_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/color/image_raw')
        self.depth_sub  = message_filters.Subscriber(
            self, Image, '/camera/camera/depth/image_rect_raw')
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.colour_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.callback)

        self.pose_pub  = self.create_publisher(PoseArray, '/ss1/raw_detected_objects', 10)
        self.label_pub = self.create_publisher(String,    '/object_labels', 10)

        # Per-colour, per-cluster: {colour: {cluster_id: [(X,Y,Z), ...]}}
        self._cluster_buffers  = {}
        # Last known stable pose per colour per cluster
        self._last_known_poses = {}
        self._last_log         = {}

        self.get_logger().info(
            'PoseEstimator ready — publishing on /ss1/raw_detected_objects and /object_labels')

    def _find_or_create_cluster(self, colour, X, Y, Z):
        if colour not in self._cluster_buffers:
            self._cluster_buffers[colour] = {}
        clusters = self._cluster_buffers[colour]
        for cid, buf in clusters.items():
            if len(buf) == 0:
                continue
            cx, cy, cz = buf[-1]
            dist = math.sqrt((X-cx)**2 + (Y-cy)**2 + (Z-cz)**2)
            if dist < CLUSTER_DIST:
                return cid
        new_id = len(clusters)
        clusters[new_id] = []
        return new_id

    def _update_cluster(self, colour, cid, X, Y, Z):
        buf = self._cluster_buffers[colour][cid]
        buf.append((X, Y, Z))
        if len(buf) > BUFFER_SIZE:
            buf.pop(0)

    def _get_cluster_avg(self, colour, cid):
        buf = self._cluster_buffers[colour][cid]
        if len(buf) < BUFFER_SIZE:
            return None
        return (
            sum(p[0] for p in buf) / BUFFER_SIZE,
            sum(p[1] for p in buf) / BUFFER_SIZE,
            sum(p[2] for p in buf) / BUFFER_SIZE,
        )

    def callback(self, colour_msg, depth_msg):
        frame       = self.bridge.imgmsg_to_cv2(colour_msg, desired_encoding='bgr8')
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg,  desired_encoding='passthrough')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        now = self.get_clock().now().nanoseconds / 1e9

        try:
            self.tf_buffer.lookup_transform(
                'base_link',
                'camera_color_optical_frame',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            tf_available = True
        except Exception:
            tf_available = False
            self.get_logger().warn(
                'TF not available yet (base_link -> camera_color_optical_frame).',
                throttle_duration_sec=5.0)

        for colour_name in COLOUR_ORDER:
            cfg = COLOURS[colour_name]
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for (lo, hi) in cfg['ranges']:
                mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))
            kernel = np.ones((11, 11), np.uint8)
            mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
            mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < MIN_AREA:
                    continue

                rect = cv2.minAreaRect(cnt)
                centre, (rw, rh), angle = rect
                cx_px = int(centre[0])
                cy_px = int(centre[1])

                if max(rw, rh) == 0:
                    continue
                squareness = min(rw, rh) / max(rw, rh)
                if squareness < MIN_SQUARENESS:
                    continue

                if rw < rh:
                    angle = angle + 90
                yaw = math.radians(angle)

                patch = depth_image[
                    max(0, cy_px - DEPTH_WINDOW): cy_px + DEPTH_WINDOW,
                    max(0, cx_px - DEPTH_WINDOW): cx_px + DEPTH_WINDOW]
                valid = patch[patch > 0]
                if valid.size == 0:
                    continue
                depth_m = float(np.median(valid)) / 1000.0

                X_cam = (cx_px - CX) * depth_m / FX
                Y_cam = (cy_px - CY) * depth_m / FY
                Z_cam = depth_m

                if not tf_available:
                    bgr = cfg['bgr']
                    box = np.int0(cv2.boxPoints(rect))
                    cv2.drawContours(frame, [box], 0, bgr, 2)
                    cv2.putText(frame, f'{colour_name} (no TF)',
                        (cx_px, cy_px - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, bgr, 2)
                    continue

                cam_pose                    = PoseStamped()
                cam_pose.header.stamp       = rclpy.time.Time().to_msg()
                cam_pose.header.frame_id    = 'camera_color_optical_frame'
                cam_pose.pose.position.x    = X_cam
                cam_pose.pose.position.y    = Y_cam
                cam_pose.pose.position.z    = Z_cam
                cam_pose.pose.orientation.w = 1.0

                try:
                    base_pose = self.tf_buffer.transform(cam_pose, 'base_link')
                except Exception as e:
                    self.get_logger().warn(
                        f'TF transform failed for {colour_name}: {e}',
                        throttle_duration_sec=2.0)
                    continue

                X = base_pose.pose.position.x
                Y = base_pose.pose.position.y
                Z = base_pose.pose.position.z

                cid = self._find_or_create_cluster(colour_name, X, Y, Z)
                self._update_cluster(colour_name, cid, X, Y, Z)
                avg = self._get_cluster_avg(colour_name, cid)

                if avg is not None:
                    X_avg, Y_avg, Z_avg = avg
                    if colour_name not in self._last_known_poses:
                        self._last_known_poses[colour_name] = {}
                    self._last_known_poses[colour_name][cid] = (X_avg, Y_avg, Z_avg, yaw, now)

                bgr = cfg['bgr']
                box = np.int0(cv2.boxPoints(rect))
                cv2.drawContours(frame, [box], 0, bgr, 2)
                cv2.circle(frame, (cx_px, cy_px), 5, bgr, -1)
                cv2.putText(frame,
                    f'{colour_name}  X:{X:.2f} Y:{Y:.2f} Z:{Z:.2f} yaw:{math.degrees(yaw):.0f}',
                    (cx_px, cy_px - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, bgr, 1)

                if now - self._last_log.get(f'{colour_name}_{cid}', 0) >= 1.0:
                    self.get_logger().info(
                        f'{colour_name}[{cid}]: X={X:.4f}m Y={Y:.4f}m Z={Z:.4f}m '
                        f'yaw={math.degrees(yaw):.1f}deg sq={squareness:.2f}')
                    self._last_log[f'{colour_name}_{cid}'] = now

        # Build pose array from last known poses within timeout
        pose_array                 = PoseArray()
        pose_array.header.stamp    = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'base_link'
        labels = []

        for colour_name in COLOUR_ORDER:
            if colour_name not in self._last_known_poses:
                continue
            clusters = self._last_known_poses[colour_name]

            valid_clusters = [
                (cid, data) for cid, data in clusters.items()
                if now - data[4] <= POSE_TIMEOUT
            ]
            valid_clusters.sort(key=lambda c: c[1][1])

            for cid, (X_avg, Y_avg, Z_avg, yaw, _) in valid_clusters:
                pose = Pose()
                pose.position.x    = X_avg
                pose.position.y    = Y_avg
                pose.position.z    = Z_avg
                pose.orientation.x = 0.0
                pose.orientation.y = 0.0
                pose.orientation.z = math.sin(yaw / 2)
                pose.orientation.w = math.cos(yaw / 2)
                pose_array.poses.append(pose)
                labels.append(colour_name)

            timed_out = [
                cid for cid, data in clusters.items()
                if now - data[4] > POSE_TIMEOUT
            ]
            for cid in timed_out:
                del clusters[cid]
                if colour_name in self._cluster_buffers:
                    self._cluster_buffers[colour_name].pop(cid, None)

        self.pose_pub.publish(pose_array)
        label_msg      = String()
        label_msg.data = ','.join(labels)
        self.label_pub.publish(label_msg)
        cv2.imshow('Pose Estimator', frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()