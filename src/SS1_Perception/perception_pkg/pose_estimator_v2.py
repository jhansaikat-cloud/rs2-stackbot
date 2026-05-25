#!/usr/bin/env python3
"""
pose_estimator_v2.py  —  SS1 Perception Node
Team: Stackbot | UTS 41069 Robotics Studio 2
Author: Arjun Harish (SS1)

Detection pipeline:
    1. HSV colour mask + morphological cleaning
    2. RETR_EXTERNAL contours (kills nested/inner-edge duplicates at source)
    3. Area + squareness filter
    4. IoU-NMS in 2D image space (kills overlapping bbox duplicates)
    5. Top-N selection by area (enforces known cube counts: 3R/2Y/1B)
    6. Depth lookup (median, zero-rejection, fallback window)
    7. Yaw from minAreaRect angle
    8. Per-detection EMA smoothing on 3D pose (stable index by X position)
    9. TF into base_link → publish /raw_detected_objects + /object_labels

Topics:
    Published: /raw_detected_objects  (geometry_msgs/PoseArray, frame: base_link)
    Published: /object_labels         (std_msgs/String)
"""

import math

import cv2
import numpy as np
import rclpy
import rclpy.time
import tf2_geometry_msgs  # noqa: F401
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

# ─────────────────────────────────────────────────────────────
#  CAMERA INTRINSICS
#  Update if serial changes:
#    rs-enumerate-devices | grep Serial
#    ros2 topic echo /camera/camera/color/camera_info --once
#  K[0]=FX  K[4]=FY  K[2]=CX  K[5]=CY
# ─────────────────────────────────────────────────────────────
FX = 915.0683
FY = 915.0022
CX = 652.6305
CY = 348.2195

# ─────────────────────────────────────────────────────────────
#  DETECTION PARAMETERS
# ─────────────────────────────────────────────────────────────
MIN_AREA           = 2000
MIN_SQUARENESS     = 0.40
IOU_THRESHOLD      = 0.30
DEPTH_WINDOW       = 20
DEPTH_WINDOW_FB    = 40
DEPTH_MIN_VALID    = 3
DEPTH_MIN_VALID_FB = 2
DEPTH_MIN_M        = 0.15
DEPTH_MAX_M        = 1.20
EMA_ALPHA          = 0.4
POSE_TIMEOUT       = 2.0

# ─────────────────────────────────────────────────────────────
#  KNOWN CUBE COUNTS
# ─────────────────────────────────────────────────────────────
EXPECTED_COUNTS = {'red': 3, 'yellow': 2, 'blue': 1}
COLOUR_ORDER    = ['red', 'yellow', 'blue']

# ─────────────────────────────────────────────────────────────
#  HSV COLOUR RANGES + BGR draw colours
# ─────────────────────────────────────────────────────────────
COLOURS = {
    'red': {
        'ranges': [
            (np.array([0,   120, 100]), np.array([10,  255, 255])),
            (np.array([165, 120, 100]), np.array([180, 255, 255])),
        ],
        'bgr': (0, 0, 220),
    },
    'yellow': {
        'ranges': [
            (np.array([18, 80, 120]), np.array([33, 255, 255])),
        ],
        'bgr': (0, 215, 255),
    },
    'blue': {
        'ranges': [
            (np.array([98, 180, 90]), np.array([108, 255, 200])),
        ],
        'bgr': (220, 80, 0),
    },
}

_MORPH_KERNEL = cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11))


# ═══════════════════════════════════════════════════════════════
class PoseEstimatorV2Node(Node):

    def __init__(self):
        super().__init__('pose_estimator_v2')

        self.bridge      = CvBridge()
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(
            Image, '/camera/camera/color/image_raw', self._cb_colour, 10)
        self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw', self._cb_depth, 10)

        self.pub_poses  = self.create_publisher(PoseArray, '/raw_detected_objects', 10)
        self.pub_labels = self.create_publisher(String,    '/object_labels',         10)

        self._depth_image = None
        self._ema_poses: dict = {}
        self._last_seen: dict = {}
        self._last_log:  dict = {}

        self.get_logger().info(
            'PoseEstimatorV2 ready — RETR_EXTERNAL + IoU NMS + Top-N + EMA')
        self.get_logger().info(
            'Publishing → /raw_detected_objects  /object_labels')

    # ──────────────────────────────────────────────
    #  Depth cache
    # ──────────────────────────────────────────────

    def _cb_depth(self, msg: Image):
        try:
            self._depth_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f'Depth decode error: {e}')

    # ──────────────────────────────────────────────
    #  Main colour callback
    # ──────────────────────────────────────────────

    def _cb_colour(self, msg: Image):
        if self._depth_image is None:
            self.get_logger().warn(
                'No depth frame yet — skipping', throttle_duration_sec=5.0)
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Colour decode error: {e}')
            return

        now     = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9
        hsv     = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        all_poses  = []
        all_labels = []

        for colour in COLOUR_ORDER:
            cfg       = COLOURS[colour]
            max_count = EXPECTED_COUNTS[colour]

            # detections: list of (cx, cy, area, yaw, box_pts)
            detections = self._detect_colour(hsv, colour, cfg['ranges'], max_count)

            for idx, (cx_px, cy_px, area, yaw, box_pts) in enumerate(detections):
                pose = self._to_base_pose(cx_px, cy_px, yaw, colour, idx, now)

                bgr = cfg['bgr']

                # Always draw bounding box (even if TF not available yet)
                cv2.drawContours(frame, [box_pts], 0, bgr, 2)
                cv2.circle(frame, (cx_px, cy_px), 5, bgr, -1)

                if pose is None:
                    cv2.putText(frame, f'{colour} (no TF)',
                                (cx_px + 8, cy_px - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, bgr, 1)
                    continue

                all_poses.append(pose)
                all_labels.append(colour)

                # Label with 3D position
                cv2.putText(
                    frame,
                    f'{colour}[{idx}] '
                    f'X:{pose.position.x:.2f} '
                    f'Y:{pose.position.y:.2f} '
                    f'Z:{pose.position.z:.2f}',
                    (cx_px + 8, cy_px - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, bgr, 1)

                # 1Hz log
                log_key = (colour, idx)
                if now_sec - self._last_log.get(log_key, 0) >= 1.0:
                    self.get_logger().info(
                        f'{colour}[{idx}]: '
                        f'X={pose.position.x*1000:.1f}mm '
                        f'Y={pose.position.y*1000:.1f}mm '
                        f'Z={pose.position.z*1000:.1f}mm '
                        f'yaw={math.degrees(yaw):.1f}deg '
                        f'area={area:.0f}px')
                    self._last_log[log_key] = now_sec

        self._purge_stale(now)

        # Publish
        pose_array                 = PoseArray()
        pose_array.header.stamp    = now.to_msg()
        pose_array.header.frame_id = 'base_link'
        pose_array.poses           = all_poses
        self.pub_poses.publish(pose_array)

        label_msg      = String()
        label_msg.data = ','.join(all_labels)
        self.pub_labels.publish(label_msg)

        cv2.imshow('Pose Estimator V2', frame)
        cv2.waitKey(1)

    # ──────────────────────────────────────────────
    #  Colour detection → NMS → Top-N
    # ──────────────────────────────────────────────

    def _detect_colour(self, hsv, colour: str,
                       ranges: list, max_count: int) -> list:
        """
        Returns list of (cx_px, cy_px, area, yaw_rad, box_pts).
        Sorted by X position for stable EMA indexing.
        box_pts: np.int0 array of 4 corners for cv2.drawContours.
        """
        # Build mask
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for (lo, hi) in ranges:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))

        # Morphological clean
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  _MORPH_KERNEL)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _MORPH_KERNEL)

        # RETR_EXTERNAL — only outer contours
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        candidates = []  # (area, x, y, w, h, cx, cy, yaw, box_pts)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_AREA:
                self.get_logger().debug(
                    f'[{colour}] SKIP area={area:.0f} < {MIN_AREA}')
                continue

            rect                    = cv2.minAreaRect(cnt)
            centre, (rw, rh), angle = rect
            cx_px = int(centre[0])
            cy_px = int(centre[1])

            if max(rw, rh) == 0:
                continue
            squareness = min(rw, rh) / max(rw, rh)
            if squareness < MIN_SQUARENESS:
                self.get_logger().debug(
                    f'[{colour}] SKIP squareness={squareness:.2f} < {MIN_SQUARENESS}')
                continue

            # Yaw from minAreaRect
            if rw < rh:
                angle = angle + 90
            yaw = math.radians(angle)

            # Bounding box points for drawing
            box_pts = np.int0(cv2.boxPoints(rect))

            x, y, w, h = cv2.boundingRect(cnt)
            candidates.append((area, x, y, w, h, cx_px, cy_px, yaw, box_pts))

        if not candidates:
            return []

        # IoU NMS — largest first
        candidates.sort(key=lambda c: c[0], reverse=True)
        kept = self._nms(candidates, IOU_THRESHOLD)

        # Top-N
        kept = kept[:max_count]

        # Sort by X for stable EMA index
        kept.sort(key=lambda c: c[5])

        return [(c[5], c[6], c[0], c[7], c[8]) for c in kept]

    # ──────────────────────────────────────────────
    #  IoU NMS
    # ──────────────────────────────────────────────

    @staticmethod
    def _nms(candidates: list, iou_thresh: float) -> list:
        kept       = []
        suppressed = [False] * len(candidates)

        for i in range(len(candidates)):
            if suppressed[i]:
                continue
            kept.append(candidates[i])
            _, xi, yi, wi, hi = candidates[i][:5]

            for j in range(i + 1, len(candidates)):
                if suppressed[j]:
                    continue
                _, xj, yj, wj, hj = candidates[j][:5]

                ix1   = max(xi, xj)
                iy1   = max(yi, yj)
                ix2   = min(xi + wi, xj + wj)
                iy2   = min(yi + hi, yj + hj)
                inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)

                if inter == 0:
                    continue

                union = wi * hi + wj * hj - inter
                iou   = inter / union if union > 0 else 0.0

                if iou >= iou_thresh:
                    suppressed[j] = True

        return kept

    # ──────────────────────────────────────────────
    #  Depth sampling
    # ──────────────────────────────────────────────

    def _sample_depth(self, cx: int, cy: int, colour: str) -> float | None:
        depth_mm = self._depth_image
        h, w     = depth_mm.shape

        for win, min_valid, label in [
            (DEPTH_WINDOW,    DEPTH_MIN_VALID,    'primary'),
            (DEPTH_WINDOW_FB, DEPTH_MIN_VALID_FB, 'fallback'),
        ]:
            patch = depth_mm[
                max(0, cy - win):min(h, cy + win),
                max(0, cx - win):min(w, cx + win)
            ].astype(np.float32)

            valid = patch[
                (patch > 0) &
                (patch >= DEPTH_MIN_M * 1000) &
                (patch <= DEPTH_MAX_M * 1000)
            ]

            if valid.size < min_valid:
                self.get_logger().debug(
                    f'[{colour}] depth {label}: {valid.size} valid px '
                    f'(need {min_valid}) at ({cx},{cy})')
                continue

            depth_m = float(np.median(valid)) / 1000.0
            if DEPTH_MIN_M <= depth_m <= DEPTH_MAX_M:
                return depth_m

        self.get_logger().warn(
            f'[{colour}] DEPTH FAILED at ({cx},{cy}) — '
            f'both windows invalid. Check USB 3.0 port.',
            throttle_duration_sec=2.0)
        return None

    # ──────────────────────────────────────────────
    #  3D projection + TF + EMA → Pose
    # ──────────────────────────────────────────────

    def _to_base_pose(self, cx_px: int, cy_px: int, yaw: float,
                      colour: str, idx: int, now) -> Pose | None:
        depth_m = self._sample_depth(cx_px, cy_px, colour)
        if depth_m is None:
            return None

        x_cam = (cx_px - CX) * depth_m / FX
        y_cam = (cy_px - CY) * depth_m / FY
        z_cam = depth_m

        cam_pose                    = PoseStamped()
        cam_pose.header.stamp       = rclpy.time.Time().to_msg()
        cam_pose.header.frame_id    = 'camera_color_optical_frame'
        cam_pose.pose.position.x    = x_cam
        cam_pose.pose.position.y    = y_cam
        cam_pose.pose.position.z    = z_cam
        cam_pose.pose.orientation.w = 1.0

        try:
            base_pose = self.tf_buffer.transform(
                cam_pose, 'base_link', timeout=Duration(seconds=0.5))
        except tf2_ros.LookupException as e:
            self.get_logger().warn(
                f'[{colour}#{idx}] TF LookupException: {e} — '
                f'is publish_handeye_tf running?',
                throttle_duration_sec=2.0)
            return None
        except tf2_ros.ConnectivityException as e:
            self.get_logger().warn(
                f'[{colour}#{idx}] TF ConnectivityException: {e}',
                throttle_duration_sec=2.0)
            return None
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warn(
                f'[{colour}#{idx}] TF ExtrapolationException: {e}',
                throttle_duration_sec=2.0)
            return None
        except Exception as e:
            self.get_logger().warn(
                f'[{colour}#{idx}] TF error: {e}',
                throttle_duration_sec=2.0)
            return None

        raw = np.array([
            base_pose.pose.position.x,
            base_pose.pose.position.y,
            base_pose.pose.position.z,
        ])

        if raw[2] < 0:
            self.get_logger().warn(
                f'[{colour}#{idx}] Z={raw[2]:.3f}m negative — '
                f'hand-eye calibration may be wrong.',
                throttle_duration_sec=5.0)
            return None

        key = (colour, idx)
        self._last_seen[key] = now

        if key not in self._ema_poses:
            self._ema_poses[key] = raw.copy()
        else:
            self._ema_poses[key] = (
                EMA_ALPHA * raw +
                (1.0 - EMA_ALPHA) * self._ema_poses[key])

        smoothed = self._ema_poses[key]

        pose = Pose()
        pose.position.x    = float(smoothed[0])
        pose.position.y    = float(smoothed[1])
        pose.position.z    = float(smoothed[2])
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = math.sin(yaw / 2)
        pose.orientation.w = math.cos(yaw / 2)

        return pose

    # ──────────────────────────────────────────────
    #  Stale cleanup
    # ──────────────────────────────────────────────

    def _purge_stale(self, now):
        stale = [
            k for k, t in self._last_seen.items()
            if (now - t).nanoseconds * 1e-9 > POSE_TIMEOUT
        ]
        for k in stale:
            self._ema_poses.pop(k, None)
            self._last_seen.pop(k, None)


# ═══════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimatorV2Node()
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