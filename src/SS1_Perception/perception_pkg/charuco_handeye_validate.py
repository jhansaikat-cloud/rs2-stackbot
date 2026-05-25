#!/usr/bin/env python3
"""
ChArUco Hand-Eye Calibration — Validation Script
=================================================
Run this AFTER charuco_handeye_calibration.py has saved its JSON files.

What it does:
  1. Loads a calibration JSON (default: park method)
  2. Subscribes to the RealSense image
  3. Detects the ChArUco board in the current camera view
  4. Uses calibration + current tool0 TF to project board position into base_link
  5. Prints the predicted board position in base_link every second
  6. You physically measure where the board is and compare

HOW TO VALIDATE:
  - Place the board on the table at a known position
  - Move the robot to a DIFFERENT pose than used during calibration
  - Run this script
  - Look at the printed X,Y,Z in base_link
  - Use the robot teach pendant to jog TCP to the board corner — compare coordinates
  - Difference should be <15mm for pass grade

Run:
  python3 charuco_handeye_validate.py
  python3 charuco_handeye_validate.py --method tsai
  python3 charuco_handeye_validate.py --method daniilidis
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
import json
import os
import argparse
import tf2_ros
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation

# ── Board parameters — must match calibration script ──────────────────────
CHARUCO_COLS   = 9
CHARUCO_ROWS   = 6
SQUARE_SIZE_M  = 0.028
MARKER_SIZE_M  = 0.021
ARUCO_DICT_ID  = cv2.aruco.DICT_6X6_250

# ── Camera intrinsics ──────────────────────────────────────────────────────
FX = 917.625244140625
FY = 915.7077026367188
CX = 636.296875
CY = 351.4231872558594
CAMERA_MATRIX = np.array([[FX,  0, CX],
                           [ 0, FY, CY],
                           [ 0,  0,  1]], dtype=np.float64)
DIST_COEFFS = np.zeros((5, 1), dtype=np.float64)

BASE_FRAME  = 'base_link'
TOOL_FRAME  = 'tool0'
OUTPUT_DIR  = os.path.expanduser('~/rs2-stackbot/src/perception_pkg/calibration')


def load_calibration(method='park'):
    path = os.path.join(OUTPUT_DIR, f'handeye_charuco_{method}.json')
    if not os.path.exists(path):
        raise FileNotFoundError(
            f'Calibration file not found: {path}\n'
            f'Run charuco_handeye_calibration.py first.')
    with open(path) as f:
        data = json.load(f)
    T = np.array(data['matrix_4x4'])
    print(f'[OK] Loaded calibration: {path}')
    print(f'     translation: x={data["translation"]["x"]:.4f}  '
          f'y={data["translation"]["y"]:.4f}  z={data["translation"]["z"]:.4f}')
    euler = data['rotation_euler_deg']
    print(f'     euler (deg): rx={euler["rx"]:.1f}  ry={euler["ry"]:.1f}  rz={euler["rz"]:.1f}')
    print(f'     convention:  {data["convention"]}')
    return T


def tf_to_matrix(transform):
    t = transform.transform.translation
    r = transform.transform.rotation
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([r.x, r.y, r.z, r.w]).as_matrix()
    T[:3,  3] = [t.x, t.y, t.z]
    return T


class HandEyeValidator(Node):

    def __init__(self, method='park'):
        super().__init__('handeye_validator')
        self.bridge      = CvBridge()
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.image_sub   = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self._image_cb, 10)

        self.aruco_dict  = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)
        self.board = cv2.aruco.CharucoBoard_create(
            CHARUCO_COLS, CHARUCO_ROWS,
            SQUARE_SIZE_M, MARKER_SIZE_M, self.aruco_dict)
        self.detector_params = cv2.aruco.DetectorParameters_create()

        self.T_cam2tool  = load_calibration(method)
        self._latest_frame = None
        self._last_print   = 0.0

        print('\n[Validation running]')
        print('Move robot to a NEW pose (not used during calibration).')
        print('Board must be visible. Predicted position prints every 1 second.')
        print('Compare to physical board position — difference = your calibration error.\n')

    def _image_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.detector_params)

        board_pose = None
        if ids is not None and len(ids) >= 4:
            ret, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(
                corners, ids, gray, self.board)
            if ret and ch_corners is not None and len(ch_corners) >= 6:
                ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                    ch_corners, ch_ids, self.board, CAMERA_MATRIX, DIST_COEFFS,
                    None, None)
                if ok:
                    board_pose = (rvec, tvec)
                    cv2.aruco.drawDetectedCornersCharuco(frame, ch_corners, ch_ids)
                    cv2.drawFrameAxes(frame, CAMERA_MATRIX, DIST_COEFFS,
                                      rvec, tvec, SQUARE_SIZE_M * 2)

        now = self.get_clock().now().nanoseconds / 1e9

        if board_pose and (now - self._last_print) > 1.0:
            rvec, tvec = board_pose
            R_b2c, _   = cv2.Rodrigues(rvec)
            t_b2c       = tvec.flatten()

            # board in camera frame → board in tool0 frame
            R_c2t = self.T_cam2tool[:3, :3]
            t_c2t = self.T_cam2tool[:3,  3]
            R_b2t = R_c2t @ R_b2c
            t_b2t = R_c2t @ t_b2c + t_c2t

            # board in tool0 frame → board in base_link frame
            try:
                tf_base_tool = self.tf_buffer.lookup_transform(
                    BASE_FRAME, TOOL_FRAME,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5))
                T_base_tool = tf_to_matrix(tf_base_tool)
                R_t2b = T_base_tool[:3, :3]
                t_t2b = T_base_tool[:3,  3]

                t_board_base = R_t2b @ t_b2t + t_t2b

                print(f'Board in base_link:  '
                      f'X={t_board_base[0]*1000:.1f}mm  '
                      f'Y={t_board_base[1]*1000:.1f}mm  '
                      f'Z={t_board_base[2]*1000:.1f}mm')

                # Check: Z should be positive and roughly table height (~100-500mm)
                z_mm = t_board_base[2] * 1000
                if z_mm < 0:
                    print('  ⚠ WARNING: Z is negative — calibration transform direction may be wrong!')
                    print('     Try a different method JSON (tsai, daniilidis).')
                elif z_mm > 800:
                    print('  ⚠ WARNING: Z seems very high — check calibration.')
                else:
                    print('  ✓ Z looks plausible (positive, reasonable height)')

            except Exception as e:
                print(f'  [TF error] {e}')

            self._last_print = now

        status = 'BOARD DETECTED ✓' if board_pose else 'Board NOT detected'
        colour = (0, 255, 0) if board_pose else (0, 0, 255)
        cv2.putText(frame, status, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, colour, 2)
        cv2.putText(frame, 'Q to quit',
                    (10, frame.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
        self._latest_frame = frame

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._latest_frame is not None:
                cv2.imshow('Hand-Eye Validation', self._latest_frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == ord('Q'):
                break
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--method', default='park',
                        choices=['tsai', 'park', 'daniilidis', 'horaud', 'andreff'],
                        help='Which calibration JSON to validate (default: park)')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = HandEyeValidator(method=args.method)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
