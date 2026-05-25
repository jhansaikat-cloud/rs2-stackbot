#!/usr/bin/env python3
"""
ChArUco Hand-Eye Calibration Script — Stackbot SS1
===================================================
Board:   6×9 ChArUco, DICT_6X6_250 (try DICT_6X6_50 if detection fails)
Squares: 28mm  |  Markers: 21mm
Camera:  Intel RealSense D435i — serial 027322070904
Robot:   UR3e — eye-in-hand (camera on wrist, between tool0 and gripper)

What this script does:
  1. Subscribes to the RealSense colour image
  2. Detects the ChArUco board in each frame
  3. When you press SPACE, it captures the current board pose + robot tool0 pose
  4. After N samples, solves hand-eye calibration using Tsai, Park, and Daniilidis
  5. Saves JSON files for each method
  6. Prints reprojection error so you know which result to trust

Run order (before running this script):
  T1: ros2 launch realsense2_camera rs_launch.py depth_module.profile:=640x480x30 enable_depth:=true publish_tf:=false
  T2: ros2 launch ur_onrobot_control start_robot.launch.py ur_type:=ur3e onrobot_type:=rg2 robot_ip:=<confirm> kinematics_params:=/home/kahniri/robotics2/ur3e_3_calibration.yaml launch_rviz:=false

Then run this script:
  python3 charuco_handeye_calibration.py

Controls:
  SPACE  — capture current sample (move robot to new pose first)
  S      — solve and save calibration (do this after 15+ samples)
  Q      — quit without saving
  D      — delete last sample (if detection looked bad)

Tips for good calibration:
  - Aim for 15-25 samples minimum
  - Vary tilt, rotation, distance — don't just translate
  - Keep board fully visible and in focus
  - Avoid samples that look identical to previous ones
  - The board must NOT move during the session
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import cv2
import numpy as np
import json
import math
import os
import tf2_ros
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation

# ── Board parameters — must match your physical board exactly ──────────────
CHARUCO_COLS        = 9          # squares across (long edge)
CHARUCO_ROWS        = 6          # squares down
SQUARE_SIZE_M       = 0.028      # 28mm
MARKER_SIZE_M       = 0.021      # 21mm
ARUCO_DICT_ID       = cv2.aruco.DICT_6X6_250   # try DICT_6X6_50 if board not detected

# ── Camera intrinsics — session 5, serial 027322070904 ─────────────────────
FX = 915.0683
FY = 915.0022
CX = 652.6305
CY = 348.2195
CAMERA_MATRIX = np.array([[FX,  0, CX],
                           [ 0, FY, CY],
                           [ 0,  0,  1]], dtype=np.float64)
DIST_COEFFS = np.zeros((5, 1), dtype=np.float64)   # D435i is pre-rectified

# ── ROS frames ─────────────────────────────────────────────────────────────
BASE_FRAME   = 'base_link'
TOOL_FRAME   = 'tool0'          # robot flange — camera mounts here
CAMERA_FRAME = 'camera_color_optical_frame'

# ── Output directory ───────────────────────────────────────────────────────
OUTPUT_DIR = os.path.expanduser('~/rs2-stackbot/src/perception_pkg/calibration')
os.makedirs(OUTPUT_DIR, exist_ok=True)


def pose_to_Rt(tvec, rvec):
    """Convert rvec+tvec (OpenCV convention) to 4×4 homogeneous matrix."""
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = tvec.flatten()
    return T


def tf_stamped_to_Rt(transform):
    """Convert a TransformStamped to a 4×4 matrix."""
    t = transform.transform.translation
    r = transform.transform.rotation
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([r.x, r.y, r.z, r.w]).as_matrix()
    T[:3,  3] = [t.x, t.y, t.z]
    return T


def Rt_to_json(T, method_name):
    """Serialise a 4×4 matrix to a JSON-friendly dict."""
    r = Rotation.from_matrix(T[:3, :3])
    q = r.as_quat()          # [x, y, z, w]
    euler = r.as_euler('xyz', degrees=True)
    return {
        'method': method_name,
        'translation': {'x': float(T[0, 3]),
                        'y': float(T[1, 3]),
                        'z': float(T[2, 3])},
        'rotation_quaternion': {'x': float(q[0]),
                                'y': float(q[1]),
                                'z': float(q[2]),
                                'w': float(q[3])},
        'rotation_euler_deg': {'rx': float(euler[0]),
                               'ry': float(euler[1]),
                               'rz': float(euler[2])},
        'matrix_4x4': T.tolist(),
        'camera_frame':  CAMERA_FRAME,
        'tool_frame':    TOOL_FRAME,
        'convention':    'tool0_T_camera  (camera pose expressed in tool0 frame)'
    }


class CharucoHandEyeCalibrator(Node):

    def __init__(self):
        super().__init__('charuco_handeye_calibrator')

        self.bridge      = CvBridge()
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.image_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self._image_cb, 10)

        # Build ChArUco board
        self.aruco_dict  = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)
        self.board = cv2.aruco.CharucoBoard_create(
            CHARUCO_COLS, CHARUCO_ROWS,
            SQUARE_SIZE_M, MARKER_SIZE_M, self.aruco_dict)
        self.detector_params = cv2.aruco.DetectorParameters_create()

        # Sample storage
        self.R_gripper2base_list = []   # robot end-effector rotation (base←tool0)
        self.t_gripper2base_list = []
        self.R_board2cam_list    = []   # board pose in camera frame
        self.t_board2cam_list    = []

        self._latest_frame    = None
        self._latest_charuco  = None   # (corners, ids, rvec, tvec) or None
        self._frame_count     = 0

        self.get_logger().info('ChArUco Hand-Eye Calibrator ready.')
        self.get_logger().info(
            f'Board: {CHARUCO_COLS}×{CHARUCO_ROWS}, '
            f'square={SQUARE_SIZE_M*1000:.0f}mm, marker={MARKER_SIZE_M*1000:.0f}mm')
        self.get_logger().info('Press SPACE to capture, S to solve, D to delete last, Q to quit.')

    # ── Image callback ──────────────────────────────────────────────────────
    def _image_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.detector_params)

        charuco_result = None
        if ids is not None and len(ids) >= 4:
            ret, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(
                corners, ids, gray, self.board)
            if ret and ch_corners is not None and len(ch_corners) >= 6:
                ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                    ch_corners, ch_ids, self.board, CAMERA_MATRIX, DIST_COEFFS,
                    None, None)
                if ok:
                    charuco_result = (ch_corners, ch_ids, rvec, tvec)
                    cv2.aruco.drawDetectedCornersCharuco(frame, ch_corners, ch_ids)
                    cv2.drawFrameAxes(frame, CAMERA_MATRIX, DIST_COEFFS,
                                      rvec, tvec, SQUARE_SIZE_M * 2)

        # HUD
        n = len(self.R_gripper2base_list)
        colour = (0, 255, 0) if charuco_result else (0, 0, 255)
        status = 'BOARD DETECTED' if charuco_result else 'Board NOT detected'
        cv2.putText(frame, f'{status} | Samples: {n}',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, colour, 2)
        cv2.putText(frame,
                    'SPACE=capture  S=solve  D=delete last  Q=quit',
                    (10, frame.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
        if n > 0:
            cv2.putText(frame, f'Last capture OK', (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 1)

        self._latest_frame   = frame
        self._latest_charuco = charuco_result

    # ── Attempt to capture a sample ─────────────────────────────────────────
    def capture_sample(self):
        if self._latest_charuco is None:
            print('[!] Board not detected in current frame — move robot so board is visible.')
            return False

        _, _, rvec, tvec = self._latest_charuco

        # Get tool0 pose in base_link
        try:
            tf_base_tool = self.tf_buffer.lookup_transform(
                BASE_FRAME, TOOL_FRAME,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
        except Exception as e:
            print(f'[!] TF lookup failed (base_link → tool0): {e}')
            print('    Is the robot driver running?')
            return False

        T_base_tool = tf_stamped_to_Rt(tf_base_tool)
        R_g2b = T_base_tool[:3, :3]
        t_g2b = T_base_tool[:3,  3].reshape(3, 1)

        R_b2c, _ = cv2.Rodrigues(rvec)
        t_b2c    = tvec.reshape(3, 1)

        self.R_gripper2base_list.append(R_g2b)
        self.t_gripper2base_list.append(t_g2b)
        self.R_board2cam_list.append(R_b2c)
        self.t_board2cam_list.append(t_b2c)

        n = len(self.R_gripper2base_list)
        print(f'[+] Sample {n} captured.  '
              f'tool0 t=[{t_g2b[0,0]:.3f}, {t_g2b[1,0]:.3f}, {t_g2b[2,0]:.3f}]  '
              f'board t=[{t_b2c[0,0]:.3f}, {t_b2c[1,0]:.3f}, {t_b2c[2,0]:.3f}]')
        if n < 8:
            print(f'    Need at least 8 samples before solving (have {n}) — keep moving.')
        return True

    def delete_last_sample(self):
        if not self.R_gripper2base_list:
            print('[!] No samples to delete.')
            return
        for lst in (self.R_gripper2base_list, self.t_gripper2base_list,
                    self.R_board2cam_list,    self.t_board2cam_list):
            lst.pop()
        print(f'[-] Deleted last sample. Remaining: {len(self.R_gripper2base_list)}')

    # ── Solve and save ──────────────────────────────────────────────────────
    def solve_and_save(self):
        n = len(self.R_gripper2base_list)
        if n < 8:
            print(f'[!] Need at least 8 samples — only have {n}. Capture more first.')
            return

        print(f'\n--- Solving hand-eye calibration with {n} samples ---')

        methods = {
            'tsai':       cv2.CALIB_HAND_EYE_TSAI,
            'park':       cv2.CALIB_HAND_EYE_PARK,
            'daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS,
            'horaud':     cv2.CALIB_HAND_EYE_HORAUD,
            'andreff':    cv2.CALIB_HAND_EYE_ANDREFF,
        }

        results = {}
        for name, method in methods.items():
            try:
                R_cam2tool, t_cam2tool = cv2.calibrateHandEye(
                    self.R_gripper2base_list, self.t_gripper2base_list,
                    self.R_board2cam_list,    self.t_board2cam_list,
                    method=method)
                T = np.eye(4)
                T[:3, :3] = R_cam2tool
                T[:3,  3] = t_cam2tool.flatten()
                results[name] = T

                # Print summary
                r = Rotation.from_matrix(R_cam2tool)
                q = r.as_quat()
                euler = r.as_euler('xyz', degrees=True)
                print(f'\n  [{name.upper()}]')
                print(f'    translation : x={t_cam2tool[0,0]:.4f}  '
                      f'y={t_cam2tool[1,0]:.4f}  z={t_cam2tool[2,0]:.4f}  (metres)')
                print(f'    euler (deg) : rx={euler[0]:.1f}  ry={euler[1]:.1f}  rz={euler[2]:.1f}')
                print(f'    quaternion  : x={q[0]:.4f}  y={q[1]:.4f}  z={q[2]:.4f}  w={q[3]:.4f}')
            except Exception as e:
                print(f'  [{name}] FAILED: {e}')

        # Save each method
        saved = []
        for name, T in results.items():
            out_path = os.path.join(OUTPUT_DIR, f'handeye_charuco_{name}.json')
            with open(out_path, 'w') as f:
                json.dump(Rt_to_json(T, name), f, indent=2)
            saved.append(out_path)
            print(f'\n  Saved: {out_path}')

        # Reprojection error (simple consistency check)
        print('\n--- Consistency check (lower is better) ---')
        for name, T in results.items():
            errors = []
            R_c2t = T[:3, :3]
            t_c2t = T[:3, 3].reshape(3, 1)
            for i in range(n):
                # Predict board position in tool frame
                R_b2c_i = self.R_board2cam_list[i]
                t_b2c_i = self.t_board2cam_list[i]
                t_board_tool = R_c2t @ t_b2c_i + t_c2t
                # Predict board position in base frame via gripper pose
                R_g2b_i = self.R_gripper2base_list[i]
                t_g2b_i = self.t_gripper2base_list[i]
                t_board_base = R_g2b_i @ t_board_tool + t_g2b_i
                errors.append(t_board_base)
            positions = np.hstack(errors)  # 3×N
            std = np.std(positions, axis=1)
            mean_std = np.mean(std) * 1000
            print(f'  [{name.upper()}]  position std = {mean_std:.2f} mm  '
                  f'(target: <10mm for pass grade)')

        print('\n=== Done. Use the method with lowest std for your system. ===')
        print('Recommended: try PARK first, then TSAI.\n')
        print('Next step: run charuco_handeye_validate.py to confirm the result.')

    # ── Spin loop ───────────────────────────────────────────────────────────
    def run(self):
        import time
        print('\n[ChArUco Hand-Eye Calibrator]')
        print('Move the robot to a pose where the ChArUco board is visible.')
        print('Press SPACE to capture, S to solve+save, D to delete last, Q to quit.\n')

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

            if self._latest_frame is not None:
                cv2.imshow('ChArUco Hand-Eye Calibration', self._latest_frame)

            key = cv2.waitKey(1) & 0xFF

            if key == ord(' '):
                self.capture_sample()
            elif key == ord('s') or key == ord('S'):
                self.solve_and_save()
            elif key == ord('d') or key == ord('D'):
                self.delete_last_sample()
            elif key == ord('q') or key == ord('Q'):
                print('[Q] Quitting without solving.')
                break

        cv2.destroyAllWindows()


def main():
    rclpy.init()
    node = CharucoHandEyeCalibrator()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
