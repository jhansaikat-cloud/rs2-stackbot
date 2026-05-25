#!/usr/bin/env python3
"""
publish_handeye_tf.py — Stackbot SS1
=====================================
Replaces the easy_handeye2 publish.launch.py step.

Reads the ChArUco hand-eye calibration JSON and publishes the static TF:
    tool0 → camera_color_optical_frame

This is what pose_estimator.py needs to transform cube poses from camera
frame into base_link frame.

Run INSTEAD of:
    ros2 launch easy_handeye2 publish.launch.py name:=ur3e_eye_on_hand

Usage:
    python3 publish_handeye_tf.py
    python3 publish_handeye_tf.py --method tsai
    python3 publish_handeye_tf.py --method daniilidis

Where to put this file:
    ~/rs2-stackbot/src/perception_pkg/perception_pkg/publish_handeye_tf.py

Then add to setup.py entry_points:
    'publish_handeye_tf = perception_pkg.publish_handeye_tf:main',

And run with:
    ros2 run perception_pkg publish_handeye_tf
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np
import json
import os
import argparse
from scipy.spatial.transform import Rotation

CALIBRATION_DIR = os.path.expanduser(
    '~/rs2-stackbot/src/perception_pkg/calibration')

# These must match what the robot driver and RealSense use
PARENT_FRAME = 'tool0'
CHILD_FRAME  = 'camera_color_optical_frame'


def load_calibration(method='park'):
    path = os.path.join(CALIBRATION_DIR, f'handeye_charuco_{method}.json')
    if not os.path.exists(path):
        raise FileNotFoundError(
            f'\n[ERROR] Calibration file not found: {path}\n'
            f'  Run charuco_handeye_calibration.py first, then re-run this node.\n')
    with open(path) as f:
        data = json.load(f)
    T = np.array(data['matrix_4x4'])
    print(f'[OK] Loaded calibration ({method}): {path}')
    t = data['translation']
    e = data['rotation_euler_deg']
    print(f'     translation (m):  x={t["x"]:.4f}  y={t["y"]:.4f}  z={t["z"]:.4f}')
    print(f'     euler (deg):      rx={e["rx"]:.1f}  ry={e["ry"]:.1f}  rz={e["rz"]:.1f}')
    print(f'     convention:       {data["convention"]}')
    print(f'     publishing:       {PARENT_FRAME} → {CHILD_FRAME}')
    return T


class HandEyeTFPublisher(Node):

    def __init__(self, method='park'):
        super().__init__('handeye_tf_publisher')

        T = load_calibration(method)

        # Extract translation and quaternion
        t_vec = T[:3, 3]
        R_mat = T[:3, :3]
        q     = Rotation.from_matrix(R_mat).as_quat()  # [x, y, z, w]

        # Build static transform
        tf_msg                           = TransformStamped()
        tf_msg.header.frame_id           = PARENT_FRAME
        tf_msg.child_frame_id            = CHILD_FRAME
        tf_msg.transform.translation.x   = float(t_vec[0])
        tf_msg.transform.translation.y   = float(t_vec[1])
        tf_msg.transform.translation.z   = float(t_vec[2])
        tf_msg.transform.rotation.x      = float(q[0])
        tf_msg.transform.rotation.y      = float(q[1])
        tf_msg.transform.rotation.z      = float(q[2])
        tf_msg.transform.rotation.w      = float(q[3])

        self._broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Stamp and publish once (static broadcaster latches it)
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        self._broadcaster.sendTransform(tf_msg)

        self.get_logger().info(
            f'Static TF published: {PARENT_FRAME} → {CHILD_FRAME}  '
            f'(method={method})')
        self.get_logger().info(
            'pose_estimator.py can now transform camera poses to base_link.')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--method', default='park',
                        choices=['tsai', 'park', 'daniilidis', 'horaud', 'andreff'],
                        help='Which calibration JSON to use (default: park)')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = HandEyeTFPublisher(method=args.method)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
