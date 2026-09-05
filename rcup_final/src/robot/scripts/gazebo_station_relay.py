#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np

ROBOT_MODEL_NAME = 'mecanum_arm_robot'

STATION_MODELS = {
    's_top':        's_top_shelf_unit',
    's_left_mid':   's_left_mid_shelf_unit',
    's_center_mid': 's_center_mid_shelf_unit',
    's_bot_left':   's_bot_left_shelf_unit',
    's_bot_center': 's_bot_center_shelf_unit',
    'cc_1':         'cc_1',
    'wb_top':       'wb_top',
    'wb_left':      'wb_left',
    'wb_bottom':    'wb_bottom',
}

BLOCK_MODELS = [
    's_top_block_red',        's_top_block_green',        's_top_block_blue',
    's_left_mid_block_red',   's_left_mid_block_green',   's_left_mid_block_blue',
    's_center_mid_block_red', 's_center_mid_block_green', 's_center_mid_block_blue',
    's_bot_left_block_red',   's_bot_left_block_green',   's_bot_left_block_blue',
    's_bot_center_block_red', 's_bot_center_block_green', 's_bot_center_block_blue',
]

TRACKED_MODELS = {**STATION_MODELS, **{name: f'{name}_link' for name in BLOCK_MODELS}}

def quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """ROS-convention [x,y,z,w] quaternion -> 4x4 homogeneous rotation matrix."""
    n = x * x + y * y + z * z + w * w
    if n < 1e-10:
        return np.identity(4)
    s = 2.0 / n
    X, Y, Z = x * s, y * s, z * s
    wX, wY, wZ = w * X, w * Y, w * Z
    xX, xY, xZ = x * X, x * Y, x * Z
    yY, yZ = y * Y, y * Z
    zZ = z * Z
    return np.array([
        [1.0 - (yY + zZ), xY - wZ,         xZ + wY,         0.0],
        [xY + wZ,         1.0 - (xX + zZ), yZ - wX,         0.0],
        [xZ - wY,         yZ + wX,         1.0 - (xX + yY), 0.0],
        [0.0,              0.0,             0.0,             1.0],
    ])


def to_matrix(translation, rotation) -> np.ndarray:
    m = quaternion_matrix(rotation.x, rotation.y, rotation.z, rotation.w)
    m[0][3] = translation.x
    m[1][3] = translation.y
    m[2][3] = translation.z
    return m


def quaternion_from_matrix(matrix: np.ndarray):
    """3x3/4x4 rotation matrix -> ROS-convention [x,y,z,w] quaternion.
    Robust eigenvalue method (Bar-Itzhack / Shepperd), avoids the usual
    trace-based method's edge cases near +/-180 degree rotations."""
    m = np.asarray(matrix, dtype=np.float64)[:4, :4]
    m00, m01, m02 = m[0, 0], m[0, 1], m[0, 2]
    m10, m11, m12 = m[1, 0], m[1, 1], m[1, 2]
    m20, m21, m22 = m[2, 0], m[2, 1], m[2, 2]

    K = np.array([
        [m00 - m11 - m22, 0.0,             0.0,             0.0],
        [m01 + m10,       m11 - m00 - m22, 0.0,             0.0],
        [m02 + m20,       m12 + m21,       m22 - m00 - m11, 0.0],
        [m21 - m12,       m02 - m20,       m10 - m01,       m00 + m11 + m22],
    ])
    K /= 3.0

    eigvals, eigvecs = np.linalg.eigh(K)
    q = eigvecs[[3, 0, 1, 2], np.argmax(eigvals)]  # [w, x, y, z]
    if q[0] < 0.0:
        q = -q
    return np.array([q[1], q[2], q[3], q[0]])  # [x, y, z, w]


class GazeboStationRelay(Node):

    def __init__(self):
        super().__init__('gazebo_station_relay')

        self.latest = {}

        self.sub = self.create_subscription(
            TFMessage, '/world/empty/pose/info', self._on_pose_info, 50
        )
        self.broadcaster = TransformBroadcaster(self)

        self.publish_timer = self.create_timer(0.1, self._publish_live_transforms)  # 10 Hz

        self.get_logger().info(
            'GazeboStationRelay started (SIM ONLY -- reads /world/empty/pose/info).'
        )

    def _on_pose_info(self, msg: TFMessage):
 
        for tr in msg.transforms:
            self.latest[tr.child_frame_id] = to_matrix(
                tr.transform.translation, tr.transform.rotation
            )

    def _publish_live_transforms(self):
        if ROBOT_MODEL_NAME not in self.latest:
            return

        try:
            robot_world_inv = np.linalg.inv(self.latest[ROBOT_MODEL_NAME])
        except np.linalg.LinAlgError:
            self.get_logger().warn('Robot world pose matrix not invertible, skipping this update.')
            return

        stamp = self.get_clock().now().to_msg()

        for friendly_name, gz_model_name in TRACKED_MODELS.items():
            if gz_model_name not in self.latest:
                continue

            relative = robot_world_inv @ self.latest[gz_model_name]
            t = relative[:3, 3]
            q = quaternion_from_matrix(relative)

            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = 'base_footprint'
            tf_msg.child_frame_id = f'{friendly_name}_live'
            tf_msg.transform.translation.x = float(t[0])
            tf_msg.transform.translation.y = float(t[1])
            tf_msg.transform.translation.z = float(t[2])
            tf_msg.transform.rotation.x = float(q[0])
            tf_msg.transform.rotation.y = float(q[1])
            tf_msg.transform.rotation.z = float(q[2])
            tf_msg.transform.rotation.w = float(q[3])

            self.broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboStationRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()