#!/usr/bin/python3

import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node


class OdomToPath(Node):
    def __init__(self):
        super().__init__('odom_to_path')

        self.declare_parameter('min_distance', 0.05)
        self.declare_parameter('min_angle', 0.05)
        self.declare_parameter('max_poses', 5000)

        self._min_distance = float(self.get_parameter('min_distance').value)
        self._min_angle = float(self.get_parameter('min_angle').value)
        self._max_poses = int(self.get_parameter('max_poses').value)

        self._path = Path()
        self._last_pose = None
        self._last_yaw = None

        self._path_publisher = self.create_publisher(Path, '/odom_path', 10)
        self.create_subscription(Odometry, '/odom', self._odom_callback, 10)

    @staticmethod
    def _yaw_from_quaternion(orientation):
        sin_yaw = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        )
        return math.atan2(sin_yaw, cos_yaw)

    def _odom_callback(self, odom):
        frame_id = odom.header.frame_id
        if not frame_id:
            self.get_logger().warning('Ignoring odometry without frame_id')
            return

        if self._path.header.frame_id not in ('', frame_id):
            self.get_logger().warning(
                'Odometry frame changed; clearing the accumulated path'
            )
            self._path.poses.clear()
            self._last_pose = None
            self._last_yaw = None

        current_pose = odom.pose.pose
        current_yaw = self._yaw_from_quaternion(current_pose.orientation)

        if self._last_pose is not None:
            dx = current_pose.position.x - self._last_pose.position.x
            dy = current_pose.position.y - self._last_pose.position.y
            distance = math.hypot(dx, dy)
            angle = abs(math.atan2(
                math.sin(current_yaw - self._last_yaw),
                math.cos(current_yaw - self._last_yaw),
            ))
            if distance < self._min_distance and angle < self._min_angle:
                return

        stamped_pose = PoseStamped()
        stamped_pose.header = odom.header
        stamped_pose.pose = current_pose

        self._path.header = odom.header
        self._path.poses.append(stamped_pose)
        if len(self._path.poses) > self._max_poses:
            self._path.poses = self._path.poses[-self._max_poses:]

        self._last_pose = current_pose
        self._last_yaw = current_yaw
        self._path_publisher.publish(self._path)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToPath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
