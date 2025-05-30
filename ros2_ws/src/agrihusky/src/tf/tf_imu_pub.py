#!/usr/bin/env python3

import rclpy


from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class StaticTfPublisher(Node):
    def __init__(self):
        super().__init__('static_tf_publisher')
        self.broadcaster = StaticTransformBroadcaster(self)

        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'base_link'
        static_transform.child_frame_id = 'imu_link'

        # Set translation (in meters)
        static_transform.transform.translation.x = 0.0
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.0

        # Set rotation (quaternion)
        q = R.from_euler('z', 1.5708).as_quat()
        static_transform.transform.rotation.x = q[0]
        static_transform.transform.rotation.y = q[1]
        static_transform.transform.rotation.z = q[2]
        static_transform.transform.rotation.w = q[3]

        self.broadcaster.sendTransform(static_transform)
        self.get_logger().info('Broadcasting static transform imu_link -> base_link')


def main(args=None):
    rclpy.init(args=args)
    node = StaticTfPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()