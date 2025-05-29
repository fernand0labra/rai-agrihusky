import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class TfPublisher(Node):

    def __init__(self):
        super().__init__('tf_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer to publish the transform at a regular interval
        self.timer = self.create_timer(0.5, self.broadcast_transform)

    def broadcast_transform(self):
        # Create a TransformStamped message
        transform_stamped = TransformStamped()

        # Set the header information
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = 'world'  # Parent frame
        transform_stamped.child_frame_id = 'base_link'  # Child frame

        # Set the transform values (translation and rotation)
        transform_stamped.transform.translation.x = 1.0  # Translation along X axis (meters)
        transform_stamped.transform.translation.y = 2.0  # Translation along Y axis (meters)
        transform_stamped.transform.translation.z = 0.0  # Translation along Z axis (meters)

        # Using quaternion for rotation (rotation about Z-axis)
        transform_stamped.transform.rotation.x = 0.0
        transform_stamped.transform.rotation.y = 0.0
        transform_stamped.transform.rotation.z = math.sin(math.radians(45) / 2)
        transform_stamped.transform.rotation.w = math.cos(math.radians(45) / 2)

        # Publish the transform
        self.tf_broadcaster.sendTransform(transform_stamped)

def main(args=None):
    rclpy.init(args=args)
    tf_publisher = TfPublisher()
    rclpy.spin(tf_publisher)
    tf_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
