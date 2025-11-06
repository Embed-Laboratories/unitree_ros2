#!/usr/bin/env python3
"""
Odometry to TF Broadcaster

Subscribes to /utlidar/robot_odom and broadcasts the odom→base_link transform
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')

        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            Odometry,
            '/utlidar/robot_odom',
            self.odom_callback,
            10
        )

        self.get_logger().info('Odometry to TF broadcaster started')
        self.get_logger().info('Subscribing to: /utlidar/robot_odom')
        self.get_logger().info('Broadcasting: odom → base_link')

    def odom_callback(self, msg):
        t = TransformStamped()

        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id  # odom
        t.child_frame_id = msg.child_frame_id    # base_link

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    odom_to_tf = OdomToTF()
    rclpy.spin(odom_to_tf)
    odom_to_tf.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
