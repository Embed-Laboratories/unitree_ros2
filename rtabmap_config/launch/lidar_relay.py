#!/usr/bin/env python3
"""
LiDAR Relay Node - Republishes LiDAR data with current timestamp

This node solves the problem of buffered LiDAR data with old timestamps
by subscribing to /utlidar/cloud and republishing to /utlidar/cloud_relay
with updated timestamps set to 'now'.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class LidarRelay(Node):
    def __init__(self):
        super().__init__('lidar_relay')

        self.subscription = self.create_subscription(
            PointCloud2,
            '/utlidar/cloud',
            self.lidar_callback,
            10
        )

        self.publisher = self.create_publisher(
            PointCloud2,
            '/utlidar/cloud_relay',
            10
        )

        self.get_logger().info('LiDAR relay node started')
        self.get_logger().info('Subscribing to: /utlidar/cloud')
        self.get_logger().info('Publishing to: /utlidar/cloud_relay')

    def lidar_callback(self, msg):
        # Create new message with current timestamp
        new_msg = PointCloud2()
        new_msg.header.stamp = self.get_clock().now().to_msg()
        new_msg.header.frame_id = msg.header.frame_id
        new_msg.height = msg.height
        new_msg.width = msg.width
        new_msg.fields = msg.fields
        new_msg.is_bigendian = msg.is_bigendian
        new_msg.point_step = msg.point_step
        new_msg.row_step = msg.row_step
        new_msg.data = msg.data
        new_msg.is_dense = msg.is_dense

        self.publisher.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    lidar_relay = LidarRelay()
    rclpy.spin(lidar_relay)
    lidar_relay.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
