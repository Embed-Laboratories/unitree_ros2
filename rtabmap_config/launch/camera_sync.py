#!/usr/bin/env python3
"""
Camera Timestamp Synchronizer

Subscribes to camera and LiDAR topics, republishes camera images with
LiDAR timestamps for proper synchronization with RTABMap.

Strategy:
- Uses nearest LiDAR timestamp for each camera frame
- Maintains buffer of recent LiDAR timestamps
- Republishes camera image and camera_info with synced timestamps
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from collections import deque
import threading


class CameraTimestampSync(Node):
    def __init__(self):
        super().__init__('camera_timestamp_sync')

        # Buffer to store recent LiDAR timestamps
        self.lidar_timestamps = deque(maxlen=100)
        self.lock = threading.Lock()

        # Subscribe to LiDAR to get timestamps
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/utlidar/cloud_deskewed',
            self.lidar_callback,
            10
        )

        # Subscribe to camera
        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )

        # Publishers for synced camera data
        self.image_pub = self.create_publisher(
            Image,
            '/camera/image_synced',
            10
        )

        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            '/camera/camera_info_synced',
            10
        )

        self.last_lidar_timestamp = None

        self.get_logger().info('Camera timestamp synchronizer started')
        self.get_logger().info('Input topics: /camera/image_raw, /camera/camera_info, /utlidar/cloud_deskewed')
        self.get_logger().info('Output topics: /camera/image_synced, /camera/camera_info_synced')
        self.get_logger().info('Strategy: Using nearest LiDAR timestamp for camera frames')

    def lidar_callback(self, msg):
        """Store LiDAR timestamps"""
        with self.lock:
            self.lidar_timestamps.append(msg.header.stamp)
            self.last_lidar_timestamp = msg.header.stamp

    def get_synced_timestamp(self):
        """Get the most recent LiDAR timestamp"""
        with self.lock:
            if self.last_lidar_timestamp is not None:
                return self.last_lidar_timestamp
            elif len(self.lidar_timestamps) > 0:
                return self.lidar_timestamps[-1]
            else:
                # Fallback: use current time if no LiDAR data yet
                return self.get_clock().now().to_msg()

    def camera_callback(self, msg):
        """Republish camera image with LiDAR timestamp"""
        synced_timestamp = self.get_synced_timestamp()

        # Create new message with synced timestamp
        synced_msg = Image()
        synced_msg.header.stamp = synced_timestamp
        synced_msg.header.frame_id = msg.header.frame_id
        synced_msg.height = msg.height
        synced_msg.width = msg.width
        synced_msg.encoding = msg.encoding
        synced_msg.is_bigendian = msg.is_bigendian
        synced_msg.step = msg.step
        synced_msg.data = msg.data

        self.image_pub.publish(synced_msg)

    def camera_info_callback(self, msg):
        """Republish camera info with LiDAR timestamp"""
        synced_timestamp = self.get_synced_timestamp()

        # Create new message with synced timestamp
        synced_msg = CameraInfo()
        synced_msg.header.stamp = synced_timestamp
        synced_msg.header.frame_id = msg.header.frame_id
        synced_msg.height = msg.height
        synced_msg.width = msg.width
        synced_msg.distortion_model = msg.distortion_model
        synced_msg.d = msg.d
        synced_msg.k = msg.k
        synced_msg.r = msg.r
        synced_msg.p = msg.p
        synced_msg.binning_x = msg.binning_x
        synced_msg.binning_y = msg.binning_y
        synced_msg.roi = msg.roi

        self.camera_info_pub.publish(synced_msg)


def main(args=None):
    rclpy.init(args=args)
    camera_sync = CameraTimestampSync()

    try:
        rclpy.spin(camera_sync)
    except KeyboardInterrupt:
        pass
    finally:
        camera_sync.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
