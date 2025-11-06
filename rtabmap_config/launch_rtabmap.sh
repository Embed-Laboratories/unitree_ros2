#!/bin/bash

# RTABMap Launch Script - Complete Setup
# Starts static TFs, odom→base_link broadcaster, and RTABMap

echo "========================================="
echo "  RTABMap SLAM for Unitree Go2"
echo "========================================="
echo ""

# Source environment
source ~/unitree_ros2/setup.sh

# Launch static TF transforms in background
echo "Starting static TF transforms..."
ros2 launch ~/unitree_ros2/rtabmap_config/launch/static_transforms.launch.py &
STATIC_TF_PID=$!
sleep 1

# Launch odometry→TF broadcaster in background
echo "Starting odom→base_link TF broadcaster..."
/usr/bin/python3 ~/unitree_ros2/rtabmap_config/launch/odom_to_tf.py &
ODOM_TF_PID=$!
sleep 1

# Launch camera timestamp synchronizer in background
echo "Starting camera timestamp synchronizer..."
/usr/bin/python3 ~/unitree_ros2/rtabmap_config/launch/camera_sync.py &
CAMERA_SYNC_PID=$!
sleep 1

# Launch RTABMap
echo "Launching RTABMap SLAM with camera..."
ros2 launch ~/unitree_ros2/rtabmap_config/launch/rtabmap_go2.launch.py

# Cleanup on exit
trap "kill $STATIC_TF_PID $ODOM_TF_PID $CAMERA_SYNC_PID 2>/dev/null" EXIT
