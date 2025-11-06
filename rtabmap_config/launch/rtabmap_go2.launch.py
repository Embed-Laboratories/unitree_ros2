#!/usr/bin/env python3

"""
RTABMap Launch File for Unitree Go2 Robot

This launch file configures RTABMap for 3D mapping using:
- Unitree LiDAR point cloud (/utlidar/cloud)
- Camera images (/camera/image_raw) for visual loop closure
- LiDAR odometry (/utlidar/robot_odom)

Usage:
    ros2 launch rtabmap_go2.launch.py

Parameters:
    use_sim_time: Use simulation time (default: false)
    localization: Start in localization mode instead of mapping (default: false)
    rviz: Launch RViz2 for visualization (default: true)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([

        # Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),


        # Static TF transforms
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(os.path.dirname(__file__), 'static_transforms.launch.py')
            )
        ),

        # RTABMap SLAM node
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'frame_id': 'base_link',
                'odom_frame_id': '',  # Empty = subscribe to odom topic directly

                # Subscribe to these topics
                'subscribe_depth': False,
                'subscribe_rgb': True,   # ENABLED: Camera with timestamp sync
                'subscribe_rgbd': False,
                'subscribe_stereo': False,
                'subscribe_scan': False,
                'subscribe_scan_cloud': True,
                'subscribe_user_data': False,

                # Synchronization - AGGRESSIVE SETTINGS
                'approx_sync': True,           # Allow approximate timestamp matching
                'approx_sync_max_interval': 0.5,  # AGGRESSIVE: 500ms max sync window
                'sync_queue_size': 200,        # LARGE: buffer 200 messages
                'topic_queue_size': 200,       # LARGE: queue size for topics

                # Publish TF
                'publish_tf': True,

                # Transform tolerance - VERY PERMISSIVE
                'wait_for_transform_duration': 0.5,  # Wait up to 500ms for transforms
                'tf_tolerance': 2.0,           # AGGRESSIVE: 2 second tolerance for TF lookup

                # Memory management - BALANCED for rotation stability
                'Mem/IncrementalMemory': 'true',
                'Mem/InitWMWithAllNodes': 'false',
                'Mem/RehearsalSimilarity': '0.3',      # Moderate threshold for node creation
                'Mem/BadSignaturesIgnored': 'false',   # Don't reject scans
                'Mem/STMSize': '30',                   # FIXED: Keep 30 nodes in working memory for rotation matching

                # RTABMap core - FORCE HIGH FREQUENCY
                'Rtabmap/CreateIntermediateNodes': 'true',   # Create nodes between closures
                'Rtabmap/ImageBufferSize': '0',              # Process all data
                'Rtabmap/DetectionRate': '10.0',             # High frequency processing

                # SLAM mode - BALANCED THRESHOLDS
                'RGBD/NeighborLinkRefining': 'true',
                'RGBD/ProximityBySpace': 'true',
                'RGBD/ProximityPathMaxNeighbors': '10',
                'RGBD/AngularUpdate': '0.05',    # 2.86° = reasonable rotation threshold
                'RGBD/LinearUpdate': '0.10',     # 10cm = reasonable movement threshold
                'RGBD/OptimizeFromGraphEnd': 'false',
                'RGBD/MaxOdomCacheSize': '100',  # Large odometry cache
                'RGBD/OptimizeMaxError': '5.0',  # Increased tolerance for loop closure errors

                # ICP parameters - STRICT for rotation stability
                'Icp/MaxTranslation': '0.2',                 # Limit: 20cm max correction per frame
                'Icp/MaxRotation': '0.78',                   # Limit: ~45° max rotation correction
                'Icp/VoxelSize': '0.1',
                'Icp/MaxCorrespondenceDistance': '0.05',     # Strict: 5cm correspondence distance
                'Icp/CorrespondenceRatio': '0.1',            # Accept low correspondence
                'Icp/PointToPlaneK': '20',
                'Icp/PointToPlaneRadius': '0',
                'Icp/Iterations': '10',
                'Icp/Epsilon': '0.001',
                'Icp/PM': 'true',                            # Use libpointmatcher for robust ICP
                'Icp/PMOutlierRatio': '0.85',                # libpointmatcher outlier filtering

                # Visual features - AGGRESSIVE for rotation stability
                'Kp/MaxFeatures': '1000',           # AGGRESSIVE: Extract many features
                'Kp/DetectorStrategy': '6',         # GFTT/Brief - faster, works in low texture
                'Kp/MaxDepth': '0',                 # No depth filtering (2D only)
                'Kp/MinDepth': '0',
                'Vis/MinInliers': '10',             # RELAXED: Accept fewer inliers
                'Vis/InlierDistance': '0.15',       # RELAXED: Allow larger inlier distance
                'Vis/MaxFeatures': '1000',          # Match many features
                'Vis/CorType': '0',                 # Features Matching
                'Vis/FeatureType': '6',             # GFTT/Brief
                'Vis/EstimationType': '1',          # PnP (Essential for camera-only registration)
                'Reg/Force3DoF': 'false',           # Allow full 6DOF for camera
                'Reg/Strategy': '2',                # Visual+ICP: Use both visual and ICP for registration

                # Grid map generation
                'Grid/FromDepth': 'false',
                'Grid/3D': 'true',
                'Grid/RangeMax': '20',
                'Grid/CellSize': '0.05',
            }],
            remappings=[
                ('scan_cloud', '/utlidar/cloud_deskewed'),
                ('odom', '/utlidar/robot_odom'),
                ('rgb/image', '/camera/image_synced'),        # SYNCED: Using timestamp-synchronized camera
                ('rgb/camera_info', '/camera/camera_info_synced'),  # SYNCED: Using timestamp-synchronized camera_info
            ],
            arguments=[
                '--delete_db_on_start',  # Start fresh each time
                # Remove this argument to keep map between runs
            ]
        ),

    ])
