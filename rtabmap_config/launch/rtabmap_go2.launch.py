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
                'subscribe_rgb': False,  # Temporarily disabled for testing
                'subscribe_rgbd': False,
                'subscribe_stereo': False,
                'subscribe_scan': False,
                'subscribe_scan_cloud': True,
                'subscribe_user_data': False,

                # Synchronization
                'approx_sync': True,
                'sync_queue_size': 100,
                'topic_queue_size': 100,

                # Publish TF
                'publish_tf': True,

                # Ignore old data - only process recent scans
                'wait_for_transform_duration': 0.2,
                'tf_tolerance': 1.0,  # Allow 1 second tolerance for TF lookup

                # Memory management - AGGRESSIVE NODE CREATION
                'Mem/IncrementalMemory': 'true',
                'Mem/InitWMWithAllNodes': 'false',
                'Mem/RehearsalSimilarity': '0.1',      # FORCE: Low threshold = more nodes
                'Mem/BadSignaturesIgnored': 'false',   # FORCE: Don't reject any scans
                'Mem/STMSize': '1',                    # FORCE: Move to LTM immediately

                # RTABMap core - FORCE HIGH FREQUENCY
                'Rtabmap/CreateIntermediateNodes': 'true',   # Create nodes between closures
                'Rtabmap/ImageBufferSize': '0',              # Process all data
                'Rtabmap/DetectionRate': '10.0',             # High frequency processing

                # SLAM mode - ULTRA SENSITIVE THRESHOLDS
                'RGBD/NeighborLinkRefining': 'true',
                'RGBD/ProximityBySpace': 'true',
                'RGBD/ProximityPathMaxNeighbors': '10',
                'RGBD/AngularUpdate': '0.001',   # FORCE: 0.057° = create on tiny rotation
                'RGBD/LinearUpdate': '0.001',    # FORCE: 1mm = create on tiny movement
                'RGBD/OptimizeFromGraphEnd': 'false',
                'RGBD/MaxOdomCacheSize': '100',  # Large odometry cache

                # ICP parameters - VERY LENIENT MATCHING
                'Icp/MaxTranslation': '5',                   # FORCE: Accept large translations
                'Icp/VoxelSize': '0.1',
                'Icp/MaxCorrespondenceDistance': '2.0',      # FORCE: Lenient matching
                'Icp/CorrespondenceRatio': '0.1',            # FORCE: Accept low correspondence
                'Icp/PointToPlaneK': '20',
                'Icp/PointToPlaneRadius': '0',
                'Icp/Iterations': '10',
                'Icp/Epsilon': '0.001',

                # Visual features for loop closure
                'Kp/MaxFeatures': '400',
                'Kp/DetectorStrategy': '0',  # SURF
                'Vis/MinInliers': '15',
                'Vis/InlierDistance': '0.1',

                # Grid map generation
                'Grid/FromDepth': 'false',
                'Grid/3D': 'true',
                'Grid/RangeMax': '20',
                'Grid/CellSize': '0.05',
            }],
            remappings=[
                ('scan_cloud', '/utlidar/cloud_deskewed'),
                ('odom', '/utlidar/robot_odom'),
                ('rgb/image', '/camera/image_raw'),
                ('rgb/camera_info', '/camera/camera_info'),
            ],
            arguments=[
                '--delete_db_on_start',  # Start fresh each time
                # Remove this argument to keep map between runs
            ]
        ),

    ])
