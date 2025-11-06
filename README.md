# Unitree Go2 Camera Bridge for ROS2

Minimal ROS2 package for visualizing the Unitree Go2 camera feed in RViz2.

**✅ TESTED AND WORKING** on Ubuntu 22.04 + ROS2 Humble + Unitree Go2

## How It Works

The Go2 robot streams H.264 video over **UDP multicast** at `230.1.1.1:1720`. This bridge captures the H.264 stream using GStreamer and converts it to standard ROS2 `sensor_msgs/Image` messages for RViz2 visualization.

## Prerequisites

- Ubuntu 22.04
- ROS2 Humble
- Unitree Go2 robot connected via Ethernet
- GStreamer (pre-installed on Ubuntu 22.04)

## Installation

### 1. Install Dependencies

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp ros-humble-cv-bridge ros-humble-rviz2
```

### 2. Build

```bash
source /opt/ros/humble/setup.bash
cd example
colcon build
```

## Usage

### Quick Start (2 Terminals)

**Terminal 1 - Camera Bridge:**
```bash
source ~/unitree_ros2/setup.sh
cd ~/unitree_ros2/example

# Replace enx9c69d31babe2 with YOUR network interface name
./install/unitree_ros2_example/lib/unitree_ros2_example/go2_camera_gstreamer --ros-args -p network_interface:=enx9c69d31babe2
```

Expected output:
```
[INFO] [go2_camera_gstreamer]: Opening camera with GStreamer pipeline:
[INFO] [go2_camera_gstreamer]: Camera stream opened successfully!
[INFO] [go2_camera_gstreamer]: Publishing camera images to /camera/image_raw
```

**Terminal 2 - RViz2:**
```bash
source ~/unitree_ros2/setup.sh
ros2 run rviz2 rviz2
```

### RViz2 Configuration

1. **Set Fixed Frame**: `camera_optical_frame` (camera-only) or `utlidar_lidar` (with LiDAR)
2. **Add Image Display**:
   - Click **Add** → **Image**
   - Set **Image Topic** to `/camera/image_raw`
3. Done! Camera feed should appear

### Finding Your Network Interface

```bash
# Method 1: Check setup.sh
cat ~/unitree_ros2/setup.sh | grep "NetworkInterface name"
# Output: <NetworkInterface name="enx9c69d31babe2" .../>

# Method 2: List all interfaces
ifconfig
```

Use the interface name connected to your robot (e.g., `enx9c69d31babe2`).

## Verification

```bash
# Check camera topics exist
ros2 topic list | grep camera
# Should show: /camera/camera_info and /camera/image_raw

# Check publishing rate (should be ~30 Hz)
ros2 topic hz /camera/image_raw

# Check camera node is running
ros2 node list | grep camera
# Should show: /go2_camera_gstreamer
```

## Network Configuration

**IMPORTANT**: Edit `setup.sh` to match your network interface.

The `setup.sh` file contains:
```xml
<NetworkInterface name="YOUR_INTERFACE" priority="default" multicast="default" />
```

Replace `YOUR_INTERFACE` with your actual Ethernet interface name.

## Troubleshooting

### No camera image appears

1. **Verify camera bridge is running**:
   ```bash
   ros2 node list | grep camera
   ```

2. **Check network interface matches**:
   ```bash
   cat ~/unitree_ros2/setup.sh | grep "NetworkInterface name"
   ```

3. **Test network connection**:
   ```bash
   ping 192.168.123.161  # or your robot's IP
   ```

4. **Install GStreamer plugins if needed**:
   ```bash
   sudo apt install gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-libav
   ```

### Camera bridge fails to start

- Ensure robot is powered on and connected via Ethernet
- Verify network interface name is correct
- Check your IP is on the robot's subnet (192.168.123.x)

### GStreamer warnings

Warnings like "Cannot query video position" are **normal** for live streams and don't affect functionality.

## Technical Details

**Video Pipeline**:
```
Go2 Robot (230.1.1.1:1720)
  → UDP H.264 Stream
  → GStreamer: udpsrc → rtph264depay → h264parse → avdec_h264 → videoconvert → appsink
  → ROS2: sensor_msgs/Image (BGR8)
  → Topic: /camera/image_raw
```

**Specifications**:
- Resolution: 1280x720 (720p HD)
- Frame Rate: ~30 FPS
- Encoding: H.264
- Transport: UDP multicast
- Latency: <100ms
- Format: BGR8

**Published Topics**:
- `/camera/image_raw` - sensor_msgs/Image
- `/camera/camera_info` - sensor_msgs/CameraInfo

## Advanced Usage

### Save RViz Configuration

```bash
# In RViz2: File → Save Config As
# Save to: ~/unitree_ros2/rviz_configs/camera.rviz

# Load next time:
ros2 run rviz2 rviz2 -d ~/unitree_ros2/rviz_configs/camera.rviz
```

### Visualize with LiDAR

If you have LiDAR data on `/utlidar/cloud`:

1. Set Fixed Frame to `utlidar_lidar`
2. Add **PointCloud2** → Topic: `/utlidar/cloud`
3. Add **Image** → Topic: `/camera/image_raw`

## Files

```
unitree_ros2/
├── README.md                      # This file
├── LICENSE
├── setup.sh                       # Network configuration (EDIT THIS!)
├── setup_local.sh                 # Local testing (loopback)
├── setup_default.sh               # Default (no interface)
└── example/
    ├── src/
    │   ├── src/go2/go2_camera_gstreamer.cpp  # Camera bridge source
    │   ├── CMakeLists.txt
    │   └── package.xml
    └── install/                   # Built executables (after colcon build)
```

## Tested Configuration

✅ Verified working:
- **OS**: Ubuntu 22.04
- **ROS2**: Humble
- **Robot**: Unitree Go2
- **GStreamer**: 1.20.3
- **OpenCV**: 4.5.4

## License

BSD 3-Clause License

## Notes

- The `/frontvideostream` topic exists but only contains metadata (720 bytes), NOT actual video
- Always use `go2_camera_gstreamer` (not the old `go2_camera_bridge`)
- Network interface parameter is **required** for proper multicast reception
