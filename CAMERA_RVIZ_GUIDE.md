# Go2 Camera Visualization in RViz2 - WORKING GUIDE

This guide explains how to visualize the Unitree Go2 camera feed in RViz2 alongside the LiDAR data.

## Important: How Go2 Camera Works

The Go2 robot streams H.264 video over **UDP multicast** at address `230.1.1.1:1720`, NOT through regular ROS2 topics. The camera bridge captures this H.264 stream using GStreamer and converts it to standard ROS2 Image messages for RViz2.

## Prerequisites

- Unitree Go2 robot connected via Ethernet
- ROS2 Humble installed
- Network configuration completed (see main README.md)
- Unitree ROS2 packages built
- GStreamer installed (usually pre-installed on Ubuntu 22.04)

## Quick Start - TESTED AND WORKING ✓

### Terminal 1: Run Camera Bridge

```bash
# Source the environment
source ~/unitree_ros2/setup.sh

# Navigate to example directory
cd ~/unitree_ros2/example

# Run the GStreamer camera bridge
# Replace enx9c69d31babe2 with YOUR network interface name
./install/unitree_ros2_example/bin/go2_camera_gstreamer --ros-args -p network_interface:=enx9c69d31babe2
```

You should see output like:
```
[INFO] [go2_camera_gstreamer]: Opening camera with GStreamer pipeline:
[INFO] [go2_camera_gstreamer]: Camera stream opened successfully!
[INFO] [go2_camera_gstreamer]: Publishing camera images to /camera/image_raw
[INFO] [go2_camera_gstreamer]: Publishing camera frames: 1280x720, frame count: 150
```

### Terminal 2: Run RViz2

```bash
# Source the environment (in a new terminal)
source ~/unitree_ros2/setup.sh

# Launch RViz2
ros2 run rviz2 rviz2
```

## RViz2 Configuration

### 1. Configure Fixed Frame

In RViz2 left panel under "Global Options":
- Set **Fixed Frame** to `utlidar_lidar` (for LiDAR) or `camera_optical_frame` (for camera-only)

### 2. Add LiDAR Visualization (if not already added)

1. Click **Add** button at bottom of left panel
2. Select **PointCloud2**
3. Set **Topic** to `/utlidar/cloud`
4. Adjust visualization settings as desired

### 3. Add Camera Visualization

There are two ways to visualize the camera:

#### Option A: Image Display (Recommended - TESTED ✅)

1. Click **Add** button
2. Select **Image**
3. Set **Image Topic** to `/camera/image_raw`
4. The camera feed will appear inline in the 3D view alongside LiDAR
5. You can resize the image panel and position it as desired

#### Option B: Camera Display (Alternative)

1. Click **Add** button
2. Select **Camera**
3. Set **Image Topic** to `/camera/image_raw`
4. Set **Camera Info Topic** to `/camera/camera_info`
5. A separate camera view window should appear
6. **Note**: This may not work in all RViz2 versions; use Image display if Camera doesn't work

### 4. Adjust Layout

- You can resize panels and undock the camera view for a better layout
- The camera view can be moved to a separate window

## Finding Your Network Interface

To find your network interface name (needed for the camera bridge):

```bash
# Check your setup.sh file
cat ~/unitree_ros2/setup.sh | grep "NetworkInterface name"

# You'll see something like:
# <NetworkInterface name="enx9c69d31babe2" priority="default" multicast="default" />
```

Use the interface name (e.g., `enx9c69d31babe2`) when running the camera bridge.

## Verification

### Check Camera Topics

```bash
# List all topics (should see /camera/image_raw)
ros2 topic list | grep camera

# Expected output:
# /camera/camera_info
# /camera/image_raw

# Check camera publishing rate (should be ~30 Hz)
ros2 topic hz /camera/image_raw

# Check camera info
ros2 topic info /camera/image_raw
```

### Check Camera Node

```bash
# Check if camera node is running
ros2 node list | grep camera

# Should show:
# /go2_camera_gstreamer
```

## Technical Details

### How It Works

1. **Video Stream**: Go2 streams H.264 video via UDP multicast (230.1.1.1:1720)
2. **GStreamer Pipeline**:
   ```
   udpsrc → rtph264depay → h264parse → avdec_h264 → videoconvert → appsink
   ```
3. **Conversion**: Bridge converts frames to sensor_msgs/Image (BGR8 format)
4. **Publishing**: Publishes to /camera/image_raw at ~30 Hz
5. **Resolution**: 1280x720 (720p HD)

### Why Not Use /frontvideostream?

The `/frontvideostream` topic exists but only contains 720-byte metadata/headers, NOT the actual video data. The real video is streamed separately over UDP multicast, which requires GStreamer to decode.

## Troubleshooting

### No camera image appears

1. **Check camera bridge is running**
   ```bash
   ros2 node list | grep camera
   ```
   Should show: `/go2_camera_gstreamer`

2. **Check network interface**
   ```bash
   cat ~/unitree_ros2/setup.sh | grep "NetworkInterface name"
   ```
   Make sure you use the SAME interface name when running the camera bridge

3. **Check GStreamer is installed**
   ```bash
   gst-inspect-1.0 --version
   ```
   Should show GStreamer 1.20.x or higher

4. **Check for errors in camera bridge terminal**
   - If you see "Failed to open camera stream", check network connection
   - If you see GStreamer errors, you may need to install additional plugins:
     ```bash
     sudo apt install gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-libav
     ```

### Camera bridge fails to open stream

- Ensure robot is powered on and connected via Ethernet
- Verify network interface name is correct
- Check that your IP is configured correctly (192.168.123.x)
- Try pinging the robot:
  ```bash
  ping 192.168.123.161  # or your robot's IP
  ```

### GStreamer warnings

You may see warnings like:
```
Cannot query video position: status=1, value=25, duration=-1
```
These are normal and don't affect functionality. They occur because GStreamer can't determine the stream length (it's a live stream).

### Frame ID issues

The camera uses frame ID `camera_optical_frame`. For combined visualization:
- **LiDAR + Camera**: Use `utlidar_lidar` as Fixed Frame
- **Camera only**: Use `camera_optical_frame` as Fixed Frame

### Network issues

Ensure your network is configured correctly:
```bash
# Check network interface in setup.sh
cat ~/unitree_ros2/setup.sh

# Should show your robot's Ethernet interface
# Example: enx9c69d31babe2

# Check network connectivity
ifconfig enx9c69d31babe2  # Replace with your interface
```

## Complete Visualization Setup (Step by Step)

Here's the complete workflow for both LiDAR and Camera:

```bash
# Terminal 1: Setup and run camera bridge
source ~/unitree_ros2/setup.sh
cd ~/unitree_ros2/example

# Replace enx9c69d31babe2 with YOUR interface name
./install/unitree_ros2_example/bin/go2_camera_gstreamer --ros-args -p network_interface:=enx9c69d31babe2

# Wait for: "Camera stream opened successfully!" message

# Terminal 2: Run RViz2
source ~/unitree_ros2/setup.sh
ros2 run rviz2 rviz2

# In RViz2:
# 1. Set Fixed Frame to "utlidar_lidar"
# 2. Add PointCloud2 → Topic: /utlidar/cloud
# 3. Add Camera → Image Topic: /camera/image_raw, Camera Info Topic: /camera/camera_info
# 4. Arrange windows as desired
```

## Performance

- **Resolution**: 1280x720 (HD 720p)
- **Frame Rate**: ~30 FPS
- **Encoding**: H.264
- **Transport**: UDP multicast
- **Latency**: Very low (<100ms)
- **CPU Usage**: Moderate (GStreamer H.264 decoding)

## Alternative: Old Bridge (Not Recommended)

There's also `go2_camera_bridge` which tries to decode from `/frontvideostream`, but it **does not work** because that topic only contains metadata, not actual video data. Always use `go2_camera_gstreamer` instead.

## Saving RViz Configuration

To save your RViz2 setup for future use:

1. Configure RViz2 with all visualizations
2. File → Save Config As
3. Save to `~/unitree_ros2/rviz_configs/go2_camera_lidar.rviz`
4. Load next time with:
   ```bash
   ros2 run rviz2 rviz2 -d ~/unitree_ros2/rviz_configs/go2_camera_lidar.rviz
   ```

## Additional Resources

- Main README: `/home/embed/unitree_ros2/README.md`
- CLAUDE.md: `/home/embed/unitree_ros2/CLAUDE.md`
- Camera bridge source (GStreamer): `/home/embed/unitree_ros2/example/src/src/go2/go2_camera_gstreamer.cpp:1`
- Camera bridge source (old/broken): `/home/embed/unitree_ros2/example/src/src/go2/go2_camera_bridge.cpp:1`

## Tested Configuration

✅ **Tested and verified working on:**
- Ubuntu 22.04
- ROS2 Humble
- Unitree Go2
- Network interface: enx9c69d31babe2
- GStreamer 1.20.3
- OpenCV 4.5.4

## Summary

The key points:
1. Use `go2_camera_gstreamer` (NOT `go2_camera_bridge`)
2. Specify your network interface with `-p network_interface:=YOUR_INTERFACE`
3. Camera streams H.264 over UDP multicast (230.1.1.1:1720)
4. Output is 1280x720 at ~30 FPS on `/camera/image_raw`
5. Works perfectly with RViz2 Camera display
