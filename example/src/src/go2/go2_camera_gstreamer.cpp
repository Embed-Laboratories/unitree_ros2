/**
 * Go2 Camera GStreamer Bridge Node
 *
 * This node captures H.264 video from the Unitree Go2 front camera via UDP multicast
 * and publishes it as standard ROS2 sensor_msgs/Image for visualization in RViz2.
 *
 * The Go2 streams H.264 video over UDP multicast at 230.1.1.1:1720
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class Go2CameraGStreamer : public rclcpp::Node
{
public:
    Go2CameraGStreamer() : Node("go2_camera_gstreamer")
    {
        // GStreamer pipeline for H.264 decoding from UDP multicast
        // Based on Unitree Go2 SDK documentation
        std::string gst_pipeline =
            "udpsrc address=230.1.1.1 port=1720 multicast-iface=eth0 ! "
            "application/x-rtp,media=video,encoding-name=H264 ! "
            "rtph264depay ! "
            "h264parse ! "
            "avdec_h264 ! "
            "videoconvert ! "
            "appsink";

        // Allow user to override network interface
        this->declare_parameter("network_interface", "eth0");
        std::string net_interface = this->get_parameter("network_interface").as_string();

        // Replace eth0 with actual interface
        size_t pos = gst_pipeline.find("multicast-iface=eth0");
        if (pos != std::string::npos) {
            gst_pipeline.replace(pos, 20, "multicast-iface=" + net_interface);
        }

        RCLCPP_INFO(this->get_logger(), "Opening camera with GStreamer pipeline:");
        RCLCPP_INFO(this->get_logger(), "%s", gst_pipeline.c_str());

        // Open video capture with GStreamer
        cap_.open(gst_pipeline, cv::CAP_GSTREAMER);

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(),
                "Failed to open camera stream. Check network connection and GStreamer installation.");
            RCLCPP_ERROR(this->get_logger(),
                "Make sure the robot is connected and streaming video to 230.1.1.1:1720");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Camera stream opened successfully!");

        // Publishers
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);
        camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/camera_info", 10);

        // Timer for frame capture (30 Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&Go2CameraGStreamer::captureFrame, this));

        RCLCPP_INFO(this->get_logger(), "Publishing camera images to /camera/image_raw");
    }

    ~Go2CameraGStreamer()
    {
        if (cap_.isOpened()) {
            cap_.release();
        }
    }

private:
    void captureFrame()
    {
        if (!cap_.isOpened()) {
            return;
        }

        cv::Mat frame;
        if (!cap_.read(frame)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Failed to read frame from camera");
            return;
        }

        if (frame.empty()) {
            return;
        }

        // Convert to ROS2 Image message
        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "camera_optical_frame";

        sensor_msgs::msg::Image::SharedPtr image_msg =
            cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();

        // Publish image
        image_pub_->publish(*image_msg);

        // Publish camera info with estimated calibration
        // Typical parameters for 1280x720 camera
        auto camera_info_msg = std::make_shared<sensor_msgs::msg::CameraInfo>();
        camera_info_msg->header = header;
        camera_info_msg->height = frame.rows;
        camera_info_msg->width = frame.cols;
        camera_info_msg->distortion_model = "plumb_bob";

        // Estimated camera intrinsics for 1280x720
        // fx, fy = focal length (pixels) - typical ~900 for this resolution
        // cx, cy = optical center (image width/2, height/2)
        double fx = 900.0;
        double fy = 900.0;
        double cx = frame.cols / 2.0;  // 640.0
        double cy = frame.rows / 2.0;  // 360.0

        // K: Camera intrinsic matrix (3x3)
        // [fx  0  cx]
        // [ 0 fy  cy]
        // [ 0  0   1]
        camera_info_msg->k = {
            fx,  0.0, cx,
            0.0, fy,  cy,
            0.0, 0.0, 1.0
        };

        // D: Distortion coefficients (k1, k2, t1, t2, k3)
        // Assuming minimal distortion for now
        camera_info_msg->d = {0.0, 0.0, 0.0, 0.0, 0.0};

        // R: Rectification matrix (3x3) - identity for monocular camera
        camera_info_msg->r = {
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        };

        // P: Projection matrix (3x4)
        // [fx  0  cx  0]
        // [ 0 fy  cy  0]
        // [ 0  0   1  0]
        camera_info_msg->p = {
            fx,  0.0, cx,  0.0,
            0.0, fy,  cy,  0.0,
            0.0, 0.0, 1.0, 0.0
        };

        camera_info_pub_->publish(*camera_info_msg);

        frame_count_++;
        if (frame_count_ % 150 == 0) {  // Log every 5 seconds at 30 Hz
            RCLCPP_INFO(this->get_logger(),
                "Publishing camera frames: %dx%d, frame count: %ld",
                frame.cols, frame.rows, frame_count_);
        }
    }

    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    long frame_count_ = 0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Go2CameraGStreamer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
