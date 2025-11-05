/**
 * Go2 Camera Bridge Node
 *
 * This node subscribes to /frontvideostream (Go2FrontVideoData) and converts
 * the video data to standard ROS2 sensor_msgs/Image for visualization in RViz2.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include "unitree_go/msg/go2_front_video_data.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>

class Go2CameraBridge : public rclcpp::Node
{
public:
    Go2CameraBridge() : Node("go2_camera_bridge")
    {
        // Subscribe to front video stream
        video_sub_ = this->create_subscription<unitree_go::msg::Go2FrontVideoData>(
            "/frontvideostream",
            10,
            std::bind(&Go2CameraBridge::videoCallback, this, std::placeholders::_1));

        // Publisher for decoded image
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);
        camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/camera_info", 10);

        // Parameters for video resolution selection
        this->declare_parameter("resolution", "360p");  // Options: 720p, 360p, 180p
        resolution_ = this->get_parameter("resolution").as_string();

        RCLCPP_INFO(this->get_logger(), "Go2 Camera Bridge started. Resolution: %s", resolution_.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing decoded images to /camera/image_raw");
    }

private:
    void videoCallback(const unitree_go::msg::Go2FrontVideoData::SharedPtr msg)
    {
        // Select video data based on resolution parameter
        const std::vector<uint8_t>* video_data = nullptr;

        if (resolution_ == "720p" && !msg->video720p.empty()) {
            video_data = &msg->video720p;
        } else if (resolution_ == "360p" && !msg->video360p.empty()) {
            video_data = &msg->video360p;
        } else if (resolution_ == "180p" && !msg->video180p.empty()) {
            video_data = &msg->video180p;
        } else {
            // Fallback: try any available resolution
            if (!msg->video360p.empty()) {
                video_data = &msg->video360p;
            } else if (!msg->video720p.empty()) {
                video_data = &msg->video720p;
            } else if (!msg->video180p.empty()) {
                video_data = &msg->video180p;
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "No video data available in any resolution");
                return;
            }
        }

        if (video_data->empty()) {
            return;
        }

        try {
            // Log video data info
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Received video data: size=%zu bytes, 720p=%zu, 360p=%zu, 180p=%zu",
                video_data->size(), msg->video720p.size(), msg->video360p.size(), msg->video180p.size());

            // Check if data looks like JPEG (starts with FF D8 FF)
            if (video_data->size() >= 3) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Video data header: %02X %02X %02X",
                    (*video_data)[0], (*video_data)[1], (*video_data)[2]);
            }

            // Decode JPEG data
            cv::Mat decoded_image = cv::imdecode(*video_data, cv::IMREAD_COLOR);

            if (decoded_image.empty()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Failed to decode image data (size=%zu bytes)", video_data->size());
                return;
            }

            // Convert to ROS2 Image message
            std_msgs::msg::Header header;
            header.stamp = this->now();
            header.frame_id = "camera_optical_frame";

            sensor_msgs::msg::Image::SharedPtr image_msg =
                cv_bridge::CvImage(header, "bgr8", decoded_image).toImageMsg();

            // Publish image
            image_pub_->publish(*image_msg);

            // Publish camera info (with basic/placeholder values)
            auto camera_info_msg = std::make_shared<sensor_msgs::msg::CameraInfo>();
            camera_info_msg->header = header;
            camera_info_msg->height = decoded_image.rows;
            camera_info_msg->width = decoded_image.cols;
            camera_info_msg->distortion_model = "plumb_bob";

            camera_info_pub_->publish(*camera_info_msg);

            // Log success (throttled to avoid spam)
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Publishing camera images: %dx%d (%s)",
                decoded_image.cols, decoded_image.rows, resolution_.c_str());

        } catch (const cv::Exception& e) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "OpenCV exception: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Exception: %s", e.what());
        }
    }

    rclcpp::Subscription<unitree_go::msg::Go2FrontVideoData>::SharedPtr video_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    std::string resolution_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Go2CameraBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
