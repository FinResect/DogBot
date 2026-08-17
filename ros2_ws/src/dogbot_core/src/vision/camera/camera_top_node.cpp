#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <atomic>
#include <string>
#include <thread>

namespace dogbot_core::camera {

class CameraTopNode : public rclcpp::Node {
public:
    explicit CameraTopNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
        : Node("camera_top_node", opts) {
        device_                = declare_parameter("device", "/dev/video0");
        width_                 = declare_parameter("width", 640);
        height_                = declare_parameter("height", 480);
        fps_                   = declare_parameter("fps", 30);
        std::string fourcc_str = declare_parameter("fourcc", "MJPG");
        topic_                 = declare_parameter("topic", "/camera/top/image_raw");
        rotate_                = declare_parameter("rotate", 180);

        int fourcc =
            cv::VideoWriter::fourcc(fourcc_str[0], fourcc_str[1], fourcc_str[2], fourcc_str[3]);

        pub_ = create_publisher<sensor_msgs::msg::Image>(topic_, 10);

        cap_.open(device_, cv::CAP_V4L2);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(get_logger(), "Failed to open camera: %s", device_.c_str());
            return;
        }

        cap_.set(cv::CAP_PROP_FOURCC, fourcc);
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
        cap_.set(cv::CAP_PROP_FPS, fps_);

        int actual_w = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
        int actual_h = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
        if (actual_w != width_ || actual_h != height_) {
            RCLCPP_WARN(
                get_logger(), "Format negotiation mismatch: requested %dx%d, camera provided %dx%d",
                width_, height_, actual_w, actual_h);
        }

        RCLCPP_INFO(
            get_logger(), "Camera opened: %s %dx%d@%d (fourcc %s), publishing to %s",
            device_.c_str(), actual_w, actual_h, fps_, fourcc_str.c_str(), topic_.c_str());

        capture_thread_ = std::thread(&CameraTopNode::captureLoop, this);
    }

    ~CameraTopNode() override {
        running_ = false;
        if (capture_thread_.joinable()) {
            capture_thread_.join();
        }
        cap_.release();
    }

private:
    void captureLoop() {
        while (running_ && rclcpp::ok()) {
            cv::Mat frame;
            cap_ >> frame;
            if (frame.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            switch (rotate_) {
            case 90: cv::rotate(frame, frame, cv::ROTATE_90_CLOCKWISE); break;
            case 180: cv::rotate(frame, frame, cv::ROTATE_180); break;
            case 270: cv::rotate(frame, frame, cv::ROTATE_90_COUNTERCLOCKWISE); break;
            default: break;
            }

            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            msg->header.stamp    = now();
            msg->header.frame_id = "camera_top";
            pub_->publish(*msg);
        }
    }

    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    std::thread capture_thread_;
    std::atomic<bool> running_{true};
    std::string device_;
    std::string topic_;
    int width_;
    int height_;
    int fps_;
    int rotate_;
};

} // namespace dogbot_core::camera

RCLCPP_COMPONENTS_REGISTER_NODE(dogbot_core::camera::CameraTopNode)
