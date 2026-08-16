#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <string>

namespace dogbot_core::camera
{

class CameraNode : public rclcpp::Node
{
public:
  explicit CameraNode(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions())
  : Node("camera_node", opts)
  {
    device_ = declare_parameter("device", "/dev/video2");
    width_ = declare_parameter("width", 640);
    height_ = declare_parameter("height", 480);
    fps_ = declare_parameter("fps", 30);
    std::string fourcc_str = declare_parameter("fourcc", "YUYV");

    int fourcc = cv::VideoWriter::fourcc(
      fourcc_str[0], fourcc_str[1], fourcc_str[2], fourcc_str[3]);

    pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", 10);

    cap_.open(device_, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(get_logger(), "Failed to open camera: %s", device_.c_str());
      return;
    }

    cap_.set(cv::CAP_PROP_FOURCC, fourcc);
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    cap_.set(cv::CAP_PROP_FPS, fps_);

    RCLCPP_INFO(get_logger(), "Camera opened: %s %dx%d@%d", device_.c_str(), width_, height_, fps_);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / fps_),
      std::bind(&CameraNode::capture, this));
  }

  ~CameraNode() override
  {
    cap_.release();
  }

private:
  void capture()
  {
    cv::Mat frame;
    cap_ >> frame;
    if (frame.empty()) {
      return;
    }

    auto msg = cv_bridge::CvImage(
      std_msgs::msg::Header(), "bgr8", frame
    ).toImageMsg();
    msg->header.stamp = now();
    msg->header.frame_id = "camera";
    pub_->publish(*msg);
  }

  cv::VideoCapture cap_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string device_;
  int width_;
  int height_;
  int fps_;
};

}  // namespace dogbot_core::camera

RCLCPP_COMPONENTS_REGISTER_NODE(dogbot_core::camera::CameraNode)
