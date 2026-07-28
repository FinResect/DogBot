#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>

#include <string>
#include <utility>
#include <vector>

namespace dogbot_core::vision {

class ColorDetectNode : public rclcpp::Node {
public:
    explicit ColorDetectNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
        : Node("color_detect_node", opts) {
        min_area_ = declare_parameter("min_area", 500);

        sub_ = create_subscription<sensor_msgs::msg::Image>(
            "image_raw", 10,
            std::bind(&ColorDetectNode::image_callback, this, std::placeholders::_1));

        image_pub_ = create_publisher<sensor_msgs::msg::Image>("/vision/color_detect/image", 10);

        reg("red", std::vector<int64_t>{0, 120, 70}, std::vector<int64_t>{10, 255, 255},
            std::vector<int64_t>{170, 120, 70}, std::vector<int64_t>{179, 255, 255},
            cv::Scalar(0, 0, 255));

        reg("blue", std::vector<int64_t>{100, 120, 70}, std::vector<int64_t>{130, 255, 255},
            std::vector<int64_t>(), std::vector<int64_t>(), cv::Scalar(255, 0, 0));

        reg("green", std::vector<int64_t>{40, 50, 50}, std::vector<int64_t>{80, 255, 255},
            std::vector<int64_t>(), std::vector<int64_t>(), cv::Scalar(0, 255, 0));

        reg("brown", std::vector<int64_t>{10, 50, 30}, std::vector<int64_t>{25, 200, 150},
            std::vector<int64_t>(), std::vector<int64_t>(), cv::Scalar(42, 42, 165));

        reg("purple", std::vector<int64_t>{130, 50, 50}, std::vector<int64_t>{160, 255, 255},
            std::vector<int64_t>(), std::vector<int64_t>(), cv::Scalar(240, 32, 160));

        reg("orange", std::vector<int64_t>{5, 120, 70}, std::vector<int64_t>{18, 255, 255},
            std::vector<int64_t>(), std::vector<int64_t>(), cv::Scalar(0, 165, 255));
    }

private:
    struct ColorEntry {
        cv::Scalar lower1;
        cv::Scalar upper1;
        cv::Scalar lower2;
        cv::Scalar upper2;
        bool has_range2;
        std::string name;
        cv::Scalar bgr;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub;
    };

    void
        reg(const std::string& name, const std::vector<int64_t>& def_low1,
            const std::vector<int64_t>& def_up1, const std::vector<int64_t>& def_low2,
            const std::vector<int64_t>& def_up2, const cv::Scalar& bgr) {
        ColorEntry e;
        e.name = name;
        e.bgr  = bgr;

        auto low1 = declare_parameter("hsv_" + name + "_lower1", def_low1);
        auto up1  = declare_parameter("hsv_" + name + "_upper1", def_up1);
        e.lower1  = cv::Scalar(low1[0], low1[1], low1[2]);
        e.upper1  = cv::Scalar(up1[0], up1[1], up1[2]);

        if (!def_low2.empty()) {
            auto low2    = declare_parameter("hsv_" + name + "_lower2", def_low2);
            auto up2     = declare_parameter("hsv_" + name + "_upper2", def_up2);
            e.lower2     = cv::Scalar(low2[0], low2[1], low2[2]);
            e.upper2     = cv::Scalar(up2[0], up2[1], up2[2]);
            e.has_range2 = true;
        } else {
            e.has_range2 = false;
        }

        e.pub = create_publisher<std_msgs::msg::Bool>("/vision/is_" + name, 10);

        entries_.push_back(std::move(e));
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        auto cv_ptr   = cv_bridge::toCvCopy(msg, "bgr8");
        cv::Mat frame = cv_ptr->image;

        cv::Mat output = frame.clone();
        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(4, 4));

        for (auto& entry : entries_) {
            cv::Mat mask;
            cv::inRange(hsv, entry.lower1, entry.upper1, mask);

            if (entry.has_range2) {
                cv::Mat mask2;
                cv::inRange(hsv, entry.lower2, entry.upper2, mask2);
                cv::bitwise_or(mask, mask2, mask);
            }

            cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
            cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            bool detected    = false;
            double best_area = min_area_;
            cv::Rect best_rect;

            for (const auto& cnt : contours) {
                double area = cv::contourArea(cnt);
                if (area >= min_area_) {
                    detected = true;
                    if (area > best_area) {
                        best_area = area;
                        best_rect = cv::boundingRect(cnt);
                    }
                }
            }

            auto bool_msg = std_msgs::msg::Bool();
            bool_msg.data = detected;
            entry.pub->publish(bool_msg);

            if (detected) {
                cv::rectangle(output, best_rect, entry.bgr, 2);
                cv::putText(
                    output, entry.name, cv::Point(best_rect.x, best_rect.y - 8),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, entry.bgr, 2);
            }
        }

        auto image_msg = cv_bridge::CvImage(msg->header, "bgr8", output).toImageMsg();
        image_pub_->publish(*image_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    std::vector<ColorEntry> entries_;
    int min_area_;
};

} // namespace dogbot_core::vision

RCLCPP_COMPONENTS_REGISTER_NODE(dogbot_core::vision::ColorDetectNode)
