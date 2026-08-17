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

class ColorDetectNonblueNode : public rclcpp::Node {
public:
    explicit ColorDetectNonblueNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
        : Node("color_detect_nonblue_node", opts) {
        mode_     = declare_parameter("mode", 1);
        min_area_ = declare_parameter("min_area", 1125);

        sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/top/image_raw", 10,
            std::bind(&ColorDetectNonblueNode::image_callback, this, std::placeholders::_1));

        image_pub_ =
            create_publisher<sensor_msgs::msg::Image>("/vision/color_detect_nonblue/image", 10);

        reg(ColorSpec{
            .name          = "red",
            .bgr           = cv::Scalar(0, 0, 255),
            .hsv_lower1    = {  0, 120,  70},
            .hsv_upper1    = { 10, 255, 255},
            .hsv_lower2    = {170, 120,  70},
            .hsv_upper2    = {179, 255, 255},
            .rgb_color     = {255,   0,   0},
            .rgb_tolerance = 100,
            .lab_color     = {136, 208, 195},
            .lab_tolerance = 20
        });

        reg(ColorSpec{
            .name          = "green",
            .bgr           = cv::Scalar(0, 255, 0),
            .hsv_lower1    = {40, 50, 50},
            .hsv_upper1    = {80, 255, 255},
            .hsv_lower2    = {},
            .hsv_upper2    = {},
            .rgb_color     = {0, 255, 0},
            .rgb_tolerance = 100,
            .lab_color     = {224, 42, 211},
            .lab_tolerance = 20
        });

        reg(ColorSpec{
            .name          = "brown",
            .bgr           = cv::Scalar(42, 42, 165),
            .hsv_lower1    = {10, 50, 30},
            .hsv_upper1    = {25, 255, 180},
            .hsv_lower2    = {},
            .hsv_upper2    = {},
            .rgb_color     = {139, 69, 19},
            .rgb_tolerance = 80,
            .lab_color     = {96, 178, 158},
            .lab_tolerance = 20
        });

        reg(ColorSpec{
            .name          = "purple",
            .bgr           = cv::Scalar(240, 32, 160),
            .hsv_lower1    = {130, 50, 50},
            .hsv_upper1    = {160, 255, 255},
            .hsv_lower2    = {},
            .hsv_upper2    = {},
            .rgb_color     = {128, 0, 128},
            .rgb_tolerance = 80,
            .lab_color     = {116, 207, 51},
            .lab_tolerance = 20
        });

        reg(ColorSpec{
            .name          = "orange",
            .bgr           = cv::Scalar(0, 165, 255),
            .hsv_lower1    = {5, 120, 70},
            .hsv_upper1    = {18, 255, 255},
            .hsv_lower2    = {},
            .hsv_upper2    = {},
            .rgb_color     = {255, 165, 0},
            .rgb_tolerance = 80,
            .lab_color     = {191, 152, 207},
            .lab_tolerance = 20
        });
    }

private:
    struct ColorSpec {
        std::string name;
        cv::Scalar bgr;
        std::vector<int64_t> hsv_lower1;
        std::vector<int64_t> hsv_upper1;
        std::vector<int64_t> hsv_lower2;
        std::vector<int64_t> hsv_upper2;
        std::vector<int64_t> rgb_color; // [R,G,B]
        int64_t rgb_tolerance;
        std::vector<int64_t> lab_color; // [L,a,b]
        int64_t lab_tolerance;
    };

    struct ColorEntry {
        cv::Scalar lower1;
        cv::Scalar upper1;
        cv::Scalar lower2;
        cv::Scalar upper2;
        bool has_range2;
        cv::Scalar rgb_color;           // BGR 顺序，供 RGB 模式使用
        int rgb_tolerance;
        cv::Scalar lab_color;           // [L,a,b]，供 LAB 模式使用
        int lab_tolerance;
        std::string name;
        cv::Scalar bgr;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub;
    };

    void reg(const ColorSpec& spec) {
        ColorEntry e;
        e.name = spec.name;
        e.bgr  = spec.bgr;

        auto low1 = declare_parameter("hsv_" + spec.name + "_lower1", spec.hsv_lower1);
        auto up1  = declare_parameter("hsv_" + spec.name + "_upper1", spec.hsv_upper1);
        e.lower1  = cv::Scalar(low1[0], low1[1], low1[2]);
        e.upper1  = cv::Scalar(up1[0], up1[1], up1[2]);

        if (!spec.hsv_lower2.empty()) {
            auto low2    = declare_parameter("hsv_" + spec.name + "_lower2", spec.hsv_lower2);
            auto up2     = declare_parameter("hsv_" + spec.name + "_upper2", spec.hsv_upper2);
            e.lower2     = cv::Scalar(low2[0], low2[1], low2[2]);
            e.upper2     = cv::Scalar(up2[0], up2[1], up2[2]);
            e.has_range2 = true;
        } else {
            e.has_range2 = false;
        }

        auto rgb_color = declare_parameter("rgb_" + spec.name + "_color", spec.rgb_color);
        e.rgb_color    = cv::Scalar(rgb_color[2], rgb_color[1], rgb_color[0]); // [R,G,B] -> BGR

        e.rgb_tolerance = declare_parameter("rgb_" + spec.name + "_tolerance", spec.rgb_tolerance);

        auto lab_color = declare_parameter("lab_" + spec.name + "_color", spec.lab_color);
        e.lab_color    = cv::Scalar(lab_color[0], lab_color[1], lab_color[2]); // [L,a,b]

        e.lab_tolerance = declare_parameter("lab_" + spec.name + "_tolerance", spec.lab_tolerance);

        e.pub = create_publisher<std_msgs::msg::Bool>("/vision/is_" + spec.name, 10);

        entries_.push_back(std::move(e));
    }
    static void hsv_update(const cv::Mat& hsv_roi, const ColorEntry& entry, cv::Mat& mask) {
        cv::inRange(hsv_roi, entry.lower1, entry.upper1, mask);

        if (entry.has_range2) {
            cv::Mat mask2;
            cv::inRange(hsv_roi, entry.lower2, entry.upper2, mask2);
            cv::bitwise_or(mask, mask2, mask);
        }
    }

    static void rgb_update(const cv::Mat& bgr_roi, const ColorEntry& entry, cv::Mat& mask) {
        cv::Mat diff;
        cv::absdiff(bgr_roi, entry.rgb_color, diff);
        cv::Scalar tolerance(entry.rgb_tolerance, entry.rgb_tolerance, entry.rgb_tolerance);
        cv::inRange(diff, cv::Scalar(0, 0, 0), tolerance, mask);
    }

    static void lab_update(const cv::Mat& lab_roi, const ColorEntry& entry, cv::Mat& mask) {
        cv::Mat diff;
        cv::absdiff(lab_roi, entry.lab_color, diff);
        cv::Scalar tolerance(entry.lab_tolerance, entry.lab_tolerance, entry.lab_tolerance);
        cv::inRange(diff, cv::Scalar(0, 0, 0), tolerance, mask);
    }

    static void draw_mode_text(
        cv::Mat& output, int text_x, int text_y, const cv::Vec3b& bgr_px, const cv::Vec3b& hsv_px,
        const cv::Vec3b& lab_px, int mode) {
        std::vector<std::string> lines;
        bool all = (mode == 4);
        if (mode == 2 || all) {
            lines.push_back(
                "RGB: (" + std::to_string(bgr_px[2]) + ", " + std::to_string(bgr_px[1]) + ", "
                + std::to_string(bgr_px[0]) + ")");
        }
        if (mode == 1 || all) {
            lines.push_back(
                "HSV: (" + std::to_string(hsv_px[0]) + ", " + std::to_string(hsv_px[1]) + ", "
                + std::to_string(hsv_px[2]) + ")");
        }
        if (mode == 3 || all) {
            lines.push_back(
                "LAB: (" + std::to_string(lab_px[0]) + ", " + std::to_string(lab_px[1]) + ", "
                + std::to_string(lab_px[2]) + ")");
        }

        for (size_t i = 0; i < lines.size(); ++i) {
            cv::putText(
                output, lines[i], cv::Point(text_x, text_y + i * 22), cv::FONT_HERSHEY_SIMPLEX,
                0.55, cv::Scalar(0, 0, 0), 4);
            cv::putText(
                output, lines[i], cv::Point(text_x, text_y + i * 22), cv::FONT_HERSHEY_SIMPLEX,
                0.55, cv::Scalar(255, 255, 255), 1);
        }
    }

    void draw_color_probe(cv::Mat& output, const cv::Mat& bgr_frame) {
        int cx = output.cols / 2;
        int cy = output.rows * 5 / 6;

        cv::Scalar cross_color(0, 255, 255);
        cv::line(output, cv::Point(cx - 20, cy), cv::Point(cx + 20, cy), cross_color, 2);
        cv::line(output, cv::Point(cx, cy - 20), cv::Point(cx, cy + 20), cross_color, 2);

        cv::Rect patch_rect(cx - 1, cy - 1, 3, 3);
        cv::Scalar bgr = cv::mean(bgr_frame(patch_rect));

        cv::Mat px(1, 1, CV_8UC3, bgr);
        cv::Mat hsv, lab;
        cv::cvtColor(px, hsv, cv::COLOR_BGR2HSV);
        cv::cvtColor(px, lab, cv::COLOR_BGR2Lab);

        cv::Vec3b b = px.at<cv::Vec3b>(0, 0);
        cv::Vec3b h = hsv.at<cv::Vec3b>(0, 0);
        cv::Vec3b l = lab.at<cv::Vec3b>(0, 0);

        draw_mode_text(output, 10, cy - 20, b, h, l, 4);
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (mode_ == 0) {
            return;
        }

        auto cv_ptr   = cv_bridge::toCvCopy(msg, "bgr8");
        cv::Mat frame = cv_ptr->image;

        cv::Mat output = frame.clone();
        cv::Mat converted;
        if (mode_ == 1) {
            cv::cvtColor(frame, converted, cv::COLOR_BGR2HSV);
        } else if (mode_ == 3) {
            cv::cvtColor(frame, converted, cv::COLOR_BGR2Lab);
        }

        int h_start = frame.rows * 2 / 3;
        cv::Rect roi_rect(0, h_start, frame.cols, frame.rows - h_start);
        cv::Mat roi = (mode_ == 2) ? cv::Mat(frame, roi_rect) : cv::Mat(converted, roi_rect);

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(4, 4));

        for (auto& entry : entries_) {
            cv::Mat mask;
            if (mode_ == 1) {
                hsv_update(roi, entry, mask);
            } else if (mode_ == 2) {
                rgb_update(roi, entry, mask);
            } else if (mode_ == 3) {
                lab_update(roi, entry, mask);
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
                best_rect.y += h_start;
                cv::rectangle(output, best_rect, entry.bgr, 2);
                cv::putText(
                    output, entry.name, cv::Point(best_rect.x, best_rect.y - 8),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, entry.bgr, 2);
            }
        }

        draw_color_probe(output, frame);

        auto image_msg = cv_bridge::CvImage(msg->header, "bgr8", output).toImageMsg();
        image_pub_->publish(*image_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    std::vector<ColorEntry> entries_;
    int min_area_;
    int mode_;
};

} // namespace dogbot_core::vision

RCLCPP_COMPONENTS_REGISTER_NODE(dogbot_core::vision::ColorDetectNonblueNode)
