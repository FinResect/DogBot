#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace dogbot_core::vision {

class FollowingNode : public rclcpp::Node {
public:
    explicit FollowingNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
        : Node("following_node", opts) {
        threshold_               = declare_parameter("threshold", 120);
        threshold_mode_          = declare_parameter("threshold_mode", std::string("fixed"));
        threshold_channel_       = declare_parameter("threshold_channel", std::string("value"));
        width_continuity_weight_ = declare_parameter("width_continuity_weight", 0.1);
        blur_ksize_              = declare_parameter("gaussian_kernel_size", 5);
        morph_ksize_             = declare_parameter("morph_kernel_size", 3);
        roi_bottom_ratio_        = declare_parameter("roi_bottom_ratio", 0.6);
        max_gap_rows_            = declare_parameter("max_gap_rows", 5);
        center_smooth_window_    = declare_parameter("center_smooth_window", 9);
        lookahead_ratio_         = declare_parameter("lookahead_ratio", 0.7);
        lookahead_band_ratio_    = declare_parameter("lookahead_band_ratio", 0.12);
        ema_alpha_               = declare_parameter("ema_alpha", 0.25);
        vx_max_                  = declare_parameter("vx_max", 0.4);
        kp_angular_              = declare_parameter("kp_angular", 10.0);

        sub_ = create_subscription<sensor_msgs::msg::Image>(
            "image_raw", 10,
            std::bind(&FollowingNode::image_callback, this, std::placeholders::_1));

        image_pub_  = create_publisher<sensor_msgs::msg::Image>("/vision/following/image", 10);
        binary_pub_ = create_publisher<sensor_msgs::msg::Image>("/vision/following/binary", 10);
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/vision/following/controller", 10);
        theta_pub_ = create_publisher<std_msgs::msg::Float64>("/vision/following/theta", 10);
    }

private:
    static std::vector<cv::Point> extract_centerline(
        const cv::Mat& binary, int roi_start_row, int max_gap_rows,
        double width_continuity_weight) {
        struct Run {
            int start;
            int end;
        };
        std::vector<cv::Point> centerline;
        int bottom   = binary.rows - 1;
        int prev_mid = -1;
        int prev_w   = -1;
        int gap      = 0;

        for (int row = bottom; row >= roi_start_row; --row) {
            const uchar* p = binary.ptr<uchar>(row);
            std::vector<Run> runs;
            int run_start = -1;
            for (int col = 0; col <= binary.cols; ++col) {
                bool fg = (col < binary.cols) && (p[col] > 0);
                if (fg && run_start < 0) {
                    run_start = col;
                } else if (!fg && run_start >= 0) {
                    runs.push_back({run_start, col - 1});
                    run_start = -1;
                }
            }

            int best_mid      = -1;
            int best_w        = -1;
            double best_score = std::numeric_limits<double>::max();
            for (size_t i = 0; i < runs.size(); ++i) {
                for (size_t j = i; j < runs.size(); ++j) {
                    int span_start = runs[i].start;
                    int span_end   = runs[j].end;
                    if (span_start == 0 && span_end == binary.cols - 1) {
                        continue;
                    }
                    int mid   = (span_start + span_end) / 2;
                    int width = span_end - span_start + 1;
                    double score;
                    if (prev_mid < 0) {
                        score = -static_cast<double>(width);
                    } else {
                        score = std::abs(mid - prev_mid)
                              + width_continuity_weight * std::abs(width - prev_w);
                    }
                    if (score < best_score) {
                        best_score = score;
                        best_mid   = mid;
                        best_w     = width;
                    }
                }
            }

            if (best_mid >= 0) {
                centerline.emplace_back(best_mid, row);
                prev_mid = best_mid;
                prev_w   = best_w;
                gap      = 0;
            } else if (runs.empty()) {
                ++gap;
                if (gap > max_gap_rows) {
                    break;
                }
            }
        }
        return centerline;
    }

    static void smooth_centerline(std::vector<cv::Point>& pts, int window) {
        if (window <= 1 || pts.size() < 3) {
            return;
        }
        std::vector<cv::Point> smoothed;
        smoothed.reserve(pts.size());
        int half = window / 2;
        for (size_t i = 0; i < pts.size(); ++i) {
            int lo   = std::max(0, static_cast<int>(i) - half);
            int hi   = std::min(static_cast<int>(pts.size()) - 1, static_cast<int>(i) + half);
            long sum = 0;
            for (int j = lo; j <= hi; ++j) {
                sum += pts[j].x;
            }
            smoothed.emplace_back(static_cast<int>(sum / (hi - lo + 1)), pts[i].y);
        }
        pts.swap(smoothed);
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        auto cv_ptr   = cv_bridge::toCvCopy(msg, "bgr8");
        cv::Mat frame = cv_ptr->image;

        cv::Mat channel;
        if (threshold_channel_ == "value") {
            cv::Mat hsv;
            cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
            cv::extractChannel(hsv, channel, 2);
        } else {
            cv::cvtColor(frame, channel, cv::COLOR_BGR2GRAY);
        }

        int ksize = blur_ksize_;
        if (ksize % 2 == 0) {
            ++ksize;
        }
        if (ksize > 0) {
            cv::GaussianBlur(channel, channel, cv::Size(ksize, ksize), 0);
        }

        cv::Mat binary;
        if (threshold_mode_ == "otsu") {
            cv::threshold(channel, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        } else {
            cv::threshold(channel, binary, threshold_, 255, cv::THRESH_BINARY);
        }

        int mksize = morph_ksize_;
        if (mksize % 2 == 0) {
            ++mksize;
        }
        if (mksize > 1) {
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(mksize, mksize));
            cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);
            cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);
        }

        int bottom_row    = frame.rows - 1;
        int roi_start_row = static_cast<int>(frame.rows * (1.0 - roi_bottom_ratio_));
        int mid_col       = frame.cols / 2;

        std::vector<cv::Point> centerline =
            extract_centerline(binary, roi_start_row, max_gap_rows_, width_continuity_weight_);
        const std::vector<cv::Point> raw_points = centerline;
        smooth_centerline(centerline, center_smooth_window_);

        int roi_height    = bottom_row - roi_start_row + 1;
        double la_ratio   = std::clamp(lookahead_ratio_, 0.0, 1.0);
        int lookahead_row = roi_start_row + static_cast<int>(la_ratio * roi_height);
        int band          = std::max(1, static_cast<int>(lookahead_band_ratio_ * roi_height));

        long sum_x = 0;
        int cnt    = 0;
        for (const auto& pt : centerline) {
            if (std::abs(pt.y - lookahead_row) <= band) {
                sum_x += pt.x;
                ++cnt;
            }
        }
        if (cnt == 0) {
            int best_d = std::numeric_limits<int>::max();
            for (const auto& pt : centerline) {
                int d = std::abs(pt.y - lookahead_row);
                if (d < best_d) {
                    best_d = d;
                    sum_x  = pt.x;
                    cnt    = 1;
                }
            }
        }

        double raw_err = 0.0;
        double err     = 0.0;
        bool valid     = cnt > 0;
        if (valid) {
            raw_err = (static_cast<double>(sum_x) / cnt - mid_col) / (frame.cols * 0.5);
            raw_err = std::clamp(raw_err, -1.0, 1.0);

            if (ema_init_) {
                err = ema_alpha_ * raw_err + (1.0 - ema_alpha_) * err_prev_;
            } else {
                err       = raw_err;
                ema_init_ = true;
            }
            err_prev_ = err;

            std_msgs::msg::Float64 theta;
            theta.data = raw_err;
            theta_pub_->publish(theta);
        }

        double vx    = 0.0;
        double omega = 0.0;
        if (valid) {
            omega = kp_angular_ * err;
            vx    = vx_max_ * (1.0 - std::fabs(err));
        }

        auto cmd      = geometry_msgs::msg::Twist();
        cmd.linear.x  = vx;
        cmd.angular.z = omega;
        cmd_pub_->publish(cmd);

        cv::Mat output = frame.clone();
        cv::line(
            output, cv::Point(mid_col, roi_start_row), cv::Point(mid_col, bottom_row),
            cv::Scalar(0, 255, 255), 1);
        cv::line(
            output, cv::Point(0, roi_start_row), cv::Point(frame.cols - 1, roi_start_row),
            cv::Scalar(0, 255, 255), 1);
        cv::line(
            output, cv::Point(0, lookahead_row - band),
            cv::Point(frame.cols - 1, lookahead_row - band), cv::Scalar(0, 255, 255), 2);
        cv::line(
            output, cv::Point(0, lookahead_row + band),
            cv::Point(frame.cols - 1, lookahead_row + band), cv::Scalar(0, 255, 255), 2);
        for (const auto& pt : raw_points) {
            cv::circle(output, pt, 1, cv::Scalar(0, 140, 255), -1);
        }
        if (centerline.size() >= 2) {
            cv::polylines(output, centerline, false, cv::Scalar(0, 255, 0), 3);
        }
        if (valid) {
            cv::Point lookahead_pt(static_cast<int>(sum_x / cnt), lookahead_row);
            cv::circle(output, lookahead_pt, 7, cv::Scalar(0, 0, 255), -1);
            cv::circle(output, lookahead_pt, 9, cv::Scalar(255, 255, 255), 2);
        } else {
            cv::putText(
                output, "NO LINE", cv::Point(10, roi_start_row + 30), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                cv::Scalar(0, 0, 255), 2);
        }

        auto image_msg = cv_bridge::CvImage(msg->header, "bgr8", output).toImageMsg();
        image_pub_->publish(*image_msg);

        cv_bridge::CvImage binary_msg(msg->header, "mono8", binary);
        binary_pub_->publish(*binary_msg.toImageMsg());
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr binary_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr theta_pub_;

    int threshold_;
    std::string threshold_mode_;
    std::string threshold_channel_;
    double width_continuity_weight_;
    int blur_ksize_;
    int morph_ksize_;
    double roi_bottom_ratio_;
    int max_gap_rows_;
    int center_smooth_window_;
    double lookahead_ratio_;
    double lookahead_band_ratio_;
    double ema_alpha_;
    double vx_max_;
    double kp_angular_;
    double err_prev_ = 0.0;
    bool ema_init_   = false;
};

} // namespace dogbot_core::vision

RCLCPP_COMPONENTS_REGISTER_NODE(dogbot_core::vision::FollowingNode)
