#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace dogbot_core::vision {

namespace {
constexpr int kDx[8] = {0, 1, 1, 1, 0, -1, -1, -1};
constexpr int kDy[8] = {-1, -1, 0, 1, 1, 1, 0, -1};
} // namespace

class FollowingNode : public rclcpp::Node {
public:
    explicit FollowingNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
        : Node("following_node", opts) {
        threshold_        = declare_parameter("threshold", 120);
        blur_ksize_       = declare_parameter("gaussian_kernel_size", 5);
        roi_bottom_ratio_ = declare_parameter("roi_bottom_ratio", 0.6);
        max_trace_steps_  = declare_parameter("max_trace_steps", 2000);
        vx_max_           = declare_parameter("vx_max", 0.3);
        kp_angular_       = declare_parameter("kp_angular", 1.0);

        sub_ = create_subscription<sensor_msgs::msg::Image>(
            "image_raw", 10,
            std::bind(&FollowingNode::image_callback, this, std::placeholders::_1));

        image_pub_ = create_publisher<sensor_msgs::msg::Image>("/vision/following/image", 10);
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/vision/following/controller", 10);
    }

private:
    static std::vector<cv::Point>
        trace_line(const cv::Mat& binary, cv::Point seed, int roi_start_row, int max_steps) {
        std::vector<cv::Point> pts;
        pts.reserve(max_steps);
        pts.push_back(seed);

        cv::Point cur  = seed;
        cv::Point prev = cv::Point(-1, -1);
        bool have_prev = false;

        for (int step = 0; step < max_steps; ++step) {
            if (cur.y <= roi_start_row) {
                break;
            }

            bool found = false;
            for (int d = 0; d < 8; ++d) {
                cv::Point cand(cur.x + kDx[d], cur.y + kDy[d]);
                if (have_prev && cand == prev) {
                    continue;
                }
                if (cand.x < 0 || cand.x >= binary.cols || cand.y < 0 || cand.y >= binary.rows) {
                    continue;
                }
                if (binary.at<uchar>(cand) > 0) {
                    prev      = cur;
                    cur       = cand;
                    have_prev = true;
                    pts.push_back(cur);
                    found = true;
                    break;
                }
            }
            if (!found) {
                break;
            }
        }
        return pts;
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        auto cv_ptr   = cv_bridge::toCvCopy(msg, "bgr8");
        cv::Mat frame = cv_ptr->image;

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        int ksize = blur_ksize_;
        if (ksize % 2 == 0) {
            ++ksize;
        }
        if (ksize > 0) {
            cv::GaussianBlur(gray, gray, cv::Size(ksize, ksize), 0);
        }

        cv::Mat binary;
        cv::threshold(gray, binary, threshold_, 255, cv::THRESH_BINARY);

        int bottom_row    = frame.rows - 1;
        int roi_start_row = static_cast<int>(frame.rows * (1.0 - roi_bottom_ratio_));
        int mid_col       = frame.cols / 2;

        cv::Point seed(-1, -1);
        for (int col = mid_col; col >= 0; --col) {
            if (binary.at<uchar>(bottom_row, col) > 0) {
                seed = cv::Point(col, bottom_row);
                break;
            }
        }
        if (seed.x < 0) {
            for (int col = mid_col + 1; col < frame.cols; ++col) {
                if (binary.at<uchar>(bottom_row, col) > 0) {
                    seed = cv::Point(col, bottom_row);
                    break;
                }
            }
        }

        std::vector<cv::Point> chain;
        if (seed.x >= 0) {
            chain = trace_line(binary, seed, roi_start_row, max_trace_steps_);
        }

        int num_rows = bottom_row - roi_start_row + 1;
        std::vector<double> row_sum(num_rows, 0.0);
        std::vector<int> row_cnt(num_rows, 0);
        for (const auto& pt : chain) {
            int idx = bottom_row - pt.y;
            if (idx >= 0 && idx < num_rows) {
                row_sum[idx] += pt.x;
                ++row_cnt[idx];
            }
        }

        std::vector<cv::Point> centerline;
        for (int i = 0; i < num_rows; ++i) {
            if (row_cnt[i] > 0) {
                centerline.emplace_back(static_cast<int>(row_sum[i] / row_cnt[i]), bottom_row - i);
            }
        }

        double vx    = 0.0;
        double omega = 0.0;
        if (!centerline.empty()) {
            double err = (centerline.front().x - mid_col) / (frame.cols * 0.5);
            err        = std::clamp(err, -1.0, 1.0);
            omega      = kp_angular_ * err;
            vx         = vx_max_ * (1.0 - std::fabs(err));
        }

        auto cmd      = geometry_msgs::msg::Twist();
        cmd.linear.x  = vx;
        cmd.angular.z = omega;
        cmd_pub_->publish(cmd);

        cv::Mat output = frame.clone();
        cv::line(
            output, cv::Point(mid_col, roi_start_row), cv::Point(mid_col, bottom_row),
            cv::Scalar(200, 200, 200), 1);
        if (seed.x >= 0) {
            cv::circle(output, seed, 4, cv::Scalar(0, 0, 255), -1);
        }
        if (chain.size() >= 2) {
            cv::polylines(output, chain, false, cv::Scalar(255, 0, 0), 1);
        }
        if (centerline.size() >= 2) {
            cv::polylines(output, centerline, false, cv::Scalar(0, 255, 0), 2);
        }

        auto image_msg = cv_bridge::CvImage(msg->header, "bgr8", output).toImageMsg();
        image_pub_->publish(*image_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    int threshold_;
    int blur_ksize_;
    double roi_bottom_ratio_;
    int max_trace_steps_;
    double vx_max_;
    double kp_angular_;
};

} // namespace dogbot_core::vision

RCLCPP_COMPONENTS_REGISTER_NODE(dogbot_core::vision::FollowingNode)
