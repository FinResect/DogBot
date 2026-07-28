#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64.hpp>

#include <limits>
#include <string>
#include <vector>

namespace dogbot_core::vision {

class FollowingNode : public rclcpp::Node {
public:
    explicit FollowingNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
        : Node("following_node", opts) {
        canny_low_        = declare_parameter("canny_low_threshold", 50);
        canny_high_       = declare_parameter("canny_high_threshold", 150);
        blur_ksize_       = declare_parameter("gaussian_kernel_size", 5);
        roi_bottom_ratio_ = declare_parameter("roi_bottom_ratio", 0.6);
        lookahead_dist_   = declare_parameter("lookahead_distance", 100);

        int ksize = blur_ksize_;
        if (ksize % 2 == 0) {
            ++ksize;
        }

        sub_ = create_subscription<sensor_msgs::msg::Image>(
            "image_raw", 10,
            std::bind(&FollowingNode::image_callback, this, std::placeholders::_1));

        image_pub_ = create_publisher<sensor_msgs::msg::Image>("/vision/following/image", 10);

        theta_pub_ = create_publisher<std_msgs::msg::Float64>("/vision/following/theta", 10);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        auto cv_ptr   = cv_bridge::toCvCopy(msg, "bgr8");
        cv::Mat frame = cv_ptr->image;

        cv::Mat gray, blurred, edges;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        int ksize = blur_ksize_;
        if (ksize % 2 == 0) {
            ++ksize;
        }
        cv::GaussianBlur(gray, blurred, cv::Size(ksize, ksize), 0);

        cv::Canny(blurred, edges, canny_low_, canny_high_);

        int bottom_row    = frame.rows - 1;
        int roi_start_row = static_cast<int>(frame.rows * (1.0 - roi_bottom_ratio_));
        int mid_col       = frame.cols / 2;

        std::vector<cv::Point> center_points;

        for (int row = bottom_row; row >= roi_start_row; --row) {
            int left_edge = -1;
            for (int col = mid_col; col >= 0; --col) {
                if (edges.at<uchar>(row, col) > 0) {
                    left_edge = col;
                    break;
                }
            }

            int right_edge = -1;
            for (int col = mid_col; col < frame.cols; ++col) {
                if (edges.at<uchar>(row, col) > 0) {
                    right_edge = col;
                    break;
                }
            }

            if (left_edge >= 0 && right_edge >= 0) {
                center_points.emplace_back((left_edge + right_edge) / 2, row);
            }
        }

        cv::Mat output;
        cv::cvtColor(edges, output, cv::COLOR_GRAY2BGR);

        for (const auto& pt : center_points) {
            cv::circle(output, pt, 2, cv::Scalar(255, 0, 0), -1);
        }

        double theta = 0.0;

        if (center_points.size() >= 2) {
            const auto& bottom_midpoint = center_points.front();

            cv::Point ahead_pt = bottom_midpoint;
            int target_row     = bottom_midpoint.y - lookahead_dist_;
            if (target_row < roi_start_row) {
                target_row = roi_start_row;
            }
            int min_diff = std::numeric_limits<int>::max();
            for (const auto& pt : center_points) {
                int diff = std::abs(pt.y - target_row);
                if (diff < min_diff) {
                    min_diff = diff;
                    ahead_pt = pt;
                }
            }

            int dy = bottom_midpoint.y - ahead_pt.y;
            if (dy > 0) {
                theta = std::atan2(
                    static_cast<double>(bottom_midpoint.x - ahead_pt.x), static_cast<double>(dy));
            }

            cv::Point ref_end(bottom_midpoint.x, bottom_midpoint.y - lookahead_dist_);
            cv::line(output, bottom_midpoint, ref_end, cv::Scalar(0, 0, 255), 2);

            cv::line(output, bottom_midpoint, ahead_pt, cv::Scalar(0, 255, 0), 2);
        }

        RCLCPP_INFO(this->get_logger(), "theta:%f", theta);
        auto theta_msg = std_msgs::msg::Float64();
        theta_msg.data = theta;
        theta_pub_->publish(theta_msg);

        auto image_msg = cv_bridge::CvImage(msg->header, "bgr8", output).toImageMsg();
        image_pub_->publish(*image_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr theta_pub_;

    int canny_low_;
    int canny_high_;
    int blur_ksize_;
    double roi_bottom_ratio_;
    int lookahead_dist_;
};

} // namespace dogbot_core::vision

RCLCPP_COMPONENTS_REGISTER_NODE(dogbot_core::vision::FollowingNode)
