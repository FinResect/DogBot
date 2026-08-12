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
        roi_bottom_ratio_  = declare_parameter("roi_bottom_ratio", 0.6);
        lookahead_dist_    = declare_parameter("lookahead_distance", 100);
        smooth_window_size_ = declare_parameter("smooth_window_size", 7);
        interpolate_gaps_  = declare_parameter("interpolate_gaps", true);

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

        int num_rows = bottom_row - roi_start_row + 1;
        std::vector<double> x_by_row(num_rows, -1.0);
        for (const auto& pt : center_points) {
            int idx = bottom_row - pt.y;
            if (idx >= 0 && idx < num_rows) {
                x_by_row[idx] = static_cast<double>(pt.x);
            }
        }

        if (interpolate_gaps_) {
            int last_known = -1;
            for (int i = 0; i < num_rows; ++i) {
                if (x_by_row[i] >= 0) {
                    if (last_known >= 0 && i > last_known + 1) {
                        double x0 = x_by_row[last_known];
                        double x1 = x_by_row[i];
                        int gap = i - last_known;
                        for (int j = 1; j < gap; ++j) {
                            double t = static_cast<double>(j) / gap;
                            x_by_row[last_known + j] = x0 + (x1 - x0) * t;
                        }
                    }
                    last_known = i;
                }
            }
            int first = -1;
            for (int i = 0; i < num_rows; ++i) {
                if (x_by_row[i] >= 0) {
                    first = i;
                    break;
                }
            }
            int last = -1;
            for (int i = num_rows - 1; i >= 0; --i) {
                if (x_by_row[i] >= 0) {
                    last = i;
                    break;
                }
            }
            if (first > 0) {
                for (int i = 0; i < first; ++i) {
                    x_by_row[i] = x_by_row[first];
                }
            }
            if (last >= 0 && last < num_rows - 1) {
                for (int i = last + 1; i < num_rows; ++i) {
                    x_by_row[i] = x_by_row[last];
                }
            }
        }

        int half = smooth_window_size_ / 2;
        std::vector<double> smoothed(num_rows, -1.0);
        for (int i = 0; i < num_rows; ++i) {
            int start = std::max(0, i - half);
            int end = std::min(num_rows - 1, i + half);
            double sum = 0.0;
            int cnt = 0;
            for (int j = start; j <= end; ++j) {
                if (x_by_row[j] >= 0) {
                    sum += x_by_row[j];
                    ++cnt;
                }
            }
            if (cnt > 0) {
                smoothed[i] = sum / cnt;
            }
        }

        cv::Mat output;
        cv::cvtColor(edges, output, cv::COLOR_GRAY2BGR);

        for (const auto& pt : center_points) {
            cv::circle(output, pt, 2, cv::Scalar(255, 0, 0), -1);
        }

        std::vector<cv::Point> smoothed_pts;
        for (int i = 0; i < num_rows; ++i) {
            if (smoothed[i] >= 0) {
                smoothed_pts.emplace_back(
                    cv::Point(static_cast<int>(smoothed[i]), bottom_row - i));
            }
        }
        if (smoothed_pts.size() >= 2) {
            cv::polylines(output, smoothed_pts, false, cv::Scalar(0, 255, 0), 1);
        }

        double theta = 0.0;

        if (!smoothed_pts.empty() && smoothed[0] >= 0) {
            double bottom_x = smoothed[0];
            int lookahead_idx = std::min(lookahead_dist_, num_rows - 1);
            double ahead_x = smoothed[lookahead_idx];

            if (ahead_x >= 0 && lookahead_idx > 0) {
                theta = std::atan2(bottom_x - ahead_x,
                                   static_cast<double>(lookahead_idx));
            }

            cv::Point bottom_pt(static_cast<int>(bottom_x), bottom_row);
            cv::Point ref_end(bottom_pt.x, bottom_pt.y - lookahead_dist_);
            cv::line(output, bottom_pt, ref_end, cv::Scalar(0, 0, 255), 2);

            cv::Point ahead_pt(static_cast<int>(ahead_x >= 0 ? ahead_x : bottom_x),
                               bottom_row - lookahead_idx);
            cv::line(output, bottom_pt, ahead_pt, cv::Scalar(0, 255, 255), 2);
        }

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
    int smooth_window_size_;
    bool interpolate_gaps_;
};

} // namespace dogbot_core::vision

RCLCPP_COMPONENTS_REGISTER_NODE(dogbot_core::vision::FollowingNode)
