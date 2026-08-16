#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int64.hpp>

#include <chrono>
#include <functional>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace dogbot_core::vision {

class ColorListenerNode : public rclcpp::Node {
public:
    explicit ColorListenerNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
        : Node("color_listener_node", opts) {
        for (const auto& name : kColors) {
            subscriptions_.push_back(create_subscription<std_msgs::msg::Bool>(
                "/vision/is_" + name, 10,
                [this, name](const std_msgs::msg::Bool::SharedPtr msg) {
                    colors_[name] = msg->data;
                }));
        }

        using namespace std::chrono_literals;
        timer_ = create_wall_timer(1ms, [this]() { update(); turn_update(); });

        turn_omega_pub_ = create_publisher<std_msgs::msg::Float64>("/vision/color/turn_omega", 10);
    }

    bool isDetected(const std::string& name) const {
        auto it = colors_.find(name);
        return it != colors_.end() && it->second;
    }

private:
    void update() {
        turn_update();

    }

    void turn_update() {
        bool condition = isDetected("green");
        if (condition && !prev_turn_condition_) {
            ++turn_count_;
            turn_active_ticks_ = 2000;
        }
        auto msg = std_msgs::msg::Float64();
        msg.data = 0.0;
        if (turn_active_ticks_ > 0) {
            if (turn_count_ == 1) {
                msg.data = kTurnOmega;
            } else if (turn_count_ == 2) {
                msg.data = -kTurnOmega;
            }
            --turn_active_ticks_;
        }
        turn_omega_pub_->publish(msg);
        prev_turn_condition_ = condition;
    }

    static const inline std::vector<std::string> kColors = {
        "red", "blue", "green", "brown", "purple", "orange"};
    static constexpr double kTurnOmega = 0.5;

    std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> subscriptions_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr turn_omega_pub_;
    std::map<std::string, bool> colors_;
    int64_t turn_count_ = 0;
    int turn_active_ticks_ = 0;
    bool prev_turn_condition_ = false;
};

} // namespace dogbot_core::vision

RCLCPP_COMPONENTS_REGISTER_NODE(dogbot_core::vision::ColorListenerNode)
