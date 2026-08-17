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

#include "controller/tick/tick_timer.hpp"

namespace dogbot_core::vision {

class ColorListenerNode : public rclcpp::Node {
public:
    explicit ColorListenerNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
        : Node("color_listener_node", opts) {
        for (const auto& name : kColors) {
            subscriptions_.push_back(create_subscription<std_msgs::msg::Bool>(
                "/vision/is_" + name, 10, [this, name](const std_msgs::msg::Bool::SharedPtr msg) {
                    colors_[name] = msg->data;
                }));
        }

        using namespace std::chrono_literals;
        timer_ = this->create_wall_timer(1ms, std::bind(&ColorListenerNode::update, this));

        turn_omega_pub_ = create_publisher<std_msgs::msg::Float64>("/vision/color/turn_omega", 10);

        thrower_pub_ =
            create_publisher<std_msgs::msg::Int64>("/vision/color/thrower_controller", 10);

        place_pub_ = create_publisher<std_msgs::msg::Bool>("/vision/color/place_controller", 10);

        climb_pub_ = create_publisher<std_msgs::msg::Bool>("/vision/color/climb_controller", 10);

        turn_watchdog.reset(0);
        turn_omega_delay_watchdog.reset(0);
        thrower_watchdog.reset(0);
        thrower_detect_delay_watchdog.reset(0);
        place_watchdog.reset(0);
        place_detect_delay_watchdog.reset(0);
        climb_watchdog.reset(0);
        climb_detect_delay_watchdog.reset(0);
    }

    bool isDetected(const std::string& name) const {
        auto it = colors_.find(name);
        return it != colors_.end() && it->second;
    }

private:
    void update() {
        task_turn_update();
        task_thrower_update();
        task_place_update();
        task_climb_update();
    }

    void task_turn_update() {
        static auto msg = std_msgs::msg::Float64();
        static bool is_first{true};
        static bool is_detected{false};

        if (isDetected("green")) {
            turn_watchdog.reset(200);                  // time from 'color disappear' to 'turn'
            is_detected = true;
        }

        if (is_detected) {

            static bool is_omega_pub{false};

            if (turn_watchdog.tick()) {
                if (is_first) {
                    msg.data = -kTurnOmega;
                } else {
                    msg.data = kTurnOmega;
                }

                turn_omega_pub_->publish(msg);
                is_omega_pub = true;
                turn_omega_delay_watchdog.reset(200);  // turn delay time
                return;
            }
            if (is_omega_pub) {
                if (turn_omega_delay_watchdog.tick()) {
                    is_detected  = false;
                    is_omega_pub = false;
                    msg.data     = 0.0;
                }
            }
        }

        turn_omega_pub_->publish(msg);
    }

    void task_thrower_update() {
        static auto msg = std_msgs::msg::Int64();
        static bool is_detected{false};

        static bool if_detect{true};

        if (is_detected) {
            if (thrower_watchdog.tick()) {
                msg.data    = 0;
                is_detected = false;
            }
            thrower_detect_delay_watchdog.reset(2000); // ignore color time
            return;
        }

        if (thrower_detect_delay_watchdog.tick()) {
            if_detect = true;
            msg.data  = 0;
        }

        if (if_detect) {
            if (isDetected("brown")) {
                msg.data    = 1;
                is_detected = true;
                if_detect   = false;
            } else if (isDetected("purple")) {
                msg.data    = 2;
                is_detected = true;
                if_detect   = false;
            } else {
                msg.data = 0;
            }
        }

        thrower_watchdog.reset(1000);                  // execution time

        thrower_pub_->publish(msg);
    }

    void task_place_update() {
        static auto msg = std_msgs::msg::Bool();
        static bool is_detected{false};

        static bool if_detect{true};

        if (is_detected) {
            if (place_watchdog.tick()) {
                msg.data = false;

                is_detected = false;
            }
            place_detect_delay_watchdog.reset(2000);   // ignore color time
            return;
        }

        if (place_detect_delay_watchdog.tick()) {
            if_detect = false;
            msg.data  = false;
        }

        if (if_detect) {
            if (isDetected("orange")) {
                msg.data = true;

                is_detected = true;
                if_detect   = false;
            } else {
                msg.data = 0;
            }
        }

        place_watchdog.reset(200);                     // execution time

        place_pub_->publish(msg);
    }

    void task_climb_update() {
        static auto msg = std_msgs::msg::Bool();
        static bool is_detected{false};

        static bool if_detect{true};

        if (is_detected) {
            if (climb_watchdog.tick()) {
                msg.data = false;

                is_detected = false;
            }
            climb_detect_delay_watchdog.reset(2000);   // ignore color time
            return;
        }

        if (climb_detect_delay_watchdog.tick()) {
            if_detect = true;
        }

        if (if_detect) {
            if (isDetected("blue")) {
                msg.data = true;

                is_detected = true;
                if_detect   = false;
            } else {
                msg.data = 0;
            }
        }

        climb_watchdog.reset(200);                     // execution time

        climb_pub_->publish(msg);
    }

    static const inline std::vector<std::string> kColors = {"red",   "blue",   "green",
                                                            "brown", "purple", "orange"};
    static constexpr double kTurnOmega                   = 5.0;

    controller::tick::TickTimer turn_watchdog;
    controller::tick::TickTimer turn_omega_delay_watchdog;
    controller::tick::TickTimer thrower_watchdog;
    controller::tick::TickTimer thrower_detect_delay_watchdog;
    controller::tick::TickTimer place_watchdog;
    controller::tick::TickTimer place_detect_delay_watchdog;
    controller::tick::TickTimer climb_watchdog;
    controller::tick::TickTimer climb_detect_delay_watchdog;

    std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> subscriptions_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr turn_omega_pub_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr thrower_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr place_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr climb_pub_;

    std::map<std::string, bool> colors_;
};

} // namespace dogbot_core::vision

RCLCPP_COMPONENTS_REGISTER_NODE(dogbot_core::vision::ColorListenerNode)
