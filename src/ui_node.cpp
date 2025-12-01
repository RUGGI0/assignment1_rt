#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <iostream>
#include <chrono>

using namespace std::chrono_literals;

static const std::string Y = "\033[33m";   // Yellow
static const std::string R = "\033[0m";    // Reset

class UINode : public rclcpp::Node
{
public:
    UINode() : Node("ui_node")
    {
        // =============================
        // VERSION PRINT (ONLY ADDITION)
        // =============================
        RCLCPP_INFO(this->get_logger(), "%sui_node v3.2 loaded%s",
                    Y.c_str(), R.c_str());
        // =============================

        pub1_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        pub2_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 10);

        freeze_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/freeze_turtles", 10,
            std::bind(&UINode::freezeCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&UINode::loop, this));

        freeze_ = false;
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub1_, pub2_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr freeze_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool waiting_input_ = true;
    bool freeze_;
    geometry_msgs::msg::Twist cmd_;
    rclcpp::Time start_time_;
    int selected_turtle_ = 1;

    //---------------------------------------------------------
    // FREEZE
    //---------------------------------------------------------
    void freezeCallback(const std_msgs::msg::Bool &msg)
    {
        freeze_ = msg.data;

        if (freeze_)
        {
            publish_cmd(geometry_msgs::msg::Twist());
            waiting_input_ = true;
            std::cout << Y << "\n[UI] INPUT BLOCKED BY COLLISION\n" << R;
        }
        else
        {
            std::cout << Y << "\n[UI] INPUT UNBLOCKED\n" << R;
        }
    }

    //---------------------------------------------------------
    // MAIN LOOP
    //---------------------------------------------------------
    void loop()
    {
        if (freeze_)
        {
            publish_cmd(geometry_msgs::msg::Twist());
            return;
        }

        if (waiting_input_)
        {
            get_user_input();
            waiting_input_ = false;
            start_time_ = now();
        }
        else
        {
            if ((now() - start_time_) < rclcpp::Duration(1s)) {
                publish_cmd(cmd_);
            } else {
                publish_cmd(geometry_msgs::msg::Twist());
                waiting_input_ = true;
                std::cout << Y << "\nCommand completed. Ready for next.\n" << R;
            }
        }
    }

    //---------------------------------------------------------
    // PUBLISH
    //---------------------------------------------------------
    void publish_cmd(const geometry_msgs::msg::Twist &msg)
    {
        if (freeze_)
            return;

        if (selected_turtle_ == 1)
            pub1_->publish(msg);
        else
            pub2_->publish(msg);
    }

    //---------------------------------------------------------
    // USER INPUT (YELLOW)
    //---------------------------------------------------------
    void get_user_input()
    {
        std::cout << Y << "\nSelect turtle to control (1 or 2): " << R;
        std::cin >> selected_turtle_;

        if (selected_turtle_ != 1 && selected_turtle_ != 2) {
            selected_turtle_ = 1;
            std::cout << Y << "Invalid input, default = turtle1\n" << R;
        }

        std::cout << Y << "Linear velocity: " << R;
        std::cin >> cmd_.linear.x;

        std::cout << Y << "Angular velocity: " << R;
        std::cin >> cmd_.angular.z;

        std::cout << Y << "Sending command for 1 second...\n" << R;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UINode>());
    rclcpp::shutdown();
    return 0;
}
