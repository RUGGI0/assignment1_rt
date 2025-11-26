#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <chrono>

using namespace std::chrono_literals;

class UINode : public rclcpp::Node
{
public:
    UINode() : Node("ui_node")
    {
        pub1_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        pub2_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&UINode::loop, this));

    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub1_, pub2_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool waiting_input_ = true;
    geometry_msgs::msg::Twist cmd_;
    rclcpp::Time start_time_;
    int selected_turtle_ = 1;

    void loop()
    {
        if(waiting_input_)
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
                std::cout << "\nCommand completed. Ready for next.\n";
            }
        }
    }

    void publish_cmd(const geometry_msgs::msg::Twist &msg)
    {
        if (selected_turtle_ == 1)
            pub1_->publish(msg);
        else
            pub2_->publish(msg);
    }

    void get_user_input()
    {
        std::cout << "\nSelect turtle to control (1 or 2): ";
        std::cin >> selected_turtle_;
        if (selected_turtle_ != 1 && selected_turtle_ != 2) {
            selected_turtle_ = 1;
            std::cout << "Invalid input, default = turtle1\n";
        }

        std::cout << "Linear velocity: ";
        std::cin >> cmd_.linear.x;

        std::cout << "Angular velocity: ";
        std::cin >> cmd_.angular.z;

        std::cout << "Sending command for 1 second...\n";
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UINode>());
    rclcpp::shutdown();
    return 0;
}
