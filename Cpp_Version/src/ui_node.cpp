
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>

#include <iostream>
#include <string>
#include <chrono>

// =====================================================
// STANDARD LIBRARY ALIASES
// =====================================================
using std::string;
using std::cin;
using std::cout;
using std::chrono::milliseconds;

// =====================================================
// ROS2 TYPE ALIASES
// =====================================================
using rclcpp::Node;
using rclcpp::Duration;
using rclcpp::Publisher;
using rclcpp::Subscription;
using rclcpp::TimerBase;
using rclcpp::init;
using rclcpp::spin;
using rclcpp::shutdown;

using Twist   = geometry_msgs::msg::Twist;
using BoolMsg = std_msgs::msg::Bool;

template<typename T>
using Pub = Publisher<T>;

template<typename T>
using Sub = Subscription<T>;

using Timer = TimerBase;


// =====================================================
// ANSI COLORS
// =====================================================
static const string Y = "\033[33m";
static const string R = "\033[0m";


// =====================================================
// UI NODE
// =====================================================
class UINode : public Node
{
public:
    UINode() : Node("ui_node")
    {
        RCLCPP_INFO(
            this->get_logger(),
            "%sui_node v3.3 loaded%s",
            Y.c_str(), R.c_str()
        );

        pub1_ = create_publisher<Twist>("/turtle1/cmd_vel", 10);
        pub2_ = create_publisher<Twist>("/turtle2/cmd_vel", 10);

        freeze_sub_ = create_subscription<BoolMsg>(
            "/freeze_turtles", 10,
            [this](const BoolMsg &msg) { onFreeze(msg); }
        );

        timer_ = create_wall_timer(
            milliseconds(100),
            [this]() { loop(); }
        );
    }


private:

    // =====================================================
    // FREEZE CALLBACK
    // =====================================================
    void onFreeze(const BoolMsg &msg)
    {
        freeze_ = msg.data;

        if (freeze_)
        {
            publish(Twist{});
            waiting_input_ = true;
            cout << Y << "\n[UI] INPUT BLOCKED BY COLLISION\n" << R;
        }
        else
        {
            cout << Y << "\n[UI] INPUT UNBLOCKED\n" << R;
        }
    }


    // =====================================================
    // MAIN LOOP
    // =====================================================
    void loop()
    {
        if (freeze_)
        {
            publish(Twist{});
            return;
        }

        if (waiting_input_)
        {
            getUserInput();
            waiting_input_ = false;
            start_time_ = now();
        }
        else
        {
            if ((now() - start_time_) < Duration::from_seconds(1))
            {
                publish(cmd_);
            }
            else
            {
                publish(Twist{});
                waiting_input_ = true;
                cout << Y << "\nCommand completed. Ready for next.\n" << R;
            }
        }
    }


    // =====================================================
    // PUBLISH
    // =====================================================
    void publish(const Twist &msg)
    {
        if (selected_turtle_ == 1)
            pub1_->publish(msg);
        else
            pub2_->publish(msg);
    }


    // =====================================================
    // USER INPUT
    // =====================================================
    void getUserInput()
    {
        cout << Y << "\nSelect turtle to control (1 or 2): " << R;
        cin >> selected_turtle_;

        if (selected_turtle_ != 1 && selected_turtle_ != 2)
        {
            selected_turtle_ = 1;
            cout << Y << "Invalid input, default = turtle1\n" << R;
        }

        cout << Y << "Linear velocity: " << R;
        cin >> cmd_.linear.x;

        cout << Y << "Angular velocity: " << R;
        cin >> cmd_.angular.z;

        cout << Y << "Sending command for 1 second...\n" << R;
    }


    // =====================================================
    // MEMBER VARIABLES
    // =====================================================
    Pub<Twist>::SharedPtr    pub1_;
    Pub<Twist>::SharedPtr    pub2_;
    Sub<BoolMsg>::SharedPtr  freeze_sub_;
    Timer::SharedPtr         timer_;

    bool waiting_input_ = true;
    bool freeze_ = false;

    Twist cmd_;
    rclcpp::Time start_time_;

    int selected_turtle_ = 1;
};


// =====================================================
// MAIN
// =====================================================
int main(int argc, char **argv)
{
    init(argc, argv);
    spin(std::make_shared<UINode>());
    shutdown();
    return 0;
}
