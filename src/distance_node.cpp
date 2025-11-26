#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cmath>

class DistanceNode : public rclcpp::Node
{
public:
    DistanceNode() : Node("distance_node")
    {
        sub1_ = create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, std::bind(&DistanceNode::pose1_callback, this, std::placeholders::_1));

        sub2_ = create_subscription<turtlesim::msg::Pose>(
            "/turtle2/pose", 10, std::bind(&DistanceNode::pose2_callback, this, std::placeholders::_1));

        dist_pub_ = create_publisher<std_msgs::msg::Float32>("/turtles_distance", 10);

        stop1_ = create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        stop2_ = create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 10);

       timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&DistanceNode::loop, this));

    }

private:
    turtlesim::msg::Pose pose1_, pose2_;
    bool got1_ = false, got2_ = false;

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub1_, sub2_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dist_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr stop1_, stop2_;
    rclcpp::TimerBase::SharedPtr timer_;

    void pose1_callback(const turtlesim::msg::Pose &msg) { pose1_ = msg; got1_ = true; }
    void pose2_callback(const turtlesim::msg::Pose &msg) { pose2_ = msg; got2_ = true; }

    void loop()
    {
        if(!got1_ || !got2_) return;

        float dist = std::hypot(pose1_.x - pose2_.x, pose1_.y - pose2_.y);
        std_msgs::msg::Float32 msg;
        msg.data = dist;
        dist_pub_->publish(msg);

        if (dist < 1.0) {
            RCLCPP_WARN(this->get_logger(), "Turtles too close! Stopping.");
            stop1_->publish(geometry_msgs::msg::Twist());
            stop2_->publish(geometry_msgs::msg::Twist());
        }

        check_boundary(pose1_, stop1_);
        check_boundary(pose2_, stop2_);
    }

    void check_boundary(const turtlesim::msg::Pose &p, 
                        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub)
    {
        if (p.x < 1.0 || p.x > 10.0 || p.y < 1.0 || p.y > 10.0)
        {
            RCLCPP_WARN(this->get_logger(), "Boundary reached! Stopping.");
            pub->publish(geometry_msgs::msg::Twist());
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DistanceNode>());
    rclcpp::shutdown();
    return 0;
}
