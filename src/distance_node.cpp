#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"

#include <cmath>
#include <chrono>

using std::placeholders::_1;

// ONLY FOR VERSION COLOR
static const std::string Y = "\033[33m";  // yellow
static const std::string R = "\033[0m";   // reset

class DistanceNode : public rclcpp::Node
{
public:
    DistanceNode() : Node("distance_node")
    {
        // VERSION (ONLY CHANGE)
        RCLCPP_INFO(this->get_logger(),
                    "%sdistance_node v4.3 loaded%s",
                    Y.c_str(), R.c_str());

        // Subscribers
        sub1_ = create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, std::bind(&DistanceNode::pose1Callback, this, _1));

        sub2_ = create_subscription<turtlesim::msg::Pose>(
            "/turtle2/pose", 10, std::bind(&DistanceNode::pose2Callback, this, _1));

        // Publishers
        pub1_ = create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        pub2_ = create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 10);

        // Freeze publisher
        freeze_pub_ = create_publisher<std_msgs::msg::Bool>("/freeze_turtles", 10);

        // Service clients
        pen1_client_ = create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");
        pen2_client_ = create_client<turtlesim::srv::SetPen>("/turtle2/set_pen");
        tp1_client_  = create_client<turtlesim::srv::TeleportAbsolute>("/turtle1/teleport_absolute");
        tp2_client_  = create_client<turtlesim::srv::TeleportAbsolute>("/turtle2/teleport_absolute");

        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&DistanceNode::update, this)
        );
    }

private:

    //------------------------------
    //  FREEZE CONTROL
    //------------------------------
    void setFreeze(bool state)
    {
        std_msgs::msg::Bool msg;
        msg.data = state;
        freeze_pub_->publish(msg);
    }

    //------------------------------
    //  UTILITIES
    //------------------------------
    void setPen(rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr client,
                int r, int g, int b, int width, bool off)
    {
        if (!client->wait_for_service(std::chrono::milliseconds(100)))
            return;

        auto req = std::make_shared<turtlesim::srv::SetPen::Request>();
        req->r = r;
        req->g = g;
        req->b = b;
        req->width = width;
        req->off = off ? 1 : 0;

        client->async_send_request(req);
    }

    void teleport(rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr client,
                  const turtlesim::msg::Pose &p)
    {
        if (!client->wait_for_service(std::chrono::milliseconds(100)))
            return;

        auto req = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
        req->x = p.x;
        req->y = p.y;
        req->theta = p.theta;

        client->async_send_request(req);
    }

    void stopTurtle(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub)
    {
        geometry_msgs::msg::Twist stop;
        stop.linear.x = 0;
        stop.angular.z = 0;
        pub->publish(stop);
    }

    bool isNearWall(const turtlesim::msg::Pose &p)
    {
        return (p.x < 1.5 || p.x > 9.5 || p.y < 1.5 || p.y > 9.5);
    }

    //------------------------------
    // POSE CALLBACKS
    //------------------------------
    void pose1Callback(const turtlesim::msg::Pose &p)
    {
        pose1_ = p;
        pose1_ready_ = true;
        if (!init1_) {
            last_stop1_ = p;
            init1_ = true;
            setPen(pen1_client_, 0, 0, 255, 3, false);
        }
    }

    void pose2Callback(const turtlesim::msg::Pose &p)
    {
        pose2_ = p;
        pose2_ready_ = true;
        if (!init2_) {
            last_stop2_ = p;
            init2_ = true;
            setPen(pen2_client_, 0, 0, 255, 3, false);
        }
    }

    //------------------------------
    // MAIN LOOP
    //------------------------------
    void update()
    {
        if (!pose1_ready_ || !pose2_ready_)
            return;

        // Save last stop
        if (pose1_.linear_velocity == 0 && pose1_.angular_velocity == 0)
            last_stop1_ = pose1_;
        if (pose2_.linear_velocity == 0 && pose2_.angular_velocity == 0)
            last_stop2_ = pose2_;

        //-----------------------------------------------------
        //  COLLISION TURTLE–TURTLE
        //-----------------------------------------------------
        float dx = pose1_.x - pose2_.x;
        float dy = pose1_.y - pose2_.y;
        if (std::sqrt(dx*dx + dy*dy) < 1.5)
        {
            RCLCPP_ERROR(this->get_logger(), "[COLLISION] turtles");

            setFreeze(true);
            stopTurtle(pub1_);
            stopTurtle(pub2_);

            setPen(pen1_client_, 255, 0, 0, 3, false);
            setPen(pen2_client_, 255, 0, 0, 3, false);
            rclcpp::sleep_for(std::chrono::milliseconds(400));

            setPen(pen1_client_, 0, 0, 0, 3, true);
            setPen(pen2_client_, 0, 0, 0, 3, true);

            teleport(tp1_client_, last_stop1_);
            teleport(tp2_client_, last_stop2_);

            stopTurtle(pub1_);
            stopTurtle(pub2_);

            setPen(pen1_client_, 0, 0, 255, 3, false);
            setPen(pen2_client_, 0, 0, 255, 3, false);

            stopTurtle(pub1_);
            stopTurtle(pub2_);

            setFreeze(false);
            return;
        }

        //-----------------------------------------------------
        //  WALL COLLISION TURTLE 1
        //-----------------------------------------------------
        if (isNearWall(pose1_))
        {
            RCLCPP_ERROR(this->get_logger(), "[COLLISION] turtle1 wall");

            setFreeze(true);
            stopTurtle(pub1_);

            setPen(pen1_client_, 255, 0, 0, 3, false);
            rclcpp::sleep_for(std::chrono::milliseconds(400));

            setPen(pen1_client_, 0, 0, 0, 3, true);
            teleport(tp1_client_, last_stop1_);

            stopTurtle(pub1_);
            setPen(pen1_client_, 0, 0, 255, 3, false);
            stopTurtle(pub1_);

            setFreeze(false);
            return;
        }

        //-----------------------------------------------------
        //  WALL COLLISION TURTLE 2
        //-----------------------------------------------------
        if (isNearWall(pose2_))
        {
            RCLCPP_ERROR(this->get_logger(), "[COLLISION] turtle2 wall");

            setFreeze(true);
            stopTurtle(pub2_);

            setPen(pen2_client_, 255, 0, 0, 3, false);
            rclcpp::sleep_for(std::chrono::milliseconds(400));

            setPen(pen2_client_, 0, 0, 0, 3, true);
            teleport(tp2_client_, last_stop2_);

            stopTurtle(pub2_);
            setPen(pen2_client_, 0, 0, 255, 3, false);
            stopTurtle(pub2_);

            setFreeze(false);
            return;
        }
    }

    //------------------------------
    // VARIABLES
    //------------------------------
    turtlesim::msg::Pose pose1_, pose2_;
    turtlesim::msg::Pose last_stop1_, last_stop2_;

    bool pose1_ready_ = false;
    bool pose2_ready_ = false;
    bool init1_ = false;
    bool init2_ = false;

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub1_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub2_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub1_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub2_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr freeze_pub_;

    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr pen1_client_;
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr pen2_client_;

    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr tp1_client_;
    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr tp2_client_;

    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DistanceNode>());
    rclcpp::shutdown();
    return 0;
}
