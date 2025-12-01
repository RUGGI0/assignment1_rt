#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"

#include <cmath>
#include <chrono>
#include <memory>
#include <string>
#include <functional>

// -------------------------
// Standard Library aliases
// -------------------------
using std::string;
using std::bind;
using std::placeholders::_1;
using std::sqrt;
using std::chrono::milliseconds;


// -------------------------
// ROS2 Type Aliases
// -------------------------
using rclcpp::Node;
using rclcpp::sleep_for;
using rclcpp::init;
using rclcpp::spin;
using rclcpp::shutdown;

// -------------------------
// Generic Aliases
// -------------------------
template<typename T>
using Sub = rclcpp::Subscription<T>;

template<typename T>
using Pub = rclcpp::Publisher<T>;

using Timer = rclcpp::TimerBase;

using Pose           = turtlesim::msg::Pose;
using Twist          = geometry_msgs::msg::Twist;
using BoolMsg        = std_msgs::msg::Bool;

using SetPenSrv      = turtlesim::srv::SetPen;
using TeleportSrv    = turtlesim::srv::TeleportAbsolute;

using ClientSetPen   = rclcpp::Client<SetPenSrv>;
using ClientTeleport = rclcpp::Client<TeleportSrv>;

using SetPenReq      = SetPenSrv::Request;
using TeleportReq    = TeleportSrv::Request;

// -------------------------
// ANSI Colors
// -------------------------
static const string Y = "\033[33m";
static const string R = "\033[0m";


// =====================================================
// CLASS NODE
// =====================================================
class DistanceNode : public Node
{
public:
    DistanceNode() : Node("distance_node")
    {
        RCLCPP_INFO(
            this->get_logger(),
            "%sdistance_node v4.3 loaded%s",
            Y.c_str(), R.c_str()
        );

        // -------------------------
        // Subscribers
        // -------------------------
        sub1_ = create_subscription<Pose>(
            "/turtle1/pose", 10,
            bind(&DistanceNode::pose1Callback, this, _1)
        );

        sub2_ = create_subscription<Pose>(
            "/turtle2/pose", 10,
            bind(&DistanceNode::pose2Callback, this, _1)
        );

        // -------------------------
        // Publishers
        // -------------------------
        pub1_ = create_publisher<Twist>("/turtle1/cmd_vel", 10);
        pub2_ = create_publisher<Twist>("/turtle2/cmd_vel", 10);

        freeze_pub_ = create_publisher<BoolMsg>("/freeze_turtles", 10);

        // -------------------------
        // Service Clients
        // -------------------------
        pen1_client_ = create_client<SetPenSrv>("/turtle1/set_pen");
        pen2_client_ = create_client<SetPenSrv>("/turtle2/set_pen");
        tp1_client_  = create_client<TeleportSrv>("/turtle1/teleport_absolute");
        tp2_client_  = create_client<TeleportSrv>("/turtle2/teleport_absolute");

        // -------------------------
        // Timer
        // -------------------------
        timer_ = create_wall_timer(
            milliseconds(50),
            bind(&DistanceNode::update, this)
        );
    }

private:

    // =====================================================
    // FREEZE CONTROL
    // =====================================================
    void setFreeze(bool state)
    {
        BoolMsg msg;
        msg.data = state;
        freeze_pub_->publish(msg);
    }


    // =====================================================
    // UTILITIES
    // =====================================================
    void setPen(const ClientSetPen::SharedPtr &client,
                int r, int g, int b,
                int width, bool off)
    {
        if (!client->wait_for_service(milliseconds(100)))
            return;

        auto req = std::make_shared<SetPenReq>();
        req->r = r;
        req->g = g;
        req->b = b;
        req->width = width;
        req->off = off ? 1 : 0;

        client->async_send_request(req);
    }

    void teleport(const ClientTeleport::SharedPtr &client,
                  const Pose &p)
    {
        if (!client->wait_for_service(milliseconds(100)))
            return;

        auto req = std::make_shared<TeleportReq>();
        req->x = p.x;
        req->y = p.y;
        req->theta = p.theta;

        client->async_send_request(req);
    }

    void stopTurtle(const Pub<Twist>::SharedPtr &pub)
    {
        Twist msg;
        msg.linear.x = 0;
        msg.angular.z = 0;
        pub->publish(msg);
    }

    bool isNearWall(const Pose &p)
    {
        return (p.x < 1.5 || p.x > 9.5 ||
                p.y < 1.5 || p.y > 9.5);
    }


    // =====================================================
    // CALLBACKS
    // =====================================================
    void pose1Callback(const Pose &p)
    {
        pose1_ = p;
        pose1_ready_ = true;

        if (!init1_) {
            last_stop1_ = p;
            init1_ = true;
            setPen(pen1_client_, 0, 0, 255, 3, false);
        }
    }

    void pose2Callback(const Pose &p)
    {
        pose2_ = p;
        pose2_ready_ = true;

        if (!init2_) {
            last_stop2_ = p;
            init2_ = true;
            setPen(pen2_client_, 0, 0, 255, 3, false);
        }
    }


    // =====================================================
    // MAIN LOOP
    // =====================================================
    void update()
    {
        if (!pose1_ready_ || !pose2_ready_)
            return;

        // Save stop positions
        if (pose1_.linear_velocity == 0 && pose1_.angular_velocity == 0)
            last_stop1_ = pose1_;

        if (pose2_.linear_velocity == 0 && pose2_.angular_velocity == 0)
            last_stop2_ = pose2_;


        // -------------------------
        // Turtle–turtle collision
        // -------------------------
        float dx = pose1_.x - pose2_.x;
        float dy = pose1_.y - pose2_.y;

        if (sqrt(dx * dx + dy * dy) < 1.5)
        {
            RCLCPP_ERROR(this->get_logger(), "[COLLISION] turtles");

            setFreeze(true);
            stopTurtle(pub1_);
            stopTurtle(pub2_);

            setPen(pen1_client_, 255, 0, 0, 3, false);
            setPen(pen2_client_, 255, 0, 0, 3, false);

            sleep_for(milliseconds(400));

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


        // -------------------------
        // Wall collision (turtle1)
        // -------------------------
        if (isNearWall(pose1_))
        {
            RCLCPP_ERROR(this->get_logger(), "[COLLISION] turtle1 wall");

            setFreeze(true);
            stopTurtle(pub1_);

            setPen(pen1_client_, 255, 0, 0, 3, false);
            sleep_for(milliseconds(400));

            setPen(pen1_client_, 0, 0, 0, 3, true);
            teleport(tp1_client_, last_stop1_);

            stopTurtle(pub1_);
            setPen(pen1_client_, 0, 0, 255, 3, false);
            stopTurtle(pub1_);

            setFreeze(false);
            return;
        }


        // -------------------------
        // Wall collision (turtle2)
        // -------------------------
        if (isNearWall(pose2_))
        {
            RCLCPP_ERROR(this->get_logger(), "[COLLISION] turtle2 wall");

            setFreeze(true);
            stopTurtle(pub2_);

            setPen(pen2_client_, 255, 0, 0, 3, false);
            sleep_for(milliseconds(400));

            setPen(pen2_client_, 0, 0, 0, 3, true);
            teleport(tp2_client_, last_stop2_);

            stopTurtle(pub2_);
            setPen(pen2_client_, 0, 0, 255, 3, false);
            stopTurtle(pub2_);

            setFreeze(false);
            return;
        }
    }

    // =====================================================
    // MEMBER VARIABLES
    // =====================================================
    Pose pose1_, pose2_;
    Pose last_stop1_, last_stop2_;
    
    bool pose1_ready_ = false;
    bool pose2_ready_ = false;
    bool init1_ = false;
    bool init2_ = false;
    
    // Subscribers
    Sub<Pose>::SharedPtr sub1_;
    Sub<Pose>::SharedPtr sub2_;
    
    // Publishers
    Pub<Twist>::SharedPtr    pub1_;
    Pub<Twist>::SharedPtr    pub2_;
    Pub<BoolMsg>::SharedPtr  freeze_pub_;
    
    // Service clients
    ClientSetPen::SharedPtr    pen1_client_;
    ClientSetPen::SharedPtr    pen2_client_;
    ClientTeleport::SharedPtr  tp1_client_;
    ClientTeleport::SharedPtr  tp2_client_;
    
    // Timer
    Timer::SharedPtr timer_;
};

// =====================================================
// MAIN
// =====================================================
int main(int argc, char **argv)
{
    init(argc, argv);
    spin(std::make_shared<DistanceNode>());
    shutdown();
    return 0;
}
