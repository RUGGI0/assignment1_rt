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
// CONFIG
// -------------------------
static const float PRECOLLISION_DISTANCE = 1.5f;
static const float COLLISION_DISTANCE   = 0.5f;

static const float WALL_MIN = 0.5f;
static const float WALL_MAX = 10.5f;

static const int COLLISION_SLEEP_MS = 400;

// -------------------------
using std::string;
using std::bind;
using std::placeholders::_1;
using std::sqrt;
using std::chrono::milliseconds;

using rclcpp::Node;
using rclcpp::sleep_for;
using rclcpp::init;
using rclcpp::spin;
using rclcpp::shutdown;

template<typename T> using Sub = rclcpp::Subscription<T>;
template<typename T> using Pub = rclcpp::Publisher<T>;
using Timer = rclcpp::TimerBase;

using Pose = turtlesim::msg::Pose;
using Twist = geometry_msgs::msg::Twist;
using BoolMsg = std_msgs::msg::Bool;

using SetPenSrv = turtlesim::srv::SetPen;
using TeleportSrv = turtlesim::srv::TeleportAbsolute;

using ClientSetPen = rclcpp::Client<SetPenSrv>;
using ClientTeleport = rclcpp::Client<TeleportSrv>;

using SetPenReq = SetPenSrv::Request;
using TeleportReq = TeleportSrv::Request;

// ANSI Colors
static const string Y = "\033[33m";
static const string R = "\033[0m";


// =====================================================
// NODE
// =====================================================
class DistanceNode : public Node
{
public:
    DistanceNode() : Node("distance_node")
    {
        RCLCPP_INFO(
            this->get_logger(),
            "%sdistance_node v5.1 loaded%s",
            Y.c_str(), R.c_str()
        );

        sub1_ = create_subscription<Pose>(
            "/turtle1/pose", 10,
            bind(&DistanceNode::pose1Callback, this, _1));

        sub2_ = create_subscription<Pose>(
            "/turtle2/pose", 10,
            bind(&DistanceNode::pose2Callback, this, _1));

        pub1_ = create_publisher<Twist>("/turtle1/cmd_vel", 10);
        pub2_ = create_publisher<Twist>("/turtle2/cmd_vel", 10);
        freeze_pub_ = create_publisher<BoolMsg>("/freeze_turtles", 10);

        pen1_client_ = create_client<SetPenSrv>("/turtle1/set_pen");
        pen2_client_ = create_client<SetPenSrv>("/turtle2/set_pen");

        tp1_client_ = create_client<TeleportSrv>("/turtle1/teleport_absolute");
        tp2_client_ = create_client<TeleportSrv>("/turtle2/teleport_absolute");

        timer_ = create_wall_timer(
            milliseconds(50),
            bind(&DistanceNode::update, this));
    }

private:

    void setFreeze(bool s)
    {
        BoolMsg msg;
        msg.data = s;
        freeze_pub_->publish(msg);
    }

    void setPen(const ClientSetPen::SharedPtr &client,
                int r, int g, int b, int w, bool off)
    {
        if (!client->wait_for_service(milliseconds(100)))
            return;

        auto req = std::make_shared<SetPenReq>();
        req->r = r; req->g = g; req->b = b;
        req->width = w;
        req->off = off ? 1 : 0;

        client->async_send_request(req);
    }

    void teleport(const ClientTeleport::SharedPtr &client, const Pose &p)
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
        msg.linear.x  = 0;
        msg.angular.z = 0;
        pub->publish(msg);
    }

    bool isNearWall(const Pose &p)
    {
        return (p.x < WALL_MIN || p.x > WALL_MAX ||
                p.y < WALL_MIN || p.y > WALL_MAX);
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

        if (p.linear_velocity == 0 && p.angular_velocity == 0)
            last_stop1_ = p;
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

        if (p.linear_velocity == 0 && p.angular_velocity == 0)
            last_stop2_ = p;
    }

    // =====================================================
    // MAIN LOOP
    // =====================================================
    void update()
    {
        if (!pose1_ready_ || !pose2_ready_)
            return;

        float dx   = pose1_.x - pose2_.x;
        float dy   = pose1_.y - pose2_.y;
        float dist = sqrt(dx * dx + dy * dy);

        handlePreCollision(dist);

        if (dist < COLLISION_DISTANCE) {
            handleCollision(true, true);
            return;
        }

        if (isNearWall(pose1_)) {
            handleCollision(true, false);
            return;
        }

        if (isNearWall(pose2_)) {
            handleCollision(false, true);
            return;
        }
    }

    // =====================================================
    // PRE COLLISION
    // =====================================================
    void handlePreCollision(float dist)
    {
        bool in_pre = (dist < PRECOLLISION_DISTANCE);

        if (in_pre && !pre1_active_) {
            setPen(pen1_client_, 255, 0, 0, 3, false);
            pre1_active_ = true;
        }
        if (!in_pre && pre1_active_) {
            setPen(pen1_client_, 0, 0, 255, 3, false);
            pre1_active_ = false;
        }

        if (in_pre && !pre2_active_) {
            setPen(pen2_client_, 255, 0, 0, 3, false);
            pre2_active_ = true;
        }
        if (!in_pre && pre2_active_) {
            setPen(pen2_client_, 0, 0, 255, 3, false);
            pre2_active_ = false;
        }
    }

    // =====================================================
    // COLLISION (PATCHED WITH RED MICRO-MOVE)
    // =====================================================
    void handleCollision(bool t1, bool t2)
    {
        setFreeze(true);

        // Micro-move twist
        Twist micro;
        micro.linear.x = 1.5;
        micro.angular.z = 0.0;

        // 1) Red pen + micro-move = guaranteed red dash
        if (t1) setPen(pen1_client_, 255, 0, 0, 4, false);
        if (t2) setPen(pen2_client_, 255, 0, 0, 4, false);

        if (t1) pub1_->publish(micro);
        if (t2) pub2_->publish(micro);

        sleep_for(milliseconds(120)); // red dash duration

        // 2) Stop movement and disable pen (no teleport streaks)
        if (t1) stopTurtle(pub1_);
        if (t2) stopTurtle(pub2_);

        if (t1) setPen(pen1_client_, 0, 0, 0, 4, true);
        if (t2) setPen(pen2_client_, 0, 0, 0, 4, true);

        // 3) Teleport
        if (t1) teleport(tp1_client_, last_stop1_);
        if (t2) teleport(tp2_client_, last_stop2_);

        sleep_for(milliseconds(50));

        // 4) Restore blue
        if (t1) setPen(pen1_client_, 0, 0, 255, 3, false);
        if (t2) setPen(pen2_client_, 0, 0, 255, 3, false);

        setFreeze(false);
    }


    // =====================================================
    // VARIABLES
    // =====================================================
    Pose pose1_, pose2_;
    Pose last_stop1_, last_stop2_;

    bool pose1_ready_ = false;
    bool pose2_ready_ = false;
    bool init1_ = false;
    bool init2_ = false;

    bool pre1_active_ = false;
    bool pre2_active_ = false;

    Sub<Pose>::SharedPtr sub1_, sub2_;
    Pub<Twist>::SharedPtr pub1_, pub2_;
    Pub<BoolMsg>::SharedPtr freeze_pub_;

    ClientSetPen::SharedPtr pen1_client_, pen2_client_;
    ClientTeleport::SharedPtr tp1_client_, tp2_client_;

    Timer::SharedPtr timer_;
};

// =====================================================
int main(int argc, char **argv)
{
    init(argc, argv);
    spin(std::make_shared<DistanceNode>());
    shutdown();
    return 0;
}




