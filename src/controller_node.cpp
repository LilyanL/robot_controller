#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/trigger.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode()
    : Node("controller_node"), x_(0.0), y_(0.0), theta_(0.0)
    {
        // Subscribe to velocity commands
        vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&ControllerNode::cmdVelCallback, this, _1)
        );

        // Publish simulated odometry
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        // Update odometry at 10Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ControllerNode::updateOdom, this)
        );

        // Create a service to reset pose
        // reset_service_ = this->create_service<std_srvs::srv::Trigger>(
        //    "/reset_pose",
        //    std::bind(&ControllerNode::resetPoseCallback, this, _1, _2)
        //);

        // Create a service to reset pose (lambda version)
        reset_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/reset_pose",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> res)
            {
                this->resetPoseCallback(req, res);
            }
        );


        RCLCPP_INFO(this->get_logger(), "ControllerNode has started.");
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        vx_ = msg->linear.x;
        vy_ = msg->linear.y;
        vtheta_ = msg->angular.z;
    }

    void updateOdom()
    {
        double dt = 0.1; // 100ms
        x_ += vx_ * dt;
        y_ += vy_ * dt;
        theta_ += vtheta_ * dt;

        auto msg = nav_msgs::msg::Odometry();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "odom";
        msg.child_frame_id = "base_link";
        msg.pose.pose.position.x = x_;
        msg.pose.pose.position.y = y_;
        msg.pose.pose.orientation.z = sin(theta_ / 2.0);
        msg.pose.pose.orientation.w = cos(theta_ / 2.0);
        msg.twist.twist.linear.x = vx_;
        msg.twist.twist.linear.y = vy_;
        msg.twist.twist.angular.z = vtheta_;

        odom_pub_->publish(msg);
    }

    void resetPoseCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request; // unused
        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;
        response->success = true;
        response->message = "Pose reset to (0,0,0)";
        RCLCPP_INFO(this->get_logger(), "Pose has been reset.");
    }


    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;

    // Internal state
    double x_, y_, theta_;
    double vx_ = 0.0, vy_ = 0.0, vtheta_ = 0.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}
