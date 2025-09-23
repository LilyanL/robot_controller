#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/go_to_pose.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"


using std::placeholders::_1;
using std::placeholders::_2;

class ControllerNode : public rclcpp::Node
{

using GoToPose = robot_interfaces::action::GoToPose;
using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPose>;

public:
    ControllerNode()
    : Node("controller_node"), x_(0.0), y_(0.0), theta_(0.0)
    {

        // Declare configurable parameters
        this->declare_parameter<double>("max_speed", 1.0);
        this->declare_parameter<double>("odom_frequency", 10.0);


        // Subscribe to velocity commands
        vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&ControllerNode::cmdVelCallback, this, _1)
        );

        // Publish simulated odometry
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        // Setup parameter change callback
        param_callback_handle_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &params)
            {
                for (const auto &param : params)
                {
                    if (param.get_name() == "odom_frequency")
                    {
                        double freq = param.as_double();
                        if (freq <= 0.0)
                        {
                            RCLCPP_WARN(this->get_logger(), "odom_frequency must be positive. Reverting to previous value.");
                            this->set_parameter(rclcpp::Parameter("odom_frequency", this->get_parameter("odom_frequency").as_double()));
                        }
                        else
                        {
                            // Update timer period
                            auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / freq));
                            timer_->cancel();
                            timer_ = this->create_wall_timer(
                                period,
                                std::bind(&ControllerNode::updateOdom, this)
                            );
                            RCLCPP_INFO(this->get_logger(), "Updated odom_frequency to %.2f Hz", freq);
                        }
                    }
                }
                return rcl_interfaces::msg::SetParametersResult().set__successful(true);
            }
        );

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

        // Create action server for GoToPose 
        action_server_ = rclcpp_action::create_server<GoToPose>(
            this,
            "go_to_pose",
            std::bind(&ControllerNode::handleGoal, this, _1, _2),
            std::bind(&ControllerNode::handleCancel, this, _1),
            std::bind(&ControllerNode::handleAccepted, this, _1)
        );

        // Initialize TF broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
            

        RCLCPP_INFO(this->get_logger(), "ControllerNode has started.");
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {

        // Get max speed parameter
        double max_speed = this->get_parameter("max_speed").as_double();
        
        // Clamp velocities to max speed
        if (msg->linear.x > max_speed)
        {
            vx_ = max_speed;
        }
        else if (msg->linear.x < -max_speed)
        {
            vx_ = -max_speed;
        }
        else
        {
            vx_ = msg->linear.x;
        }

        if (msg->linear.y > max_speed)
        {
            vy_ = max_speed;
        }
        else if (msg->linear.y < -max_speed)
        {
            vy_ = -max_speed;
        }
        else
        {
            vy_ = msg->linear.y;
        }

        // Angular velocity is not clamped for simplicity
        vtheta_ = msg->angular.z;
    }

    void updateOdom()
    {

    double dt = 0.1; // 100ms
    x_ += vx_ * dt;
    y_ += vy_ * dt;
    theta_ += vtheta_ * dt;

    // Create quaternion from yaw
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    q.normalize();  // for safety

    // Publish odom
    auto msg = nav_msgs::msg::Odometry();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_link";
    msg.pose.pose.position.x = x_;
    msg.pose.pose.position.y = y_;
    msg.pose.pose.position.z = 0.0; // robot au sol
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    msg.twist.twist.linear.x = vx_;
    msg.twist.twist.linear.y = vy_;
    msg.twist.twist.angular.z = vtheta_;

    odom_pub_->publish(msg);

    // Publish TF (odom -> base_link)
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";

    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);
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



    // Action server callbacks
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const GoToPose::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with target (%.2f, %.2f)", goal->target_x, goal->target_y);
        (void)uuid;
        // Accept all goals
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandleGoToPose> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleAccepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle)
    {
        // This needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&ControllerNode::execute, this, _1), goal_handle}.detach();
    }

    // Execute the goal
    void execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle)
{
    auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<GoToPose::Feedback>();
    auto result = std::make_shared<GoToPose::Result>();

    double target_x = goal->target_x;
    double target_y = goal->target_y;

    rclcpp::Rate loop_rate(10);

    while (rclcpp::ok()) {
        double dx = target_x - x_;
        double dy = target_y - y_;
        double dist = std::sqrt(dx*dx + dy*dy);

        feedback->distance_remaining = dist;
        goal_handle->publish_feedback(feedback);

        if (dist < 0.1) {
            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
            return;
        }

        // Simple proportional control
        vx_ = 0.2 * dx;
        vy_ = 0.2 * dy;

        loop_rate.sleep();
    }
}


    // ROS interfaces

    // Subscriptions and Publications
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Services 
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;

    // Parameter callback handle
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // Action server
    rclcpp_action::Server<GoToPose>::SharedPtr action_server_;

    // TF broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

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
