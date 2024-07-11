#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class Cmd_vel2odom_node : public rclcpp::Node
{
public:
    Cmd_vel2odom_node() : Node("cmd_vel2odom_node"), x_(0.0), y_(0.0), theta_(0.0)

    {
        this->declare_parameter<double>("wheel_base", 0.5);

        this->get_parameter("wheel_base", wheel_base_);

        cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&Cmd_vel2odom_node::cmdVelCallback, this, std::placeholders::_1));

        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

        timer_ = this->create_wall_timer(
            100ms, std::bind(&Cmd_vel2odom_node::publishOdometry, this));
        last_time_ = this->now();
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        current_velocity_ = msg->linear.x;
        current_steering_angle_ = msg->angular.z;
    }

    void publishOdometry()
    {
        auto current_time = this->now();
        double dt = (current_time - last_time_).seconds();

        // Compute odometry
        double delta_x = current_velocity_ * std::cos(theta_) * dt;
        double delta_y = current_velocity_ * std::sin(theta_) * dt;
        double delta_theta = (current_velocity_ / wheel_base_) * std::tan(current_steering_angle_) * dt;

        x_ += delta_x;
        y_ += delta_y;
        theta_ += delta_theta;

        // Create quaternion from yaw
        tf2::Quaternion odom_quat;
        odom_quat.setRPY(0, 0, theta_);

        // Publish odometry message
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";

        // Set the position
        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation.x = odom_quat.x();
        odom_msg.pose.pose.orientation.y = odom_quat.y();
        odom_msg.pose.pose.orientation.z = odom_quat.z();
        odom_msg.pose.pose.orientation.w = odom_quat.w();

        // Set the velocity
        odom_msg.child_frame_id = "base_link";
        odom_msg.twist.twist.linear.x = current_velocity_;
        odom_msg.twist.twist.angular.z = delta_theta / dt;

        odom_publisher_->publish(odom_msg);

        // Update time
        last_time_ = current_time;
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    double wheel_base_;
    double current_velocity_;
    double current_steering_angle_;
    double x_, y_, theta_;

    rclcpp::Time last_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Cmd_vel2odom_node>());
    rclcpp::shutdown();
    return 0;
}
