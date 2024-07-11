#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class OdomToTFNode : public rclcpp::Node
{
public:
    OdomToTFNode()
        : Node("odom_to_tf_node")
    {
        // Create a TransformBroadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Create a subscription to the odometry topic
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&OdomToTFNode::odomCallback, this, std::placeholders::_1));
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Create a TransformStamped message
        geometry_msgs::msg::TransformStamped transform_stamped;

        // Set the frame ID and child frame ID
        transform_stamped.header.stamp = msg->header.stamp;
        transform_stamped.header.frame_id = msg->header.frame_id;
        transform_stamped.child_frame_id = msg->child_frame_id;

        // Set the translation
        transform_stamped.transform.translation.x = msg->pose.pose.position.x;
        transform_stamped.transform.translation.y = msg->pose.pose.position.y;
        transform_stamped.transform.translation.z = msg->pose.pose.position.z;

        // Set the rotation
        transform_stamped.transform.rotation = msg->pose.pose.orientation;

        // Broadcast the transform
        tf_broadcaster_->sendTransform(transform_stamped);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomToTFNode>());
    rclcpp::shutdown();
    return 0;
}
