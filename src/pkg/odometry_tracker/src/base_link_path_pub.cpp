#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>

class BaseLinkPathPublisher : public rclcpp::Node
{
public:
    BaseLinkPathPublisher() : Node("base_link_path_publisher")
    {
        // Declare parameter for topic name
        this->declare_parameter<std::string>("odom_topic", "/front/base_link_odom");
        std::string odom_topic;
        this->get_parameter("odom_topic", odom_topic);

        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/base_link_path", 10);
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/base_link_pose", 10);
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10,
            std::bind(&BaseLinkPathPublisher::odom_callback, this, std::placeholders::_1));

        path_msg_.header.frame_id = "map";

        RCLCPP_INFO(this->get_logger(), "Base link path publisher started. Listening to %s", odom_topic.c_str());
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = msg->header.stamp;
        pose.header.frame_id = "map";
        
        pose.pose = msg->pose.pose;

        path_msg_.poses.push_back(pose);
        path_msg_.header.stamp = msg->header.stamp;
        
        path_publisher_->publish(path_msg_);
        pose_publisher_->publish(pose);
    }

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    nav_msgs::msg::Path path_msg_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BaseLinkPathPublisher>());
    rclcpp::shutdown();
    return 0;
}
