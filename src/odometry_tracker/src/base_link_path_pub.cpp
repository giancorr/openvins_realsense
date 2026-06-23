#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>

class BaseLinkPathPublisher : public rclcpp::Node
{
public:
    BaseLinkPathPublisher() : Node("base_link_path_publisher")
    {
        publisher_ = this->create_publisher<nav_msgs::msg::Path>("/base_link_path", 10);
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/base_link_pose", 10);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        path_msg_.header.frame_id = "global_ned";
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),  // 20 Hz
            std::bind(&BaseLinkPathPublisher::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Base link path publisher started");
    }

private:
    void timer_callback()
    {
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform(
                "global_ned", "base_link", tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            return;
        }

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = this->get_clock()->now();
        pose.header.frame_id = "global_ned";
        pose.pose.position.x = transform.transform.translation.x;
        pose.pose.position.y = transform.transform.translation.y;
        pose.pose.position.z = transform.transform.translation.z;
        pose.pose.orientation = transform.transform.rotation;

        path_msg_.poses.push_back(pose);
        path_msg_.header.stamp = transform.header.stamp;
        publisher_->publish(path_msg_);
        pose_publisher_->publish(pose);
    }

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    nav_msgs::msg::Path path_msg_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BaseLinkPathPublisher>());
    rclcpp::shutdown();
    return 0;
}
