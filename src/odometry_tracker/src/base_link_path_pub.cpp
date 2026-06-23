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

        tf2::Transform T_global_base;
        T_global_base.setOrigin(tf2::Vector3(
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z));
        T_global_base.setRotation(tf2::Quaternion(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w));

        if (first_) {
            double roll, pitch, yaw;
            T_global_base.getBasis().getRPY(roll, pitch, yaw);
            yaw_offset_ = yaw;
            first_ = false;
        }

        // Apply yaw offset to rotate global_ned so that initial base_link X is aligned with global_ned X
        tf2::Quaternion q_rot;
        q_rot.setRPY(0, 0, -yaw_offset_);
        tf2::Transform T_rot(q_rot, tf2::Vector3(0, 0, 0));
        
        tf2::Transform T_corrected = T_rot * T_global_base;

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = this->get_clock()->now();
        pose.header.frame_id = "global_ned";
        pose.pose.position.x = T_corrected.getOrigin().x();
        pose.pose.position.y = T_corrected.getOrigin().y();
        pose.pose.position.z = T_corrected.getOrigin().z();
        
        pose.pose.orientation.x = T_corrected.getRotation().x();
        pose.pose.orientation.y = T_corrected.getRotation().y();
        pose.pose.orientation.z = T_corrected.getRotation().z();
        pose.pose.orientation.w = T_corrected.getRotation().w();

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
    bool first_ = true;
    double yaw_offset_ = 0.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BaseLinkPathPublisher>());
    rclcpp::shutdown();
    return 0;
}
