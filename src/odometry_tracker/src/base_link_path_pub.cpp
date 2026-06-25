#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class BaseLinkPathPublisher : public rclcpp::Node
{
public:
    BaseLinkPathPublisher() : Node("base_link_path_publisher")
    {
        publisher_ = this->create_publisher<nav_msgs::msg::Path>("/base_link_path", 10);
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/base_link_pose", 10);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        path_msg_.header.frame_id = "odom";
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
        tf2::fromMsg(transform.transform, T_global_base);

        if (first_msg_) {
            tf2::Matrix3x3 m(T_global_base.getRotation());
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            
            tf2::Quaternion q_yaw;
            q_yaw.setRPY(0.0, 0.0, yaw);
            T_init_.setRotation(q_yaw);
            T_init_.setOrigin(T_global_base.getOrigin());
            first_msg_ = false;
        }

        tf2::Transform T_odom_base = T_init_.inverse() * T_global_base;

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = this->get_clock()->now();
        pose.header.frame_id = "odom";
        pose.pose.position.x = T_odom_base.getOrigin().x();
        pose.pose.position.y = T_odom_base.getOrigin().y();
        pose.pose.position.z = T_odom_base.getOrigin().z();
        
        tf2::Quaternion q_out = T_odom_base.getRotation();
        pose.pose.orientation.x = q_out.x();
        pose.pose.orientation.y = q_out.y();
        pose.pose.orientation.z = q_out.z();
        pose.pose.orientation.w = q_out.w();

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

    // Zeroing
    bool first_msg_ = true;
    tf2::Transform T_init_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BaseLinkPathPublisher>());
    rclcpp::shutdown();
    return 0;
}
