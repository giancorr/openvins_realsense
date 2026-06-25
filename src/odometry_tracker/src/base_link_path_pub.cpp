#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

class BaseLinkPathPublisher : public rclcpp::Node
{
public:
    BaseLinkPathPublisher() : Node("base_link_path_publisher")
    {
        // Publishers & TF Setup
        publisher_ = this->create_publisher<nav_msgs::msg::Path>("/base_link_path", 10);
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/base_link_pose", 10);
        
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        path_msg_.header.frame_id = "odom";

        // Main Processing Loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), 
            std::bind(&BaseLinkPathPublisher::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Base link path publisher started");
    }

private:
    void timer_callback()
    {
        geometry_msgs::msg::TransformStamped transform;
        try {
            // Retrieve current global_ned -> base_link transform
            transform = tf_buffer_->lookupTransform(
                "global_ned", "base_link", tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            return;
        }

        tf2::Transform T_global_base;
        tf2::fromMsg(transform.transform, T_global_base);

        // Zeroing Mechanism (Initialization)
        if (first_msg_) {
            tf2::Matrix3x3 m(T_global_base.getRotation());
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            // Create initial pose removing pitch and roll (keeping horizontal alignment)
            tf2::Quaternion q_yaw;
            q_yaw.setRPY(0.0, 0.0, yaw);
            T_init_.setRotation(q_yaw);
            T_init_.setOrigin(T_global_base.getOrigin());

            // Broadcast the odometry origin frame to the TF tree
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = transform.header.stamp;
            t.header.frame_id = "global_ned";
            t.child_frame_id = "odom";
            t.transform.translation.x = T_init_.getOrigin().x();
            t.transform.translation.y = T_init_.getOrigin().y();
            t.transform.translation.z = T_init_.getOrigin().z();
            t.transform.rotation.x = q_yaw.x();
            t.transform.rotation.y = q_yaw.y();
            t.transform.rotation.z = q_yaw.z();
            t.transform.rotation.w = q_yaw.w();
            tf_static_broadcaster_->sendTransform(t);

            first_msg_ = false;
        }

        // Calculate Odometry (Pose relative to starting point)
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

        // Publish Path and Pose
        path_msg_.poses.push_back(pose);
        path_msg_.header.stamp = transform.header.stamp;
        
        publisher_->publish(path_msg_);
        pose_publisher_->publish(pose);
    }

    // State Variables
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    
    nav_msgs::msg::Path path_msg_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Initialization flags
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
