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

        path_msg_.header.frame_id = "map";

        // ENU-to-NED rotation (rotX 180°), applied internally
        tf2::Quaternion q_ned;
        q_ned.setRPY(M_PI, 0.0, 0.0);
        T_enu_to_ned_.setRotation(q_ned);
        T_enu_to_ned_.setOrigin(tf2::Vector3(0, 0, 0));

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
            // Retrieve current global -> base_link transform
            transform = tf_buffer_->lookupTransform(
                "global", "base_link", tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            return;
        }

        tf2::Transform T_global_base;
        tf2::fromMsg(transform.transform, T_global_base);

        // Convert to NED coordinates internally
        tf2::Transform T_ned_base = T_enu_to_ned_.inverse() * T_global_base;

        // Zeroing Mechanism (Initialization)
        if (first_msg_) {
            tf2::Matrix3x3 m(T_ned_base.getRotation());
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            // Create initial pose removing pitch and roll (keeping horizontal alignment)
            tf2::Quaternion q_yaw;
            q_yaw.setRPY(0.0, 0.0, yaw);
            T_init_ned_.setRotation(q_yaw);
            T_init_ned_.setOrigin(T_ned_base.getOrigin());

            // Publish map as child of global (baking in the NED rotation)
            tf2::Transform T_global_map = T_enu_to_ned_ * T_init_ned_;

            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = transform.header.stamp;
            t.header.frame_id = "global";
            t.child_frame_id = "map";
            t.transform.translation.x = T_global_map.getOrigin().x();
            t.transform.translation.y = T_global_map.getOrigin().y();
            t.transform.translation.z = T_global_map.getOrigin().z();
            t.transform.rotation.x = T_global_map.getRotation().x();
            t.transform.rotation.y = T_global_map.getRotation().y();
            t.transform.rotation.z = T_global_map.getRotation().z();
            t.transform.rotation.w = T_global_map.getRotation().w();
            tf_static_broadcaster_->sendTransform(t);

            first_msg_ = false;
        }

        // Calculate Odometry (Pose relative to starting point in NED)
        tf2::Transform T_map_base = T_init_ned_.inverse() * T_ned_base;

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = this->get_clock()->now();
        pose.header.frame_id = "map";
        
        pose.pose.position.x = T_map_base.getOrigin().x();
        pose.pose.position.y = T_map_base.getOrigin().y();
        pose.pose.position.z = T_map_base.getOrigin().z();

        tf2::Quaternion q_out = T_map_base.getRotation();
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

    // ENU-to-NED conversion (rotX 180°)
    tf2::Transform T_enu_to_ned_;

    // Initialization
    bool first_msg_ = true;
    tf2::Transform T_init_ned_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BaseLinkPathPublisher>());
    rclcpp::shutdown();
    return 0;
}
