#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

class OdomToBaselinkNode : public rclcpp::Node
{
public:
    OdomToBaselinkNode() : Node("odom_to_baselink_node")
    {
        // Publishers & Subscribers
        odom_front_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/front/odomimu", 10,
            std::bind(&OdomToBaselinkNode::odom_front_callback, this, std::placeholders::_1));

        odom_back_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/back/odomimu", 10,
            std::bind(&OdomToBaselinkNode::odom_back_callback, this, std::placeholders::_1));

        odom_front_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/front/base_link_odom", 10);
        odom_back_pub_  = this->create_publisher<nav_msgs::msg::Odometry>("/back/base_link_odom", 10);

        // TF Setup
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // --- Static transform: cam0 IMU -> base_link ---
        // cam0 at (+0.13, 0, +0.19) in base_link NED; lens 4cm left of IMU (-Y)
        // Camera points forward: +Z_cam = +X_base, +X_cam = +Y_base, +Y_cam = +Z_base
        tf2::Quaternion q_front(0.5, 0.5, 0.5, -0.5);
        T_imu_front_base_.setRotation(q_front);
        T_imu_front_base_.setOrigin(tf2::Vector3(0.0, -0.19, -0.13));

        // --- Static transform: cam1 IMU -> base_link ---
        // cam1 at (-0.15, +0.0, +0.18) in base_link NED; 180 deg yaw + 30 deg pitch down
        tf2::Quaternion q_back(-0.35355339, 0.35355339, 0.61237244, 0.61237244);
        T_imu_back_base_.setRotation(q_back);
        T_imu_back_base_.setOrigin(tf2::Vector3(0.0, -0.080885, -0.219904));

        // --- Conversion to User FRD frame (X=Forward, Y=Right, Z=Down) ---
        // Quaternion (x, y, z, w) for R_enu_to_user (Roll=180°, Pitch=0°, Yaw=90°)
        tf2::Quaternion q_user(0.70710678, 0.70710678, 0.0, 0.0);
        T_enu_to_ned_.setRotation(q_user);
        T_enu_to_ned_.setOrigin(tf2::Vector3(0, 0, 0));

        RCLCPP_INFO(this->get_logger(), "OdomToBaselinkNode started. Listening to OpenVINS odometry...");
    }

private:
    void odom_front_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (first_msg_front_) {
            auto global_out = transform_odometry_global(msg, T_imu_front_base_);
            initialize_odom_frame("front", global_out);
            first_msg_front_ = false;
        }

        auto out_msg = transform_odometry_map(msg, T_imu_front_base_, T_init_front_);
        odom_front_pub_->publish(out_msg);
    }

    void odom_back_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (first_msg_back_) {
            auto global_out = transform_odometry_global(msg, T_imu_back_base_);
            initialize_odom_frame("back", global_out);
            first_msg_back_ = false;
        }

        auto out_msg = transform_odometry_map(msg, T_imu_back_base_, T_init_back_);
        odom_back_pub_->publish(out_msg);
    }

    void initialize_odom_frame(const std::string& session_type, const nav_msgs::msg::Odometry& out_msg)
    {
        // Define initial origin and yaw in User FRD frame
        tf2::Transform T_global_base_enu;
        tf2::fromMsg(out_msg.pose.pose, T_global_base_enu);
        tf2::Transform T_user_base = T_enu_to_ned_ * T_global_base_enu;

        tf2::Matrix3x3 m(T_user_base.getRotation());
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        tf2::Quaternion q_yaw;
        q_yaw.setRPY(0.0, 0.0, yaw);

        if (session_type == "front") {
            T_init_front_.setRotation(q_yaw);
            T_init_front_.setOrigin(T_user_base.getOrigin());
            if (!static_map_published_) {
                publish_static_map(T_init_front_, out_msg.header.stamp);
                static_map_published_ = true;
            }
            RCLCPP_INFO(this->get_logger(), "Zeroing complete for FRONT session.");
        } else {
            T_init_back_.setRotation(q_yaw);
            T_init_back_.setOrigin(T_user_base.getOrigin());
            if (!static_map_published_) {
                publish_static_map(T_init_back_, out_msg.header.stamp);
                static_map_published_ = true;
            }
            RCLCPP_INFO(this->get_logger(), "Zeroing complete for BACK session.");
        }
    }

    void publish_static_map(const tf2::Transform& T_init, const builtin_interfaces::msg::Time& stamp)
    {
        // T_map_base = T_init^-1 * T_enu_to_ned * T_global_base
        // We want TF tree: T_map_global = T_init^-1 * T_enu_to_ned
        // So T_global_map = T_enu_to_ned^-1 * T_init
        tf2::Transform T_global_map = T_enu_to_ned_.inverse() * T_init;

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = stamp;
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
    }

    nav_msgs::msg::Odometry transform_odometry_global(const nav_msgs::msg::Odometry::SharedPtr msg, const tf2::Transform& T_imu_base)
    {
        // T_global_imu
        tf2::Transform T_global_imu;
        tf2::fromMsg(msg->pose.pose, T_global_imu);

        // T_global_base = T_global_imu * T_imu_base
        tf2::Transform T_global_base = T_global_imu * T_imu_base;

        nav_msgs::msg::Odometry out;
        out.header.stamp = msg->header.stamp;
        out.header.frame_id = "global";
        out.child_frame_id = "base_link";

        tf2::Vector3 pos = T_global_base.getOrigin();
        tf2::Quaternion rot = T_global_base.getRotation();

        out.pose.pose.position.x = pos.x();
        out.pose.pose.position.y = pos.y();
        out.pose.pose.position.z = pos.z();
        out.pose.pose.orientation.x = rot.x();
        out.pose.pose.orientation.y = rot.y();
        out.pose.pose.orientation.z = rot.z();
        out.pose.pose.orientation.w = rot.w();

        return out;
    }

    nav_msgs::msg::Odometry transform_odometry_map(const nav_msgs::msg::Odometry::SharedPtr msg, const tf2::Transform& T_imu_base, const tf2::Transform& T_init)
    {
        // Get the global base_link pose in ENU
        tf2::Transform T_global_imu;
        tf2::fromMsg(msg->pose.pose, T_global_imu);
        tf2::Transform T_global_base_enu = T_global_imu * T_imu_base;

        // Convert to User FRD frame
        tf2::Transform T_user_base = T_enu_to_ned_ * T_global_base_enu;
        tf2::Transform T_map_base = T_init.inverse() * T_user_base;

        nav_msgs::msg::Odometry out;
        out.header.stamp = msg->header.stamp;
        out.header.frame_id = "map";
        out.child_frame_id = "base_link";

        tf2::Vector3 pos = T_map_base.getOrigin();
        tf2::Quaternion rot = T_map_base.getRotation();

        out.pose.pose.position.x = pos.x();
        out.pose.pose.position.y = pos.y();
        out.pose.pose.position.z = pos.z();
        out.pose.pose.orientation.x = rot.x();
        out.pose.pose.orientation.y = rot.y();
        out.pose.pose.orientation.z = rot.z();
        out.pose.pose.orientation.w = rot.w();

        // Pass covariance
        out.pose.covariance = msg->pose.covariance;

        // Twist needs to be rotated because it is expressed in body frame (cam_link). 
        // We want it in the new base_link frame.
        tf2::Vector3 linear_vel_imu(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
        tf2::Vector3 angular_vel_imu(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);

        // Apply rotation from IMU to base_link
        tf2::Matrix3x3 R_imu_base(T_imu_base.getRotation());
        tf2::Vector3 linear_vel_base = R_imu_base.inverse() * linear_vel_imu;
        tf2::Vector3 angular_vel_base = R_imu_base.inverse() * angular_vel_imu;

        out.twist.twist.linear.x = linear_vel_base.x();
        out.twist.twist.linear.y = linear_vel_base.y();
        out.twist.twist.linear.z = linear_vel_base.z();
        
        out.twist.twist.angular.x = angular_vel_base.x();
        out.twist.twist.angular.y = angular_vel_base.y();
        out.twist.twist.angular.z = angular_vel_base.z();

        // Pass Twist covariance 
        out.twist.covariance = msg->twist.covariance;

        return out;
    }

    // ROS2 constructs
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_front_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_back_sub_;
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_front_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_back_pub_;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

    // Static transforms
    tf2::Transform T_imu_front_base_;  
    tf2::Transform T_imu_back_base_;   
    tf2::Transform T_enu_to_ned_;

    // Zeroing
    bool first_msg_front_ = true;
    bool first_msg_back_ = true;
    bool static_map_published_ = false;
    tf2::Transform T_init_front_;
    tf2::Transform T_init_back_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomToBaselinkNode>());
    rclcpp::shutdown();
    return 0;
}
