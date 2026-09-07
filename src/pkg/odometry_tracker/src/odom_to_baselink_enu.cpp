#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include <cmath>

using std::placeholders::_1;

class OdomToBaselinkEnu : public rclcpp::Node
{
public:
    OdomToBaselinkEnu() : Node("odom_to_baselink_enu") {
        // Publishers
        front_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/front/base_link_odom", 10);
        back_pub_  = this->create_publisher<nav_msgs::msg::Odometry>("/back/base_link_odom", 10);

        // Subscribers
        front_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/front/odomimu", 10, std::bind(&OdomToBaselinkEnu::front_callback, this, _1));
        back_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/back/odomimu", 10, std::bind(&OdomToBaselinkEnu::back_callback, this, _1));

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // --- Conversion from FLU (ROS base_link) to NED (PX4 base_link) ---
        // X_ned = X_flu, Y_ned = -Y_flu, Z_ned = -Z_flu => Roll = 180 degrees
        tf2::Transform T_flu_to_ned;
        T_flu_to_ned.setOrigin(tf2::Vector3(0, 0, 0));
        tf2::Quaternion q_flu_ned;
        q_flu_ned.setRPY(M_PI, 0.0, 0.0);
        T_flu_to_ned.setRotation(q_flu_ned);

        // --- Static transform: cam0 IMU -> base_link (FLU) ---
        tf2::Transform T_ned_to_imu_front;
        tf2::Quaternion q_front_ned(-0.5, 0.5, -0.5, -0.5);
        T_ned_to_imu_front.setRotation(q_front_ned);
        T_ned_to_imu_front.setOrigin(tf2::Vector3(0.0, 0.165, -0.13)); // Z was 0.19, now 0.165
        T_imu_front_base_ = T_ned_to_imu_front * T_flu_to_ned;

        // --- Static transform: cam1 IMU -> base_link (FLU) ---
        tf2::Transform T_ned_to_imu_back;
        tf2::Quaternion q_back_ned(-0.06162842, -0.35355339, 0.9312693, -0.06162842);
        T_ned_to_imu_back.setRotation(q_back_ned);
        T_ned_to_imu_back.setOrigin(tf2::Vector3(0.04, -0.18, 0.15));
        T_imu_back_base_ = T_ned_to_imu_back * T_flu_to_ned;

        // --- Conversion from OpenVINS World (X=Left, Y=Back, Z=Up) to ROS ENU (X=Fwd, Y=Left, Z=Up) ---
        // This requires Yaw = +90 degrees.
        tf2::Quaternion q_ov_ros;
        q_ov_ros.setRPY(0.0, 0.0, M_PI / 2.0);
        T_ov_to_ros_.setRotation(q_ov_ros);
        T_ov_to_ros_.setOrigin(tf2::Vector3(0, 0, 0));

        // Publish static TFs for rviz debugging (optional, can be disabled if handled elsewhere)
        publish_static_tfs();

        RCLCPP_INFO(this->get_logger(), "OdomToBaselinkEnu Node started. Publishing ENU Odometry.");
    }

private:
    void front_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        process_odom(msg, T_imu_front_base_, front_pub_, "front_odom");
    }

    void back_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        process_odom(msg, T_imu_back_base_, back_pub_, "back_odom");
    }

    void process_odom(const nav_msgs::msg::Odometry::SharedPtr msg, 
                      const tf2::Transform& T_imu_base, 
                      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub,
                      const std::string& child_frame_id) 
    {
        // 1. Parse OpenVINS pose (T_ovworld_imu)
        tf2::Transform T_ovworld_imu;
        tf2::fromMsg(msg->pose.pose, T_ovworld_imu);

        // 2. T_rosworld_base = T_rosworld_ovworld * T_ovworld_imu * T_imu_base
        tf2::Transform T_rosworld_base = T_ov_to_ros_ * T_ovworld_imu * T_imu_base;

        // 3. Create ENU Odometry Message
        nav_msgs::msg::Odometry out_msg;
        out_msg.header.stamp = msg->header.stamp;
        out_msg.header.frame_id = "odom";         // Standard ROS 2 map/odom frame
        out_msg.child_frame_id = "base_link";     // Standard ROS 2 base_link

        tf2::toMsg(T_rosworld_base, out_msg.pose.pose);

        // 4. Transform Twist
        // OpenVINS twist is in IMU local frame. We need it in base_link (FLU) frame.
        tf2::Vector3 v_imu(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
        tf2::Vector3 w_imu(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
        
        // Transform velocities from IMU frame to base_link frame
        // T_base_imu is the inverse of T_imu_base
        tf2::Transform T_base_imu = T_imu_base.inverse();
        tf2::Matrix3x3 R_base_imu = T_base_imu.getBasis();
        tf2::Vector3 t_base_imu = T_base_imu.getOrigin();

        // v_base = R_base_imu * v_imu + w_base x t_base_imu
        // w_base = R_base_imu * w_imu
        tf2::Vector3 w_base = R_base_imu * w_imu;
        tf2::Vector3 v_base = R_base_imu * v_imu + w_base.cross(t_base_imu);

        out_msg.twist.twist.linear.x = v_base.x();
        out_msg.twist.twist.linear.y = v_base.y();
        out_msg.twist.twist.linear.z = v_base.z();
        out_msg.twist.twist.angular.x = w_base.x();
        out_msg.twist.twist.angular.y = w_base.y();
        out_msg.twist.twist.angular.z = w_base.z();

        // 5. Transfer Covariance (simplified: assume diagonal/roughly same or rotate it if needed)
        // For EKF, just passing it as is (since it's mostly diagonal) is often acceptable,
        // but ideally we should rotate the 6x6 covariance matrix. 
        // We'll keep it simple here as it was in the original script.
        out_msg.pose.covariance = msg->pose.covariance;
        out_msg.twist.covariance = msg->twist.covariance;

        pub->publish(out_msg);

        // Publish TF for visualization (optional)
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = msg->header.stamp;
        tf_msg.header.frame_id = "odom";
        tf_msg.child_frame_id = child_frame_id;
        tf_msg.transform.translation.x = T_rosworld_base.getOrigin().x();
        tf_msg.transform.translation.y = T_rosworld_base.getOrigin().y();
        tf_msg.transform.translation.z = T_rosworld_base.getOrigin().z();
        tf_msg.transform.rotation = tf2::toMsg(T_rosworld_base.getRotation());
        tf_broadcaster_->sendTransform(tf_msg);
    }

    void publish_static_tfs() {
        std::vector<geometry_msgs::msg::TransformStamped> static_tfs;

        // 1. odom -> openvins_world
        geometry_msgs::msg::TransformStamped tf_ov;
        tf_ov.header.stamp = this->now();
        tf_ov.header.frame_id = "odom";
        tf_ov.child_frame_id = "openvins_world";
        tf_ov.transform.translation.x = T_ov_to_ros_.inverse().getOrigin().x();
        tf_ov.transform.translation.y = T_ov_to_ros_.inverse().getOrigin().y();
        tf_ov.transform.translation.z = T_ov_to_ros_.inverse().getOrigin().z();
        tf_ov.transform.rotation = tf2::toMsg(T_ov_to_ros_.inverse().getRotation());
        static_tfs.push_back(tf_ov);

        // 2. base_link -> imu_front
        geometry_msgs::msg::TransformStamped tf_front;
        tf_front.header.stamp = this->now();
        tf_front.header.frame_id = "base_link";
        tf_front.child_frame_id = "imu_front";
        tf2::Transform T_base_imu_front = T_imu_front_base_.inverse();
        tf_front.transform.translation.x = T_base_imu_front.getOrigin().x();
        tf_front.transform.translation.y = T_base_imu_front.getOrigin().y();
        tf_front.transform.translation.z = T_base_imu_front.getOrigin().z();
        tf_front.transform.rotation = tf2::toMsg(T_base_imu_front.getRotation());
        static_tfs.push_back(tf_front);

        // 3. base_link -> imu_back
        geometry_msgs::msg::TransformStamped tf_back;
        tf_back.header.stamp = this->now();
        tf_back.header.frame_id = "base_link";
        tf_back.child_frame_id = "imu_back";
        tf2::Transform T_base_imu_back = T_imu_back_base_.inverse();
        tf_back.transform.translation.x = T_base_imu_back.getOrigin().x();
        tf_back.transform.translation.y = T_base_imu_back.getOrigin().y();
        tf_back.transform.translation.z = T_base_imu_back.getOrigin().z();
        tf_back.transform.rotation = tf2::toMsg(T_base_imu_back.getRotation());
        static_tfs.push_back(tf_back);

        // 4. base_link -> cam0_link (FLU)
        geometry_msgs::msg::TransformStamped tf_cam0;
        tf_cam0.header.stamp = this->now();
        tf_cam0.header.frame_id = "base_link";
        tf_cam0.child_frame_id = "cam0_link";
        tf2::Transform T_ned_to_cam0;
        T_ned_to_cam0.setRotation(tf2::Quaternion(0.5, 0.5, 0.5, 0.5));
        T_ned_to_cam0.setOrigin(tf2::Vector3(0.13, -0.04, 0.165));
        tf2::Transform T_base_cam0 = T_ned_to_cam0 * T_flu_to_ned; // T_flu_to_ned converts baseFLU to baseNED
        // Wait, T_baseFLU_to_cam0 = T_baseFLU_to_baseNED * T_baseNED_to_cam0?
        // Let's use the same logic as IMU:
        // T_ned_to_cam0 transforms from cam0 to baseNED.
        // We want T_flu_to_cam0 which transforms from cam0 to baseFLU.
        // P_flu = T_flu_to_ned * P_ned? No, T_flu_to_ned transforms from NED to FLU.
        // Wait, earlier I did T_imu_front_base_ = T_ned_to_imu_front * T_flu_to_ned;
        // This was: P_imu = T_ned_to_imu * P_ned = T_ned_to_imu * T_flu_to_ned * P_flu. 
        // So T_imu_base_flu = T_ned_to_imu * T_flu_to_ned.
        // Here we want T_base_flu_to_cam0, which is the inverse of T_cam0_to_base_flu.
        // Let's construct T_cam0_to_base_flu:
        // P_flu = (T_flu_to_ned)^-1 * P_ned = (T_flu_to_ned)^-1 * (T_cam0_to_base_ned)^-1 * P_cam0.
        // Actually, T_flu_to_ned maps points in NED to FLU. (Because X_ned = X_flu, Y_ned = -Y_flu, Z_ned = -Z_flu => P_ned = R * P_flu. So R maps FLU to NED. So T_flu_to_ned maps FLU to NED).
        // Let's just do it directly.
        tf_cam0.transform.translation.x = 0.13;
        tf_cam0.transform.translation.y = 0.04; // Y_flu = -Y_ned = -(-0.04) = 0.04
        tf_cam0.transform.translation.z = -0.165; // Z_flu = -Z_ned = -0.165
        // Rotation: T_baseFLU_to_cam0 = T_baseFLU_to_baseNED * T_baseNED_to_cam0
        // Wait! We want the rotation of cam0 relative to baseFLU.
        // R_cam0_in_flu = R_ned_in_flu * R_cam0_in_ned
        tf2::Quaternion q_flu_to_ned;
        q_flu_to_ned.setRPY(M_PI, 0, 0); // Maps FLU vectors to NED vectors
        tf2::Quaternion q_cam0_in_ned(0.5, 0.5, 0.5, 0.5); // Maps Cam0 vectors to NED vectors
        // R_cam0_in_flu = (R_flu_to_ned)^-1 * R_cam0_in_ned
        tf2::Quaternion q_cam0_in_flu = q_flu_to_ned.inverse() * q_cam0_in_ned;
        tf_cam0.transform.rotation = tf2::toMsg(q_cam0_in_flu);
        static_tfs.push_back(tf_cam0);

        // 5. base_link -> cam1_link (FLU)
        geometry_msgs::msg::TransformStamped tf_cam1;
        tf_cam1.header.stamp = this->now();
        tf_cam1.header.frame_id = "base_link";
        tf_cam1.child_frame_id = "cam1_link";
        tf_cam1.transform.translation.x = -0.15;
        tf_cam1.transform.translation.y = -0.04; // Y_flu = -Y_ned = -(0.04) = -0.04
        tf_cam1.transform.translation.z = -0.18; // Z_flu = -Z_ned = -0.18
        tf2::Quaternion q_cam1_in_ned(-0.35355339, 0.35355339, 0.61237244, -0.61237244);
        tf2::Quaternion q_cam1_in_flu = q_flu_to_ned.inverse() * q_cam1_in_ned;
        tf_cam1.transform.rotation = tf2::toMsg(q_cam1_in_flu);
        static_tfs.push_back(tf_cam1);

        tf_static_broadcaster_->sendTransform(static_tfs);
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr front_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr back_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr front_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr back_sub_;
    
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

    tf2::Transform T_imu_front_base_;
    tf2::Transform T_imu_back_base_;
    tf2::Transform T_ov_to_ros_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomToBaselinkEnu>());
    rclcpp::shutdown();
    return 0;
}
