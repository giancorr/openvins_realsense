#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include <cmath>

using std::placeholders::_1;

class OdomToBaselinkEnuDirect : public rclcpp::Node
{
public:
    OdomToBaselinkEnuDirect() : Node("odom_to_baselink_enu_direct") {
        // Publisher (directly to /odometry/filtered since there is no EKF)
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry/filtered", 10);

        // Subscriber (reads from the single OpenVINS instance)
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ov_msckf/odomimu", 10, std::bind(&OdomToBaselinkEnuDirect::odom_callback, this, _1));

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // --- Conversion from FLU (ROS base_link) to NED (PX4 base_link) ---
        tf2::Transform T_flu_to_ned;
        T_flu_to_ned.setOrigin(tf2::Vector3(0, 0, 0));
        tf2::Quaternion q_flu_ned;
        q_flu_ned.setRPY(M_PI, 0.0, 0.0);
        T_flu_to_ned.setRotation(q_flu_ned);

        // --- Static transform: cam0 IMU -> base_link (FLU) ---
        // OpenVINS is running relative to imu_front, so we use its transform
        tf2::Transform T_ned_to_imu_front;
        tf2::Quaternion q_front_ned(-0.5, 0.5, -0.5, -0.5);
        T_ned_to_imu_front.setRotation(q_front_ned);
        T_ned_to_imu_front.setOrigin(tf2::Vector3(0.0, 0.165, -0.13)); 
        T_imu_front_base_ = T_ned_to_imu_front * T_flu_to_ned;

        // --- Static transform: cam1 IMU -> base_link (FLU) ---
        tf2::Transform T_ned_to_imu_back;
        tf2::Quaternion q_back_ned(-0.06162842, -0.35355339, 0.9312693, -0.06162842);
        T_ned_to_imu_back.setRotation(q_back_ned);
        T_ned_to_imu_back.setOrigin(tf2::Vector3(0.04, -0.18, 0.15));
        T_imu_back_base_ = T_ned_to_imu_back * T_flu_to_ned;

        // --- Conversion from OpenVINS World (X=Left, Y=Back, Z=Up) to ROS ENU (X=Fwd, Y=Left, Z=Up) ---
        tf2::Quaternion q_ov_ros;
        q_ov_ros.setRPY(0.0, 0.0, M_PI / 2.0);
        T_ov_to_ros_.setRotation(q_ov_ros);
        T_ov_to_ros_.setOrigin(tf2::Vector3(0, 0, 0));

        publish_static_tfs();

        RCLCPP_INFO(this->get_logger(), "OdomToBaselinkEnuDirect Node started. Publishing ENU Odometry to /odometry/filtered.");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // 1. Parse OpenVINS pose (T_ovworld_imu)
        tf2::Transform T_ovworld_imu;
        tf2::fromMsg(msg->pose.pose, T_ovworld_imu);

        // 2. T_rosworld_base = T_rosworld_ovworld * T_ovworld_imu * T_imu_base
        // Note: we use T_imu_front_base_ because OpenVINS is running on imu_front
        tf2::Transform T_rosworld_base = T_ov_to_ros_ * T_ovworld_imu * T_imu_front_base_;

        // 3. Create ENU Odometry Message
        nav_msgs::msg::Odometry out_msg;
        out_msg.header.stamp = msg->header.stamp;
        out_msg.header.frame_id = "odom";         // Standard ROS 2 map/odom frame
        out_msg.child_frame_id = "base_link";     // Standard ROS 2 base_link

        tf2::toMsg(T_rosworld_base, out_msg.pose.pose);

        // 4. Transform Twist
        tf2::Vector3 v_imu(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
        tf2::Vector3 w_imu(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
        
        tf2::Transform T_base_imu = T_imu_front_base_.inverse();
        tf2::Matrix3x3 R_base_imu = T_base_imu.getBasis();
        tf2::Vector3 t_base_imu = T_base_imu.getOrigin();

        tf2::Vector3 w_base = R_base_imu * w_imu;
        tf2::Vector3 v_base = R_base_imu * v_imu + w_base.cross(t_base_imu);

        out_msg.twist.twist.linear.x = v_base.x();
        out_msg.twist.twist.linear.y = v_base.y();
        out_msg.twist.twist.linear.z = v_base.z();
        out_msg.twist.twist.angular.x = w_base.x();
        out_msg.twist.twist.angular.y = w_base.y();
        out_msg.twist.twist.angular.z = w_base.z();

        out_msg.pose.covariance = msg->pose.covariance;
        out_msg.twist.covariance = msg->twist.covariance;

        odom_pub_->publish(out_msg);

        // Publish dynamic TF odom -> base_link (Since there is no EKF to do it for us!)
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = msg->header.stamp;
        tf_msg.header.frame_id = "odom";
        tf_msg.child_frame_id = "base_link";
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

        tf_static_broadcaster_->sendTransform(static_tfs);
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

    tf2::Transform T_imu_front_base_;
    tf2::Transform T_imu_back_base_;
    tf2::Transform T_ov_to_ros_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomToBaselinkEnuDirect>());
    rclcpp::shutdown();
    return 0;
}
