// odom_to_baselink: transforms two IMU odometries to base_link odometries.
//
// Subscribes to:
//   /front/odomimu  (nav_msgs/Odometry, global_front -> imu_front)
//   /back/odomimu   (nav_msgs/Odometry, global_back  -> imu_back)
//
// Publishes:
//   /front/base_link_odom  (nav_msgs/Odometry, odom -> base_link)
//   /back/base_link_odom   (nav_msgs/Odometry, odom -> base_link)
//
// Each odometry is transformed from its IMU frame to base_link using
// the known static transform (different for front and back IMU).

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>

class OdomToBaselink : public rclcpp::Node
{
public:
    OdomToBaselink() : Node("odom_to_baselink")
    {
        // Publishers
        pub_front_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/front/base_link_odom", 10);
        pub_back_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/back/base_link_odom", 10);

        // Subscribers
        sub_front_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/front/odomimu", 10,
            std::bind(&OdomToBaselink::front_callback, this, std::placeholders::_1));
        sub_back_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/back/odomimu", 10,
            std::bind(&OdomToBaselink::back_callback, this, std::placeholders::_1));

        // Static transform: cam0 IMU -> base_link
        tf2::Quaternion q_front(0.5047690, -0.4984747, 0.4937537, 0.5029300);
        T_imu_front_base_.setRotation(q_front);
        T_imu_front_base_.setOrigin(tf2::Vector3(0.008649, 0.144703, -0.103955));

        // Static transform: cam1 IMU -> base_link
        tf2::Quaternion q_back(0.3567783, -0.3462856, 0.6116574, 0.6153622);
        T_imu_back_base_.setRotation(q_back);
        T_imu_back_base_.setOrigin(tf2::Vector3(-0.010360, 0.252312, -0.206836));

        // Static transform: global -> global_ned 
        // User explicitly requested EXACTLY 180 deg around Y for ALL cameras.
        // RotY(180) quaternion is (x=0, y=1, z=0, w=0).
        // Since we need the inverse (global_ned -> global) for this specific variable,
        // the inverse of (0, 1, 0, 0) is (0, -1, 0, 0).
        tf2::Quaternion q_global_ned(0.0, -1.0, 0.0, 0.0);

        T_global_globalned_front_.setRotation(q_global_ned);
        T_global_globalned_front_.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));

        T_global_globalned_back_.setRotation(q_global_ned);
        T_global_globalned_back_.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));

        RCLCPP_INFO(this->get_logger(),
            "odom_to_baselink started: transforming IMU odometries to base_link in global_ned");
    }

private:
    void front_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        tf2::Transform T_ovglobal_imu;
        T_ovglobal_imu.setRotation(tf2::Quaternion(
            msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
        T_ovglobal_imu.setOrigin(tf2::Vector3(
            msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));

        tf2::Transform T_raw = T_ovglobal_imu * T_imu_front_base_;

        tf2::Transform T_global_base = T_global_globalned_front_ * T_raw;

        auto out = build_output_msg(msg, T_global_base, T_imu_front_base_);
        pub_front_->publish(out);
    }

    void back_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        tf2::Transform T_ovglobal_imu;
        T_ovglobal_imu.setRotation(tf2::Quaternion(
            msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
        T_ovglobal_imu.setOrigin(tf2::Vector3(
            msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));

        tf2::Transform T_raw = T_ovglobal_imu * T_imu_back_base_;

        tf2::Transform T_global_base = T_global_globalned_back_ * T_raw;

        auto out = build_output_msg(msg, T_global_base, T_imu_back_base_);
        pub_back_->publish(out);
    }

    nav_msgs::msg::Odometry build_output_msg(
        const nav_msgs::msg::Odometry::SharedPtr& msg,
        const tf2::Transform& T_global_base,
        const tf2::Transform& T_imu_base)
    {
        // Build output message
        nav_msgs::msg::Odometry out;
        out.header.stamp = msg->header.stamp;
        out.header.frame_id = "global";  // Tell EKF this is in the global frame
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

        // Copy and INFLATE covariance from input (if available)
        // VIO covariances are microscopically small. When fusing two VIOs,
        // we must inflate them so the EKF can find a smooth tradeoff without diverging.
        for (int i = 0; i < 36; ++i) {
            out.pose.covariance[i] = msg->pose.covariance[i] * 100.0;
        }

        // Transform twist (velocity) from imu frame to base_link frame
        // v_base = R_base_imu * v_imu
        tf2::Matrix3x3 R_base_imu = T_imu_base.inverse().getBasis();

        tf2::Vector3 v_lin(msg->twist.twist.linear.x,
                           msg->twist.twist.linear.y,
                           msg->twist.twist.linear.z);
        tf2::Vector3 v_ang(msg->twist.twist.angular.x,
                           msg->twist.twist.angular.y,
                           msg->twist.twist.angular.z);

        tf2::Vector3 v_lin_base = R_base_imu * v_lin;
        tf2::Vector3 v_ang_base = R_base_imu * v_ang;

        out.twist.twist.linear.x = v_lin_base.x();
        out.twist.twist.linear.y = v_lin_base.y();
        out.twist.twist.linear.z = v_lin_base.z();
        out.twist.twist.angular.x = v_ang_base.x();
        out.twist.twist.angular.y = v_ang_base.y();
        out.twist.twist.angular.z = v_ang_base.z();
        out.twist.covariance = msg->twist.covariance;

        return out;
    }

    // Static transforms
    tf2::Transform T_imu_front_base_;  
    tf2::Transform T_imu_back_base_;   
    tf2::Transform T_global_globalned_front_; 
    tf2::Transform T_global_globalned_back_;  

    // Publishers & Subscribers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_front_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_back_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_front_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_back_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomToBaselink>());
    rclcpp::shutdown();
    return 0;
}
