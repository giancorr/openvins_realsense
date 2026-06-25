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
        tf2::Quaternion q_front(0.5, 0.5, 0.5, -0.5);
        T_imu_front_base_.setRotation(q_front);
        T_imu_front_base_.setOrigin(tf2::Vector3(0.0, -0.15, -0.13));

        // --- Static transform: cam1 IMU -> base_link ---
        // Derived from: 15cm behind, 19cm below, yaw 180°, pitch 30° down
        tf2::Quaternion q_back(-0.35355339, 0.35355339, 0.61237244, 0.61237244);
        T_imu_back_base_.setRotation(q_back);
        T_imu_back_base_.setOrigin(tf2::Vector3(0.0, -0.089545, -0.224904));

        // --- Static transform: global -> global_ned ---
        tf2::Quaternion q_ned;
        q_ned.setRPY(M_PI, 0.0, 0.0);
        T_global_global_ned_.setRotation(q_ned);
        T_global_global_ned_.setOrigin(tf2::Vector3(0, 0, 0));

        RCLCPP_INFO(this->get_logger(), "OdomToBaselinkNode started. Listening to OpenVINS odometry...");
    }

private:
    void odom_front_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Process Odometry
        auto out_msg = transform_odometry(msg, T_imu_front_base_);
        odom_front_pub_->publish(out_msg);

        // Zeroing mechanism on the first message
        if (first_msg_front_) {
            initialize_odom_frame("front", out_msg);
            first_msg_front_ = false;
        }
    }

    void odom_back_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Process Odometry
        auto out_msg = transform_odometry(msg, T_imu_back_base_);
        odom_back_pub_->publish(out_msg);

        // Zeroing mechanism on the first message
        if (first_msg_back_) {
            initialize_odom_frame("back", out_msg);
            first_msg_back_ = false;
        }
    }

    void initialize_odom_frame(const std::string& session_type, const nav_msgs::msg::Odometry& out_msg)
    {
        // We use the first output Odometry (which is global -> base_link) to define the 'odom' frame
        // 'odom' represents the starting position and yaw of 'base_link' in the global_ned frame.
        
        tf2::Transform T_global_base;
        tf2::fromMsg(out_msg.pose.pose, T_global_base);

        tf2::Transform T_global_ned_base = T_global_global_ned_.inverse() * T_global_base;

        tf2::Matrix3x3 m(T_global_ned_base.getRotation());
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        tf2::Quaternion q_yaw;
        q_yaw.setRPY(0.0, 0.0, yaw);

        if (session_type == "front") {
            T_init_front_.setRotation(q_yaw);
            T_init_front_.setOrigin(T_global_ned_base.getOrigin());
            publish_static_odom(T_init_front_, out_msg.header.stamp);
            RCLCPP_INFO(this->get_logger(), "Zeroing complete for FRONT session.");
        } else {
            T_init_back_.setRotation(q_yaw);
            T_init_back_.setOrigin(T_global_ned_base.getOrigin());
            publish_static_odom(T_init_back_, out_msg.header.stamp);
            RCLCPP_INFO(this->get_logger(), "Zeroing complete for BACK session.");
        }
    }

    void publish_static_odom(const tf2::Transform& T_init, const builtin_interfaces::msg::Time& stamp)
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = stamp;
        t.header.frame_id = "global_ned";
        t.child_frame_id = "odom";
        
        t.transform.translation.x = T_init.getOrigin().x();
        t.transform.translation.y = T_init.getOrigin().y();
        t.transform.translation.z = T_init.getOrigin().z();
        
        t.transform.rotation.x = T_init.getRotation().x();
        t.transform.rotation.y = T_init.getRotation().y();
        t.transform.rotation.z = T_init.getRotation().z();
        t.transform.rotation.w = T_init.getRotation().w();
        
        tf_static_broadcaster_->sendTransform(t);
    }

    nav_msgs::msg::Odometry transform_odometry(const nav_msgs::msg::Odometry::SharedPtr msg, const tf2::Transform& T_imu_base)
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

        // Copy and inflate covariance (EKF tuning)
        for (int i = 0; i < 36; ++i) {
            out.pose.covariance[i] = msg->pose.covariance[i] * 100.0;
        }

        // Transform twist from imu frame to base_link frame
        tf2::Matrix3x3 R_base_imu = T_imu_base.inverse().getBasis();

        tf2::Vector3 v_lin(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
        tf2::Vector3 v_ang(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);

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

    // --- State Variables ---
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
    tf2::Transform T_global_global_ned_;

    // Zeroing
    bool first_msg_front_ = true;
    bool first_msg_back_ = true;
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
