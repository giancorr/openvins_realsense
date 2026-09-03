#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

class VioBridge : public rclcpp::Node {
public:
    VioBridge() : Node("simple_vio_bridge") {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10,
            std::bind(&VioBridge::odom_callback, this, std::placeholders::_1)
        );

        px4_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
            "/fmu/in/vehicle_visual_odometry", 10
        );

        RCLCPP_INFO(this->get_logger(), "Simple VIO Bridge (C++) avviato. In attesa di odometria...");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        px4_msgs::msg::VehicleOdometry px4_msg;

        // Timestamp: microseconds
        uint64_t timestamp = this->get_clock()->now().nanoseconds() / 1000ULL;
        px4_msg.timestamp = timestamp;
        px4_msg.timestamp_sample = timestamp;

        px4_msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;

        px4_msg.position[0] = msg->pose.pose.position.x;
        px4_msg.position[1] = msg->pose.pose.position.y;
        px4_msg.position[2] = msg->pose.pose.position.z;

        px4_msg.q[0] = msg->pose.pose.orientation.w;
        px4_msg.q[1] = msg->pose.pose.orientation.x;
        px4_msg.q[2] = msg->pose.pose.orientation.y;
        px4_msg.q[3] = msg->pose.pose.orientation.z;

        px4_msg.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD;
        px4_msg.velocity[0] = msg->twist.twist.linear.x;
        px4_msg.velocity[1] = msg->twist.twist.linear.y;
        px4_msg.velocity[2] = msg->twist.twist.linear.z;

        px4_msg.angular_velocity[0] = msg->twist.twist.angular.x;
        px4_msg.angular_velocity[1] = msg->twist.twist.angular.y;
        px4_msg.angular_velocity[2] = msg->twist.twist.angular.z;

        px4_msg.position_variance = {0.1f, 0.1f, 0.1f};
        px4_msg.orientation_variance = {0.1f, 0.1f, 0.1f};
        px4_msg.velocity_variance = {0.1f, 0.1f, 0.1f};

        px4_pub_->publish(px4_msg);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_pub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VioBridge>());
    rclcpp::shutdown();
    return 0;
}
