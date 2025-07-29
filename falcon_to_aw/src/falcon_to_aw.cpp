#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "autoware_vehicle_msgs/msg/velocity_report.hpp"
#include "autoware_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_vehicle_msgs/msg/control_mode_report.hpp"
#include "autoware_vehicle_msgs/msg/gear_report.hpp"
#include "autoware_vehicle_msgs/msg/hazard_lights_report.hpp"
#include "autoware_vehicle_msgs/msg/turn_indicators_report.hpp"
#include "autoware_control_msgs/msg/control.hpp"
#include "std_msgs/msg/header.hpp"

class FalconToAutoware : public rclcpp::Node
{
public:
  FalconToAutoware()
  : Node("falcon_to_autoware")
  {
    using std::placeholders::_1;

    // Subscriber to Twist messages
    twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/sensing/imu/imu_raw/twist", 10,
      std::bind(&FalconToAutoware::twistCallback, this, _1));
    cmd_sub_ = this->create_subscription<autoware_control_msgs::msg::Control>(
      "/control/command/control_cmd", 10,
      std::bind(&FalconToAutoware::steeringCallback, this, _1));
    wheel_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/vehicle_status/falcon/steering_status", 10,
      std::bind(&FalconToAutoware::wheelPoseCallback, this, _1));

    // Publisher for messages
    velocity_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::VelocityReport>(
      "/vehicle/status/velocity_status", 10);
    steering_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::SteeringReport>(
      "/vehicle/status/steering_status", 10);
    twist_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/control/command/twist_cmd", 10);
    
    control_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>(
      "/vehicle/status/control_mode", 10);
    gear_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::GearReport>(
      "/vehicle/status/gear_status", 10);
    hazard_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::HazardLightsReport>(
      "/vehicle/status/hazard_lights_status", 10);
    turn_indicators_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>(
      "/vehicle/status/turn_indicators_status", 10);
  }

private:
  void twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    autoware_vehicle_msgs::msg::VelocityReport velocity_msg;

    velocity_msg.header.stamp = msg->header.stamp;
    velocity_msg.header.frame_id = "base_link";

    velocity_msg.longitudinal_velocity = static_cast<float>(msg->twist.linear.x);
    velocity_msg.lateral_velocity = static_cast<float>(msg->twist.linear.y);
    velocity_msg.heading_rate = static_cast<float>(msg->twist.angular.z);

    // Publish the message
    velocity_pub_->publish(velocity_msg);
  }

  void steeringCallback(const autoware_control_msgs::msg::Control msg)
  {
    geometry_msgs::msg::Twist twist_cmd;
    twist_cmd.linear.x = msg.longitudinal.acceleration;
    twist_cmd.angular.z = msg.lateral.steering_tire_angle;

    twist_cmd_pub_->publish(twist_cmd);
  }

  void wheelPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    autoware_vehicle_msgs::msg::SteeringReport steering_msg;
    steering_msg.stamp = msg->header.stamp;
    steering_msg.steering_tire_angle = msg->pose.orientation.z;
    steering_pub_->publish(steering_msg);

    autoware_vehicle_msgs::msg::ControlModeReport control_msg;
    control_msg.stamp = msg->header.stamp;
    control_msg.mode = autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
    control_pub_->publish(control_msg);

    autoware_vehicle_msgs::msg::GearReport gear_msg;
    gear_msg.stamp = msg->header.stamp;
    gear_msg.report = autoware_vehicle_msgs::msg::GearReport::DRIVE;
    gear_pub_->publish(gear_msg);

    autoware_vehicle_msgs::msg::HazardLightsReport hazard_msg;
    hazard_msg.stamp = msg->header.stamp;
    hazard_msg.report = autoware_vehicle_msgs::msg::HazardLightsReport::DISABLE;
    hazard_pub_->publish(hazard_msg);

    autoware_vehicle_msgs::msg::TurnIndicatorsReport turn_indicators_msg;
    turn_indicators_msg.stamp = msg->header.stamp;
    turn_indicators_msg.report = autoware_vehicle_msgs::msg::TurnIndicatorsReport::DISABLE;
    turn_indicators_pub_->publish(turn_indicators_msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr cmd_sub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_cmd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr wheel_pose_sub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr control_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::GearReport>::SharedPtr gear_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::HazardLightsReport>::SharedPtr hazard_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr turn_indicators_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FalconToAutoware>());
  rclcpp::shutdown();
  return 0;
}
