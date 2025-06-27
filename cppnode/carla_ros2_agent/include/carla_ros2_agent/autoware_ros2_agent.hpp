#ifndef AUTOWARE_ROS2_AGENT_H
#define AUTOWARE_ROS2_AGENT_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/header.hpp>
#include <autoware_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/hand_brake_report.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <tier4_vehicle_msgs/msg/actuation_status_stamped.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include <carla_ros2_agent/autoware_ros2_agent.hpp>

#include <memory>
#include <thread>

namespace CarlaAutowareRos2
{

class AutowareAgent : public rclcpp::Node
{
public:
  AutowareAgent();

  void pub_camera(sensor_msgs::msg::Image::UniquePtr msg);
  void pub_camera_info(sensor_msgs::msg::CameraInfo::UniquePtr msg);
  void pub_lidar_loc(sensor_msgs::msg::PointCloud2::UniquePtr msg);
  void pub_nav_sat(sensor_msgs::msg::NavSatFix::UniquePtr msg);
  void pub_ecef_vel(geometry_msgs::msg::TwistWithCovarianceStamped::UniquePtr msg);
  void pub_imu(sensor_msgs::msg::Imu::UniquePtr msg);
  void pub_gnss_ins(autoware_sensing_msgs::msg::GnssInsOrientationStamped::UniquePtr msg);
  void pub_auto_steering_status(autoware_auto_vehicle_msgs::msg::SteeringReport::UniquePtr msg);
  void pub_auto_hazard_status(autoware_auto_vehicle_msgs::msg::HazardLightsReport::UniquePtr msg);
  void pub_auto_turning_status(autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::UniquePtr msg);
  void pub_auto_gear_report(autoware_auto_vehicle_msgs::msg::GearReport::UniquePtr msg);
  void pub_auto_control_mode_report(autoware_auto_vehicle_msgs::msg::ControlModeReport::UniquePtr msg);
  void pub_velocity_report(autoware_auto_vehicle_msgs::msg::VelocityReport::UniquePtr msg);
  void pub_actuation_status(tier4_vehicle_msgs::msg::ActuationStatusStamped::UniquePtr msg);
  void pub_handbrake_status(autoware_auto_vehicle_msgs::msg::HandBrakeReport::UniquePtr msg);
  void pub_headlights_status(std_msgs::msg::Int16::UniquePtr msg);
  void pub_twist(geometry_msgs::msg::TwistWithCovarianceStamped::UniquePtr msg);
  void pub_clock(rosgraph_msgs::msg::Clock::UniquePtr msg);

private:
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image, std::allocator<void>>> camera_pub;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo, std::allocator<void>>> camera_info_pub;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2, std::allocator<void>>> lidar_pub_loc;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::NavSatFix, std::allocator<void>>> nav_sat_pub;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped, std::allocator<void>>> ecef_vel_pub;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu, std::allocator<void>>> imu_pub;
  std::shared_ptr<rclcpp::Publisher<autoware_sensing_msgs::msg::GnssInsOrientationStamped, std::allocator<void>>> gnss_ins_pub;
  std::shared_ptr<rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport, std::allocator<void>>> auto_steering_status_pub;
  std::shared_ptr<rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::HazardLightsReport, std::allocator<void>>> auto_hazard_status_pub;
  std::shared_ptr<rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport, std::allocator<void>>> auto_turning_status_pub;
  std::shared_ptr<rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearReport, std::allocator<void>>> auto_gear_report_pub;
  // std::shared_ptr<rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport, std::allocator<void>>> auto_control_mode_report_pub;
  std::shared_ptr<rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport, std::allocator<void>>> velocity_report_pub;
  std::shared_ptr<rclcpp::Publisher<tier4_vehicle_msgs::msg::ActuationStatusStamped, std::allocator<void>>> actuation_status_pub;
  std::shared_ptr<rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::HandBrakeReport, std::allocator<void>>> handbrake_status_pub;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int16, std::allocator<void>>> headlights_status_pub;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped, std::allocator<void>>> twist_pub;

  std::shared_ptr<rclcpp::Publisher<rosgraph_msgs::msg::Clock, std::allocator<void>>> ros2_clock_pub;
};


}

#endif
