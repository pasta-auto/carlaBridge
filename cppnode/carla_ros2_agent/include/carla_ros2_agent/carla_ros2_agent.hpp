#ifndef CARLA_ROS2_AGENT_HPP
#define CARLA_ROS2_AGENT_HPP

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
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/hand_brake_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/hand_brake_report.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <tier4_vehicle_msgs/msg/actuation_status_stamped.hpp>
#include <tier4_external_api_msgs/msg/control_command_stamped.hpp>
#include <tier4_external_api_msgs/msg/gear_shift_stamped.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>

#include <tier4_vehicle_msgs/msg/actuation_command_stamped.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>

#include <carla/client/Sensor.h>

#include <carla/client/World.h>
#include <carla/client/Vehicle.h>

#include <carla_ros2_agent/autoware_ros2_agent.hpp>

#include <memory>
#include <thread>
#include <mutex>

#include "server/remote_process.hpp"

// #define DEBUG_MSG_PRINT

namespace CarlaAutowareRos2
{

class CarlaAgent : public rclcpp::Node
{
public:
  CarlaAgent(std::weak_ptr<AutowareAgent> autoware_agent_ptr);

  inline std::string get_agent_role_name()
  {
    return agent_role_name;
  }

  inline std::string get_carla_ip()
  {
    return carla_ip;
  }

  inline int get_carla_port()
  {
    return carla_port;
  }

  inline int get_frame_rate()
  {
    return frame_rate;
  }

  inline std::string get_spawn_objects_path()
  {
    return spawn_objects_path;
  }

  void set_world(std::weak_ptr<carla::client::World>);

  void sensors_check();
  void sensors_destroy();
  
  void carla_vehicle_status_update();

private:

  static constexpr double CARLA_IDLE_THROTTLE = 0.25;
  static constexpr double CARLA_IDLE_COMPRESSION = 1 - CARLA_IDLE_THROTTLE;

  double carla_throttle_compression = CARLA_IDLE_COMPRESSION;
  double carla_throttle_idle = CARLA_IDLE_THROTTLE;
  double carla_throttle_compression_inv = 1 / CARLA_IDLE_COMPRESSION;

  const static int16_t PASTA_HEADLIGHT_OFF = 0;
  const static int16_t PASTA_HEADLIGHT_POSITION = 1;
  const static int16_t PASTA_HEADLIGHT_LOW = 2;
  const static int16_t PASTA_HEADLIGHT_PASS = 4;

  inline double to_radians(double degrees)
  {
    return (degrees * M_PI) / 180.0;
  }

  inline double to_degrees(double radians)
  {
    return (radians * 180.0) / M_PI;
  }

  inline builtin_interfaces::msg::Time time_to_stamp(double time)
  {
    builtin_interfaces::msg::Time stamp;
    stamp.sec = (int)time;
    stamp.nanosec = (time - stamp.sec) * 1000000000;
    return stamp;
  }

  inline bool LightFlagEnabled(carla::rpc::VehicleLightState::LightState value, carla::rpc::VehicleLightState::LightState flag)
  {
    return static_cast<carla::rpc::VehicleLightState::flag_type>(value) & static_cast<carla::rpc::VehicleLightState::flag_type>(flag);
  }

  inline carla::rpc::VehicleLightState::LightState LightFlagSet(carla::rpc::VehicleLightState::LightState value, carla::rpc::VehicleLightState::LightState flag)
  {
    return static_cast<carla::rpc::VehicleLightState::LightState>(static_cast<carla::rpc::VehicleLightState::flag_type>(value) | static_cast<carla::rpc::VehicleLightState::flag_type>(flag));
  }

  inline carla::rpc::VehicleLightState::LightState LightFlagClear(carla::rpc::VehicleLightState::LightState value, carla::rpc::VehicleLightState::LightState flag)
  {
    return static_cast<carla::rpc::VehicleLightState::LightState>(static_cast<carla::rpc::VehicleLightState::flag_type>(value) & ~static_cast<carla::rpc::VehicleLightState::flag_type>(flag));
  }
 

  std::string spawn_objects_path;

  double current_timestamp;

  std::weak_ptr<AutowareAgent> autoware_agent_ptr;
  std::weak_ptr<carla::client::World> world_ptr;

  bool turn_left_enabled;
  bool turn_right_enabled;
  
  double carla_steer_angle_rad_unit;
  double carla_steer_angle_rad;
  double max_steer_angle;
  std::array<double, 200> steerCurve;
  int steerCurveMaxSpeed;

  std::vector<carla::geom::Vector2D> torqueCurve;

  float max_rpm;
  int currentGear = autoware_auto_vehicle_msgs::msg::GearCommand::NONE;
  int currentGearCmd = autoware_auto_vehicle_msgs::msg::GearCommand::NONE;
  float currentSpeed = 0;
  bool parkingHandbrakeCmd = true;
  bool pastaHandbrakeCmd = false;

  carla::client::Vehicle::LightState currentLightState;
  std::mutex lightLock;

  carla::client::Vehicle::Control currentControl;
  std::mutex controlLock;

  std::array<double, 3> m_llh;
  std::mutex llhLock;

  void camera_sensor_setup(carla::SharedPtr<carla::client::ActorList> actorList, carla::ActorId parent_id);
  void lidar_sensor_setup(carla::SharedPtr<carla::client::ActorList> actorList, carla::ActorId parent_id);
  void gnss_sensor_setup(carla::SharedPtr<carla::client::ActorList> actorList, carla::ActorId parent_id);
  void imu_sensor_setup(carla::SharedPtr<carla::client::ActorList> actorList, carla::ActorId parent_id);

  void carla_steer_update();
  void carla_lights_update();
  void carla_control_update();
  void carla_speedometer_update();
  void carla_autonomous_update();

  std::string agent_role_name;
  std::string carla_ip;
  int carla_port;
  int frame_rate;

  uint8_t control_mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;

  rclcpp::Subscription<tier4_external_api_msgs::msg::ControlCommandStamped>::SharedPtr external_control_cmd_sub_;
  rclcpp::Subscription<tier4_vehicle_msgs::msg::ActuationCommandStamped>::SharedPtr control_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr gear_cmd_sub_;
  rclcpp::Subscription<tier4_external_api_msgs::msg::GearShiftStamped>::SharedPtr external_gear_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr turn_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr hazard_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::HandBrakeCommand>::SharedPtr handbrake_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr headlights_cmd_sub_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr max_wheel_angle_pub_;

  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr auto_control_mode_report_sub_;

  void control_cmd_common(double accel, double brake, double steer);
  void control_cmd_callback(tier4_vehicle_msgs::msg::ActuationCommandStamped::UniquePtr msg);
  void external_control_cmd_callback(tier4_external_api_msgs::msg::ControlCommandStamped::UniquePtr msg);
  void gear_cmd_common(int gear);
  void gear_cmd_callback(autoware_auto_vehicle_msgs::msg::GearCommand::UniquePtr msg);
  void external_gear_cmd_callback(tier4_external_api_msgs::msg::GearShiftStamped::UniquePtr msg);
  void turn_cmd_callback(autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::UniquePtr msg);
  void hazard_cmd_callback(autoware_auto_vehicle_msgs::msg::HazardLightsCommand::UniquePtr msg);
  void handbrake_cmd_callback(autoware_auto_vehicle_msgs::msg::HandBrakeCommand::UniquePtr msg);
  void headlights_cmd_callback(std_msgs::msg::Int16::UniquePtr msg);
  void control_mod_callback(autoware_auto_vehicle_msgs::msg::ControlModeReport::UniquePtr msg);

  void publish_max_wheel_angle(float wheel_angle);
  
  carla::SharedPtr<carla::client::Sensor> imu_sensor;
  void imu_callback(carla::SharedPtr<carla::sensor::SensorData> data);

  carla::SharedPtr<carla::client::Sensor> gnss_sensor;
  void gnss_callback(carla::SharedPtr<carla::sensor::SensorData> data);

  carla::SharedPtr<carla::client::Sensor> lidar_sensor;
  void lidar_callback(carla::SharedPtr<carla::sensor::SensorData> data);

  carla::SharedPtr<carla::client::Vehicle> egoVehicle;

  carla::SharedPtr<carla::client::Sensor> camera_sensor;
  sensor_msgs::msg::CameraInfo camera_sensor_info_tmpl;
  void camera_callback(carla::SharedPtr<carla::sensor::SensorData> data);

  remote_process apiRPC; 
};
}  // namespace CarlaAutowareRos2

#endif
