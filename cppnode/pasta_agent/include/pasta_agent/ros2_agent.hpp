#ifndef CARLA_ROS2_AGENT_HPP
#define CARLA_ROS2_AGENT_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int16.hpp>

#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/hand_brake_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/hand_brake_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/headlights_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/headlights_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/hand_brake_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/control_mode_command.hpp>
#include <autoware_auto_vehicle_msgs/srv/control_mode_command.hpp>
#include <tier4_external_api_msgs/msg/control_command_stamped.hpp>
#include <tier4_external_api_msgs/msg/gear_shift_stamped.hpp>
#include <tier4_vehicle_msgs/msg/actuation_status_stamped.hpp>
#include <tier4_vehicle_msgs/msg/actuation_command_stamped.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>

#include <tier4_external_api_msgs/msg/control_command_stamped.hpp>
#include <tier4_external_api_msgs/msg/gear_shift_stamped.hpp>
#include <tier4_external_api_msgs/msg/turn_signal_stamped.hpp>
#include <tier4_external_api_msgs/msg/heartbeat.hpp>

#include <pasta_agent/gear_conversion.hpp>

#include <memory>
#include <thread>
#include <mutex>

class Ros2Agent : public rclcpp::Node
{

public:
  Ros2Agent();

  static std::shared_ptr<Ros2Agent> currentAgent;
  static std::shared_ptr<Ros2Agent> getAgent();
  static void createAgent();
  static void destroyAgent();

  std::string get_pasta_config();

  void pub_control_command(uint16_t breakCmd, uint16_t throttleCmd, int16_t steerCmd);
  void pub_shift_command(uint8_t data);
  void pub_turnsignal_command(uint8_t data);
  void pub_handbrake_command(bool handbrake);
  void pub_headlights_command(int headlight);
  void timer_periodic();
  void pub_controlmode();
  void pub_heartbeat();

  std::string get_pasta_ip();
  int get_pasta_port();
private:
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

  std::string pasta_ip;
  int pasta_port;

  static constexpr int PASTA_STEER_RANGE = 0x1FF;
  static constexpr double PASTA_STEER_UNIT = 1 / (double)PASTA_STEER_RANGE;

  static constexpr int PASTA_THROTTLE_RANGE = 0x3FF;
  static constexpr double PASTA_THORTTLE_UNIT = 1 / (double)PASTA_THROTTLE_RANGE;

  static constexpr int PASTA_BRAKE_RANGE = 0x3FF;
  static constexpr double PASTA_BRAKE_UNIT = 1 / (double)PASTA_BRAKE_RANGE;

  static constexpr int PASTA_RPM_RANGE = 0xFFFF; 
  static constexpr double PASTA_RPM_UNIT = 1;
  static constexpr int PASTA_RPM_THROTTLE_RANGE = 10000 * PASTA_RPM_UNIT;

  const static int PASTA_GEAR_LOWEST = Gears::PASTA_GEAR_PARK;
  const static int PASTA_GEAR_HIGHEST = Gears::PASTA_GEAR_DRIVE;

  std::string pasta_mappings_path;

  float max_steer_angle = 35.0;
  double max_steer_angle_rads = to_radians(max_steer_angle);
  double max_steer_angle_units = PASTA_STEER_RANGE / max_steer_angle;

  double currentSpeed;

  bool currentHandbrake = false;
  int currentPastaGear = Gears::PASTA_GEAR_NONE;

  std::mutex lightReportLock;
  int currentTurnSignal = 0;
  bool currentHazard = false;

  std::mutex lightCmdLock;
  int currentCmdTurnSignal = 0;
  bool currentCmdHazard = false;

  //Timers
  rclcpp::TimerBase::SharedPtr control_mode_timer_;

  //Parameters
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr max_wheel_angle_sub_;

  void max_wheel_angle_callback(std_msgs::msg::Float32::UniquePtr msg);

  //Reports
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr veloticy_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr gear_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr turn_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::HazardLightsReport>::SharedPtr hazard_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr steer_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::HandBrakeReport>::SharedPtr handbrake_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr headlights_sub_;
  rclcpp::Subscription<tier4_vehicle_msgs::msg::ActuationStatusStamped>::SharedPtr actuation_sub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr controlmode_pub_;

  void velocity_callback(autoware_auto_vehicle_msgs::msg::VelocityReport::UniquePtr msg);
  void gear_callback(autoware_auto_vehicle_msgs::msg::GearReport::UniquePtr msg);
  void turn_callback(autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::UniquePtr msg);
  void hazard_callback(autoware_auto_vehicle_msgs::msg::HazardLightsReport::UniquePtr msg);
  void steer_callback(autoware_auto_vehicle_msgs::msg::SteeringReport::UniquePtr msg);
  void handbrake_callback(autoware_auto_vehicle_msgs::msg::HandBrakeReport::UniquePtr msg);
  void headlights_callback(std_msgs::msg::Int16::UniquePtr msg);
  void actuation_callback(tier4_vehicle_msgs::msg::ActuationStatusStamped::UniquePtr msg);

  //Commands
  rclcpp::Subscription<tier4_vehicle_msgs::msg::ActuationCommandStamped>::SharedPtr control_cmd_sub_;
  rclcpp::Subscription<tier4_external_api_msgs::msg::ControlCommandStamped>::SharedPtr ext_control_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr gear_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr turn_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr hazard_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::HandBrakeCommand>::SharedPtr handbrake_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr headlights_cmd_sub_;

  rclcpp::Service<autoware_auto_vehicle_msgs::srv::ControlModeCommand>::SharedPtr controlMode_cmd_srv;

  void control_cmd_callback(tier4_vehicle_msgs::msg::ActuationCommandStamped::UniquePtr msg);
  void ext_control_cmd_callback(tier4_external_api_msgs::msg::ControlCommandStamped::UniquePtr msg);
  void gear_cmd_callback(autoware_auto_vehicle_msgs::msg::GearCommand::UniquePtr msg);
  void turn_cmd_callback(autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::UniquePtr msg);
  void hazard_cmd_callback(autoware_auto_vehicle_msgs::msg::HazardLightsCommand::UniquePtr msg);
  void handbrake_cmd_callback(autoware_auto_vehicle_msgs::msg::HandBrakeCommand::UniquePtr msg);
  void headlights_cmd_callback(std_msgs::msg::Int16::UniquePtr msg);
  void controlMode_srv_callback(const autoware_auto_vehicle_msgs::srv::ControlModeCommand_Request::SharedPtr request, autoware_auto_vehicle_msgs::srv::ControlModeCommand_Response::SharedPtr response);

  //Mode1 Ros2 inserts
  rclcpp::Publisher<tier4_external_api_msgs::msg::ControlCommandStamped>::SharedPtr ext_control_pub_;
  rclcpp::Publisher<tier4_external_api_msgs::msg::GearShiftStamped>::SharedPtr ext_shift_pub_;
  rclcpp::Publisher<tier4_external_api_msgs::msg::TurnSignalStamped>::SharedPtr ext_turnsignal_pub_;
  rclcpp::Publisher<tier4_external_api_msgs::msg::Heartbeat>::SharedPtr ext_heartbeat_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::HandBrakeCommand>::SharedPtr ext_handbrake_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr ext_headlights_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr ext_directgear_pub_;

  //Mode Control
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::OperationModeState>::SharedPtr autoware_mode_sub_;

  rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedPtr modechange_client_;

  void autoware_mode_callback(autoware_adapi_v1_msgs::msg::OperationModeState::UniquePtr msg);

  rclcpp::executors::SingleThreadedExecutor executor;
};

#endif