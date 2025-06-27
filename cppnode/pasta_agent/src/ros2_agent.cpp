#include <cstdio>
#include <cstdlib>

#include <memory>
#include <cmath>

#include <builtin_interfaces/msg/time.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <rclcpp/utilities.hpp>

#include <pasta_agent/ros2_agent.hpp>
#include <pasta_agent/pasta_interface.hpp>
#include <rclcpp/create_timer.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

std::shared_ptr<Ros2Agent> Ros2Agent::currentAgent;

Ros2Agent::Ros2Agent() : Node("pasta_agent", rclcpp::NodeOptions().use_intra_process_comms(true))
{
  // on_shutdown(std::bind(&Ros2Agent::sensors_destroy, this));

  this->set_parameter(rclcpp::Parameter("use_sim_time", true));
 
  pasta_mappings_path = this->declare_parameter("pasta_mappings", "../config/pastaIDs.json");

  //Parameters
  pasta_ip = this->declare_parameter("host", "172.25.240.1");
  pasta_port = (int)this->declare_parameter("port", 3000);
  std::string carla_vehicle_max_wheel_angle = this->declare_parameter("carla.max_wheel_angle_path", "/carla/max_wheel_angle");

  //Reports
  std::string velocity_path = this->declare_parameter("velocity_status", "/vehicle/status/velocity_status");
  std::string turn_indicators_path = this->declare_parameter("turn_indicators_status", "/vehicle/status/turn_indicators_status");
  std::string hazard_lights_path = this->declare_parameter("hazard_lights_status", "/vehicle/status/hazard_lights_status");
  std::string gear_path = this->declare_parameter("gear_status", "/vehicle/status/gear_status");
  std::string actuation_path = this->declare_parameter("actuation_status", "/vehicle/status/actuation_status");
  std::string steer_path = this->declare_parameter("steering_status", "/vehicle/status/steering_status");
  std::string handbrake_path = this->declare_parameter("handbrake_status", "/vehicle/status/handbrake_status");
  std::string headlights_path = this->declare_parameter("headlight_status", "/vehicle/status/headlights_status");
  std::string control_mode_status_path = this->declare_parameter("control_mode_status_path", "/vehicle/status/control_mode");

  //Commands
  std::string actuation_cmd_path = this->declare_parameter("actuation_cmd_path", "/control/command/actuation_cmd"); 
  std::string ext_actuation_cmd_path = this->declare_parameter("ext_actuation_cmd_path", "/api/external/get/command/selected/control");
  std::string turn_cmd_indicators_path = this->declare_parameter("turn_indicators_path", "/control/command/turn_indicators_cmd");
  std::string hazard_cmd_lights_path = this->declare_parameter("hazard_lights_path", "/control/command/hazard_lights_cmd");
  std::string gear_cmd_path = this->declare_parameter("gear_cmd_path", "/control/command/gear_cmd");
  std::string handbrake_cmd_path = this->declare_parameter("handbrake_cmd_path", "/control/command/handbrake_cmd");
  std::string headlights_cmd_path = this->declare_parameter("headlight_cmd_path", "/control/command/headlight_cmd");
  std::string control_mode_request_path = this->declare_parameter("control_mode_request", "/control/control_mode_request");

  //Mode1 Ros2 inserts
  std::string ext_control_path = this->declare_parameter("external_control_cmd", "/api/external/set/command/remote/control");
  std::string ext_shift_path = this->declare_parameter("external_shift_cmd", "/api/external/set/command/remote/shift");
  std::string ext_turnsignal_path = this->declare_parameter("external_turnsignal_cmd", "/api/external/set/command/remote/turn_signal");
  std::string ext_heartbeat_path = this->declare_parameter("external_heartbeat", "/api/external/set/command/remote/heartbeat");
  std::string ext_handbrake_path = this->declare_parameter("external_handbrake", "/control/command/handbrake_cmd");
  std::string ext_headlights_path = this->declare_parameter("external_headlights", "/control/command/headlights_cmd");

  std::string autoware_mode_path = this->declare_parameter("autoware_mode_status", "/system/operation_mode/state");

  rclcpp::QoS statusQoS = rclcpp::SensorDataQoS();
  statusQoS.reliability(rclcpp::ReliabilityPolicy::Reliable);

  rclcpp::QoS systemQoS = rclcpp::QoS(1);
  systemQoS.reliability(rclcpp::ReliabilityPolicy::Reliable);
  systemQoS.history(rclcpp::HistoryPolicy::KeepLast);

  systemQoS.liveliness(rclcpp::LivelinessPolicy::Automatic);

  //Parameters
  max_wheel_angle_sub_ = this->create_subscription<std_msgs::msg::Float32>(carla_vehicle_max_wheel_angle, rclcpp::ParametersQoS(), std::bind(&Ros2Agent::max_wheel_angle_callback, this, _1));

  //Reports
  veloticy_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>(
      velocity_path, statusQoS, std::bind(&Ros2Agent::velocity_callback, this, _1));

  gear_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::GearReport>(
      gear_path, statusQoS, std::bind(&Ros2Agent::gear_callback, this, _1));

  turn_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>(
      turn_indicators_path, statusQoS, std::bind(&Ros2Agent::turn_callback, this, _1));

  hazard_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::HazardLightsReport>(
      hazard_lights_path, statusQoS, std::bind(&Ros2Agent::hazard_callback, this, _1));

  steer_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>(
      steer_path, statusQoS, std::bind(&Ros2Agent::steer_callback, this, _1));

  actuation_sub_ = this->create_subscription<tier4_vehicle_msgs::msg::ActuationStatusStamped>(
      actuation_path, statusQoS, std::bind(&Ros2Agent::actuation_callback, this, _1));

  handbrake_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::HandBrakeReport>(
      handbrake_path, statusQoS, std::bind(&Ros2Agent::handbrake_callback, this, _1));

  headlights_sub_ = this->create_subscription<std_msgs::msg::Int16>(
      headlights_path, statusQoS, std::bind(&Ros2Agent::headlights_callback, this, _1));

  controlmode_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>(control_mode_status_path, statusQoS);

  //Commands
  control_cmd_sub_ = this->create_subscription<tier4_vehicle_msgs::msg::ActuationCommandStamped>(
      actuation_cmd_path, statusQoS, std::bind(&Ros2Agent::control_cmd_callback, this, _1));

  ext_control_cmd_sub_ = this->create_subscription<tier4_external_api_msgs::msg::ControlCommandStamped>(
      ext_actuation_cmd_path, statusQoS, std::bind(&Ros2Agent::ext_control_cmd_callback, this, _1));

  turn_cmd_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>(
      turn_cmd_indicators_path, statusQoS, std::bind(&Ros2Agent::turn_cmd_callback, this, _1));

  hazard_cmd_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>(
      hazard_cmd_lights_path, statusQoS, std::bind(&Ros2Agent::hazard_cmd_callback, this, _1));

  gear_cmd_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::GearCommand>(
      gear_cmd_path, statusQoS, std::bind(&Ros2Agent::gear_cmd_callback, this, _1));

  handbrake_cmd_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::HandBrakeCommand>(
      handbrake_cmd_path, statusQoS, std::bind(&Ros2Agent::handbrake_cmd_callback, this, _1));

  headlights_cmd_sub_ = this->create_subscription<std_msgs::msg::Int16>(
      headlights_cmd_path, statusQoS, std::bind(&Ros2Agent::headlights_cmd_callback, this, _1));

  // controlMode_cmd_srv = this->create_service<autoware_auto_vehicle_msgs::srv::ControlModeCommand>(control_mode_request_path, std::bind(&Ros2Agent::controlMode_srv_callback, this, _1, _2));

  //Mode1 Ros2 inserts
  ext_control_pub_ = this->create_publisher<tier4_external_api_msgs::msg::ControlCommandStamped>(ext_control_path, statusQoS);
  ext_shift_pub_ = this->create_publisher<tier4_external_api_msgs::msg::GearShiftStamped>(ext_shift_path, statusQoS);
  ext_turnsignal_pub_ = this->create_publisher<tier4_external_api_msgs::msg::TurnSignalStamped>(ext_turnsignal_path, statusQoS);
  ext_heartbeat_pub_ = this->create_publisher<tier4_external_api_msgs::msg::Heartbeat>(ext_heartbeat_path, statusQoS);
  ext_handbrake_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::HandBrakeCommand>(ext_handbrake_path, statusQoS);
  ext_headlights_pub_ = this->create_publisher<std_msgs::msg::Int16>(ext_headlights_path, statusQoS);
  ext_directgear_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::GearCommand>(gear_cmd_path, statusQoS);

  //Mode Control
  autoware_mode_sub_ = this->create_subscription<autoware_adapi_v1_msgs::msg::OperationModeState>(
      autoware_mode_path, systemQoS, std::bind(&Ros2Agent::autoware_mode_callback, this, _1));

  modechange_client_ = this->create_client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>("/api/operation_mode/disable_autoware_control");

  //Control mode timer
  control_mode_timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&Ros2Agent::timer_periodic, this));
}

std::shared_ptr<Ros2Agent> Ros2Agent::getAgent()
{
  return currentAgent;
}

void Ros2Agent::createAgent()
{
  currentAgent = std::make_shared<Ros2Agent>();
}

void Ros2Agent::destroyAgent()
{
  currentAgent.reset();
}

std::string Ros2Agent::get_pasta_ip()
{
  return pasta_ip;
}

int Ros2Agent::get_pasta_port()
{
  return pasta_port;
}

std::string Ros2Agent::get_pasta_config()
{
  return pasta_mappings_path;
}

void Ros2Agent::timer_periodic()
{
  //Send ignition report
  pasta_interface::getAgent()->set("ignitionRpt", 1);

  pub_controlmode();
}

void Ros2Agent::max_wheel_angle_callback(std_msgs::msg::Float32::UniquePtr msg)
{
  max_steer_angle = msg->data;
  max_steer_angle_rads = to_radians(max_steer_angle);
  max_steer_angle_units = PASTA_STEER_RANGE / max_steer_angle;
}

void Ros2Agent::velocity_callback(autoware_auto_vehicle_msgs::msg::VelocityReport::UniquePtr msg)
{
  auto pasta = pasta_interface::getAgent();

  if (!pasta) 
  {
    return; 
  }

  float speed = std::abs(msg->longitudinal_velocity) * 3.6;

  currentSpeed = speed;

  pasta->set("speedRpt", speed);
}

void Ros2Agent::gear_callback(autoware_auto_vehicle_msgs::msg::GearReport::UniquePtr msg)
{
  auto pasta = pasta_interface::getAgent();

  if (!pasta) 
  {
    return; 
  }

  //Set Can Updates
  currentPastaGear = Gears::autoware_2_pasta_gears(msg->report);

  pasta->set("gearRpt", currentPastaGear);
}

void Ros2Agent::handbrake_callback(autoware_auto_vehicle_msgs::msg::HandBrakeReport::UniquePtr msg)
{
  auto pasta = pasta_interface::getAgent();

  if (!pasta) 
  {
    return; 
  }

  currentHandbrake = msg->report;

  pasta->set("handbrakeRpt", currentHandbrake);
}

void Ros2Agent::headlights_callback(std_msgs::msg::Int16::UniquePtr msg)
{
  auto pasta = pasta_interface::getAgent();

  if (!pasta) 
  {
    return; 
  }

  int headlightsState = msg->data;

  pasta->set("frontLightsRpt", headlightsState);
}

void Ros2Agent::turn_callback(autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::UniquePtr msg)
{
  auto pasta = pasta_interface::getAgent();

  if (!pasta) 
  {
    return; 
  }

  {
    const std::lock_guard<std::mutex> lock(lightReportLock);
    
    switch(msg->report)
    {
      case autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::DISABLE:
        currentTurnSignal = PASTA_TURN_NONE;
        break;
      case autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_RIGHT:
        currentTurnSignal = PASTA_TURN_RIGHT;
        break;
      case autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_LEFT:
        currentTurnSignal = PASTA_TURN_LEFT;
      default:
      break;
    }

    int canLight = currentTurnSignal;
    
    if (currentHazard) 
    {
      canLight = PASTA_TURN_RPT_HAZARD;
    }

    pasta->set("turnLightsRpt", canLight);
  }
}

void Ros2Agent::hazard_callback(autoware_auto_vehicle_msgs::msg::HazardLightsReport::UniquePtr msg)
{
  auto pasta = pasta_interface::getAgent();

  if (!pasta) 
  {
    return; 
  }

  {
    const std::lock_guard<std::mutex> lock(lightReportLock);

    switch(msg->report)
    {
      case autoware_auto_vehicle_msgs::msg::HazardLightsReport::DISABLE:
        currentHazard = false;
        break;
      case autoware_auto_vehicle_msgs::msg::HazardLightsReport::ENABLE:
        currentHazard = true;
      default:
      break;
    }

    int canLight = currentTurnSignal;
    
    if (currentHazard) 
    {
      canLight = PASTA_TURN_RPT_HAZARD;
    }

    pasta->set("turnLightsRpt", canLight);
  }
}

void Ros2Agent::steer_callback(autoware_auto_vehicle_msgs::msg::SteeringReport::UniquePtr msg)
{
  auto pasta = pasta_interface::getAgent();

  if (!pasta) 
  {
    return; 
  }

  int pastaSteer = -std::trunc(to_degrees(msg->steering_tire_angle) * max_steer_angle_units);
  int pastaPowerSteer = pastaSteer;

  int pastaPowerTorque = 50;

  //Required to double output
  if (pasta->pasta_mode == PASTA_MODE_1)
  {
    if (currentPastaGear == Gears::PASTA_GEAR_REVERSE) 
    {
      pastaPowerSteer *= 2;
    }
    else if (currentPastaGear != Gears::PASTA_GEAR_DRIVE)
    {
      pastaPowerSteer = 0;
    } 

    pastaPowerTorque = std::clamp((int)std::trunc(currentSpeed), 0, 100);
  }
  
  pastaPowerSteer = std::clamp(pastaPowerSteer, -PASTA_STEER_RANGE, PASTA_STEER_RANGE);
  pastaSteer = std::clamp(pastaSteer, -PASTA_STEER_RANGE, PASTA_STEER_RANGE);

  pasta->set("steerRpt", pastaSteer);
  pasta->set("powerSteerRpt", pastaPowerSteer, pastaPowerTorque);
}

void Ros2Agent::actuation_callback(tier4_vehicle_msgs::msg::ActuationStatusStamped::UniquePtr msg)
{
  auto pasta = pasta_interface::getAgent();

  if (!pasta) 
  {
    return; 
  }

  pasta->set("rpmSpeedRpt", (int)(msg->status.accel_status * (double)PASTA_RPM_THROTTLE_RANGE), std::trunc(currentSpeed));

  pasta->set("throttleRpt", std::trunc(msg->status.accel_status * PASTA_THROTTLE_RANGE));
  pasta->set("brakeRpt", std::trunc(msg->status.brake_status * PASTA_BRAKE_RANGE));
}

void Ros2Agent::ext_control_cmd_callback(tier4_external_api_msgs::msg::ControlCommandStamped::UniquePtr msg)
{
  auto pasta = pasta_interface::getAgent();

  if (pasta->pasta_mode != PASTA_MODE_1)
  {
    return;
  }

  float throttle = msg->control.throttle;

  int pastaSteer = -std::trunc(to_degrees(msg->control.steering_angle) * max_steer_angle_units);

  if (!pasta) 
  {
    return; 
  }

  pasta->set("throttleCmd", std::trunc(throttle * PASTA_THROTTLE_RANGE));
  pasta->set("steerCmd", pastaSteer);
  pasta->set("brakeCmd", std::trunc(msg->control.brake * PASTA_BRAKE_RANGE));
}

void Ros2Agent::control_cmd_callback(tier4_vehicle_msgs::msg::ActuationCommandStamped::UniquePtr msg)
{
  auto pasta = pasta_interface::getAgent();

  if (pasta->pasta_mode == PASTA_MODE_1)
  {
    return;
  }

  float throttle = msg->actuation.accel_cmd;

  int pastaSteer = -std::trunc(to_degrees(msg->actuation.steer_cmd) * max_steer_angle_units);

  if (!pasta) 
  {
    return; 
  }

  pasta->set("throttleCmd", std::trunc(throttle * PASTA_THROTTLE_RANGE));
  pasta->set("steerCmd", pastaSteer);
  pasta->set("brakeCmd", std::trunc(msg->actuation.brake_cmd * PASTA_BRAKE_RANGE));
}

void Ros2Agent::gear_cmd_callback(autoware_auto_vehicle_msgs::msg::GearCommand::UniquePtr msg)
{
  static int expected_gear = Gears::PASTA_GEAR_NONE;

  auto gear = Gears::autoware_2_pasta_gears(msg->command);

  auto pasta = pasta_interface::getAgent();

  if (currentPastaGear == Gears::PASTA_GEAR_NONE)
  {
    return;
  }

  //Do nothing if the gear change had not been reported yet
  if (expected_gear != Gears::PASTA_GEAR_NONE && expected_gear != currentPastaGear)
  {
    return;
  }

  //Work out shift and new gear for possible handbrake
  int shiftCommand = PASTA_SHIFT_NONE;
  if (gear > currentPastaGear)
  {
    shiftCommand = PASTA_SHIFT_DOWN;

    expected_gear = currentPastaGear + 1;
    if (expected_gear > PASTA_GEAR_HIGHEST)
    {
      expected_gear = PASTA_GEAR_HIGHEST;
    }
  } else if (gear < currentPastaGear)
  {
    shiftCommand = PASTA_SHIFT_UP;

    expected_gear = currentPastaGear - 1;
    if (expected_gear < PASTA_GEAR_LOWEST)
    {
      expected_gear = PASTA_GEAR_LOWEST;
    }
  } else 
  {
    expected_gear = gear;
  }

  if (!pasta) 
  {
    return; 
  }

  pasta->set("ShiftCmd", shiftCommand);
}

void Ros2Agent::handbrake_cmd_callback(autoware_auto_vehicle_msgs::msg::HandBrakeCommand::UniquePtr msg)
{
  auto pasta = pasta_interface::getAgent();

  if (!pasta) 
  {
    return; 
  }

  pasta->set("handbrakeCmd",  msg->active);
}

void Ros2Agent::headlights_cmd_callback(std_msgs::msg::Int16::UniquePtr msg)
{
  auto pasta = pasta_interface::getAgent();

  if (!pasta) 
  {
    return; 
  }

  pasta->set("frontLightsCmd",  msg->data);
}

void Ros2Agent::turn_cmd_callback(autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::UniquePtr msg)
{
  using TurnIndicatorsCommand = autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand;

  auto pasta = pasta_interface::getAgent();

  if (!pasta) 
  {
    return; 
  }

  {
    const std::lock_guard<std::mutex> lock(lightCmdLock);

    switch(msg->command)
    {
      case TurnIndicatorsCommand::DISABLE:
        currentCmdTurnSignal = PASTA_TURN_NONE;
        break;

      case TurnIndicatorsCommand::ENABLE_LEFT:
        currentCmdTurnSignal = PASTA_TURN_LEFT;
        break;

      case TurnIndicatorsCommand::ENABLE_RIGHT:
        currentCmdTurnSignal = PASTA_TURN_RIGHT;
        break;

      case TurnIndicatorsCommand::NO_COMMAND:
      default:
        return;
    }

    int lights = currentCmdTurnSignal;

    if (currentCmdHazard)
    {
      lights = PASTA_TURN_CMD_HAZARD;
    }

    pasta->set("turnLightsCmd", lights);
  }
}

void Ros2Agent::hazard_cmd_callback(autoware_auto_vehicle_msgs::msg::HazardLightsCommand::UniquePtr msg)
{
  auto pasta = pasta_interface::getAgent();

  if (!pasta) 
  {
    return; 
  }

  {
    const std::lock_guard<std::mutex> lock(lightCmdLock);

    switch(msg->command)
    {
      case autoware_auto_vehicle_msgs::msg::HazardLightsCommand::DISABLE:
        currentHazard = false;
        break;
      case autoware_auto_vehicle_msgs::msg::HazardLightsCommand::ENABLE:
        currentHazard = true;
        break;
      case autoware_auto_vehicle_msgs::msg::HazardLightsCommand::NO_COMMAND:
      default:
        break;
    }

    int lights = currentTurnSignal;
    
    if (currentHazard) 
    {
      lights = PASTA_TURN_CMD_HAZARD;
    }

    pasta->set("turnLightsCmd", lights);
  }
}

void Ros2Agent::autoware_mode_callback(autoware_adapi_v1_msgs::msg::OperationModeState::UniquePtr msg)
{
  auto pasta = pasta_interface::getAgent();

  if (msg->is_in_transition)
  {
    return;
  }

  if (currentHandbrake)
  {
    
  }

  switch(msg->mode)
  {
    //TODO Fix mode switching, mode 6 displays may be broken
    case autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS:
      pasta->set("modeChangeCmd", PASTA_MODE_6);
      pasta->set_mode(PASTA_MODE_6);
      break;

    case autoware_adapi_v1_msgs::msg::OperationModeState::REMOTE:
      pasta->set("modeChangeCmd", PASTA_MODE_1);
      pasta->set_mode(PASTA_MODE_1);
      break;

    case autoware_adapi_v1_msgs::msg::OperationModeState::LOCAL:
      pasta->set("modeChangeCmd", PASTA_MODE_2);
      pasta->set_mode(PASTA_MODE_2);
      break;

    default:
      break;
  }

  pub_controlmode();
}

void Ros2Agent::pub_control_command(uint16_t breakCmd, uint16_t throttleCmd, int16_t steerCmd)
{
  tier4_external_api_msgs::msg::ControlCommandStamped::UniquePtr msg = std::make_unique<tier4_external_api_msgs::msg::ControlCommandStamped>();

  msg->stamp = this->now();
  
  //Clamp this to a double digit amount of max_steer_angle so 1.00,-0.01,0.0,0.01,1.00
  static constexpr double steer_percent_unit = -PASTA_STEER_UNIT * 100.0f;
  float raw_steer_angle = std::trunc((float)steerCmd * steer_percent_unit) / 100.0f;
  msg->control.steering_angle = raw_steer_angle * max_steer_angle_rads;
  
  msg->control.brake = (float)breakCmd * PASTA_BRAKE_UNIT;
  msg->control.throttle = (float)throttleCmd * PASTA_THORTTLE_UNIT;

  if (msg.get() != nullptr)
  {
    ext_control_pub_->publish(std::move(msg));
  }
}

void Ros2Agent::pub_shift_command(uint8_t data)
{
  tier4_external_api_msgs::msg::GearShiftStamped::UniquePtr msg = std::make_unique<tier4_external_api_msgs::msg::GearShiftStamped>();

  msg->stamp = this->now();

  uint8_t shiftedGear;

  switch(data)
  {
    case PASTA_SHIFT_UP:
      shiftedGear = currentPastaGear - 1;
      break;
    case PASTA_SHIFT_DOWN: 
      shiftedGear = currentPastaGear + 1;
      break;
    case PASTA_SHIFT_NONE:
    default:
      return;
      break;
  }

  //Do not send if out of range
  if (shiftedGear > tier4_external_api_msgs::msg::GearShift::DRIVE ||
      shiftedGear < tier4_external_api_msgs::msg::GearShift::PARKING)
  {
    std::cerr << "Invalid gear requested: " << (int)shiftedGear << std::endl;
    return; 
  }

  msg->gear_shift.data = shiftedGear;

  // //To fix stupid of ignoring neutral in external api inputs of autoware ....
  // if (shiftedGear == tier4_external_api_msgs::msg::GearShift::NEUTRAL)
  // {
  //   autoware_auto_vehicle_msgs::msg::GearCommand::UniquePtr gearMsg = std::make_unique<autoware_auto_vehicle_msgs::msg::GearCommand>();

  //   gearMsg->stamp = this->now();
  //   gearMsg->command = autoware_auto_vehicle_msgs::msg::GearCommand::NEUTRAL;

  //   ext_directgear_pub_->publish(std::move(gearMsg));
  // } else 

  std::cout << "Gear shift requested: " << (int)shiftedGear << std::endl;

  if (msg.get() != nullptr)
  {
    ext_shift_pub_->publish(std::move(msg));
  }
}

void Ros2Agent::pub_turnsignal_command(uint8_t data)
{
  tier4_external_api_msgs::msg::TurnSignalStamped::UniquePtr msg = std::make_unique<tier4_external_api_msgs::msg::TurnSignalStamped>();

  msg->stamp = this->now();

  switch(data)
  {
    case PASTA_TURN_NONE:
      msg->turn_signal.data = tier4_external_api_msgs::msg::TurnSignal::NONE;
      break;
    case PASTA_TURN_LEFT:
      msg->turn_signal.data = tier4_external_api_msgs::msg::TurnSignal::LEFT;
      break;
    case PASTA_TURN_RIGHT:
      msg->turn_signal.data = tier4_external_api_msgs::msg::TurnSignal::RIGHT;
      break;
    case PASTA_TURN_CMD_HAZARD:
      msg->turn_signal.data = tier4_external_api_msgs::msg::TurnSignal::HAZARD;
      break;
    default:
      return;
  }

  if (msg.get() != nullptr)
  {
    ext_turnsignal_pub_->publish(std::move(msg));
  }
}

void Ros2Agent::pub_handbrake_command(bool handbrake)
{
  autoware_auto_vehicle_msgs::msg::HandBrakeCommand::UniquePtr msg = std::make_unique<autoware_auto_vehicle_msgs::msg::HandBrakeCommand>();

  msg->stamp = this->now();
  msg->active = handbrake;

  if (msg.get() != nullptr)
  {
    ext_handbrake_pub_->publish(std::move(msg));
  }

  if (handbrake && (pasta_interface::getAgent()->pasta_mode != 1))
  {
    pasta_interface::getAgent()->set("modeChangeCmd", PASTA_MODE_1);
    pasta_interface::getAgent()->set_mode(PASTA_MODE_1);
    pub_controlmode();
  }
}

void Ros2Agent::pub_headlights_command(int headlightState)
{
  std_msgs::msg::Int16::UniquePtr msg = std::make_unique<std_msgs::msg::Int16>();

  msg->data = headlightState;
  
  if (msg.get() != nullptr)
  {
    ext_headlights_pub_->publish(std::move(msg));
  }
}

void Ros2Agent::pub_heartbeat()
{
  tier4_external_api_msgs::msg::Heartbeat::UniquePtr msg = std::make_unique<tier4_external_api_msgs::msg::Heartbeat>();

  msg->stamp = this->now();

  if (msg.get() != nullptr)
  {
    ext_heartbeat_pub_->publish(std::move(msg));
  }
}

void Ros2Agent::pub_controlmode()
{
  autoware_auto_vehicle_msgs::msg::ControlModeReport::UniquePtr msg = std::make_unique<autoware_auto_vehicle_msgs::msg::ControlModeReport>();

  msg->stamp = this->now();

  if (pasta_interface::getAgent()->pasta_mode == 1)
  {
    msg->mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::MANUAL;
  }
  else 
  {
    msg->mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
  }

  if (msg.get() != nullptr)
  {
    controlmode_pub_->publish(std::move(msg));
  }
}

void Ros2Agent::controlMode_srv_callback(const autoware_auto_vehicle_msgs::srv::ControlModeCommand_Request::SharedPtr request, autoware_auto_vehicle_msgs::srv::ControlModeCommand_Response::SharedPtr response)
{
  auto pasta = pasta_interface::getAgent();

  switch(request->mode)
  {
    //TODO Fix mode switching, mode 6 displays may be broken
    case autoware_auto_vehicle_msgs::srv::ControlModeCommand_Request::AUTONOMOUS:
      pasta->set("modeChangeCmd", PASTA_MODE_6);
      pasta->set_mode(PASTA_MODE_6);
      response->success = true;
      break;

    case autoware_auto_vehicle_msgs::srv::ControlModeCommand_Request::MANUAL:
      pasta->set("modeChangeCmd", PASTA_MODE_1);
      pasta->set_mode(PASTA_MODE_1);
      response->success = true;
      break;

    case autoware_auto_vehicle_msgs::srv::ControlModeCommand_Request::NO_COMMAND:
      response->success = true;
      break;

    default:
      // pasta->set("modeChangeCmd", PASTA_MODE_2);
      // pasta->set_mode(PASTA_MODE_6);
      // pub_controlmode(false);
      response->success = false;
      break;
  }

  pub_controlmode();
}