#include <cstdio>
#include <cstdlib>

#include <memory>
#include <cmath>

#include <carla_ros2_agent/carla_ros2_agent.hpp>
#include <carla/client/Actor.h>
#include <carla/client/ActorList.h>
#include <carla/client/Sensor.h>
#include <carla/rpc/ActorId.h>

#include <builtin_interfaces/msg/time.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <carla/sensor/data/Image.h>
#include <carla/sensor/data/LidarMeasurement.h>
#include <carla/sensor/data/GnssMeasurement.h>
#include <carla/sensor/data/IMUMeasurement.h>

#include <carla_ros2_agent/gear_conversion.hpp>

#include <tier4_external_api_msgs/msg/gear_shift.hpp>

#include <rclcpp/utilities.hpp>

#include <GeographicLib/Geocentric.hpp>
#include <eigen3/Eigen/StdVector>

using namespace std::chrono_literals;
using std::placeholders::_1;

using CAgent = CarlaAutowareRos2::CarlaAgent;
using LightState = carla::rpc::VehicleLightState::LightState;

namespace
{
  void enu2xyz_vel(double enu_vel[3], double ecef_base_pos[3], double xyz_vel[3])
  {
    using namespace GeographicLib;
    Geocentric earth(Constants::WGS84_a(), Constants::WGS84_f());

    std::vector<double> rotation(9);
    double llh[3];
    earth.Reverse(ecef_base_pos[0], ecef_base_pos[1], ecef_base_pos[2], llh[0], llh[1], llh[2], rotation);

    Eigen::Matrix3d R(rotation.data());
    Eigen::Vector3d v_enu(enu_vel);

    Eigen::Map<Eigen::Vector3d> v_xyz(xyz_vel);
    v_xyz = R.transpose() * v_enu;
  }
}  // namespace

CAgent::CarlaAgent(std::weak_ptr<AutowareAgent> autoware_agent_ptr)
  : Node("bridge_carla_agent", rclcpp::NodeOptions().use_intra_process_comms(true))
{
  on_shutdown(std::bind(&CarlaAgent::sensors_destroy, this));

  this->set_parameter(rclcpp::Parameter("use_sim_time", true));
 
  carla_ip = this->declare_parameter("host", "172.25.240.1");
  carla_port = (int)this->declare_parameter("port", 2000);
  frame_rate = (int)this->declare_parameter("frame_rate", 20);
  agent_role_name = this->declare_parameter("ego_vehicle_role_name", "heroAW");
  spawn_objects_path = this->declare_parameter("spawn_objects", "/autoware/carlaBridge/cppnode/carla_ros2_agent/config/objects.json");

  std::string external_actuation_cmd_path = this->declare_parameter("carla.ext_actuation_cmd_path", "/api/external/get/command/selected/control");
  std::string actuation_cmd_path = this->declare_parameter("carla.actuation_cmd_path", "/control/command/actuation_cmd");
  std::string turn_indicators_path = this->declare_parameter("carla.turn_indicators_path", "/control/command/turn_indicators_cmd");
  std::string hazard_lights_path = this->declare_parameter("carla.hazard_lights_path", "/control/command/hazard_lights_cmd");
  std::string external_gear_cmd_path = this->declare_parameter("carla.ext_gear_cmd_path", "/api/external/set/command/remote/shift");
  std::string gear_cmd_path = this->declare_parameter("carla.gear_cmd_path", "/control/command/gear_cmd");
  std::string handbrake_cmd_path = this->declare_parameter("carla.handbrake_cmd_path", "/control/command/handbrake_cmd");
  std::string headlights_cmd_path = this->declare_parameter("carla.headlights_cmd_path", "/control/command/headlights_cmd");

  bool pasta_throttle_compression = this->declare_parameter("carla.pasta_throttle_compression", true);
 
  std::string control_mode_status_path = this->declare_parameter("autoware.control_mode_status_path", "/vehicle/status/control_mode");

  std::string carla_vehicle_max_wheel_angle = this->declare_parameter("carla.max_wheel_angle_path", "/carla/max_wheel_angle");

  rclcpp::QoS statusQoS = rclcpp::SensorDataQoS();
  statusQoS.reliability(rclcpp::ReliabilityPolicy::Reliable);

  rclcpp::QoS stableQoS = rclcpp::ParametersQoS();

  external_control_cmd_sub_ = this->create_subscription<tier4_external_api_msgs::msg::ControlCommandStamped>(
      external_actuation_cmd_path, statusQoS, std::bind(&CarlaAgent::external_control_cmd_callback, this, _1));

  control_cmd_sub_ = this->create_subscription<tier4_vehicle_msgs::msg::ActuationCommandStamped>(
      actuation_cmd_path, statusQoS, std::bind(&CarlaAgent::control_cmd_callback, this, _1));

  turn_cmd_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>(
      turn_indicators_path, statusQoS, std::bind(&CarlaAgent::turn_cmd_callback, this, _1));

  hazard_cmd_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>(
      hazard_lights_path, statusQoS, std::bind(&CarlaAgent::hazard_cmd_callback, this, _1));

  gear_cmd_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::GearCommand>(
      gear_cmd_path, statusQoS, std::bind(&CarlaAgent::gear_cmd_callback, this, _1));

  external_gear_cmd_sub_ = this->create_subscription<tier4_external_api_msgs::msg::GearShiftStamped>(
      external_gear_cmd_path, statusQoS, std::bind(&CarlaAgent::external_gear_cmd_callback, this, _1));

  handbrake_cmd_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::HandBrakeCommand>(
      handbrake_cmd_path, statusQoS, std::bind(&CarlaAgent::handbrake_cmd_callback, this, _1));

  headlights_cmd_sub_ = this->create_subscription<std_msgs::msg::Int16>(
      headlights_cmd_path, statusQoS, std::bind(&CarlaAgent::headlights_cmd_callback, this, _1));

  auto_control_mode_report_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::ControlModeReport>(
      control_mode_status_path, statusQoS, std::bind(&CarlaAgent::control_mod_callback, this, _1));

  max_wheel_angle_pub_ = this->create_publisher<std_msgs::msg::Float32>(carla_vehicle_max_wheel_angle, stableQoS);

  this->autoware_agent_ptr = autoware_agent_ptr;

  if (pasta_throttle_compression)
  {
    carla_throttle_compression = CARLA_IDLE_COMPRESSION;
    carla_throttle_compression_inv = 1 / CARLA_IDLE_COMPRESSION;
    carla_throttle_idle = CARLA_IDLE_THROTTLE;
  } else {
    carla_throttle_compression = 1.0;
    carla_throttle_compression_inv = 1.0;
    carla_throttle_idle = 0.0;
  }

}

void CAgent::publish_max_wheel_angle(float angle)
{
  std_msgs::msg::Float32::UniquePtr msg = std::make_unique<std_msgs::msg::Float32>();

  msg->data = angle;

  max_wheel_angle_pub_->publish(std::move(msg));
}

void CAgent::control_mod_callback(autoware_auto_vehicle_msgs::msg::ControlModeReport::UniquePtr msg)
{
  control_mode = msg->mode;
}

void CAgent::control_cmd_callback(tier4_vehicle_msgs::msg::ActuationCommandStamped::UniquePtr msg)
{
  if (control_mode == autoware_auto_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS)
  {
    control_cmd_common(msg->actuation.accel_cmd, msg->actuation.brake_cmd, msg->actuation.steer_cmd);
  }
}

void CAgent::external_control_cmd_callback(tier4_external_api_msgs::msg::ControlCommandStamped::UniquePtr msg)
{
  if (control_mode == autoware_auto_vehicle_msgs::msg::ControlModeReport::MANUAL)
  {
    control_cmd_common(msg->control.throttle, msg->control.brake, msg->control.steering_angle);
  }
}

void CAgent::control_cmd_common(double accel, double brake, double steer)
{
  if (!egoVehicle)
  {
    //some error msg here or warning
    return;
  }

  bool light_brake = false;

  {
    const std::lock_guard<std::mutex> lock(controlLock);

    currentControl.throttle = (accel * carla_throttle_compression) + carla_throttle_idle;

    currentControl.brake = brake;
    
    //Clamp the steering at max    
    double steer_cmd = -steer * carla_steer_angle_rad_unit;
    currentControl.steer = std::clamp(steer_cmd, -0.5, 0.5); //Clamped at 0.5 due to max steer range that should exist on each tire

    egoVehicle->ApplyControl(currentControl);

    light_brake = (currentControl.brake > 0.1);
  }

  //set brake light accordingly
  if (light_brake)
  {
    const std::lock_guard<std::mutex> lock(lightLock);
    if (!LightFlagEnabled(currentLightState, LightState::Brake))
    {
      currentLightState = LightFlagSet(currentLightState, LightState::Brake);
      egoVehicle->SetLightState(currentLightState);
    }
  }
  else 
  {
    const std::lock_guard<std::mutex> lock(lightLock);
    if (LightFlagEnabled(currentLightState, LightState::Brake))
    {
      currentLightState = LightFlagClear(currentLightState, LightState::Brake);
      egoVehicle->SetLightState(currentLightState);
    }
  }
}

void CAgent::external_gear_cmd_callback(tier4_external_api_msgs::msg::GearShiftStamped::UniquePtr msg)
{
  if (control_mode != autoware_auto_vehicle_msgs::msg::ControlModeReport::MANUAL)
  {
    return;
  }

  if (!egoVehicle)
  {
    //some error msg here or warning
    return;
  }

  //No command to process
  if (msg->gear_shift.data == tier4_external_api_msgs::msg::GearShift::NONE)
  {
    return;
  }

  int newGear = Gears::tier4_external_api_2_autoware_gears(msg->gear_shift.data);

  gear_cmd_common(newGear);
}

void CAgent::gear_cmd_callback(autoware_auto_vehicle_msgs::msg::GearCommand::UniquePtr msg)
{
  if (control_mode == autoware_auto_vehicle_msgs::msg::ControlModeReport::MANUAL)
  {
    return;
  }

  if (!egoVehicle)
  {
    //some error msg here or warning
    return;
  }

  //No command to process
  if (msg->command == autoware_auto_vehicle_msgs::msg::GearCommand::NONE)
  {
    return;
  }

  gear_cmd_common(msg->command);
}

void CAgent::gear_cmd_common(int newGear)
{
  auto pastaGear = Gears::autoware_2_pasta_gears(newGear);
  auto currentPastaGear = Gears::autoware_2_pasta_gears(currentGear);

  if (currentGear == autoware_auto_vehicle_msgs::msg::GearCommand::NONE)
  {
    std::cerr << "Carla gear is in unknown state" << std::endl;
    return;
  }

  if (currentGearCmd == autoware_auto_vehicle_msgs::msg::GearCommand::NONE || newGear != currentGear)
  {
    if (pastaGear > currentPastaGear)
    {
      int newPastaGear = currentPastaGear + 1;
      if (newPastaGear > Gears::PASTA_GEAR_DRIVE)
      {
        newPastaGear = Gears::PASTA_GEAR_DRIVE;
      }
      currentGearCmd = Gears::pasta_2_autoware_gears(newPastaGear);
    } else if (pastaGear < currentPastaGear)
    {
      int newPastaGear = currentPastaGear - 1;
      if (newPastaGear < Gears::PASTA_GEAR_PARK)
      {
        newPastaGear = Gears::PASTA_GEAR_PARK;
      }
      currentGearCmd = Gears::pasta_2_autoware_gears(newPastaGear);
    } else 
    {
      //Nothing to change for gears
      return;
    }

    std::cout << "Current Gear Cmd" << currentGearCmd << std::endl;
  }

  auto [gear, reverse, hand_brake] = Gears::autoware_2_carla_gears(currentGearCmd);
  {
    const std::lock_guard<std::mutex> lock(controlLock);

    //Deal with handbrake and parking
    parkingHandbrakeCmd = hand_brake;

    // if (g == Gears::CARLA_GEAR_NEUTRAL::gear && currentPastaGear == Gears::PASTA_GEAR_NEUTRAL)
    // {
    //   currentControl.brake = 0.40;
    // }

    // if (currentGear == Gears::PASTA_GEAR_DRIVE && gear != Gears::CARLA_GEAR_DRIVE::gear && currentControl.throttle == 0.0)
    // {
    //   currentControl.throttle = 0.01;
    // }

    
    // if (currentGear != currentGearCmd)
    // {
    //   currentControl.throttle += 0.05;
    // }

    currentControl.gear = gear;
    currentControl.reverse = reverse;
    currentControl.hand_brake = parkingHandbrakeCmd || pastaHandbrakeCmd;

    if (gear == Gears::CARLA_GEAR_NEUTRAL::gear) 
    {
      currentControl.manual_gear_shift = true;
    } else
    {
      currentControl.manual_gear_shift = false;
    }

    // egoVehicle->ApplyControl(currentControl);
  }

  //Apply reverse break as required
  if (currentControl.reverse)
  {
    if (!LightFlagEnabled(currentLightState, LightState::Reverse))
    {
      const std::lock_guard<std::mutex> lock(lightLock);
      
      currentLightState = LightFlagSet(currentLightState, LightState::Reverse);
      egoVehicle->SetLightState(currentLightState);
    }
  }
  else 
  {
    if (LightFlagEnabled(currentLightState, LightState::Reverse))
    {
      const std::lock_guard<std::mutex> lock(lightLock);

      currentLightState = LightFlagClear(currentLightState, LightState::Reverse);
      egoVehicle->SetLightState(currentLightState);
    }
  }
}

void CAgent::turn_cmd_callback(autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::UniquePtr msg)
{
  using TurnIndicatorsCommand = autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand;

  if (!egoVehicle)
  {
    //some error msg here or warning
    return;
  }

  switch(msg->command)
  {
    case TurnIndicatorsCommand::NO_COMMAND:
    case TurnIndicatorsCommand::DISABLE:
      turn_left_enabled = false;
      turn_right_enabled = false;
      break;

    case TurnIndicatorsCommand::ENABLE_LEFT:
      turn_left_enabled = true;
      turn_right_enabled = false;
      break;

    case TurnIndicatorsCommand::ENABLE_RIGHT:
      turn_left_enabled = false;
      turn_right_enabled = true;
      break;

    default:
      return;
  }

  const std::lock_guard<std::mutex> lock(lightLock);
  if (!(LightFlagEnabled(currentLightState, LightState::RightBlinker) && LightFlagEnabled(currentLightState, LightState::LeftBlinker)))
  {
    if (turn_left_enabled)
    {
      currentLightState = LightFlagSet(currentLightState, LightState::LeftBlinker);
    }
    else 
    {
      currentLightState = LightFlagClear(currentLightState, LightState::LeftBlinker);
    }

    if (turn_right_enabled)
    {
      currentLightState = LightFlagSet(currentLightState, LightState::RightBlinker);
    }
    else 
    {
      currentLightState = LightFlagClear(currentLightState, LightState::RightBlinker);
    }

    egoVehicle->SetLightState(currentLightState);
  }
}

void CAgent::hazard_cmd_callback(autoware_auto_vehicle_msgs::msg::HazardLightsCommand::UniquePtr msg)
{
  using HazardLightsCommand = autoware_auto_vehicle_msgs::msg::HazardLightsCommand;

  if (!egoVehicle)
  {
    //some error msg here or warning
    return;
  }

  const std::lock_guard<std::mutex> lock(lightLock);
  
  switch(msg->command)
  {
    case HazardLightsCommand::NO_COMMAND:
    case HazardLightsCommand::DISABLE:
      //Check if hazard is enabled or not, do nothing if not enabled
      if (LightFlagEnabled(currentLightState, LightState::RightBlinker) && LightFlagEnabled(currentLightState, LightState::LeftBlinker))
      {
        if (turn_left_enabled)
        {
          currentLightState = LightFlagSet(currentLightState, LightState::LeftBlinker);
        }
        else 
        {
          currentLightState = LightFlagClear(currentLightState, LightState::LeftBlinker);
        }

        if (turn_right_enabled)
        {
          currentLightState = LightFlagSet(currentLightState, LightState::RightBlinker);
        }
        else 
        {
          currentLightState = LightFlagClear(currentLightState, LightState::RightBlinker);
        }        
      }
      break;

    case HazardLightsCommand::ENABLE:
      currentLightState = LightFlagSet(currentLightState, LightState::LeftBlinker);
      currentLightState = LightFlagSet(currentLightState, LightState::RightBlinker);
      break;

    default:
      break;
  }

  egoVehicle->SetLightState(currentLightState);
}

void CAgent::handbrake_cmd_callback(autoware_auto_vehicle_msgs::msg::HandBrakeCommand::UniquePtr msg)
{
  if (!egoVehicle)
  {
    //some error msg here or warning
    return;
  }

  const std::lock_guard<std::mutex> lock(controlLock);

  pastaHandbrakeCmd = msg->active;
  
  currentControl.hand_brake = pastaHandbrakeCmd || parkingHandbrakeCmd;

  egoVehicle->ApplyControl(currentControl);
}

void CAgent::headlights_cmd_callback(std_msgs::msg::Int16::UniquePtr msg)
{
   if (!egoVehicle)
  {
    //some error msg here or warning
    return;
  }

  const std::lock_guard<std::mutex> lock(lightLock);

  if (msg->data & PASTA_HEADLIGHT_POSITION)
  {
    currentLightState = LightFlagSet(currentLightState, LightState::Position);
  } else 
  {
    currentLightState = LightFlagClear(currentLightState, LightState::Position);
  }

  if (msg->data & PASTA_HEADLIGHT_LOW)
  {
    currentLightState = LightFlagSet(currentLightState, LightState::LowBeam);
  } else 
  {
    currentLightState = LightFlagClear(currentLightState, LightState::LowBeam);
  }

  if (msg->data & PASTA_HEADLIGHT_PASS)
  {
    currentLightState = LightFlagSet(currentLightState, LightState::HighBeam);
  } else 
  {
    currentLightState = LightFlagClear(currentLightState, LightState::HighBeam);
  }
}

void CAgent::carla_vehicle_status_update()
{
  if (auto world = world_ptr.lock())
  {
    auto world_snapshot = world->GetSnapshot();
    current_timestamp = world_snapshot.GetTimestamp().elapsed_seconds;
  }
  
  carla_control_update();
  carla_lights_update();
  carla_steer_update();
  carla_speedometer_update();
  carla_autonomous_update();
}

void CAgent::carla_speedometer_update()
{
  auto velocity_msg = std::make_unique<autoware_auto_vehicle_msgs::msg::VelocityReport>();
  velocity_msg->header.frame_id = "base_link";
  velocity_msg->header.stamp = time_to_stamp(current_timestamp);

  tf2::Vector3 actorVelocity = tf2::Vector3(
    egoVehicle->GetVelocity().x,
    egoVehicle->GetVelocity().y,
    egoVehicle->GetVelocity().z
  );

  float pitch = to_radians(egoVehicle->GetTransform().rotation.pitch);
  float yaw = to_radians(egoVehicle->GetTransform().rotation.yaw);

  tf2::Vector3 planeTransform = tf2::Vector3(
    cos(pitch) * cos(yaw),
    cos(pitch) * sin(yaw),
    sin(pitch)
  );

  currentSpeed = actorVelocity.dot(planeTransform);

  velocity_msg->longitudinal_velocity = currentSpeed;
  velocity_msg->heading_rate = 0.0;

  if (auto agent = autoware_agent_ptr.lock())
  {
    agent->pub_velocity_report(std::move(velocity_msg));
  }
}

void CAgent::carla_autonomous_update()
{
  auto control_mode_msg = std::make_unique<autoware_auto_vehicle_msgs::msg::ControlModeReport>();
  control_mode_msg->stamp = time_to_stamp(current_timestamp);

  control_mode_msg->mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;

  if (auto agent = autoware_agent_ptr.lock())
  {
    // agent->pub_auto_control_mode_report(std::move(control_mode_msg));
  }
}

void CAgent::carla_lights_update()
{
  using TurnIndicatorsReport = autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport;
  using HazardLightsReport = autoware_auto_vehicle_msgs::msg::HazardLightsReport;

  auto turn_msg = std::make_unique<TurnIndicatorsReport>();
  auto hazard_msg = std::make_unique<HazardLightsReport>();
  auto headlights_msg = std::make_unique<std_msgs::msg::Int16>();
  turn_msg->stamp = time_to_stamp(current_timestamp);
  hazard_msg->stamp = time_to_stamp(current_timestamp);

  carla::client::Vehicle::LightState lightState = egoVehicle->GetLightState();

  //Check hazards first, this is if both turn signals are on as Carla does not have a concept of hazard directly
  if (LightFlagEnabled(lightState, LightState::RightBlinker) && LightFlagEnabled(lightState, LightState::LeftBlinker))
  {
    turn_msg->report = TurnIndicatorsReport::DISABLE;
    hazard_msg->report = HazardLightsReport::ENABLE;
  }
  else if (LightFlagEnabled(lightState, LightState::RightBlinker))
  {
    turn_msg->report = TurnIndicatorsReport::ENABLE_RIGHT;
    hazard_msg->report = HazardLightsReport::DISABLE;
  }
  else if (LightFlagEnabled(lightState, LightState::LeftBlinker))
  {
    turn_msg->report = TurnIndicatorsReport::ENABLE_LEFT;
    hazard_msg->report = HazardLightsReport::DISABLE;
  }
  else 
  {
    turn_msg->report = TurnIndicatorsReport::DISABLE;
    hazard_msg->report = HazardLightsReport::DISABLE;
  }

  headlights_msg->data = PASTA_HEADLIGHT_OFF;
  if (LightFlagEnabled(lightState, LightState::Position))
  {
    headlights_msg->data |= PASTA_HEADLIGHT_POSITION;
  }
  if (LightFlagEnabled(lightState, LightState::LowBeam))
  {
    headlights_msg->data |= PASTA_HEADLIGHT_LOW;
  }
  if (LightFlagEnabled(lightState, LightState::HighBeam))
  {
    headlights_msg->data |= PASTA_HEADLIGHT_PASS;
  }

  if (auto agent = autoware_agent_ptr.lock())
  {
    agent->pub_auto_hazard_status(std::move(hazard_msg));
    agent->pub_auto_turning_status(std::move(turn_msg));
    agent->pub_headlights_status(std::move(headlights_msg));
  }
}

void CAgent::carla_steer_update()
{
  //TODO Fix brake angle
  float carlaSteerL = egoVehicle->GetWheelSteerAngle(carla::client::Vehicle::WheelLocation::FL_Wheel);
  float carlaSteerR = egoVehicle->GetWheelSteerAngle(carla::client::Vehicle::WheelLocation::FR_Wheel);
  float carlaSteer = carlaSteerL;

  if (carlaSteerL > 0.0)
  {
    carlaSteer = carlaSteerR;
  }
  
  auto steer_msg = std::make_unique<autoware_auto_vehicle_msgs::msg::SteeringReport>();
  steer_msg->stamp = time_to_stamp(current_timestamp);
  steer_msg->steering_tire_angle = -to_radians(carlaSteer);

  if (auto agent = autoware_agent_ptr.lock())
  {
    agent->pub_auto_steering_status(std::move(steer_msg));
  }
}

void CAgent::carla_control_update()
{
  using GearReport = autoware_auto_vehicle_msgs::msg::GearReport;

  auto gear_msg = std::make_unique<GearReport>();
  gear_msg->stamp = time_to_stamp(current_timestamp);

  const std::lock_guard<std::mutex> lock(controlLock);
  carla::client::Vehicle::Control currentControlState = egoVehicle->GetControl();

  //This is due to carla not having a park, autoware having no handbrake, and pasta having both
  //Checks if the expected gear is park to determine whether to use handbrake status
  bool handbrake = currentControlState.hand_brake && parkingHandbrakeCmd;

  currentGear = Gears::carla_2_autoware_gears(currentControlState.gear, currentControlState.reverse, handbrake);
  gear_msg->report = currentGear;

  if (currentGear != GearReport::PARK && currentControlState.hand_brake && !parkingHandbrakeCmd)
  {
    pastaHandbrakeCmd = true;
  }

  auto actuation_msg = std::make_unique<tier4_vehicle_msgs::msg::ActuationStatusStamped>();
  actuation_msg->header.stamp = time_to_stamp(current_timestamp);
  actuation_msg->header.frame_id = "base_link";
  actuation_msg->status.accel_status = (currentControlState.throttle - carla_throttle_idle) * carla_throttle_compression_inv;
  actuation_msg->status.brake_status = currentControlState.brake;
  actuation_msg->status.steer_status = -currentControlState.steer * carla_steer_angle_rad;

  auto handbrake_msg = std::make_unique<autoware_auto_vehicle_msgs::msg::HandBrakeReport>();
  handbrake_msg->stamp = time_to_stamp(current_timestamp);
  handbrake_msg->report = currentControlState.hand_brake && pastaHandbrakeCmd;

  if (auto agent = autoware_agent_ptr.lock())
  {
    agent->pub_auto_gear_report(std::move(gear_msg));
    agent->pub_actuation_status(std::move(actuation_msg)); 
    agent->pub_handbrake_status(std::move(handbrake_msg));
  }
}

void CAgent::camera_callback(carla::SharedPtr<carla::sensor::SensorData> data)
{
  auto imageData = boost::static_pointer_cast<carla::sensor::data::Image>(data);

  auto buffer = imageData->data();

  auto image_msg = std::make_unique<sensor_msgs::msg::Image>();

  //TODO Fix timestamp
  image_msg->header.stamp = time_to_stamp(imageData->GetTimestamp());
  image_msg->header.frame_id = "camera0/camera_link";

  image_msg->height = imageData->GetHeight();
  image_msg->width = imageData->GetWidth();
  image_msg->encoding = "bgra8";

  image_msg->step = imageData->GetWidth() * 4;

  size_t size = image_msg->step * image_msg->height;

  image_msg->data.resize(size);
  image_msg->is_bigendian=0;

  memcpy(image_msg->data.data(), buffer, size);
  apiRPC.camera_process(image_msg->data.data(), image_msg->width, image_msg->height);

  //update camera info 
  auto image_info_msg = std::make_unique<sensor_msgs::msg::CameraInfo>(camera_sensor_info_tmpl);
  image_info_msg->header = image_msg->header;

  if(auto autoware_agent = autoware_agent_ptr.lock())
  {
    autoware_agent->pub_camera(std::move(image_msg));
    autoware_agent->pub_camera_info(std::move(image_info_msg));
  }
}

struct lidar_data {
    _Float32 x;
    _Float32 y;
    _Float32 z;
    _Float32 intensity;
    _Float32 azimuth;
    _Float32 distance;
    uint16_t ring;
    uint8_t return_type;
    uint8_t filler;
};

void CAgent::lidar_callback(carla::SharedPtr<carla::sensor::SensorData> newData)
{
  

  //Reduce change of object change
  carla::SharedPtr<carla::sensor::SensorData> data(newData);

  using PointField = sensor_msgs::msg::PointField;

  auto lidarData = boost::static_pointer_cast<carla::sensor::data::LidarMeasurement>(data);

  // auto buffer = lidarData->data();

  auto lidar_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  
  lidar_msg->header.stamp = time_to_stamp(lidarData->GetTimestamp());
  lidar_msg->header.frame_id = "velodyne_top";

  lidar_msg->fields = {
    PointField().set__name("x").set__offset(0).set__datatype(PointField::FLOAT32).set__count(1),
    PointField().set__name("y").set__offset(4).set__datatype(PointField::FLOAT32).set__count(1),
    PointField().set__name("z").set__offset(8).set__datatype(PointField::FLOAT32).set__count(1),
    PointField().set__name("intensity").set__offset(12).set__datatype(PointField::FLOAT32).set__count(1),
    PointField().set__name("azimuth").set__offset(16).set__datatype(PointField::FLOAT32).set__count(1),
    PointField().set__name("distance").set__offset(20).set__datatype(PointField::FLOAT32).set__count(1),
    PointField().set__name("ring").set__offset(24).set__datatype(PointField::UINT16).set__count(1),
    PointField().set__name("return_type").set__offset(26).set__datatype(PointField::UINT8).set__count(1),
    PointField().set__name("filler").set__offset(27).set__datatype(PointField::UINT8).set__count(1),
  };

  lidar_msg->width = 0;
  size_t numPoints = 0;
  for(unsigned int x = 0; x < lidarData->GetChannelCount(); x++)
  {
    lidar_msg->width += lidarData->GetPointCount(x);
    numPoints        += lidarData->GetPointCount(x);
  }

  lidar_msg->is_bigendian = false;
  lidar_msg->point_step = 28;
  lidar_msg->height = 1;
  lidar_msg->row_step = lidar_msg->point_step * lidar_msg->width;

  lidar_msg->data.resize(lidar_msg->row_step);
  // memcpy(lidar_msg->data.data(), buffer, lidar_msg->row_step);

  //Modify lidar data for reverse y due to right hand vs left hand rule
  //Start point y is the offset of the y value in the fields
  // width at this point is total points
  apiRPC.lidar_process((float*)lidarData->data(), lidar_msg->width, lidarData->GetHorizontalAngle(), lidarData->GetChannelCount());

  unsigned int carlaPtr = 0;
  unsigned int ptr = 0;
  for(unsigned int x = 0; x < lidarData->GetChannelCount(); x++)
  {
    unsigned int row_count = lidarData->GetPointCount(x);
    for(unsigned int row_current = 0; row_current < row_count; row_current++, ptr++, carlaPtr++)
    {
      lidar_data* dataStore = reinterpret_cast<lidar_data*>(lidar_msg->data.data() + (ptr * lidar_msg->point_step));
      auto carlaData = lidarData->data() + carlaPtr;

      dataStore->x = carlaData->point.x;
      dataStore->y = -carlaData->point.y;
      dataStore->z = carlaData->point.z;
      dataStore->intensity = carlaData->intensity;

      dataStore->azimuth =  lidarData->GetHorizontalAngle() + std::atan2(dataStore->y, dataStore->x);
      dataStore->distance = std::hypot(dataStore->x, dataStore->y, dataStore->z);

      dataStore->ring = x;
      dataStore->return_type = 0;
    }
  }

  if(auto autoware_agent = autoware_agent_ptr.lock())
  {
    autoware_agent->pub_lidar_loc(std::move(lidar_msg));
  }
}

void CAgent::gnss_callback(carla::SharedPtr<carla::sensor::SensorData> data)
{
  using NSS = sensor_msgs::msg::NavSatStatus;

  auto gnssData = boost::static_pointer_cast<carla::sensor::data::GnssMeasurement>(data);

  auto gnss_msg = std::make_unique<sensor_msgs::msg::NavSatFix>();
  
  gnss_msg->header.stamp = time_to_stamp(gnssData->GetTimestamp());
  gnss_msg->header.frame_id = "gnss_link";
  gnss_msg->status.status = NSS::STATUS_SBAS_FIX;
  gnss_msg->status.service = NSS::SERVICE_GPS | NSS::SERVICE_GLONASS | NSS::SERVICE_COMPASS | NSS::SERVICE_GALILEO;
  
  gnss_msg->latitude  = gnssData->GetLatitude();
  gnss_msg->longitude = gnssData->GetLongitude();
  gnss_msg->altitude  = gnssData->GetAltitude();
  apiRPC.gnss_process(gnss_msg->latitude, gnss_msg->longitude, gnss_msg->altitude);

  {
    std::lock_guard<std::mutex> lock(llhLock);
    m_llh[0] = gnss_msg->latitude;
    m_llh[1] = gnss_msg->longitude;
    m_llh[2] = gnss_msg->altitude;
  }

  if (auto agent = autoware_agent_ptr.lock())
  {
    agent->pub_nav_sat(std::move(gnss_msg));
  }
}

void CAgent::imu_callback(carla::SharedPtr<carla::sensor::SensorData> data)
{
  auto imuData = boost::static_pointer_cast<carla::sensor::data::IMUMeasurement>(data);

  auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
  // do processing on input data
  float acc_x = imuData->GetAccelerometer().x;
  float acc_y = imuData->GetAccelerometer().y;
  float acc_z = imuData->GetAccelerometer().z;

  float gyr_x = imuData->GetGyroscope().x;
  float gyr_y = imuData->GetGyroscope().y;
  float gyr_z = imuData->GetGyroscope().z;
  // TODO location isn't actually used?
  float loc_x = imuData->GetSensorTransform().location.x;
  float loc_y = imuData->GetSensorTransform().location.y;
  float loc_z = imuData->GetSensorTransform().location.z;

  float yaw   = imuData->GetSensorTransform().rotation.yaw;
  float pitch = imuData->GetSensorTransform().rotation.pitch;
  float roll  = imuData->GetSensorTransform().rotation.roll;
  apiRPC.imu_process(
      gyr_x, gyr_y, gyr_z
    , acc_x, acc_y, acc_z
    , loc_x, loc_y, loc_z
    , yaw  , pitch, roll
  );
  
  imu_msg->header.stamp = time_to_stamp(imuData->GetTimestamp());
  imu_msg->header.frame_id = "tamagawa/imu_link";
 
  imu_msg->angular_velocity.x = -gyr_x;
  imu_msg->angular_velocity.y =  gyr_y;
  imu_msg->angular_velocity.z = -gyr_z;

  imu_msg->linear_acceleration.x =  acc_x;
  imu_msg->linear_acceleration.y = -acc_y;
  imu_msg->linear_acceleration.z =  acc_z;

  //Calc Quarternion
  auto yaw_rad   = -to_radians(yaw  );
  auto pitch_rad = -to_radians(pitch);
  auto roll_rad  =  to_radians(roll );

  tf2::Quaternion quat;
  quat.setRPY(roll_rad, pitch_rad, yaw_rad);

  imu_msg->orientation.w = quat.w();
  imu_msg->orientation.x = quat.x();
  imu_msg->orientation.y = quat.y();
  imu_msg->orientation.z = quat.z();

  //Create GNSS Ins message
  auto gnss_ins_msg = std::make_unique<autoware_sensing_msgs::msg::GnssInsOrientationStamped>();

  gnss_ins_msg->header.stamp = time_to_stamp(imuData->GetTimestamp());
  gnss_ins_msg->header.frame_id = "gnss_link";

  gnss_ins_msg->orientation.rmse_rotation_x = 0.1;
  gnss_ins_msg->orientation.rmse_rotation_y = 0.1;
  gnss_ins_msg->orientation.rmse_rotation_z = 1.0;
  gnss_ins_msg->orientation.orientation = imu_msg->orientation;

  //Make Twist msg
  auto twist_msg = std::make_unique<geometry_msgs::msg::TwistWithCovarianceStamped>();
  double stddev_vx_ = 0.04; // 0.2 * 0.2 for std dev convariance
  double stddev_wz_ = 0.01; // 0.1 * 0.1 for std dev convariance

  twist_msg->header.stamp = time_to_stamp(imuData->GetTimestamp());
  twist_msg->header.frame_id = "base_link";

  twist_msg->twist.twist.angular = imu_msg->angular_velocity;

  tf2::Vector3 actorVelocity = tf2::Vector3(
    egoVehicle->GetVelocity().x,
    egoVehicle->GetVelocity().y,
    egoVehicle->GetVelocity().z
  );

  tf2::Vector3 rotatedVel = quatRotate(quat, actorVelocity);

  twist_msg->twist.twist.linear.x = rotatedVel.getX();
  twist_msg->twist.twist.linear.y = rotatedVel.getY();
  twist_msg->twist.twist.linear.z = rotatedVel.getZ();

  twist_msg->twist.covariance[0] = stddev_vx_;
  twist_msg->twist.covariance[7] = 10000.0;
  twist_msg->twist.covariance[14] = 10000.0;
  twist_msg->twist.covariance[21] = 10000.0;
  twist_msg->twist.covariance[28] = 10000.0;
  twist_msg->twist.covariance[35] = stddev_wz_;

  // convert to ECEF coordinates
  auto ecef_twist_msg = std::make_unique<geometry_msgs::msg::TwistWithCovarianceStamped>();
  ecef_twist_msg->header.stamp = time_to_stamp(imuData->GetTimestamp());
  ecef_twist_msg->header.frame_id = "base_link";

  ecef_twist_msg->twist.twist.angular = imu_msg->angular_velocity;

  double enu_vel[] = {
    actorVelocity.getX(),
    actorVelocity.getY(),
    actorVelocity.getZ()
  };
  double ecef_base_pos[3];
  double llh[3];
  double xyz_vel[3];
  {
    std::lock_guard<std::mutex> lock(llhLock);
    llh[0] = m_llh[0];
    llh[1] = m_llh[1];
    llh[2] = m_llh[2];
  }
  GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
  earth.Forward(llh[0], llh[1], llh[2], ecef_base_pos[0], ecef_base_pos[1], ecef_base_pos[2]);
  enu2xyz_vel(enu_vel, ecef_base_pos, xyz_vel);

  ecef_twist_msg->twist.twist.linear.x =  xyz_vel[0];
  ecef_twist_msg->twist.twist.linear.y =  xyz_vel[1];
  ecef_twist_msg->twist.twist.linear.z = -xyz_vel[2];  // Negative for Town01. Might be different for others

  ecef_twist_msg->twist.covariance[0] = stddev_vx_;
  ecef_twist_msg->twist.covariance[7] = stddev_vx_;
  ecef_twist_msg->twist.covariance[14] = stddev_vx_;
  ecef_twist_msg->twist.covariance[21] = stddev_wz_;
  ecef_twist_msg->twist.covariance[28] = stddev_wz_;
  ecef_twist_msg->twist.covariance[35] = stddev_wz_;

  if (auto agent = autoware_agent_ptr.lock())
  {
    agent->pub_imu(std::move(imu_msg));
    agent->pub_gnss_ins(std::move(gnss_ins_msg));
    agent->pub_twist(std::move(twist_msg));
    agent->pub_ecef_vel(std::move(ecef_twist_msg));
  }
}

void CAgent::sensors_destroy()
{
  if (camera_sensor)
  {
    camera_sensor->Stop();
    camera_sensor = nullptr;
    std::cout << "Unsubscribe camera sensor" << std::endl;
  }

  if (lidar_sensor)
  {
    lidar_sensor->Stop();
    lidar_sensor = nullptr;
    std::cout << "Unsubscribe lidar sensor" << std::endl;
  }

  if (gnss_sensor)
  {
    gnss_sensor->Stop();
    gnss_sensor = nullptr;
    std::cout << "Unsubscribe gnss sensor" << std::endl;
  }

  if (imu_sensor)
  {
    imu_sensor->Stop();
    imu_sensor = nullptr;
    std::cout << "Unsubscribe imu sensor" << std::endl;
  }
}

void CAgent::sensors_check()
{
  if (auto world = world_ptr.lock())
  {
    auto actorList = world->GetActors();
    carla::ActorId parent_id = 0;

    //Find actor parent id
    auto vehicleList = actorList->Filter("*vehicle*");
    for(const auto& actor : *vehicleList)
    {
      bool found = false;

      for (const auto& att : actor->GetAttributes())
      {
        if (att.GetId() == "role_name" && att.GetValue() == get_agent_role_name())
        {
          found = true;
          parent_id = actor->GetId();

          egoVehicle = boost::static_pointer_cast<carla::client::Vehicle>(actor);

          break;
        }
      }
      
      if (found)
      {
        break;
      }
    }

    if (parent_id == 0)
    {
      return;
    }

    //Get Max steer angle
    auto egoPhysics = egoVehicle->GetPhysicsControl();
    max_rpm = egoPhysics.max_rpm;
    for(auto wheel : egoPhysics.GetWheels())
    {
      if (wheel.max_steer_angle > 0)
      {
        max_steer_angle = wheel.max_steer_angle / 2; //Carla puts the full wheel turn range so divide by half for each wheel
        carla_steer_angle_rad_unit = 1.0 / to_radians(wheel.max_steer_angle);
        carla_steer_angle_rad = to_radians(wheel.max_steer_angle);;
        break;
      }
    }

    // auto carlaSteerCurve = egoPhysics.GetSteeringCurve();
    
    // for(int i = 0; i < carlaSteerCurve.size(); i++)
    // {
    //   steerCurve[carlaSteerCurve[i].x] = carlaSteerCurve[i].y;
    //   steerCurveMaxSpeed = carlaSteerCurve[i].x;

    //   if (i < (carlaSteerCurve.size() - 1))
    //   {
    //     int diffCount = (carlaSteerCurve[i + 1].x - carlaSteerCurve[i].x - 1);
    //     double diffUnit = (carlaSteerCurve[i].y - carlaSteerCurve[i + 1].y) / diffCount;
        
    //     for(int x = 1; x <= diffCount; x++)
    //     {
    //       steerCurve[carlaSteerCurve[i].x + x] = carlaSteerCurve[i].y - (diffUnit * x);
    //     }
    //   }
    // }

    torqueCurve = egoPhysics.GetTorqueCurve();    

    publish_max_wheel_angle(max_steer_angle);

    camera_sensor_setup(actorList, parent_id);
    gnss_sensor_setup(actorList, parent_id);
    lidar_sensor_setup(actorList, parent_id);
    imu_sensor_setup(actorList, parent_id);
  }
}

void CAgent::camera_sensor_setup(carla::SharedPtr<carla::client::ActorList> actorList, carla::ActorId parent_id)
{
  auto filteredActorList = actorList->Filter("sensor.camera.rgb");

  if (camera_sensor)
  {
    camera_sensor->Stop();
    camera_sensor = nullptr;
    std::cout << "Unsubscribe camera sensor" << std::endl;
  }

  for(const auto& actor : *filteredActorList)
  {
    if (actor->GetParentId() != parent_id)
    {
      continue;
    }

    camera_sensor = boost::static_pointer_cast<carla::client::Sensor>(actor);
    camera_sensor->Listen(std::bind(&CAgent::camera_callback, this, _1));

    std::cout << "Subscribe camera sensor" << std::endl;

    //Build Camera Info 
    double cx, cy, fx, fy;
    double fov = 0.0;
    for (const auto& att : actor->GetAttributes())
    {
      if (att.GetId() == "image_size_x")
      {
        camera_sensor_info_tmpl.width = stoi(att.GetValue());
      }
      else if (att.GetId() == "image_size_z")
      {
        camera_sensor_info_tmpl.height = stoi(att.GetValue());
      }
      else if (att.GetId() == "fov")
      {
        fov = stod(att.GetValue());
      }
    }
    cx = camera_sensor_info_tmpl.width / 2.0;
    cy = camera_sensor_info_tmpl.height / 2.0;
    fx = camera_sensor_info_tmpl.width / (2.0 * std::tan(fov * M_PI) * 360.0);
    fy = fx;

    camera_sensor_info_tmpl.distortion_model = "plumb_bob";
    camera_sensor_info_tmpl.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
    camera_sensor_info_tmpl.d = {0.0, 0.0, 0.0, 0.0, 0.0};
    camera_sensor_info_tmpl.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    camera_sensor_info_tmpl.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
  }
}

void CAgent::lidar_sensor_setup(carla::SharedPtr<carla::client::ActorList> actorList, carla::ActorId parent_id)
{
  auto filteredActorList = actorList->Filter("sensor.lidar.ray_cast");

  if (lidar_sensor)
  {
    lidar_sensor->Stop();
    lidar_sensor = nullptr;
    std::cout << "Unsubscribe lidar sensor" << std::endl;
  }

  for(const auto& actor : *filteredActorList)
  {
    if (actor->GetParentId() != parent_id)
    {
      continue;
    }

    lidar_sensor = boost::static_pointer_cast<carla::client::Sensor>(actor);
    lidar_sensor->Listen(std::bind(&CAgent::lidar_callback, this, _1));

    std::cout << "Subscribe lidar sensor" << std::endl;
  }
}

void CAgent::gnss_sensor_setup(carla::SharedPtr<carla::client::ActorList> actorList, carla::ActorId parent_id)
{
  auto filteredActorList = actorList->Filter("sensor.other.gnss");

  if (gnss_sensor)
  {
    gnss_sensor->Stop();
    gnss_sensor = nullptr;
    std::cout << "Unsubscribe gnss sensor" << std::endl;
  }

  for(const auto& actor : *filteredActorList)
  {
    if (actor->GetParentId() != parent_id)
    {
      continue;
    }

    gnss_sensor = boost::static_pointer_cast<carla::client::Sensor>(actor);
    gnss_sensor->Listen(std::bind(&CAgent::gnss_callback, this, _1));

    std::cout << "Subscribe gnss sensor" << std::endl;
  }
}

void CAgent::imu_sensor_setup(carla::SharedPtr<carla::client::ActorList> actorList, carla::ActorId parent_id)
{
  auto filteredActorList = actorList->Filter("sensor.other.imu");

  if (imu_sensor)
  {
    imu_sensor->Stop();
    imu_sensor = nullptr;
    std::cout << "Unsubscribe imu sensor" << std::endl;
  }

  for(const auto& actor : *filteredActorList)
  {
    if (actor->GetParentId() != parent_id)
    {
      continue;
    }

    imu_sensor = boost::static_pointer_cast<carla::client::Sensor>(actor);
    imu_sensor->Listen(std::bind(&CAgent::imu_callback, this, _1));

    std::cout << "Subscribe imu sensor" << std::endl;
  }
}

void CAgent::set_world(std::weak_ptr<carla::client::World> carlaWorld)
{
  world_ptr = carlaWorld;
}
