#include "attack_ethernet/attack_ethernet.hpp"

#include <rclcpp/create_timer.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node_options.hpp>

#include <chrono>

#include <rclcpp/logger.hpp>
#include <rclcpp/subscription_options.hpp>

#include <memory>

using std::placeholders::_1;

int main(int argc, char ** argv)
{
  (void)argc;
  (void)argv;

  rclcpp::init(argc, argv);

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  auto config = std::make_shared<EthernetAttackConfigNode>();
  auto attack = std::make_shared<EthernetAttackNode>(config->get_config());

  executor->add_node(config);
  executor->add_node(attack);
  executor->spin();

  return 0;
}

EthernetAttackNode::EthernetAttackNode(std::shared_ptr<EthernetAttackConfig> config)
: Node("attack_ethernet", rclcpp::NodeOptions().use_intra_process_comms(true))
{
  this->attack_config = config;

  std::string camera_sub_path = this->declare_parameter("input_camera_path", "/attack/sensing/camera/camera0/image_rect_color");
  std::string camera_info_sub_path = this->declare_parameter("input_camera_info_path", "/attack/sensing/camera/camera0/camera_info");
  std::string lidar_sub_path = this->declare_parameter("input_lidar_path", "/attack/sensing/lidar/top/outlier_filtered/pointcloud");
  std::string nav_sat_sub_path = this->declare_parameter("input_nav_sat_path", "/attack/sensing/gnss/ublox/nav_sat_fix");
  std::string ecef_velocity_sub_path = this->declare_parameter("input_ecef_velocity_path", "/attack/sensing/gnss/ecef_twist_with_covariance");
  std::string nav_sat_orientation_sub_path = this->declare_parameter("input_nav_sat_orientation_path", "/attack/autoware_orientation");
  std::string imu_sub_path = this->declare_parameter("input_imu_path", "/attack/sensing/imu/tamagawa/imu_raw");
  std::string steering_status_sub_path = this->declare_parameter("input_steering_status_path", "/attack/vehicle/status/steering_status");
  std::string hazard_status_sub_path = this->declare_parameter("input_hazard_status_path", "/attack/vehicle/status/hazard_lights_status");
  std::string turn_indicators_status_sub_path = this->declare_parameter( "input_turn_indicators_status_path", "/attack/vehicle/status/turn_indicators_status");
  std::string gear_status_sub_path = this->declare_parameter("input_gear_status_path", "/attack/vehicle/status/gear_status");
  std::string control_mode_status_sub_path = this->declare_parameter("input_control_mode_status_path", "/attack/vehicle/status/control_mode");
  std::string velocity_status_sub_path = this->declare_parameter("input_velocity_status_path", "/attack/vehicle/status/velocity_status");
  std::string actuation_status_sub_path = this->declare_parameter("input_actuation_status_path", "/attack/vehicle/status/actuation_status");
  std::string handbrake_status_sub_path = this->declare_parameter("input_handbrake_status_path", "/attack/vehicle/status/handbrake_status");
  std::string headlights_status_sub_path = this->declare_parameter("input_headlights_status_path", "/attack/vehicle/status/headlights_status");
  std::string vehicle_velocity_twist_sub_path = this->declare_parameter("input_vehicle_velocity_twist_path", "/attack/sensing/vehicle_velocity_converter/twist_with_covariance");

  std::string camera_pub_path = this->declare_parameter("camera_path", "/sensing/camera/camera0/image_rect_color");
  std::string camera_info_pub_path = this->declare_parameter("camera_info_path", "/sensing/camera/camera0/camera_info");
  std::string lidar_pub_path = this->declare_parameter("lidar_path", "/sensing/lidar/top/outlier_filtered/pointcloud");
  std::string nav_sat_pub_path = this->declare_parameter("nav_sat_path", "/sensing/gnss/ublox/nav_sat_fix");
  std::string ecef_velocity_pub_path = this->declare_parameter("ecef_velocity_path", "/sensing/gnss/ecef_twist_with_covariance");
  std::string nav_sat_orientation_pub_path = this->declare_parameter("nav_sat_orientation_path", "/autoware_orientation");
  std::string imu_pub_path = this->declare_parameter("imu_path", "/sensing/imu/tamagawa/imu_raw");
  std::string steering_status_pub_path = this->declare_parameter("steering_status_path", "/vehicle/status/steering_status");
  std::string hazard_status_pub_path = this->declare_parameter("hazard_status_path", "/vehicle/status/hazard_lights_status");
  std::string turn_indicators_status_pub_path = this->declare_parameter("turn_indicators_status_path", "/vehicle/status/turn_indicators_status");
  std::string gear_status_pub_path = this->declare_parameter("gear_status_path", "/vehicle/status/gear_status");
  std::string control_mode_status_pub_path = this->declare_parameter("control_mode_status_path", "/vehicle/status/control_mode");
  std::string velocity_status_pub_path = this->declare_parameter("velocity_status_path", "/vehicle/status/velocity_status");
  std::string actuation_status_pub_path = this->declare_parameter("actuation_status_path", "/vehicle/status/actuation_status");
  std::string handbrake_status_pub_path = this->declare_parameter("handbrake_status_path", "/vehicle/status/handbrake_status");
  std::string headlights_status_pub_path = this->declare_parameter("headlights_status_path", "/vehicle/status/headlights_status");
  std::string vehicle_velocity_twist_pub_path = this->declare_parameter("vehicle_velocity_twist_path", "/sensing/vehicle_velocity_converter/twist_with_covariance");
  std::string ethernet_status_path = this->declare_parameter("status", "/attack/ethernet/status");

  rclcpp::QoS statusQoS = rclcpp::SensorDataQoS();
  statusQoS.reliability(rclcpp::ReliabilityPolicy::Reliable);

  _callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  auto grp_opt = rclcpp::SubscriptionOptions();
  grp_opt.callback_group = _callback_group;

  camera_sub = this->create_subscription<sensor_msgs::msg::Image>(camera_sub_path, rclcpp::SensorDataQoS(), std::bind(&EthernetAttackNode::camera_callback, this, _1), grp_opt);
  camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(camera_info_sub_path, rclcpp::SensorDataQoS(), std::bind(&EthernetAttackNode::camera_info_callback, this, _1), grp_opt);
  lidar_sub_loc = this->create_subscription<sensor_msgs::msg::PointCloud2>(lidar_sub_path, rclcpp::SensorDataQoS(), std::bind(&EthernetAttackNode::lidar_callback, this, _1), grp_opt);
  nav_sat_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(nav_sat_sub_path, statusQoS, std::bind(&EthernetAttackNode::nav_sat_callback, this, _1), grp_opt);
  ecef_vel_sub = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(ecef_velocity_sub_path, statusQoS, std::bind(&EthernetAttackNode::ecef_vel_callback, this, _1), grp_opt);
  imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(imu_sub_path, statusQoS, std::bind(&EthernetAttackNode::imu_callback, this, _1), grp_opt);
  gnss_ins_sub = this->create_subscription<autoware_sensing_msgs::msg::GnssInsOrientationStamped>(nav_sat_orientation_sub_path, statusQoS, std::bind(&EthernetAttackNode::gnss_ins_callback, this, _1), grp_opt);
  auto_steering_status_sub = this->create_subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>(steering_status_sub_path, statusQoS, std::bind(&EthernetAttackNode::auto_steering_status_callback, this, _1), grp_opt);
  auto_hazard_status_sub = this->create_subscription<autoware_auto_vehicle_msgs::msg::HazardLightsReport>(hazard_status_sub_path, statusQoS, std::bind(&EthernetAttackNode::auto_hazard_status_callback, this, _1), grp_opt);
  auto_turning_status_sub = this->create_subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>(turn_indicators_status_sub_path, statusQoS, std::bind(&EthernetAttackNode::auto_turning_status_callback, this, _1), grp_opt);
  auto_gear_report_sub = this->create_subscription<autoware_auto_vehicle_msgs::msg::GearReport>(gear_status_sub_path, statusQoS, std::bind(&EthernetAttackNode::auto_gear_report_callback, this, _1), grp_opt);
  velocity_report_sub = this->create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>(velocity_status_sub_path, statusQoS, std::bind(&EthernetAttackNode::velocity_report_callback, this, _1), grp_opt);
  actuation_status_sub = this->create_subscription<tier4_vehicle_msgs::msg::ActuationStatusStamped>(actuation_status_sub_path, statusQoS, std::bind(&EthernetAttackNode::actuation_status_callback, this, _1), grp_opt);
  handbrake_status_sub = this->create_subscription<autoware_auto_vehicle_msgs::msg::HandBrakeReport>(handbrake_status_sub_path, statusQoS, std::bind(&EthernetAttackNode::handbrake_status_callback, this, _1), grp_opt);
  headlights_status_sub = this->create_subscription<std_msgs::msg::Int16>(headlights_status_sub_path, statusQoS, std::bind(&EthernetAttackNode::headlights_status_callback, this, _1), grp_opt);
  twist_sub = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(vehicle_velocity_twist_sub_path, statusQoS, std::bind(&EthernetAttackNode::twist_callback, this, _1), grp_opt);

  camera_pub = this->create_publisher<sensor_msgs::msg::Image>(camera_pub_path, rclcpp::SensorDataQoS());
  camera_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_pub_path, rclcpp::SensorDataQoS());
  lidar_pub_loc = this->create_publisher<sensor_msgs::msg::PointCloud2>(lidar_pub_path, rclcpp::SensorDataQoS());
  nav_sat_pub = this->create_publisher<sensor_msgs::msg::NavSatFix>(nav_sat_pub_path, statusQoS);
  ecef_vel_pub = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(ecef_velocity_pub_path, statusQoS);
  imu_pub = this->create_publisher<sensor_msgs::msg::Imu>(imu_pub_path, statusQoS);
  gnss_ins_pub = this->create_publisher<autoware_sensing_msgs::msg::GnssInsOrientationStamped>(nav_sat_orientation_pub_path, statusQoS);
  auto_steering_status_pub = this->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>(steering_status_pub_path, statusQoS);
  auto_hazard_status_pub = this->create_publisher<autoware_auto_vehicle_msgs::msg::HazardLightsReport>(hazard_status_pub_path, statusQoS);
  auto_turning_status_pub = this->create_publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>(turn_indicators_status_pub_path, statusQoS);
  auto_gear_report_pub = this->create_publisher<autoware_auto_vehicle_msgs::msg::GearReport>(gear_status_pub_path, statusQoS);
  velocity_report_pub = this->create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>(velocity_status_pub_path, statusQoS);
  actuation_status_pub = this->create_publisher<tier4_vehicle_msgs::msg::ActuationStatusStamped>(actuation_status_pub_path, statusQoS);
  handbrake_status_pub = this->create_publisher<autoware_auto_vehicle_msgs::msg::HandBrakeReport>(handbrake_status_pub_path, statusQoS);
  headlights_status_pub = this->create_publisher<std_msgs::msg::Int16>(headlights_status_pub_path, statusQoS);
  twist_pub = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(vehicle_velocity_twist_pub_path, statusQoS);

  status_pub = this->create_publisher<attack_ethernet_msgs::msg::EthernetAttackStatus>(ethernet_status_path, rclcpp::SensorDataQoS());

  constexpr auto replay_gruanularity = std::chrono::milliseconds(10);
  replay_timer = rclcpp::create_timer(this, this->get_clock(), replay_gruanularity, std::bind(&EthernetAttackNode::handle_replay, this), _callback_group);
}


void EthernetAttackNode::camera_callback(sensor_msgs::msg::Image::UniquePtr msg)
{
  using attack_ethernet_msgs::msg::EthernetAttackConfig;
  if(handle_switching(EthernetAttackConfig::ATTACK_TARGET_CAMERA, AttackNum::CAMERA, camera_pub, *msg))
  {
    std::lock_guard<std::mutex> guard(pub_mutex[AttackNum::CAMERA]);
    camera_pub->publish(std::move(msg));
  }
}

void EthernetAttackNode::camera_info_callback(sensor_msgs::msg::CameraInfo::UniquePtr msg)
{
  using attack_ethernet_msgs::msg::EthernetAttackConfig;
  if(handle_switching(EthernetAttackConfig::ATTACK_TARGET_CAMERA_INFO, AttackNum::CAMERA_INFO, camera_info_pub, *msg))
  {
    std::lock_guard<std::mutex> guard(pub_mutex[AttackNum::CAMERA_INFO]);
    camera_info_pub->publish(std::move(msg));
  }
}

void EthernetAttackNode::lidar_callback(sensor_msgs::msg::PointCloud2::UniquePtr msg)
{
  using attack_ethernet_msgs::msg::EthernetAttackConfig;
  if(handle_switching(EthernetAttackConfig::ATTACK_TARGET_LIDAR, AttackNum::LIDAR, lidar_pub_loc, *msg))
  {
    std::lock_guard<std::mutex> guard(pub_mutex[AttackNum::LIDAR]);
    lidar_pub_loc->publish(std::move(msg));
  }
}

void EthernetAttackNode::nav_sat_callback(sensor_msgs::msg::NavSatFix::UniquePtr msg)
{
  using attack_ethernet_msgs::msg::EthernetAttackConfig;
  if(handle_switching(EthernetAttackConfig::ATTACK_TARGET_NAV_SAT, AttackNum::NAV_SAT, nav_sat_pub, *msg))
  {
    std::lock_guard<std::mutex> guard(pub_mutex[AttackNum::NAV_SAT]);
    nav_sat_pub->publish(std::move(msg));
  }
}

void EthernetAttackNode::ecef_vel_callback(geometry_msgs::msg::TwistWithCovarianceStamped::UniquePtr msg)
{
  using attack_ethernet_msgs::msg::EthernetAttackConfig;
  if(handle_switching(EthernetAttackConfig::ATTACK_TARGET_ECEF_VEL, AttackNum::ECEF_VEL, ecef_vel_pub, *msg))
  {
    std::lock_guard<std::mutex> guard(pub_mutex[AttackNum::ECEF_VEL]);
    ecef_vel_pub->publish(std::move(msg));
  }
}

void EthernetAttackNode::imu_callback(sensor_msgs::msg::Imu::UniquePtr msg)
{
  using attack_ethernet_msgs::msg::EthernetAttackConfig;
  if(handle_switching(EthernetAttackConfig::ATTACK_TARGET_IMU, AttackNum::IMU, imu_pub, *msg))
  {
    std::lock_guard<std::mutex> guard(pub_mutex[AttackNum::IMU]);
    imu_pub->publish(std::move(msg));
  }
}

void EthernetAttackNode::gnss_ins_callback(autoware_sensing_msgs::msg::GnssInsOrientationStamped::UniquePtr msg)
{
  using attack_ethernet_msgs::msg::EthernetAttackConfig;
  if(handle_switching(EthernetAttackConfig::ATTACK_TARGET_GNSS_INS, AttackNum::GNSS_INS, gnss_ins_pub, *msg))
  {
    std::lock_guard<std::mutex> guard(pub_mutex[AttackNum::GNSS_INS]);
    gnss_ins_pub->publish(std::move(msg));
  }
}

void EthernetAttackNode::auto_steering_status_callback(autoware_auto_vehicle_msgs::msg::SteeringReport::UniquePtr msg)
{
  using attack_ethernet_msgs::msg::EthernetAttackConfig;
  if(handle_switching(EthernetAttackConfig::ATTACK_TARGET_AUTO_STEERING_STATUS, AttackNum::AUTO_STEERING_STATUS, auto_steering_status_pub, *msg))
  {
    std::lock_guard<std::mutex> guard(pub_mutex[AttackNum::AUTO_STEERING_STATUS]);
    auto_steering_status_pub->publish(std::move(msg));
  }
}

void EthernetAttackNode::auto_hazard_status_callback(autoware_auto_vehicle_msgs::msg::HazardLightsReport::UniquePtr msg)
{
  using attack_ethernet_msgs::msg::EthernetAttackConfig;
  if(handle_switching(EthernetAttackConfig::ATTACK_TARGET_AUTO_HAZARD_STATUS, AttackNum::AUTO_HAZARD_STATUS, auto_hazard_status_pub, *msg))
  {
    std::lock_guard<std::mutex> guard(pub_mutex[AttackNum::AUTO_HAZARD_STATUS]);
    auto_hazard_status_pub->publish(std::move(msg));
  }
}

void EthernetAttackNode::auto_turning_status_callback(autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::UniquePtr msg)
{
  using attack_ethernet_msgs::msg::EthernetAttackConfig;
  if(handle_switching(EthernetAttackConfig::ATTACK_TARGET_AUTO_TURNING_STATUS, AttackNum::AUTO_TURNING_STATUS, auto_turning_status_pub, *msg))
  {
    std::lock_guard<std::mutex> guard(pub_mutex[AttackNum::AUTO_TURNING_STATUS]);
    auto_turning_status_pub->publish(std::move(msg));
  }
}

void EthernetAttackNode::auto_gear_report_callback(autoware_auto_vehicle_msgs::msg::GearReport::UniquePtr msg)
{
  using attack_ethernet_msgs::msg::EthernetAttackConfig;
  if(handle_switching(EthernetAttackConfig::ATTACK_TARGET_AUTO_GEAR_REPORT, AttackNum::AUTO_GEAR_REPORT, auto_gear_report_pub, *msg))
  {
    std::lock_guard<std::mutex> guard(pub_mutex[AttackNum::AUTO_GEAR_REPORT]);
    auto_gear_report_pub->publish(std::move(msg));
  }
}

void EthernetAttackNode::velocity_report_callback(autoware_auto_vehicle_msgs::msg::VelocityReport::UniquePtr msg)
{
  using attack_ethernet_msgs::msg::EthernetAttackConfig;
  if(handle_switching(EthernetAttackConfig::ATTACK_TARGET_VELOCITY_REPORT, AttackNum::VELOCITY_REPORT, velocity_report_pub, *msg))
  {
    std::lock_guard<std::mutex> guard(pub_mutex[AttackNum::VELOCITY_REPORT]);
    velocity_report_pub->publish(std::move(msg));
  }
}

void EthernetAttackNode::actuation_status_callback(tier4_vehicle_msgs::msg::ActuationStatusStamped::UniquePtr msg)
{
  using attack_ethernet_msgs::msg::EthernetAttackConfig;
  if(handle_switching(EthernetAttackConfig::ATTACK_TARGET_ACTUATION_STATUS, AttackNum::ACTUATION_STATUS, actuation_status_pub, *msg))
  {
    std::lock_guard<std::mutex> guard(pub_mutex[AttackNum::ACTUATION_STATUS]);
    actuation_status_pub->publish(std::move(msg));
  }
}

void EthernetAttackNode::handbrake_status_callback(autoware_auto_vehicle_msgs::msg::HandBrakeReport::UniquePtr msg)
{
  using attack_ethernet_msgs::msg::EthernetAttackConfig;
  if(handle_switching(EthernetAttackConfig::ATTACK_TARGET_HANDBRAKE_STATUS, AttackNum::HANDBRAKE_STATUS, handbrake_status_pub, *msg))
  {
    std::lock_guard<std::mutex> guard(pub_mutex[AttackNum::HANDBRAKE_STATUS]);
    handbrake_status_pub->publish(std::move(msg));
  }
}

void EthernetAttackNode::headlights_status_callback(std_msgs::msg::Int16::UniquePtr msg)
{
  using attack_ethernet_msgs::msg::EthernetAttackConfig;
  if(handle_switching(EthernetAttackConfig::ATTACK_TARGET_HEADLIGHTS_STATUS, AttackNum::HEADLIGHTS_STATUS, headlights_status_pub, *msg))
  {
    std::lock_guard<std::mutex> guard(pub_mutex[AttackNum::HEADLIGHTS_STATUS]);
    headlights_status_pub->publish(std::move(msg));
  }
}

void EthernetAttackNode::twist_callback(geometry_msgs::msg::TwistWithCovarianceStamped::UniquePtr msg)
{
  using attack_ethernet_msgs::msg::EthernetAttackConfig;
  if(handle_switching(EthernetAttackConfig::ATTACK_TARGET_TWIST, AttackNum::TWIST, twist_pub, *msg))
  {
    std::lock_guard<std::mutex> guard(pub_mutex[AttackNum::TWIST]);
    twist_pub->publish(std::move(msg));
  }
}

void EthernetAttackNode::handle_replay()
{
  std::lock_guard<std::mutex> guard(replay_queue_lock);
  if(!replay_queue.empty())
  {
    auto current_time = this->now();
    auto curr_sec = static_cast<unsigned int>(current_time.nanoseconds() / 1000000000l);
    auto curr_nanosec = static_cast<unsigned int>(current_time.nanoseconds() % 1000000000l);

    uint16_t last_target = 0;
    auto replay = replay_queue.top();
    while (curr_sec >= replay.sec && curr_nanosec >= replay.nanosec)
    {
      if (replay.target == last_target)
      {
        // Avoid publishing to the same topic too quickly
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      replay.cb();
      last_target = replay.target;

      replay_queue.pop();
      if (replay_queue.empty())
      {
        break;
      }
      replay = replay_queue.top();
    }
  }
}

