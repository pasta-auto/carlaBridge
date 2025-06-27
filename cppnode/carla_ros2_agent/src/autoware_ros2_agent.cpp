#include <carla_ros2_agent/autoware_ros2_agent.hpp>

using AAgent = CarlaAutowareRos2::AutowareAgent;

AAgent::AutowareAgent() : rclcpp::Node("bridge_autoware_agent", rclcpp::NodeOptions().use_intra_process_comms(true))
{
  std::string agent_role_name = this->declare_parameter("carla_role_name", "heroAW");
  std::string agent_role_path = "/carla/" + agent_role_name;

  std::string camera_raw_path = this->declare_parameter("autoware.camera_raw_path", "/sensing/camera/camera0/image_rect_color");
  std::string camera_info_path = this->declare_parameter("autoware.camera_info_path", "/sensing/camera/camera0/camera_info");
  std::string lidar_path = this->declare_parameter("autoware.lidar_path", "/sensing/lidar/top/outlier_filtered/pointcloud");
  std::string nav_sat_path = this->declare_parameter("autoware.nav_sat_path", "/sensing/gnss/ublox/nav_sat_fix");
  std::string ecef_velocity_path = this->declare_parameter("autoware.ecef_velocity_path", "/sensing/gnss/ecef_twist_with_covariance");
  std::string nav_sat_orientation_path = this->declare_parameter("autoware.nav_sat_orientation_path", "/autoware_orientation");
  std::string imu_path = this->declare_parameter("autoware.imu_path", "/sensing/imu/tamagawa/imu_raw");
  std::string steering_status_path = this->declare_parameter("autoware.steering_status_path", "/vehicle/status/steering_status");
  std::string hazard_status_path = this->declare_parameter("autoware.hazard_status_path", "/vehicle/status/hazard_lights_status");
  std::string turn_indicators_status_path = this->declare_parameter("autoware.turn_indicators_status_path", "/vehicle/status/turn_indicators_status");
  std::string gear_status_path = this->declare_parameter("autoware.gear_status_path", "/vehicle/status/gear_status");
  std::string control_mode_status_path = this->declare_parameter("autoware.control_mode_status_path", "/vehicle/status/control_mode");
  std::string veloticy_status_path = this->declare_parameter("autoware.veloticy_status_path", "/vehicle/status/velocity_status");
  std::string actuation_status_path = this->declare_parameter("autoware.actuation_status_path", "/vehicle/status/actuation_status");
  std::string handbrake_status_path = this->declare_parameter("autoware.handbrake_status_path", "/vehicle/status/handbrake_status");
  std::string headlights_status_path = this->declare_parameter("autoware.headlights_status_path", "/vehicle/status/headlights_status");
  std::string vehicle_velocity_twist_path = this->declare_parameter("autoware.vehicle_velocity_twist_path", "/sensing/vehicle_velocity_converter/twist_with_covariance");
  std::string clock_path = this->declare_parameter("autoware.clock_path", "/clock");

  

  rclcpp::QoS statusQoS = rclcpp::SensorDataQoS();
  statusQoS.reliability(rclcpp::ReliabilityPolicy::Reliable);

  camera_pub = this->create_publisher<sensor_msgs::msg::Image>(camera_raw_path, rclcpp::SensorDataQoS());
  camera_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_path, rclcpp::SensorDataQoS());

  lidar_pub_loc = this->create_publisher<sensor_msgs::msg::PointCloud2>(lidar_path, rclcpp::SensorDataQoS());

  nav_sat_pub = this->create_publisher<sensor_msgs::msg::NavSatFix>(nav_sat_path, statusQoS);

  ecef_vel_pub = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(ecef_velocity_path, statusQoS);

  imu_pub = this->create_publisher<sensor_msgs::msg::Imu>(imu_path, statusQoS);

  gnss_ins_pub = this->create_publisher<autoware_sensing_msgs::msg::GnssInsOrientationStamped>(nav_sat_orientation_path, statusQoS);

  auto_steering_status_pub = this->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>(steering_status_path, statusQoS);

  auto_hazard_status_pub = this->create_publisher<autoware_auto_vehicle_msgs::msg::HazardLightsReport>(hazard_status_path, statusQoS);

  auto_turning_status_pub = this->create_publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>(turn_indicators_status_path, statusQoS);

  auto_gear_report_pub = this->create_publisher<autoware_auto_vehicle_msgs::msg::GearReport>(gear_status_path, statusQoS);

  // auto_control_mode_report_pub = this->create_publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>(control_mode_status_path, statusQoS);

  velocity_report_pub = this->create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>(veloticy_status_path, statusQoS);

  actuation_status_pub = this->create_publisher<tier4_vehicle_msgs::msg::ActuationStatusStamped>(actuation_status_path, statusQoS);

  handbrake_status_pub = this->create_publisher<autoware_auto_vehicle_msgs::msg::HandBrakeReport>(handbrake_status_path, statusQoS);

  headlights_status_pub = this->create_publisher<std_msgs::msg::Int16>(headlights_status_path, statusQoS);

  twist_pub = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(vehicle_velocity_twist_path, statusQoS);

  ros2_clock_pub = this->create_publisher<rosgraph_msgs::msg::Clock>(clock_path, rclcpp::ClockQoS());
}

void AAgent::pub_camera(sensor_msgs::msg::Image::UniquePtr msg)
{
  if (msg.get() != nullptr)
  {
#ifdef DEBUG_MSG_PRINT
    printf("Camera msg publish, addr: 0x%p\n", (void*)msg.get());
#endif


    camera_pub->publish(std::move(msg));
  }
}

void AAgent::pub_camera_info(sensor_msgs::msg::CameraInfo::UniquePtr msg)
{
  if (msg.get() != nullptr)
  {
#ifdef DEBUG_MSG_PRINT
    printf("CameraInfo msg publish, addr: 0x%p\n", (void*)msg.get());
#endif

    camera_info_pub->publish(std::move(msg));
  }
}

void AAgent::pub_lidar_loc(sensor_msgs::msg::PointCloud2::UniquePtr msg)
{
  if (msg.get() != nullptr)
  {
#ifdef DEBUG_MSG_PRINT
    printf("Lidar msg publish, addr: 0x%p\n", (void*)msg.get());
#endif

    lidar_pub_loc->publish(std::move(msg));
  }
}

void AAgent::pub_nav_sat(sensor_msgs::msg::NavSatFix::UniquePtr msg)
{
  if (msg.get() != nullptr)
  {
#ifdef DEBUG_MSG_PRINT
    printf("NavSatFix msg publish, addr: 0x%p\n", (void*)msg.get());
#endif

    nav_sat_pub->publish(std::move(msg));
  }
}

void AAgent::pub_ecef_vel(geometry_msgs::msg::TwistWithCovarianceStamped::UniquePtr msg)
{
  if (msg.get() != nullptr)
  {
#ifdef DEBUG_MSG_PRINT
    printf("ECEF Twist msg publish, addr: 0x%p\n", (void*)msg.get());
#endif

    ecef_vel_pub->publish(std::move(msg));
  }
}

void AAgent::pub_imu(sensor_msgs::msg::Imu::UniquePtr msg)
{
  if (msg.get() != nullptr)
  {
#ifdef DEBUG_MSG_PRINT
    printf("Imu msg publish, addr: 0x%p\n", (void*)msg.get());
#endif

    imu_pub->publish(std::move(msg));
  }
}

void AAgent::pub_gnss_ins(autoware_sensing_msgs::msg::GnssInsOrientationStamped::UniquePtr msg)
{
  if (msg.get() != nullptr)
  {
#ifdef DEBUG_MSG_PRINT
    printf("GNSS Ins msg publish, addr: 0x%p\n", (void*)msg.get());
#endif

    gnss_ins_pub->publish(std::move(msg));
  }
}

void AAgent::pub_auto_steering_status(autoware_auto_vehicle_msgs::msg::SteeringReport::UniquePtr msg)
{
  if (msg.get() != nullptr)
  {
#ifdef DEBUG_MSG_PRINT
    printf("Auto Steering Report msg publish, addr: 0x%p\n", (void*)msg.get());
#endif

    auto_steering_status_pub->publish(std::move(msg));
  }
}

void AAgent::pub_auto_hazard_status(autoware_auto_vehicle_msgs::msg::HazardLightsReport::UniquePtr msg)
{
  if (msg.get() != nullptr)
  {
#ifdef DEBUG_MSG_PRINT
    printf("Hazard Lights Report msg publish, addr: 0x%p\n", (void*)msg.get());
#endif

    auto_hazard_status_pub->publish(std::move(msg));
  }
}

void AAgent::pub_auto_turning_status(autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::UniquePtr msg)
{
  if (msg.get() != nullptr)
  {
#ifdef DEBUG_MSG_PRINT
    printf("Turning Indicators Report msg publish, addr: 0x%p\n", (void*)msg.get());
#endif

    auto_turning_status_pub->publish(std::move(msg));
  }
}

void AAgent::pub_auto_gear_report(autoware_auto_vehicle_msgs::msg::GearReport::UniquePtr msg)
{
  if (msg.get() != nullptr)
  {
#ifdef DEBUG_MSG_PRINT
    printf("Gear Report msg publish, addr: 0x%p\n", (void*)msg.get());
#endif

    auto_gear_report_pub->publish(std::move(msg));
  }
}

void AAgent::pub_auto_control_mode_report(autoware_auto_vehicle_msgs::msg::ControlModeReport::UniquePtr msg)
{
  if (msg.get() != nullptr)
  {
#ifdef DEBUG_MSG_PRINT
    printf("Gear Report msg publish, addr: 0x%p\n", (void*)msg.get());
#endif

    // auto_control_mode_report_pub->publish(std::move(msg));
  }
}

void AAgent::pub_velocity_report(autoware_auto_vehicle_msgs::msg::VelocityReport::UniquePtr msg)
{
  if (msg.get() != nullptr)
  {
#ifdef DEBUG_MSG_PRINT
    printf("Velocity Report msg publish, addr: 0x%p\n", (void*)msg.get());
#endif

    velocity_report_pub->publish(std::move(msg));
  }
}

void AAgent::pub_actuation_status(tier4_vehicle_msgs::msg::ActuationStatusStamped::UniquePtr msg)
{
  if (msg.get() != nullptr)
  {
#ifdef DEBUG_MSG_PRINT
    printf("Actuation Status msg publish, addr: 0x%p\n", (void*)msg.get());
#endif

    actuation_status_pub->publish(std::move(msg));
  }
}

void AAgent::pub_twist(geometry_msgs::msg::TwistWithCovarianceStamped::UniquePtr msg)
{
  if (msg.get() != nullptr)
  {
#ifdef DEBUG_MSG_PRINT
    printf("Twist msg publish, addr: 0x%p\n", (void*)msg.get());
#endif

    twist_pub->publish(std::move(msg));
  }
}

void AAgent::pub_clock(rosgraph_msgs::msg::Clock::UniquePtr msg)
{
  if (msg.get() != nullptr)
  {
#ifdef DEBUG_MSG_PRINT
    printf("Clock msg publish, addr: 0x%p\n", (void*)msg.get());
#endif

    ros2_clock_pub->publish(std::move(msg));
  }
}

void AAgent::pub_handbrake_status(autoware_auto_vehicle_msgs::msg::HandBrakeReport::UniquePtr msg)
{
  if (msg.get() != nullptr)
  {
#ifdef DEBUG_MSG_PRINT
    printf("Clock msg publish, addr: 0x%p\n", (void*)msg.get());
#endif

    handbrake_status_pub->publish(std::move(msg));
  }
}

void AAgent::pub_headlights_status(std_msgs::msg::Int16::UniquePtr msg)
{
  if (msg.get() != nullptr)
  {
#ifdef DEBUG_MSG_PRINT
    printf("Clock msg publish, addr: 0x%p\n", (void*)msg.get());
#endif

    headlights_status_pub->publish(std::move(msg));
  }
}
