#include <cstdio>

#include "attack_gnss/attack_gnss.hpp"
#include "attack_gnss/config_node.hpp"

#include <rclcpp/node_options.hpp>

// #include <boost/geometry/geometry.hpp>
#include <rclcpp/logger.hpp>

#include <algorithm>

#include <gnss_convert.hpp>

using std::placeholders::_1;

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);

  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  auto config = std::make_shared<GNSSAttackConfigNode>();
  auto attack = std::make_shared<GNSSAttackNode>(config->get_config());

  executor->add_node(config);
  executor->add_node(attack);
  executor->spin();

  return 0;
}

GNSSAttackNode::GNSSAttackNode(std::shared_ptr<GNSSAttackConfig> config) : Node("attack_gnss", rclcpp::NodeOptions().use_intra_process_comms(true))
{
  this->attack_config = config;

  std::string gnss_subscription_path = this->declare_parameter("input", "/attack/gnss/input");
  std::string gnss_vel_subscription_path = this->declare_parameter("input_vel", "/attack/gnss/input_vel");
  std::string gnss_publish_path = this->declare_parameter("output", "/sensing/gnss/ublox/nav_sat_fix");
  std::string gnss_vel_publish_path = this->declare_parameter("output_vel", "/sensing/gnss/ecef_twist_with_covariance");
  std::string gnss_status_path = this->declare_parameter("status", "/attack/gnss/status");

  rclcpp::QoS gnssQoS = rclcpp::SensorDataQoS();
  gnssQoS.reliability(rclcpp::ReliabilityPolicy::Reliable);

  navsat_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      gnss_subscription_path, rclcpp::SensorDataQoS(), std::bind(&GNSSAttackNode::navsat_callback, this, _1));
  gnss_vel_sub = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      gnss_vel_subscription_path, rclcpp::SensorDataQoS(), std::bind(&GNSSAttackNode::gnss_vel_callback, this, _1));

  navsat_pub = this->create_publisher<sensor_msgs::msg::NavSatFix>(gnss_publish_path, gnssQoS);
  gnss_vel_pub = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(gnss_vel_publish_path, gnssQoS);
  status_pub = this->create_publisher<attack_gnss_msgs::msg::GNSSAttackStatus>(gnss_status_path, rclcpp::SensorDataQoS());
}

void GNSSAttackNode::navsat_callback(sensor_msgs::msg::NavSatFix::UniquePtr msg)
{
  std::lock_guard<std::mutex> guard(attack_config->config_access);

  using GNSSAttackConfig = attack_gnss_msgs::msg::GNSSAttackConfig;

  auto attack_status_msg = std::make_unique<attack_gnss_msgs::msg::GNSSAttackStatus>();

  attack_status_msg->config = attack_config->get_data();
  attack_status_msg->header = msg->header;

  //Immediately publish the current if not enabled as a passthough
  if (attack_config->attack_type() == GNSSAttackConfig::ATTACK_TYPE_NONE)
  {
    navsat_pub->publish(std::move(msg));
    status_pub->publish(std::move(attack_status_msg));
    return;
  }

  auto new_gnss_msg = std::make_unique<sensor_msgs::msg::NavSatFix>(*msg);

  if (attack_config->attack_type() & GNSSAttackConfig::ATTACK_TYPE_DROP)
  {
    if (attack_config->get_random_percent() < attack_config->drop_chance())
    {
      attack_status_msg->dropped = ++attack_config->dropped;
      attack_status_msg->processed = attack_config->processed;
      status_pub->publish(std::move(attack_status_msg));

      return;
    }
  }

  if (attack_config->attack_type() & GNSSAttackConfig::ATTACK_TYPE_ACCURACY)
  {
    constexpr int lon_cov_pos = 0;
    constexpr int lat_cov_pos = 4;
    constexpr int alt_cov_pos = 8;

    if (attack_config->accuracy_lon() > 0)
    {
      new_gnss_msg->position_covariance[lon_cov_pos] = std::pow(attack_config->accuracy_lon(), 2);
      new_gnss_msg->position_covariance[lon_cov_pos + 1] = 0.0;
      new_gnss_msg->position_covariance[lon_cov_pos + 2] = 0.0;
    } 
    else if (new_gnss_msg->position_covariance[lon_cov_pos] == 0.0)    
    {
      new_gnss_msg->position_covariance[lon_cov_pos] = 10.0;
    }

    
    if (attack_config->accuracy_lat() > 0)
    {
      new_gnss_msg->position_covariance[lat_cov_pos - 1] = 0.0;
      new_gnss_msg->position_covariance[lat_cov_pos] = std::pow(attack_config->accuracy_lat(), 2);
      new_gnss_msg->position_covariance[lat_cov_pos + 1] = 0.0;
    }
    else if (new_gnss_msg->position_covariance[lat_cov_pos] == 0.0)    
    {
      new_gnss_msg->position_covariance[lat_cov_pos] = 10.0;
    }

    if (attack_config->accuracy_alt() > 0)
    {
      new_gnss_msg->position_covariance[alt_cov_pos - 2] = 0.0;
      new_gnss_msg->position_covariance[alt_cov_pos - 1] = 0.0;
      new_gnss_msg->position_covariance[alt_cov_pos] = std::pow(attack_config->accuracy_alt(), 2);
    }
    else if (new_gnss_msg->position_covariance[alt_cov_pos] == 0.0)    
    {
      new_gnss_msg->position_covariance[alt_cov_pos] = 10.0;
    }

    if (new_gnss_msg->position_covariance_type == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN)
    {
      new_gnss_msg->position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
    }

    if (new_gnss_msg->position_covariance_type > sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN)
    {
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), std::chrono::milliseconds(10000).count(), "GPS Accuracy attack is downgrading covariance data from GPS");
    }
  }

  GNSSCoords gnss_data(new_gnss_msg->latitude, new_gnss_msg->longitude, new_gnss_msg->altitude, attack_config->coordinate_system, attack_config->height_system, attack_config->origin_lat, attack_config->origin_lon, attack_config->origin_alt, attack_config->plane);
  gnss_data.convert2XYZ();

  if (attack_config->attack_type() & GNSSAttackConfig::ATTACK_TYPE_OFFSET)
  {
    if (attack_config->offset_lon() != 0.0) 
    {
      gnss_data.x(gnss_data.x() + attack_config->offset_lon());
      attack_status_msg->difference_lon += attack_config->offset_lon();
    }

    if (attack_config->offset_lat() != 0.0) 
    {
      gnss_data.y(gnss_data.y() + attack_config->offset_lat());
      attack_status_msg->difference_lat += attack_config->offset_lat();
    }

    if (attack_config->offset_alt() != 0.0) 
    {
      gnss_data.z(gnss_data.z() + attack_config->offset_alt());
      attack_status_msg->difference_alt += attack_config->offset_alt();
    }
  }
  
  if (attack_config->attack_type() & GNSSAttackConfig::ATTACK_TYPE_DRIFT_STOP_DIST)
  {
    double distance = std::sqrt(
      std::pow(attack_config->drift_lon_cumlative, 2)
      + std::pow(attack_config->drift_lat_cumlative, 2)
      + std::pow(attack_config->drift_alt_cumlative, 2)
    );

    attack_status_msg->drift_distance = distance;

    if (distance > attack_config->drift_stop_meter())
    {
      attack_config->drift_run = false;
    }
  }

  if (attack_config->attack_type() & GNSSAttackConfig::ATTACK_TYPE_DRIFT_STOP_TIME)
  {
    auto current_time = this->get_clock()->now(); 
    
    if (attack_config->drift_start < 0)
    {
      attack_config->drift_start = current_time.seconds();
      attack_config->drift_stop = attack_config->drift_start + attack_config->drift_stop_second();
    } 
    else if ( current_time.seconds() > attack_config->drift_stop)
    {
      attack_config->drift_run = false;
    }

    attack_status_msg->drift_seconds = current_time.seconds() - attack_config->drift_start;
  }

  if (attack_config->attack_type() & GNSSAttackConfig::ATTACK_TYPE_DRIFT)
  {
    auto current_time = this->get_clock()->now();

    if (attack_config->drift_last_run < 0)
    {
      attack_config->drift_last_run = current_time.seconds();
    }

    double elapsed_time = current_time.seconds() - attack_config->drift_last_run;
    attack_config->drift_last_run = current_time.seconds();

    if (attack_config->drift_lon_s != 0.0  && attack_config->drift_run)
    {
      attack_config->drift_lon_cumlative += attack_config->drift_lon_s * elapsed_time;
    }

    if (attack_config->drift_lat_s != 0.0  && attack_config->drift_run)
    {
      attack_config->drift_lat_cumlative += attack_config->drift_lat_s * elapsed_time;
    }

    if (attack_config->drift_alt_s != 0.0  && attack_config->drift_run)
    {
      attack_config->drift_alt_cumlative += attack_config->drift_alt_s * elapsed_time;
    }

    gnss_data.x(gnss_data.x() + attack_config->drift_lon_cumlative);
    attack_status_msg->difference_lon += attack_config->drift_lon_cumlative;

    gnss_data.y(gnss_data.y() + attack_config->drift_lat_cumlative);
    attack_status_msg->difference_lat += attack_config->drift_lat_cumlative;

    gnss_data.z(gnss_data.z() + attack_config->drift_alt_cumlative);
    attack_status_msg->difference_alt += attack_config->drift_alt_cumlative;
  }

  gnss_data.convert2LLA();
  new_gnss_msg->latitude = gnss_data.lat();
  new_gnss_msg->longitude = gnss_data.lon();
  new_gnss_msg->altitude = gnss_data.alt();

  attack_status_msg->drift_running = attack_config->drift_run;

  attack_status_msg->dropped = attack_config->dropped;
  attack_status_msg->processed = ++attack_config->processed;

  navsat_pub->publish(std::move(new_gnss_msg));
  status_pub->publish(std::move(attack_status_msg));
}

void GNSSAttackNode::gnss_vel_callback(geometry_msgs::msg::TwistWithCovarianceStamped::UniquePtr msg)
{
  std::lock_guard<std::mutex> guard(attack_config->config_access);
  using GNSSAttackConfig = attack_gnss_msgs::msg::GNSSAttackConfig;

  auto new_vel_msg = std::make_unique<geometry_msgs::msg::TwistWithCovarianceStamped>(*msg);

  if (attack_config->attack_type() & GNSSAttackConfig::ATTACK_TYPE_VEL_SCALE)
  {
    new_vel_msg->twist.twist.linear.x *= attack_config->vel_scale_x();
    new_vel_msg->twist.twist.linear.y *= attack_config->vel_scale_y();
    new_vel_msg->twist.twist.linear.z *= attack_config->vel_scale_z();
  }

  gnss_vel_pub->publish(std::move(new_vel_msg));
}

