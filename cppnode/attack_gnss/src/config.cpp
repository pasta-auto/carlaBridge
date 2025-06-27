#include "attack_gnss/config.hpp"
#include "attack_gnss/config_node.hpp"

static rclcpp::Node* node_ptr; 

using std::placeholders::_1;

GNSSAttackConfigNode::GNSSAttackConfigNode() : Node("attack_gnss_config", rclcpp::NodeOptions().use_intra_process_comms(false))
{
    node_ptr = this;
    
    std::string config_path = this->declare_parameter("config", "/attack/gnss/config");

    config->height_system = this->declare_parameter("height_system", 0);
    config->coordinate_system = this->declare_parameter("coordinate_system", 3);

    config->origin_lat = this->declare_parameter("origin_lat", 0.0);
    config->origin_lon = this->declare_parameter("origin_lon", 0.0);
    config->origin_alt = this->declare_parameter("origin_alt", 0.0);

    config->plane = this->declare_parameter("plane", 0);

    if (config->coordinate_system != 2 && config->coordinate_system != 3)
    {
      std::cerr << "Current unsupported coordinate system " << config->coordinate_system << std::endl;
      rclcpp::shutdown(nullptr, "Current unsupported coordinate system");
    }

    config_sub = this->create_subscription<attack_gnss_msgs::msg::GNSSAttackConfig>(
        config_path, rclcpp::SensorDataQoS(), std::bind(&GNSSAttackConfigNode::config_callback, this, _1));
}

//Setup Config
void GNSSAttackConfig::set_data(attack_gnss_msgs::msg::GNSSAttackConfig::SharedPtr newData)
{
  data = newData;
}

void GNSSAttackConfigNode::config_callback(attack_gnss_msgs::msg::GNSSAttackConfig::UniquePtr msg)
{
  std::lock_guard<std::mutex> guard(config->config_access);

  //move config
  config->set_data(std::move(msg));

  config->drift_start = -1;
  config->drift_stop = -1;
  config->drift_last_run = -1;
  config->drift_run = true;

  config->drift_lat_s = config->drift_lat() / 60;
  config->drift_lon_s = config->drift_lon() / 60;
  config->drift_alt_s = config->drift_alt() / 60;

  if (config->reset_drift())
  {
    config->drift_lat_cumlative = 0.0;
    config->drift_lon_cumlative = 0.0;
    config->drift_alt_cumlative = 0.0;
  }

  config->processed = 0;
  config->dropped = 0;
}
