#include "attack_lidar_blind/config.hpp"
#include "attack_lidar_blind/config_node.hpp"


#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>

static rclcpp::Node* node_ptr; 

using std::placeholders::_1;

LidarBlindAttackConfigNode::LidarBlindAttackConfigNode() : Node("attack_lidar_blind_config", rclcpp::NodeOptions().use_intra_process_comms(false))
{
    node_ptr = this;
    
    std::string config_path = this->declare_parameter("config", "/attack/lidar_blind/config");
    std::string map_path = this->declare_parameter("map", "/map/vector_map");

    config_sub = this->create_subscription<attack_lidar_blind_msgs::msg::LidarAttackConfig>(
        config_path, rclcpp::SensorDataQoS(), std::bind(&LidarBlindAttackConfigNode::config_callback, this, _1));

    map_bin_sub = this->create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
        map_path, rclcpp::QoS{1}.transient_local(),
        std::bind(&LidarBlindAttackConfigNode::mapbin_callback, this, _1));
}

//Setup Config
void LidarBlindAttackConfig::set_data(attack_lidar_blind_msgs::msg::LidarAttackConfig::SharedPtr newData)
{
  if (newData->attack_type & attack_lidar_blind_msgs::msg::LidarAttackConfig::ATTACK_TYPE_LANELET)
  {
    for(auto ll_candidate : lanelets)
    {
      if (ll_candidate.id() == newData->lanelet_id)
      {
        attack_lanelet = std::make_shared<lanelet::ConstLanelet>(ll_candidate);
        break;
      }
    }

    if (!attack_lanelet)
    {
      RCLCPP_ERROR(node_ptr->get_logger(), "Failed to find lanelet %li, Disabling attacks \n", lanelet_id());
      newData->attack_type = attack_lidar_blind_msgs::msg::LidarAttackConfig::ATTACK_TYPE_NONE;
    }
  }

  if (newData->attack_type & attack_lidar_blind_msgs::msg::LidarAttackConfig::ATTACK_TYPE_BOUND_BOX)
  {
    _bound_lower_x = std::min(newData->bound_box[0].x, newData->bound_box[1].x);
    _bound_upper_x = std::max(newData->bound_box[0].x, newData->bound_box[1].x);
    _bound_x_valid = _bound_lower_x != _bound_upper_x;

    _bound_lower_y = std::min(newData->bound_box[0].y, newData->bound_box[1].y);
    _bound_upper_y = std::max(newData->bound_box[0].y, newData->bound_box[1].y);
    _bound_y_valid = _bound_lower_y != _bound_upper_y;

    _bound_lower_z = std::min(newData->bound_box[0].z, newData->bound_box[1].z);
    _bound_upper_z = std::max(newData->bound_box[0].z, newData->bound_box[1].z);
    _bound_z_valid = _bound_lower_z != _bound_upper_z;

    if (!_bound_x_valid || !_bound_y_valid)
    {
      RCLCPP_ERROR(node_ptr->get_logger(), "Bound Box required minimum X & Y, Disabling attacks \n");
      newData->attack_type = attack_lidar_blind_msgs::msg::LidarAttackConfig::ATTACK_TYPE_NONE;
    }
  }

  if (newData->attack_angle_dropoff > 0.0)
  {
    _angle_blend_enabled = true;

    _angle_blend_start = (newData->attack_angle / 2) - newData->attack_angle_dropoff;
    _angle_blend_unit = newData->attack_angle_dropoff / blend_size;
    _angle_blend_bias = -_angle_blend_start + _angle_blend_unit * 0.5;

    float blend_unit;
    if (newData->attack_type & attack_lidar_blind_msgs::msg::LidarAttackConfig::ATTACK_TYPE_RAND_DROP)
    {
      blend_unit = newData->random_drop_percent / (float)blend_size;
    }
    else {
      blend_unit = 1.0 / (float)blend_size;
    }

    for(int i = 0; i < blend_size; i++)
    {
      _angle_blend_lookup[i] = (blend_size - i) * blend_unit;
    }
  } else {
    _angle_blend_enabled = false;
  }

  if (newData->distance_dropoff > 0.0)
  {
    _distance_blend_enabled = true;

    _distance_blend_start = newData->distance_less - newData->distance_dropoff;
    _distance_blend_unit = newData->distance_dropoff / blend_size;
    _distance_blend_bias = -_distance_blend_start + _distance_blend_unit * 0.5;

    float blend_unit;
    if (newData->attack_type & attack_lidar_blind_msgs::msg::LidarAttackConfig::ATTACK_TYPE_RAND_DROP)
    {
      blend_unit = newData->random_drop_percent / (float)blend_size;
    }
    else {
      blend_unit = 1.0 / (float)blend_size;
    }

    for(int i = 0; i < blend_size; i++)
    {
      _distance_blend_lookup[i] = (blend_size - i) * blend_unit;
    }
  } else {
    _distance_blend_enabled = false;;
  }

  data = newData;
  _angle_h = data->attack_angle / 2;
}

float LidarBlindAttackConfig::angle_blend_lookup(int x)
{ 
  if (x < 0 || x >= blend_size)
  {
    return 0;
  }

  return _angle_blend_lookup[x]; 
}

float LidarBlindAttackConfig::distance_blend_lookup(int x) 
{ 
  if (x < 0 || x >= blend_size)
  {
    return 0;
  }

  return _distance_blend_lookup[x]; 
}

void LidarBlindAttackConfigNode::mapbin_callback(autoware_auto_mapping_msgs::msg::HADMapBin::UniquePtr msg)
{
  lanelet::LaneletMapPtr lanelet_map(new lanelet::LaneletMap);

  lanelet::utils::conversion::fromBinMsg(*msg, lanelet_map);
  RCLCPP_INFO(this->get_logger(), "Revieved Map\n");

  // lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map);
  config->lanelets = lanelet::utils::query::laneletLayer(lanelet_map);

  //Store the crosswalk lanelets for use when needed
  // crosswalk_lanelets = lanelet::utils::query::crosswalkLanelets(all_lanelets);
  RCLCPP_INFO(this->get_logger(), "Found %lu lanelets\n", config->lanelets.size());
}

void LidarBlindAttackConfigNode::config_callback(attack_lidar_blind_msgs::msg::LidarAttackConfig::UniquePtr msg)
{
  //move config
  config->set_data(std::move(msg));
}