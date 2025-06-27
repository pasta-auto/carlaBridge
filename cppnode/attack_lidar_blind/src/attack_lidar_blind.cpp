#include <cstdio>

#include "attack_lidar_blind/attack_lidar_blind.hpp"
#include "attack_lidar_blind/config_node.hpp"

#include <rclcpp/node_options.hpp>

// #include <boost/geometry/geometry.hpp>
#include <lanelet2_core/geometry/Lanelet.h>
#include <rclcpp/logger.hpp>

#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <algorithm>

using std::placeholders::_1;

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);

  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  auto config = std::make_shared<LidarBlindAttackConfigNode>();
  auto attack = std::make_shared<LidarBlindAttackNode>(config->get_config());

  executor->add_node(config);
  executor->add_node(attack);
  executor->spin();

  return 0;
}

LidarBlindAttackNode::LidarBlindAttackNode(std::shared_ptr<LidarBlindAttackConfig> config) : Node("attack_lidar_blind", rclcpp::NodeOptions().use_intra_process_comms(true))
{
  this->attack_config = config;

  std::string lidar_subscription_path = this->declare_parameter("input", "/attack/lidar_blind/input");
  std::string lidar_publish_path = this->declare_parameter("output", "/sensing/lidar/top/outlier_filtered/pointcloud");
  std::string lidar_status_path = this->declare_parameter("status", "/attack/lidar_blind/status");
  
  tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  lidar_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_subscription_path, rclcpp::SensorDataQoS(), std::bind(&LidarBlindAttackNode::lidar_callback, this, _1));

  lidar_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(lidar_publish_path, rclcpp::SensorDataQoS());
  status_pub = this->create_publisher<attack_lidar_blind_msgs::msg::LidarBlindStatus>(lidar_status_path, rclcpp::SensorDataQoS());
}

void LidarBlindAttackNode::lidar_callback(sensor_msgs::msg::PointCloud2::UniquePtr msg)
{
  //Do this to have a shallow copy so the config is not modified on the callback but on the next run
  LidarBlindAttackConfig config = *attack_config;

  using LidarAttackConfig = attack_lidar_blind_msgs::msg::LidarAttackConfig;

  auto attack_status_msg = std::make_unique<attack_lidar_blind_msgs::msg::LidarBlindStatus>();

  attack_status_msg->config = config.get_data();
  attack_status_msg->header = msg->header;

  //Immediately publish the current if not enabled as a passthough
  if (config.attack_type() == LidarAttackConfig::ATTACK_TYPE_NONE)
  {
    lidar_pub->publish(std::move(msg));
    status_pub->publish(std::move(attack_status_msg));

    return;
  }

  geometry_msgs::msg::TransformStamped tf_msg;

  std::string fromTF = msg->header.frame_id;
  std::string toTF = "map";

  float bound_x_upper = 0, bound_y_upper = 0;
  float bound_x_lower = 0, bound_y_lower = 0;

  if (config.attack_type() & LidarAttackConfig::ATTACK_TYPE_REQUIRE_TF)
  {
    try {
      tf_msg = tf_buffer->lookupTransform(toTF, fromTF, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", toTF.c_str(), fromTF.c_str(), ex.what());
      return;
    }
  }

  if (config.attack_type() & LidarAttackConfig::ATTACK_TYPE_LANELET)
  {
    float bound_x_lower_front = std::min(config.attack_lanelet->leftBound2d().front().x(), config.attack_lanelet->rightBound2d().front().x());
    float bound_x_lower_back = std::min(config.attack_lanelet->leftBound2d().back().x(), config.attack_lanelet->rightBound2d().back().x());
    bound_x_lower = std::min(bound_x_lower_front, bound_x_lower_back);
    float bound_x_upper_front = std::max(config.attack_lanelet->leftBound2d().front().x(), config.attack_lanelet->rightBound2d().front().x());
    float bound_x_upper_back = std::max(config.attack_lanelet->leftBound2d().back().x(), config.attack_lanelet->rightBound2d().back().x());
    bound_x_upper = std::max(bound_x_upper_front, bound_x_upper_back);

    float bound_y_lower_front = std::min(config.attack_lanelet->leftBound2d().front().y(), config.attack_lanelet->rightBound2d().front().y());
    float bound_y_lower_back = std::min(config.attack_lanelet->leftBound2d().back().y(), config.attack_lanelet->rightBound2d().back().y());
    bound_y_lower = std::min(bound_y_lower_front, bound_y_lower_back);
    float bound_y_upper_front = std::max(config.attack_lanelet->leftBound2d().front().y(), config.attack_lanelet->rightBound2d().front().y());
    float bound_y_upper_back = std::max(config.attack_lanelet->leftBound2d().back().y(), config.attack_lanelet->rightBound2d().back().y());
    bound_y_upper = std::max(bound_y_upper_front, bound_y_upper_back);
  }

  auto new_lidar_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();

  uint32_t azimuth_offset = 9999;
  uint32_t distance_offset = 9999; 
  uint32_t x_offset = 9999; 
  uint32_t y_offset = 9999; 
  uint32_t z_offset = 9999;

  for(sensor_msgs::msg::PointField field : msg->fields)
  {
    //Get offsets
    if (field.name == azimuth_field_name)
    {
      azimuth_offset = field.offset;
    }

    if (field.name == distance_field_name)
    {
      distance_offset = field.offset;
    }

    if (field.name == x_field_name)
    {
      x_offset = field.offset;
    }

    if (field.name == y_field_name)
    {
      y_offset = field.offset;
    }

    if (field.name == z_field_name)
    {
      z_offset = field.offset;
    }
  }

  //Check that offsets were found that are expected
  if (azimuth_offset == 9999)
  {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *get_clock(), 5000, "Could not find offset for %s", azimuth_field_name.c_str());
    return;
  }
  if (distance_offset == 9999)
  {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *get_clock(), 5000, "Could not find offset for %s", distance_field_name.c_str());
    return;
  }
  if (x_offset == 9999)
  {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *get_clock(), 5000, "Could not find offset for %s", x_field_name.c_str());
    return;
  }
  if (y_offset == 9999)
  {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *get_clock(), 5000, "Could not find offset for %s", y_field_name.c_str());
    return;
  }
  if (z_offset == 9999)
  {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *get_clock(), 5000, "Could not find offset for %s", z_field_name.c_str());
    return;
  }

  //Setup message
  new_lidar_msg->header.stamp = msg->header.stamp;
  new_lidar_msg->header.frame_id = msg->header.frame_id;
  new_lidar_msg->fields = msg->fields;
  new_lidar_msg->is_bigendian = msg->is_bigendian;
  new_lidar_msg->point_step = msg->point_step;
  new_lidar_msg->height = msg->height;

  //Resize to current lidar msg data size to contain the max possible points assuming none removed
  new_lidar_msg->data.resize(msg->row_step);

  uint32_t new_width_count = 0;
  uint32_t i_new = 0;
  for(uint32_t i = 0; i < msg->row_step; i += msg->point_step)
  {
    auto point = msg->data.data() + i;

    LidarAttackConfig::_attack_type_type attack_check = LidarAttackConfig::ATTACK_TYPE_NONE;

    //Bool to say not to process random drop as blending occured
    bool blend_occured = false;

    float drop_chance = -1.0;

    lanelet::BasicPoint3d tf_output = lanelet::BasicPoint3d(0, 0, 0);

    if (config.attack_type() & LidarAttackConfig::ATTACK_TYPE_DISTANCE)
    {
      auto distance = *reinterpret_cast<_Float32*>(point + distance_offset);

      if(distance < config.distance_less() && distance > config.distance_greater())
      {
        if (config.distance_blend_enabled() && distance > config.distance_blend_start())
        {
          drop_chance = config.distance_blend_lookup(fabs(distance + config.distance_blend_bias()) / config.distance_blend_unit());

          blend_occured = true;
        }

        attack_check += LidarAttackConfig::ATTACK_TYPE_DISTANCE;
      }
      else{
        goto Process_Point;
      }
    }

    if (config.attack_type() & LidarAttackConfig::ATTACK_TYPE_ANGLE)
    {
      auto azumith = *reinterpret_cast<_Float32*>(point + azimuth_offset);

      if (azumith < config.angle_h() && azumith > -config.angle_h())
      {
        float azumith_abs = fabs(azumith);

        if (config.angle_blend_enabled() && azumith_abs > config.angle_blend_start())
        {
          float blend = config.angle_blend_lookup(fabs(azumith_abs + config.angle_blend_bias()) / config.angle_blend_unit());

          if (drop_chance >= 0)
          {
            constexpr float t = 0.5;
            constexpr float m_t = 1.0 - t;

            drop_chance = m_t * drop_chance + t * blend;
          }
          else
          {
            drop_chance = blend;
          }

          blend_occured = true;
        }

        attack_check += LidarAttackConfig::ATTACK_TYPE_ANGLE;
      }
      else{
        goto Process_Point;
      }
    }

    if (blend_occured)
    {
      if (config.get_random_percent() > drop_chance) 
      {
        goto Process_Point;
      }
    }

    if (!blend_occured && (config.attack_type() & LidarAttackConfig::ATTACK_TYPE_RAND_DROP))
    {

      if (config.get_random_percent() < config.random_drop_percent())
      {
        attack_check += LidarAttackConfig::ATTACK_TYPE_RAND_DROP;
      }
      else{
        goto Process_Point;
      }
    }

    //Do transform only if needed due to cost
    if (config.attack_type() & LidarAttackConfig::ATTACK_TYPE_REQUIRE_TF)
    {
      auto x = *reinterpret_cast<_Float32*>(point + x_offset);
      auto y = *reinterpret_cast<_Float32*>(point + y_offset);
      auto z = *reinterpret_cast<_Float32*>(point + z_offset);

      tf2::doTransform(lanelet::BasicPoint3d(x, y, z), tf_output, tf_msg);
    }

    if (config.attack_type() & LidarAttackConfig::ATTACK_TYPE_Z)
    {
      if (tf_output.z() >= config.z_min())
      {
        attack_check += LidarAttackConfig::ATTACK_TYPE_Z;
      }
      else{
        goto Process_Point;
      }
    }

    if (config.attack_type() & LidarAttackConfig::ATTACK_TYPE_LANELET)
    {
      if (tf_output.x() >= bound_x_lower && tf_output.x() <= bound_x_upper && tf_output.y() >= bound_y_lower && tf_output.y() <= bound_y_upper)
      {
        if (lanelet::geometry::inside(*(config.attack_lanelet), lanelet::BasicPoint2d(tf_output.x(), tf_output.y())))
        {
          attack_check += LidarAttackConfig::ATTACK_TYPE_LANELET;
        }
      }
      else{
        goto Process_Point;
      }
    }

    if (config.attack_type() & LidarAttackConfig::ATTACK_TYPE_BOUND_BOX)
    {
      if (tf_output.x() >= config.bound_x_lower() && tf_output.x() <= config.bound_x_upper() 
        && tf_output.y() >= config.bound_y_lower() && tf_output.y() <= config.bound_y_upper()
        && ( !config.bound_z_valid() || (tf_output.z() >= config.bound_z_lower() && tf_output.z() <= config.bound_z_upper())))
      {
        attack_check += LidarAttackConfig::ATTACK_TYPE_BOUND_BOX;
      }
      else{
        goto Process_Point;
      }
    }

    attack_status_msg->points_removed++;
    continue;

Process_Point:
    attack_status_msg->points_processed++;

    //Copy point data over
    auto new_point = new_lidar_msg->data.data() + i_new;
    memcpy(new_point, point, msg->point_step);

    new_width_count++;
    i_new += msg->point_step;
  }

  //Set new width and row step size, then resize to proper size for the removed points
  new_lidar_msg->width = new_width_count;
  new_lidar_msg->row_step = new_lidar_msg->point_step * new_lidar_msg->width;
  new_lidar_msg->data.resize(new_lidar_msg->row_step);

  lidar_pub->publish(std::move(new_lidar_msg));
  status_pub->publish(std::move(attack_status_msg));
}


