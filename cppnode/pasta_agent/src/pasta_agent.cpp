#include <cstdio>

#include <pasta_agent/pasta_interface.hpp>
#include <pasta_agent/ros2_agent.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  Ros2Agent::createAgent();

  //Initialize pasta agent
  pasta_interface::createAgent("can", Ros2Agent::getAgent()->get_pasta_config());

  pasta_interface::getAgent()->set_net(Ros2Agent::getAgent()->get_pasta_port(), Ros2Agent::getAgent()->get_pasta_ip());
  pasta_interface::getAgent()->start();

  pasta_interface::getAgent()->set_mode(PASTA_MODE_2);
  pasta_interface::getAgent()->set("modeChangeCmd", PASTA_MODE_2);

  rclcpp::spin_some(Ros2Agent::getAgent());

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(Ros2Agent::getAgent());
  executor.spin();

  return 0;
}