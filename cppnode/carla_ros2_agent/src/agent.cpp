#include <carla/client/Actor.h>
#include <carla/client/ActorList.h>

#include <carla_ros2_agent/agent.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <builtin_interfaces/msg/time.hpp>

CarlaAutowareRos2::Agent::Agent()
{
  autoware_node = std::make_shared<CarlaAutowareRos2::AutowareAgent>();

  std::weak_ptr<std::remove_pointer<decltype(autoware_node.get())>::type> autoware_agent_weak = autoware_node;
  carla_node = std::make_shared<CarlaAutowareRos2::CarlaAgent>(autoware_agent_weak);
  
  spawn_node = std::make_shared<carla_ego_spawn_handler>();

  rclcpp::spin_some(autoware_node);
  rclcpp::spin_some(carla_node);
  rclcpp::spin_some(spawn_node);

  executor.add_node(carla_node);
  executor.add_node(autoware_node);
  executor.add_node(spawn_node);
}

void CarlaAutowareRos2::Agent::run_step()
{
  static carla::SharedPtr<carla::client::Actor> egoVehicle;

  if (auto world = world_ptr.lock() )
  {
    auto snapshot = world->GetSnapshot();

    auto timestamp = snapshot.GetTimestamp();

    //Publish ros2 clock
    {
      double time = timestamp.elapsed_seconds;
      auto clock_msg = std::make_unique<rosgraph_msgs::msg::Clock>();
      clock_msg->clock.sec = (int)time;
      clock_msg->clock.nanosec = (time - clock_msg->clock.sec) * 1000000000;
      autoware_node->pub_clock(std::move(clock_msg));
    }

    if (!spectator)
    {
      spectator = world->GetSpectator();
    }

    //Find the ego vehicle
    if (!egoVehicle || !egoVehicle->IsAlive())
    {
      auto actorList = world->GetActors();
      auto filteredActorList = actorList->Filter("*vehicle*");
      for(const auto& actor : *filteredActorList)
      {
        for (const auto& att : actor->GetAttributes())
        {
          if (att.GetId() == "role_name" && att.GetValue() == carla_node->get_agent_role_name())
          {
            egoVehicle = actor;
          }
        }
      }

      carla_node->sensors_check();
    }

    if (egoVehicle)
    {
      carla_node->carla_vehicle_status_update();

      auto ego_trans = egoVehicle->GetTransform();
      auto ego_forward = ego_trans.GetForwardVector();
      carla::geom::Rotation egoRot = carla::geom::Rotation(-25, ego_trans.rotation.yaw, 0);
      carla::geom::Location egoLoc = ego_trans.location + carla::geom::Location(-ego_forward.x * 15, -ego_forward.y * 15 ,10);
      carla::geom::Transform spec_trans = carla::geom::Transform(egoLoc, egoRot);
      spectator->SetTransform(spec_trans);
    }
  }
}

void CarlaAutowareRos2::Agent::set_world(std::weak_ptr<carla::client::World> newWorld) 
{
  world_ptr = newWorld;

  carla_node->set_world(newWorld);
  spawn_node->set_world(newWorld);
}

void CarlaAutowareRos2::Agent::prep()
{
  std::cout << "Starting carla_ros2_agent" << std::endl << std::flush;

  spawn_node->destroy_ego(get_agent_role_name());
  spawn_node->destroy_ego(spawn_node->spawn_role_name);
  spawn_node->spawn_ego(get_spawn_object_path());
}

void CarlaAutowareRos2::Agent::start()
{
  executor.spin();

  std::cout << "Stopping carla_ros2_agent" << std::endl << std::flush;

  spawn_node->destroy_ego(get_agent_role_name());
  spawn_node->destroy_ego(spawn_node->spawn_role_name);

  rclcpp::shutdown(nullptr, "CarlaAutowareRos2 Agent stop called");
}

std::string CarlaAutowareRos2::Agent::get_carla_ip()
{
  if (carla_node)
  {
    return carla_node->get_carla_ip();
  }
  else
  {
    return "127.0.0.1";
  }
}

int CarlaAutowareRos2::Agent::get_carla_port()
{
  if (carla_node)
  {
    return carla_node->get_carla_port();
  }
  else
  {
    return 2000;
  }
}
  
int CarlaAutowareRos2::Agent::get_frame_rate()
{
  if (carla_node)
  {
    return carla_node->get_frame_rate();
  }
  else
  {
    return 20;
  }
}

std::string CarlaAutowareRos2::Agent::get_agent_role_name()
{
  if (carla_node)
  {
    return carla_node->get_agent_role_name();
  }
  else
  {
    return "";
  }
}

std::string CarlaAutowareRos2::Agent::get_spawn_object_path()
{
  if (carla_node)
  {
    return carla_node->get_spawn_objects_path();
  }
  else
  {
    return "";
  }
}

