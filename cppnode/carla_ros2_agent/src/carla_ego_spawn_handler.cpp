#include <carla_ros2_agent/carla_ego_spawn_handler.hpp>

#include <autoware_auto_perception_msgs/msg/object_classification.hpp>

#include <carla/client/Actor.h>
#include <carla/client/ActorList.h>
#include <carla/client/Vehicle.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Map.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <random>
#include <utility>

using namespace std::chrono_literals;

inline double to_degrees(double radians)
{
    return (radians * 180.0) / M_PI;
}

carla_ego_spawn_handler::carla_ego_spawn_handler() : rclcpp::Node("bridge_spawn_agent")
{
    spawn_object_sub = this->create_subscription<dummy_perception_publisher::msg::Object>("/carla/spawn", rclcpp::QoS(1), std::bind(&carla_ego_spawn_handler::spawn_handler, this, std::placeholders::_1));
}

void carla_ego_spawn_handler::spawn_handler(dummy_perception_publisher::msg::Object::UniquePtr msg)
{
    switch(msg->action)
    {
        case dummy_perception_publisher::msg::Object::DELETEALL:
            destroy_ego(spawn_role_name);
            break;

        case dummy_perception_publisher::msg::Object::ADD:
            spawn_actor_object(msg->classification.label, msg->initial_state.pose_covariance.pose);
            break;

        default:
            break;
    }
}

void carla_ego_spawn_handler::spawn_actor_object(const int type, geometry_msgs::msg::Pose pose)
{
    if (auto world = world_ptr.lock())
    {
        auto blueprints = world->GetBlueprintLibrary();
        std::shared_ptr<carla::geom::Transform> bpTransform = std::make_shared<carla::geom::Transform>();

        std::string actor_type;

        switch(type)
        {
            case autoware_auto_perception_msgs::msg::ObjectClassification::PEDESTRIAN:
                actor_type = "walker.pedestrian.0001";
                break;

            default:
                return;
        }

        auto bpMaster = blueprints->Find(actor_type);

        if (!bpMaster)
        {
            std::cerr << actor_type << " is not a known blueprint, skipping spawn" << std::endl;
            return;
        }

        auto bp = std::make_shared<carla::client::ActorBlueprint>(*bpMaster);

        bp->SetAttribute("role_name", spawn_role_name);

        carla::geom::Location loc;
        carla::geom::Rotation rot;

        loc.x = pose.position.x;
        loc.y = -pose.position.y;
        loc.z = pose.position.z;

        tf2::Quaternion q(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w);
        tf2::Matrix3x3 m(q);

        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        rot.pitch = -to_degrees(pitch);
        rot.yaw = -to_degrees(yaw);
        rot.roll = to_degrees(roll);

        bpTransform = std::make_shared<carla::geom::Transform>(loc, rot);

        world->SpawnActor(*bp, *bpTransform);
        std::cout << "Spawned " << actor_type << " at " << loc.x << ", " << loc.y << ", " << loc.z << std::endl;
    }

}

void carla_ego_spawn_handler::set_world(std::weak_ptr<carla::client::World> world_ptr)
{
  this->world_ptr = world_ptr;
}

void carla_ego_spawn_handler::spawn_ego(std::string spawn_objects_path)
{
    std::ifstream ifs(spawn_objects_path);
    json objects = json::parse(ifs);

    if (auto world = world_ptr.lock())
    {
        for(json object : objects["objects"])
        {
            if (!spawn_actor_json(object, world))
            {
                std::cerr << "Error spawning object: " << object.dump() << std::endl;
            }
        }
    }
}

void carla_ego_spawn_handler::destroy_ego(const std::string& role_name)
{
    std::set<int> actor_parents = std::set<int>();

    if (auto world = world_ptr.lock())
    {
        auto actors_list = world->GetActors();

        //Find all actors with this role_name
        for(const auto& actor : *actors_list)
        {
            for(const auto& att : actor->GetAttributes())
            {
                if (att.GetId() == "role_name" && att.GetValue() == role_name)
                {
                    actor_parents.insert(actor->GetId());
                }
            }
        }

        //Remove all actors with parents that have role_name
        for(const auto& actor : *actors_list)
        {
            auto parentSearch = actor_parents.find(actor->GetParentId());
            if (parentSearch != actor_parents.end())
            {
                if (!actor->Destroy())
                {
                    std::cerr << "Issue destroying actor id:" << actor->GetId() << " type:" << actor->GetTypeId() << std::endl;
                }
                else 
                {
                    std::cout << "Destroyed actor id:" << actor->GetId() << " type:" << actor->GetTypeId() << " with parent id: " << *parentSearch << std::endl;
                }
            }
        }

        //Remove all the found parent actors
        for(const auto& actor : *actors_list)
        {
            auto search = actor_parents.find(actor->GetId());
            if (search != actor_parents.end())
            {
                if (!actor->Destroy())
                {
                    std::cerr << "Issue destroying actor id:" << actor->GetId() << " type:" << actor->GetTypeId() << std::endl;
                }
                else 
                {
                    std::cout << "Destroyed actor id:" << actor->GetId() << " type:" << actor->GetTypeId() << std::endl;
                }
            }
        }
    }
}

bool carla_ego_spawn_handler::spawn_actor_json(json actor_json, std::shared_ptr<carla::client::World> world, carla::SharedPtr<carla::client::Actor> parent)
{
    auto blueprints = world->GetBlueprintLibrary();
    std::shared_ptr<carla::geom::Transform> bpTransform = std::make_shared<carla::geom::Transform>();

    //Do checks for required entries
    if (!actor_json.contains("id"))
    {
        std::cerr << "Spawn actor 'id' is missing, skipping entry: " << actor_json.dump() << std::endl;
        return false;
    }
    if (!actor_json.contains("type"))
    {
        std::cerr << "Spawn actor 'type' is missing, skipping entry: " << actor_json.dump() << std::endl;
        return false;
    }

    auto bpMaster = blueprints->Find(actor_json["type"]);
    if (!bpMaster)
    {
        std::cerr << actor_json["type"] << " is not a known blueprint, skipping entry" << std::endl;
        return false;
    }

    auto bp = std::make_shared<carla::client::ActorBlueprint>(*bpMaster);
  
    for(auto tag : actor_json.items())
    {
        std::string key = tag.key();
        std::string value;

        if (tag.value().is_number_float())
        {
            value = std::to_string(tag.value().get<double>());
        }
        else if (tag.value().is_number())
        {
            value = std::to_string(tag.value().get<int>());
        }
        else if (tag.value().is_string())
        {
            value = tag.value();
        }

        if (bp->ContainsAttribute(key))
        {
            bp->SetAttribute(key, value);
        }
        else if (key == "id")
        {
            bp->SetAttribute("role_name", value);
        }
        else if (key == "spawn_point")
        {
            json loc_json = actor_json[key];
            carla::geom::Location loc;
            carla::geom::Rotation rot;

            if (loc_json.contains("x"))
            {
                loc.x = loc_json["x"];
            }
            if (loc_json.contains("y"))
            {
                loc.y = loc_json["y"];
            }
            if (loc_json.contains("z"))
            {
                loc.z = loc_json["z"];
            }

            if (loc_json.contains("roll"))
            {
                rot.roll = loc_json["roll"];
            }
            if (loc_json.contains("pitch"))
            {
                rot.pitch = loc_json["pitch"];
            }
            if (loc_json.contains("yaw"))
            {
                rot.yaw = loc_json["yaw"];
            }

            bpTransform = std::make_shared<carla::geom::Transform>(loc, rot);
        }
    }

    //Get Vehicle spawn location if not set
    if (bp->ContainsTag("vehicle") && !actor_json.contains("spawn_point"))
    {
        auto spawn_points = world->GetMap()->GetRecommendedSpawnPoints();

        std::random_device r;
        std::default_random_engine re(r());
        std::uniform_int_distribution<int> uniform_dist(0, spawn_points.size() - 1);
        
        bpTransform = std::make_shared<carla::geom::Transform>(spawn_points[uniform_dist(re)]);
    }

    //Spawn actor
    carla::SharedPtr<carla::client::Actor> spawned_actor;
    spawned_actor = world->SpawnActor(*bp, *bpTransform, parent.get());
    // if (bp->ContainsTag("sensor"))
    // {
    //     std::shared_ptr<carla::geom::Transform> bpTransformZero = std::make_shared<carla::geom::Transform>();
    //     spawned_actor = world->SpawnActor(*bp, *bpTransformZero, parent.get());
    // }
    // else
    // {
    //     spawned_actor = world->SpawnActor(*bp, *bpTransform, parent.get());
    // }
    
    if (spawned_actor)
    {
        // if (bp->ContainsTag("sensor"))
        // {
        //     spawned_actor->SetTransform(*bpTransform);
        // }

        std::cout << "Spawned id:" << spawned_actor->GetId() << " type:" << spawned_actor->GetTypeId() << std::endl;
    }
    else
    {
        return false;
    }
    
    //Need to tick the world to spawn fully
    world->Tick(2s);

    //This is a vehicle so spawn sensors
    if (bp->ContainsTag("vehicle"))
    {
        //Set vehicle to park by default
        auto control = boost::static_pointer_cast<carla::client::Vehicle>(spawned_actor)->GetControl();
        control.gear = 0;
        control.hand_brake = true;
        boost::static_pointer_cast<carla::client::Vehicle>(spawned_actor)->ApplyControl(control);

        if (actor_json.contains("sensors"))
        {
            for(json sensor_json : actor_json["sensors"])
            {
                if (!spawn_actor_json(sensor_json, world, spawned_actor))
                {
                    std::cout << "FAILED to spawned id:" << sensor_json["id"] << " type:" << sensor_json["type"] << std::endl;
                }
            }
        }
    }

    return true;
}