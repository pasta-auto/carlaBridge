#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <fstream>

#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>
#include <carla/image/ImageIO.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>
#include <carla/sensor/data/GnssMeasurement.h>
#include <carla/sensor/data/IMUMeasurement.h>
#include <carla/sensor/data/LidarMeasurement.h>
#include <carla/sensor/data/RadarMeasurement.h>

#include "../../include/pasta_interface.hpp"

#include "../../server/include/remote_process.hpp"

#include "../../python/include/camera_module.hpp"
#include "../../server/include/camera.hpp"

//#include "lodepng/lodepng.h"
#include <dlfcn.h>

namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;

using namespace std::chrono_literals;
using namespace std::string_literals;

#define EXPECT_TRUE(pred) if (!(pred)) { throw std::runtime_error(#pred); }

/// Pick a random element from @a range.
template <typename RangeT, typename RNG>
static auto &RandomChoice(const RangeT &range, RNG &&generator) {
  EXPECT_TRUE(range.size() > 0u);
  std::uniform_int_distribution<size_t> dist{0u, range.size() - 1u};
  return range[dist(std::forward<RNG>(generator))];
}

/// Save a semantic segmentation image to disk converting to CityScapes palette.
// this example doesn't work the saved images clearly have wrong stride 
static void SaveSemSegImageToDisk(const csd::Image &image) {
  using namespace carla::image;

  char buffer[9u];
  std::snprintf(buffer, sizeof(buffer), "%08zu", image.GetFrame());
  auto filename = "_images/"s + buffer + ".png";

  auto view = ImageView::MakeView(image);
  ImageIO::WriteView(filename, view);
}

static auto ParseArguments(int argc, const char *argv[]) {
  EXPECT_TRUE((argc == 1u) || (argc == 3u));
  using ResultType = std::tuple<std::string, uint16_t>;
  return argc == 3u ?
      ResultType{argv[1u], std::stoi(argv[2u])} :
      ResultType{"localhost", 2000u};
}


int main(int argc, const char *argv[]) {
  try {
    remote_process apiRPC; 
    //camera_server camServ;
    //auto camThread = camServ.runAPIThreaded();

    std::string host;
    uint16_t port;
    //std::tie(host, port) = ParseArguments(argc, argv);

    std::mt19937_64 rng((std::random_device())());

    auto client = cc::Client("172.25.240.1", 2000);
    client.SetTimeout(10s);

    std::cout << "Client API version : " << client.GetClientVersion() << '\n';
    std::cout << "Server API version : " << client.GetServerVersion() << '\n';

    // Load a random town.
    auto town_name = RandomChoice(client.GetAvailableMaps(), rng);
    town_name = "Town01";
    std::cout << "Loading world: " << town_name << std::endl;
    auto world = client.LoadWorld(town_name);

    // Get a random vehicle blueprint.
    auto blueprint_library = world.GetBlueprintLibrary();
    auto vehicles = blueprint_library->Filter("vehicle");
    auto blueprint = RandomChoice(*vehicles, rng);

    // Randomize the blueprint.
    if (blueprint.ContainsAttribute("color")) {
      auto &attribute = blueprint.GetAttribute("color");
      blueprint.SetAttribute(
          "color",
          RandomChoice(attribute.GetRecommendedValues(), rng));
    }

    // Find a valid spawn point.
    auto map = world.GetMap();
    auto transform = RandomChoice(map->GetRecommendedSpawnPoints(), rng);

    // Spawn the vehicle.
    auto actor = world.SpawnActor(blueprint, transform);
    std::cout << "Spawned " << actor->GetDisplayId() << '\n';
    auto vehicle = boost::static_pointer_cast<cc::Vehicle>(actor);

    // Apply control to vehicle.
    cc::Vehicle::Control control;
    control.throttle = 1.0f;
    //vehicle->ApplyControl(control);
    vehicle->SetAutopilot(true);

    // Move spectator so we can see the vehicle from the simulator window.
    auto spectator = world.GetSpectator();
    transform.location += 32.0f * transform.GetForwardVector();
    transform.location.z += 2.0f;
    transform.rotation.yaw += 180.0f;
    transform.rotation.pitch = -15.0f;
    spectator->SetTransform(transform);

    // Find a camera blueprint.
    auto camera_bp = blueprint_library->at("sensor.camera.rgb");
    //EXPECT_TRUE(camera_bp != nullptr);
    camera_bp.SetAttribute("image_size_x", "640");
    camera_bp.SetAttribute("image_size_y", "480");

    // Spawn a camera attached to the vehicle.
    auto camera_transform = cg::Transform{
        cg::Location{-5.5f, 0.0f, 2.8f},   // x, y, z.
        cg::Rotation{-15.0f, 0.0f, 0.0f}}; // pitch, yaw, roll.
    auto cam_actor = world.SpawnActor(camera_bp, camera_transform, actor.get());
    auto camera = boost::static_pointer_cast<cc::Sensor>(cam_actor);

    // Register a callback to save images to disk.
    camera->Listen([&apiRPC](auto data) {
      //std::cout << "Run from thread: " << std::hash<std::thread::id>()(std::this_thread::get_id()) << std::endl;
      auto image = boost::static_pointer_cast<csd::Image>(data);
      EXPECT_TRUE(image != nullptr);
      auto height = image->GetHeight();
      auto width  = image->GetWidth();
      uint8_t* rawImData = (uint8_t*)image->data();
      apiRPC.camera_process(rawImData, width, height);
      //camServ.process(rawImData, width, height);
      //testNoSOLoad(rawImData, width, height);
      //SOLoadNoServer(rawImData, width, height);
      SaveSemSegImageToDisk(*image);
    });

    // Find a gnss blueprint.
    auto gnss_bp = blueprint_library->at("sensor.other.gnss");
    auto gnss_transform = cg::Transform{
        cg::Location{0.0f, 0.0f, 2.8f},   // x, y, z.
        cg::Rotation{0.0f, 0.0f, 0.0f}}; // pitch, yaw, roll.
    auto gnss_actor = world.SpawnActor(gnss_bp, gnss_transform, actor.get());
    auto gnss = boost::static_pointer_cast<cc::Sensor>(gnss_actor);
    gnss->Listen([&apiRPC](auto data) {
      auto gnssData = boost::static_pointer_cast<carla::sensor::data::GnssMeasurement>(data);

      double lat = gnssData->GetLatitude();
      double lon = gnssData->GetLongitude();
      double alt = gnssData->GetAltitude();
      //std::cout << "(" << lat << "," << lon << ")";
      apiRPC.gnss_process(lat, lon, alt);
      //std::cout << "  --> (" << lat << "," << lon << ")" << std::endl;
    });

    auto imu_bp = blueprint_library->at("sensor.other.imu");
    auto imu_transform = cg::Transform{
        cg::Location{0.0f, 0.0f, 2.8f},   // x, y, z.
        cg::Rotation{0.0f, 0.0f, 0.0f}}; // pitch, yaw, roll.
    auto imu_actor = world.SpawnActor(imu_bp, imu_transform, actor.get());
    auto imu = boost::static_pointer_cast<cc::Sensor>(imu_actor);
    imu->Listen([&apiRPC](auto data) {
      auto imuData = boost::static_pointer_cast<carla::sensor::data::IMUMeasurement>(data);
      float acc_x = imuData->GetAccelerometer().x;
      float acc_y = imuData->GetAccelerometer().y;
      float acc_z = imuData->GetAccelerometer().z;

      float gyr_x = imuData->GetGyroscope().x;
      float gyr_y = imuData->GetGyroscope().y;
      float gyr_z = imuData->GetGyroscope().z;

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
    });

    auto lidar_bp = blueprint_library->at("sensor.lidar.ray_cast");
    auto lidar_transform = cg::Transform{
        cg::Location{0.0f, 0.0f, 2.8f},   // x, y, z.
        cg::Rotation{0.0f, 0.0f, 0.0f}}; // pitch, yaw, roll.
    auto lidar_actor = world.SpawnActor(lidar_bp, lidar_transform, actor.get());
    auto lidar = boost::static_pointer_cast<cc::Sensor>(lidar_actor);
    lidar->Listen([&apiRPC](auto data) {
      auto lidarData = boost::static_pointer_cast<carla::sensor::data::LidarMeasurement>(data);
      size_t numPoints = 0;
      for(unsigned int x = 0; x < lidarData->GetChannelCount(); x++) {numPoints += lidarData->GetPointCount(x);}
      float ha = lidarData->GetHorizontalAngle();
      size_t channels = lidarData->GetChannelCount();
      float * rawData = (float*)lidarData->data();
      //if (numPoints >= 3) {
      //  std::cout << "First 3 intensities: "<< std::endl;
      //  std::cout << rawData[3] << ", " << rawData[7] <<  ", " << rawData[11] << "   ----->" << std::endl;
      //}
      apiRPC.lidar_process(rawData, numPoints, ha, channels);
      //if (numPoints >= 3) {
      //  std::cout << rawData[3] << ", " << rawData[7] <<  ", " << rawData[11] << std::endl;
      //}
    });

    auto radar_bp = blueprint_library->at("sensor.other.radar");
    auto radar_transform = cg::Transform{
        cg::Location{0.0f, 0.0f, 2.8f},   // x, y, z.
        cg::Rotation{0.0f, 0.0f, 0.0f}}; // pitch, yaw, roll.
    auto radar_actor = world.SpawnActor(radar_bp, radar_transform, actor.get());
    auto radar = boost::static_pointer_cast<cc::Sensor>(radar_actor);
    radar->Listen([&apiRPC](auto data) {
      auto radarData = boost::static_pointer_cast<carla::sensor::data::RadarMeasurement>(data);
      size_t numPoints = radarData->GetDetectionAmount();
      float * rawData = (float*)radarData->data();
      apiRPC.radar_process(rawData, numPoints);
    });
    std::cout << "Press button to continue" << std::endl;
    std::cin.get();

    // Remove actors from the simulation.
    vehicle->Destroy();
    camera->Destroy();
    gnss->Destroy();
    imu->Destroy();
    lidar->Destroy();
    radar->Destroy();
    std::cout << "Actors destroyed." << std::endl;

  } catch (const cc::TimeoutException &e) {
    std::cout << '\n' << e.what() << std::endl;
    return 1;
  } catch (const std::exception &e) {
    std::cout << "\nException: " << e.what() << std::endl;
    return 2;
  }
}