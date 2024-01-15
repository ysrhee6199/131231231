#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
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
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <carla/sensor/data/RadarMeasurement.h>
#include "sensor_msgs/msg/point_cloud2.hpp"
namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;
using namespace std::chrono_literals;
using namespace std::string_literals;




class CarlaRadarPublisher : public rclcpp::Node {

public:
  CarlaRadarPublisher(boost::shared_ptr<carla::client::BlueprintLibrary> blueprint_library, boost::shared_ptr<carla::client::Actor> actor, carla::client::World& world_);
  ~CarlaRadarPublisher(){
    radar->Destroy();
  }
private:
  boost::shared_ptr<carla::client::BlueprintLibrary> blueprint_library = nullptr;
  boost::shared_ptr<carla::client::Actor> actor = nullptr; 
  carla::client::World& world_;
  void publishRadarData(const boost::shared_ptr<csd::RadarMeasurement> &radar_data);
  sensor_msgs::msg::PointCloud2 ConvertRadarDataToROSMessage(const boost::shared_ptr<csd::RadarMeasurement> &radar_data);
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    boost::shared_ptr<carla::client::Sensor> radar;
    boost::shared_ptr<carla::client::Actor> radar_actor;
    carla::geom::Transform radar_transform;
    boost::shared_ptr<carla::client::ActorBlueprint> radar_bp;

};