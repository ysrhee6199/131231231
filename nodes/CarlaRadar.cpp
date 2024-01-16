#include "CarlaRadar.hpp"

#include <boost/make_shared.hpp>

CarlaRadarPublisher::CarlaRadarPublisher(boost::shared_ptr<carla::client::BlueprintLibrary> blueprint_library, boost::shared_ptr<carla::client::Actor> actor,carla::client::World& world_)
    : Node("carla_radar_publisher"),world_(world_) {

       rclcpp::QoS custom_qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    custom_qos.best_effort();

  this->blueprint_library = blueprint_library;
  this->actor = actor;
  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/carla/radar", custom_qos);

  radar_bp = boost::shared_ptr<carla::client::ActorBlueprint>(
    const_cast<carla::client::ActorBlueprint*>(blueprint_library->Find("sensor.other.radar"))
);
  radar_bp->SetAttribute("sensor_tick", "0.1f");
  radar_bp->SetAttribute("horizontal_fov", "15.0f");
  radar_bp->SetAttribute("points_per_second", "1500");
  radar_bp->SetAttribute("vertical_fov", "15.0f");
  radar_bp->SetAttribute("range", "70f");
  assert(radar_bp != nullptr);

  radar_transform = cg::Transform{
      cg::Location{2.3f, 0.0f, 1.9f},   // x, y, z.
      cg::Rotation{5.0f, 0.0f, 0.0f}}; // pitch, yaw, roll.
  radar_actor = world_.SpawnActor(*radar_bp, radar_transform, actor.get());
  radar = boost::static_pointer_cast<cc::Sensor>(radar_actor);

  radar->Listen([this](auto data) {
            auto radar_data = boost::static_pointer_cast<carla::sensor::data::RadarMeasurement>(data);
            assert(radar_data != nullptr);
           publishRadarData(radar_data);
  });
}




void CarlaRadarPublisher::publishRadarData(const boost::shared_ptr<csd::RadarMeasurement> &carla_radar_measurement)
    {
    sensor_msgs::msg::PointCloud2 radar_msg;
    radar_msg.header.stamp = this->now();
    radar_msg.header.frame_id = "radar_frame";

    radar_msg.height = 1;
    radar_msg.width = carla_radar_measurement->GetDetectionAmount();
    radar_msg.is_dense = false;
    radar_msg.is_bigendian = false;

    std::vector<sensor_msgs::msg::PointField> fields(7);

    // Define PointFields for x, y, z, Range, Velocity, AzimuthAngle, and ElevationAngle
    fields[0].name = "x";
    fields[0].offset = 0;
    fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[0].count = 1;

    fields[1].name = "y";
    fields[1].offset = 4;
    fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[1].count = 1;

    fields[2].name = "z";
    fields[2].offset = 8;
    fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[2].count = 1;

    fields[3].name = "Range";
    fields[3].offset = 12;
    fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[3].count = 1;

    fields[4].name = "Velocity";
    fields[4].offset = 16;
    fields[4].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[4].count = 1;

    fields[5].name = "AzimuthAngle";
    fields[5].offset = 20;
    fields[5].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[5].count = 1;

    fields[6].name = "ElevationAngle";
    fields[6].offset = 24;
    fields[6].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[6].count = 1;

    radar_msg.fields = fields;

    radar_msg.point_step = sizeof(float) * fields.size();
    radar_msg.row_step = radar_msg.point_step * radar_msg.width;

    std::vector<uint8_t> data(radar_msg.row_step * radar_msg.height);
  
    size_t offset = 0;
   
    for (size_t i = 0; i < carla_radar_measurement->GetDetectionAmount(); ++i)
    {
      const auto &detection = carla_radar_measurement->at(i);
      float x = detection.depth * std::cos(detection.azimuth) * std::cos(-detection.altitude);
      float y = detection.depth * std::sin(-detection.azimuth) * std::cos(detection.altitude);
      float z = detection.depth * std::sin(detection.altitude);
      float range = detection.depth;
      float velocity = detection.velocity;
      float azimuth_angle = detection.azimuth;
      float elevation_angle = detection.altitude;

      memcpy(&data[offset + fields[0].offset], &x, sizeof(float));
      memcpy(&data[offset + fields[1].offset], &y, sizeof(float));
      memcpy(&data[offset + fields[2].offset], &z, sizeof(float));
      memcpy(&data[offset + fields[3].offset], &range, sizeof(float));
      memcpy(&data[offset + fields[4].offset], &velocity, sizeof(float));
      memcpy(&data[offset + fields[5].offset], &azimuth_angle, sizeof(float));
      memcpy(&data[offset + fields[6].offset], &elevation_angle, sizeof(float));
 
      offset += radar_msg.point_step;
      std::cerr << "Point " << i << ": ("
              << x << ", " << y << ", " << z << ") "
              << "Distance: " << detection.depth << std::endl;
     
    }
    radar_msg.data = data;
    publisher_->publish(radar_msg);
 
    }

