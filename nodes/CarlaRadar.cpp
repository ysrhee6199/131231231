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
  assert(radar_bp != nullptr);

  radar_transform = cg::Transform{
      cg::Location{2.4f, 0.0f, 0.8f},   // x, y, z.
      cg::Rotation{0.0f, 0.0f, 0.0f}}; // pitch, yaw, roll.
  radar_actor = world_.SpawnActor(*radar_bp, radar_transform, actor.get());
  radar = boost::static_pointer_cast<cc::Sensor>(radar_actor);

  radar->Listen([this](auto data) {
            auto radar_data = boost::static_pointer_cast<carla::sensor::data::RadarMeasurement>(data);
            assert(radar_data != nullptr);
            publishRadarData(radar_data);
    
  });
}




void CarlaRadarPublisher::publishRadarData(const boost::shared_ptr<csd::RadarMeasurement> &radar_data)
    {
        // Carla Radar 데이터를 ROS 2 메시지로 변환
        sensor_msgs::msg::PointCloud2 msg;
        // radar_data를 msg로 변환하는 코드 작성
        msg = ConvertRadarDataToROSMessage(radar_data);
        // 메시지 발행
        publisher_->publish(msg);
    }

    sensor_msgs::msg::PointCloud2 CarlaRadarPublisher::ConvertRadarDataToROSMessage(const boost::shared_ptr<carla::sensor::data::RadarMeasurement> &radar_data)
{
  sensor_msgs::msg::PointCloud2 radar_msg;
   

    // Populate the PointCloud2 message
    radar_msg.header.stamp = this->now();
    radar_msg.header.frame_id = "radar_frame";

    radar_msg.height = 1;
    radar_msg.width = radar_data->GetDetectionAmount();
    radar_msg.is_dense = false;
    radar_msg.is_bigendian = false;

    std::vector<sensor_msgs::msg::PointField> fields(7);

    // Define PointFields for x, y, z, Range, Velocity, AzimuthAngle, and ElevationAngle
    fields[0].name = "x";
    fields[0].offset = 0;
    fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[0].count = 1;

    // Define other fields similarly

    radar_msg.fields = fields;

    radar_msg.point_step = sizeof(float) * fields.size();
    radar_msg.row_step = radar_msg.point_step * radar_msg.width;

    std::vector<uint8_t> data(radar_msg.row_step * radar_msg.height);
    size_t offset = 0;

    for (size_t i = 0; i < radar_data->GetDetectionAmount(); ++i)
    {
      const auto &detection = radar_data->at(i);
      float x = detection.depth * std::cos(detection.azimuth) * std::cos(-detection.altitude);
      //float y = detection.depth * std::sin(-detection.azimuth) * std::cos(detection.altitude);
      //float z = detection.depth * std::sin(detection.altitude);
      //float range = detection.depth;
      //float velocity = detection.velocity;
      //float azimuth_angle = detection.azimuth;
      //float elevation_angle = detection.altitude;

      memcpy(&data[offset + fields[0].offset], &x, sizeof(float));
      // Copy other fields similarly

      offset += radar_msg.point_step;
    }

    radar_msg.data = data;

    return radar_msg;
}