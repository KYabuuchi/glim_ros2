#pragma once
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <memory>
#include <rclcpp/node.hpp>

namespace nebula {

class Decoder {
public:
  explicit Decoder(rclcpp::Node* node);

  bool convert_packets_to_pointcloud2(const std::string& topic_type, const rclcpp::SerializedMessage& serialized_msg, sensor_msgs::msg::PointCloud2& points_msg);

private:
};

// class VelodyneDecoder {
// public:
//   explicit VelodyneDecoder(const Parameter& config_ros);

//   void convert_velodyne_packet_to_pointcloud2(const velodyne_msgs::msg::VelodyneScan& packet_msg, sensor_msgs::msg::PointCloud2& points_msg);

// private:
// };

}  // namespace nebula