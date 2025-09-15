#pragma once
#include <velodyne_msgs/msg/velodyne_packet.hpp>
#include <velodyne_msgs/msg/velodyne_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nebula_decoders/nebula_decoders_velodyne/velodyne_driver.hpp>
#include <glim/util/config.hpp>

namespace nebula {

class VelodyneDecoder {
public:
  explicit VelodyneDecoder(const glim::Config& config_ros);

  void convert_velodyne_packet_to_pointcloud2(const velodyne_msgs::msg::VelodyneScan& packet_msg, sensor_msgs::msg::PointCloud2& points_msg);

private:
};

}  // namespace nebula