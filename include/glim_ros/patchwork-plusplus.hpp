#pragma once
#include <glim/util/config.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace patchworkplusplus {

class Patchworkpp {
public:
  explicit Patchworkpp(const glim::Config& config_ros);

  void split_ground(const sensor_msgs::msg::PointCloud2& input_cloud, sensor_msgs::msg::PointCloud2& ground_cloud, sensor_msgs::msg::PointCloud2& nonground_cloud);
};
}  // namespace patchworkplusplus