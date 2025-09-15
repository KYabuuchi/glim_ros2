#include "glim_ros/patchwork-plusplus.hpp"
#include "patchwork/patchworkpp.h"
#include <memory>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <regex>

namespace patchworkplusplus {

namespace {
std::unique_ptr<patchwork::PatchWorkpp> Patchworkpp_;
}

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointField = sensor_msgs::msg::PointField;
using Header = std_msgs::msg::Header;

inline Eigen::MatrixXf PointCloud2ToEigenMat(const PointCloud2& msg) {
  sensor_msgs::PointCloud2ConstIterator<float> msg_x(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> msg_y(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> msg_z(msg, "z");

  Eigen::MatrixXf points;
  size_t num_points = msg.height * msg.width;
  points.resize(num_points, 3);

  for (size_t i = 0; i < num_points; ++i, ++msg_x, ++msg_y, ++msg_z) {
    points.row(i) << *msg_x, *msg_y, *msg_z;
  }

  return points;
}

inline std::string FixFrameId(const std::string& frame_id) {
  return std::regex_replace(frame_id, std::regex("^/"), "");
}

inline std::unique_ptr<PointCloud2> CreatePointCloud2Msg(const size_t n_points, const Header& header, bool timestamp = false) {
  auto cloud_msg = std::make_unique<PointCloud2>();
  sensor_msgs::PointCloud2Modifier modifier(*cloud_msg);
  cloud_msg->header = header;
  cloud_msg->header.frame_id = FixFrameId(cloud_msg->header.frame_id);
  cloud_msg->fields.clear();
  int offset = 0;
  offset = addPointField(*cloud_msg, "x", 1, PointField::FLOAT32, offset);
  offset = addPointField(*cloud_msg, "y", 1, PointField::FLOAT32, offset);
  offset = addPointField(*cloud_msg, "z", 1, PointField::FLOAT32, offset);
  offset += sizeOfPointField(PointField::FLOAT32);
  if (timestamp) {
    // assuming timestamp on a velodyne fashion for now (between 0.0 and 1.0)
    offset = addPointField(*cloud_msg, "time", 1, PointField::FLOAT64, offset);
    offset += sizeOfPointField(PointField::FLOAT64);
  }

  // Resize the point cloud accordingly
  cloud_msg->point_step = offset;
  cloud_msg->row_step = cloud_msg->width * cloud_msg->point_step;
  cloud_msg->data.resize(cloud_msg->height * cloud_msg->row_step);
  modifier.resize(n_points);
  return cloud_msg;
}

inline std::unique_ptr<PointCloud2> EigenMatToPointCloud2(const Eigen::MatrixX3f& points, const Header& header) {
  auto msg = CreatePointCloud2Msg(points.rows(), header);

  sensor_msgs::PointCloud2Iterator<float> msg_x(*msg, "x");
  sensor_msgs::PointCloud2Iterator<float> msg_y(*msg, "y");
  sensor_msgs::PointCloud2Iterator<float> msg_z(*msg, "z");
  for (size_t i = 0; i < points.size(); i++, ++msg_x, ++msg_y, ++msg_z) {
    const Eigen::Vector3f& point = points.row(i);
    *msg_x = point.x();
    *msg_y = point.y();
    *msg_z = point.z();
  }

  return msg;
}

Patchworkpp::Patchworkpp(const glim::Config& config_ros) {
  // TODO(KYabuuchi): load parameters from config_ros
  patchwork::Params params;

  // TODO(KYabuuchi): Not supported yet ?
  params.enable_RNR = false;

  // Construct the main Patchwork++ node
  Patchworkpp_ = std::make_unique<patchwork::PatchWorkpp>(params);
}

void Patchworkpp::split_ground(const sensor_msgs::msg::PointCloud2& input_cloud, sensor_msgs::msg::PointCloud2& ground_cloud, sensor_msgs::msg::PointCloud2& nonground_cloud) {
  const auto& cloud = PointCloud2ToEigenMat(input_cloud);

  // Estimate ground
  Patchworkpp_->estimateGround(cloud);

  // Get ground and nonground
  Eigen::MatrixX3f ground = Patchworkpp_->getGround();
  Eigen::MatrixX3f nonground = Patchworkpp_->getNonground();

  EigenMatToPointCloud2(ground, ground_cloud.header);
  EigenMatToPointCloud2(nonground, nonground_cloud.header);
}
}  // namespace patchworkplusplus