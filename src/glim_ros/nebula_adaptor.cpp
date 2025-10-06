#include "glim_ros/nebula_adaptor.hpp"
#include <nebula_common/velodyne/velodyne_common.hpp>
#include <nebula_ros/common/rclcpp_logger.hpp>
#include <nebula_decoders/nebula_decoders_velodyne/velodyne_driver.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <spdlog/spdlog.h>
#include <nebula_decoders/nebula_decoders_velodyne/velodyne_driver.hpp>
#include <nebula_decoders/nebula_decoders_hesai/hesai_driver.hpp>
#include <pandar_msgs/msg/pandar_scan.hpp>
#include <velodyne_msgs/msg/velodyne_packet.hpp>
#include <velodyne_msgs/msg/velodyne_scan.hpp>
#include <rclcpp/serialization.hpp>

namespace nebula {
namespace {

std::shared_ptr<drivers::VelodyneDriver> velodyne_driver_ptr_ = nullptr;
std::shared_ptr<drivers::HesaiDriver> hesai_driver_ptr_ = nullptr;
rclcpp::Serialization<velodyne_msgs::msg::VelodyneScan> velodyne_packets_serialization;
rclcpp::Serialization<pandar_msgs::msg::PandarScan> pandar_packets_serialization;
}  // namespace

Status initialize_velodyne_driver(
  std::shared_ptr<const drivers::VelodyneSensorConfiguration> sensor_configuration,
  std::shared_ptr<const drivers::VelodyneCalibrationConfiguration> calibration_configuration) {
  // driver should be initialized here with proper decoder
  velodyne_driver_ptr_ = std::make_shared<drivers::VelodyneDriver>(sensor_configuration, calibration_configuration);
  return velodyne_driver_ptr_->get_status();
}

Status get_parameters(
  rclcpp::Node* node,
  drivers::HesaiSensorConfiguration& sensor_configuration,
  drivers::HesaiCalibrationConfiguration& calibration_configuration,
  drivers::HesaiCorrection& correction_configuration) {
  //
  // auto sensor_model_ = this->declare_parameter<std::string>("sensor_model", "");
  // sensor_configuration.sensor_model = nebula::drivers::sensor_model_from_string(sensor_model_);
  auto return_mode_ = node->declare_parameter<std::string>("return_mode");
  sensor_configuration.return_mode = nebula::drivers::return_mode_from_string_hesai(return_mode_, sensor_configuration.sensor_model);
  sensor_configuration.frame_id = node->declare_parameter<std::string>("frame_id");

  sensor_configuration.sync_angle = node->declare_parameter<uint16_t>("sync_angle");
  sensor_configuration.cut_angle = node->declare_parameter<double>("cut_angle");

  sensor_configuration.cloud_max_angle = node->declare_parameter<int>("cloud_max_angle");
  sensor_configuration.cloud_min_angle = node->declare_parameter<int>("cloud_min_angle");
  sensor_configuration.max_range = node->declare_parameter<double>("max_range");
  sensor_configuration.min_range = node->declare_parameter<double>("min_range");

  calibration_configuration.calibration_file = node->declare_parameter<std::string>("calibration_file");
  // TODO: support PandarAT128
  // if (sensor_configuration.sensor_model == drivers::SensorModel::HESAI_PANDARAT128) {
  //   correction_file_path_ = node->declare_parameter<std::string>("correction_file");
  // }

  if (sensor_configuration.sensor_model == nebula::drivers::SensorModel::UNKNOWN) {
    return Status::INVALID_SENSOR_MODEL;
  }
  if (sensor_configuration.return_mode == nebula::drivers::ReturnMode::UNKNOWN) {
    return Status::INVALID_ECHO_MODE;
  }
  if (sensor_configuration.frame_id.empty()) {
    return Status::SENSOR_CONFIG_ERROR;
  }
  if (calibration_configuration.calibration_file.empty()) {
    return Status::INVALID_CALIBRATION_FILE;
  } else {
    auto cal_status = calibration_configuration.load_from_file(calibration_configuration.calibration_file);
    if (cal_status != Status::OK) {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Given Calibration File: '" << calibration_configuration.calibration_file << "'");
      return cal_status;
    } else {
      RCLCPP_INFO_STREAM(node->get_logger(), "Given Calibration File: '" << calibration_configuration.calibration_file << "'");
    }
  }
  // TODO: support PandarAT128
  // if (sensor_configuration.sensor_model == drivers::SensorModel::HESAI_PANDARAT128) {
  //   if (correction_file_path_.empty()) {
  //     return Status::INVALID_CALIBRATION_FILE;
  //   } else {
  //     auto cal_status = correction_configuration.load_from_file(correction_file_path_);
  //     if (cal_status != Status::OK) {
  //       RCLCPP_ERROR_STREAM(node->get_logger(), "Given Correction File: '" << correction_file_path_ << "'");
  //       return cal_status;
  //     }
  //   }
  // }

  return Status::OK;
}

nebula::Status
get_parameters(rclcpp::Node* node, drivers::VelodyneSensorConfiguration& sensor_configuration, drivers::VelodyneCalibrationConfiguration& calibration_configuration) {
  {
    auto return_mode = node->declare_parameter<std::string>("glim_ros.return_mode");
    sensor_configuration.return_mode = nebula::drivers::return_mode_from_string(return_mode);
  }
  {
    auto frame_id = node->declare_parameter<std::string>("glim_ros.frame_id");
    sensor_configuration.frame_id = frame_id;
  }
  {
    auto scan_phase = node->declare_parameter<double>("glim_ros.scan_phase");
    sensor_configuration.scan_phase = scan_phase;
  }
  {
    auto calibration_file = node->declare_parameter<std::string>("glim_ros.calibration_file");
    calibration_configuration.calibration_file = calibration_file;
  }
  {
    auto min_range = node->declare_parameter<double>("glim_ros.min_range");
    sensor_configuration.min_range = min_range;
  }
  {
    auto max_range = node->declare_parameter<double>("glim_ros.max_range");
    sensor_configuration.max_range = max_range;
  }
  double view_direction = sensor_configuration.scan_phase * M_PI / 180;
  double view_width = 360 * M_PI / 180;
  {
    auto view_direction_param = node->declare_parameter<double>("glim_ros.view_direction");
    view_width = view_direction_param * M_PI / 180;
  }

  if (sensor_configuration.sensor_model != nebula::drivers::SensorModel::VELODYNE_HDL64) {
    {
      auto cloud_min_angle = node->declare_parameter<int>("glim_ros.cloud_min_angle");
      sensor_configuration.cloud_min_angle = cloud_min_angle;
    }
    {
      auto cloud_max_angle = node->declare_parameter<int>("glim_ros.cloud_max_angle");
      sensor_configuration.cloud_max_angle = cloud_max_angle;
    }
  } else {
    double min_angle = fmod(fmod(view_direction + view_width / 2, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
    double max_angle = fmod(fmod(view_direction - view_width / 2, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
    sensor_configuration.cloud_min_angle = static_cast<int>(std::lround(100 * (2 * M_PI - min_angle) * 180 / M_PI));
    sensor_configuration.cloud_max_angle = static_cast<int>(std::lround(100 * (2 * M_PI - max_angle) * 180 / M_PI));
    if (sensor_configuration.cloud_min_angle == sensor_configuration.cloud_max_angle) {
      // avoid returning empty cloud if min_angle = max_angle
      sensor_configuration.cloud_min_angle = 0;
      sensor_configuration.cloud_max_angle = 36000;
    }
  }
  if (sensor_configuration.sensor_model == nebula::drivers::SensorModel::UNKNOWN) {
    return Status::INVALID_SENSOR_MODEL;
  }
  if (sensor_configuration.return_mode == nebula::drivers::ReturnMode::UNKNOWN) {
    return Status::INVALID_ECHO_MODE;
  }
  if (sensor_configuration.frame_id.empty() || sensor_configuration.scan_phase > 360) {
    return Status::SENSOR_CONFIG_ERROR;
  }

  if (calibration_configuration.calibration_file.empty()) {
    return Status::INVALID_CALIBRATION_FILE;
  } else {
    auto cal_status = calibration_configuration.load_from_file(calibration_configuration.calibration_file);
    if (cal_status != Status::OK) {
      spdlog::error("Failed to load calibration file: {}", calibration_configuration.calibration_file);
      return cal_status;
    }
  }

  return Status::OK;
}

bool convert_pandar_packet_to_pointcloud2(const pandar_msgs::msg::PandarScan& packet_msg, sensor_msgs::msg::PointCloud2& points_msg) {
  for (const auto& pkt : packet_msg.packets) {
    auto pointcloud_ts = hesai_driver_ptr_->parse_cloud_packet(std::vector<uint8_t>(pkt.data.begin(), std::next(pkt.data.begin(), pkt.size)));
    auto pointcloud = std::get<0>(pointcloud_ts);

    if (!pointcloud) {
      continue;
    }
    pcl::toROSMsg(*pointcloud, points_msg);

    // TODO(KYabuuchi): more precise timestamp handling
    points_msg.header.frame_id = packet_msg.header.frame_id;
    points_msg.header.stamp = packet_msg.header.stamp;

    return true;
  }
  RCLCPP_WARN(rclcpp::get_logger("nebula"), "Pandar packet conversion resulted in no points");
  return false;
}

void convert_velodyne_packet_to_pointcloud2(const velodyne_msgs::msg::VelodyneScan& packet_msg, sensor_msgs::msg::PointCloud2& points_msg) {
  for (auto& pkt : packet_msg.packets) {
    auto pointcloud_ts = velodyne_driver_ptr_->parse_cloud_packet(std::vector<uint8_t>(pkt.data.begin(), std::next(pkt.data.begin(), pkt.data.size())), pkt.stamp.sec);
    auto pointcloud = std::get<0>(pointcloud_ts);

    if (!pointcloud) {
      continue;
    }
    pcl::toROSMsg(*pointcloud, points_msg);
    // TODO(KYabuuchi): more precise timestamp handling
    points_msg.header.frame_id = packet_msg.header.frame_id;
    points_msg.header.stamp = packet_msg.header.stamp;
  }
}

Decoder::Decoder(rclcpp::Node* node) {
  // TODO:
  const auto sensor_model = node->declare_parameter<std::string>("glim_ros.sensor_model");
  spdlog::info("Sensor model: {}", sensor_model);

  if (sensor_model == "VLP16" || sensor_model == "VLS128") {
    // Velodyne Decoder
    drivers::VelodyneCalibrationConfiguration calibration_configuration;
    drivers::VelodyneSensorConfiguration sensor_configuration;

    sensor_configuration = drivers::VelodyneSensorConfiguration();
    sensor_configuration.sensor_model = nebula::drivers::sensor_model_from_string(sensor_model);

    Status wrapper_status_ = get_parameters(node, sensor_configuration, calibration_configuration);
    if (nebula::Status::OK != wrapper_status_) {
      throw std::runtime_error("Failed to get Velodyne parameters");
    }

    auto calibration_cfg_ptr_ = std::make_shared<const drivers::VelodyneCalibrationConfiguration>(calibration_configuration);
    auto sensor_cfg_ptr_ = std::make_shared<const drivers::VelodyneSensorConfiguration>(sensor_configuration);

    wrapper_status_ = initialize_velodyne_driver(sensor_cfg_ptr_, calibration_cfg_ptr_);
    if (nebula::Status::OK != wrapper_status_) {
      throw std::runtime_error("Failed to initialize Velodyne driver");
    }
  } else if (sensor_model == "Pandar128E4X") {
    spdlog::info("try to setup Hesai Driver");

    drivers::HesaiCalibrationConfiguration calibration_configuration;
    drivers::HesaiSensorConfiguration sensor_configuration;
    drivers::HesaiCorrection correction_configuration;
    sensor_configuration.hires_mode = true;
    sensor_configuration.sensor_model = nebula::drivers::sensor_model_from_string(sensor_model);
    Status wrapper_status_ = get_parameters(node, sensor_configuration, calibration_configuration, correction_configuration);
    if (Status::OK != wrapper_status_) {
      spdlog::error("Error: wrapper_status_ is not OK");
      return;
    }

    auto calibration_cfg_ptr_ = std::make_shared<drivers::HesaiCalibrationConfiguration>(calibration_configuration);
    auto sensor_cfg_ptr_ = std::make_shared<drivers::HesaiSensorConfiguration>(sensor_configuration);

    hesai_driver_ptr_ = std::make_shared<drivers::HesaiDriver>(
      std::static_pointer_cast<drivers::HesaiSensorConfiguration>(sensor_cfg_ptr_),
      calibration_cfg_ptr_,
      std::make_shared<drivers::loggers::RclcppLogger>(node->get_logger()));

    std::cout << sensor_configuration << std::endl;
  } else {
    throw std::runtime_error("Unsupported sensor model : " + sensor_model);
  }
}

bool Decoder::convert_packets_to_pointcloud2(const std::string& topic_type, const rclcpp::SerializedMessage& serialized_msg, sensor_msgs::msg::PointCloud2& points_msg) {
  if (topic_type == "velodyne_msgs/msg/VelodyneScan") {
    // Deserialize the VelodyneScan message
    velodyne_msgs::msg::VelodyneScan packet_msg;
    velodyne_packets_serialization.deserialize_message(&serialized_msg, &packet_msg);

    // Convert the VelodyneScan message to PointCloud2
    convert_velodyne_packet_to_pointcloud2(packet_msg, points_msg);
    return true;
  }

  if (topic_type == "pandar_msgs/msg/PandarScan") {
    // Deserialize the PandarScan message
    pandar_msgs::msg::PandarScan packet_msg;
    pandar_packets_serialization.deserialize_message(&serialized_msg, &packet_msg);

    // Convert the PandarScan message to PointCloud2
    const bool success = convert_pandar_packet_to_pointcloud2(packet_msg, points_msg);
    return success;
  }

  spdlog::error("Unsupported topic type: {}", topic_type);
  return false;
}

}  // namespace nebula