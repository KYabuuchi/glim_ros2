#include "glim_ros/nebula_adaptor.hpp"
#include <nebula_common/velodyne/velodyne_common.hpp>
#include <nebula_decoders/nebula_decoders_velodyne/velodyne_driver.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <spdlog/spdlog.h>

namespace nebula {
namespace {
std::shared_ptr<drivers::VelodyneDriver> driver_ptr_;
}

Status initialize_driver(
  std::shared_ptr<const drivers::VelodyneSensorConfiguration> sensor_configuration,
  std::shared_ptr<const drivers::VelodyneCalibrationConfiguration> calibration_configuration) {
  // driver should be initialized here with proper decoder
  driver_ptr_ = std::make_shared<drivers::VelodyneDriver>(sensor_configuration, calibration_configuration);
  return driver_ptr_->get_status();
}

nebula::Status
get_parameters(const glim::Config& config_ros, drivers::VelodyneSensorConfiguration& sensor_configuration, drivers::VelodyneCalibrationConfiguration& calibration_configuration) {
  {
    auto sensor_model = config_ros.param<std::string>("glim_ros", "sensor_model");
    if (!sensor_model) {
      spdlog::error("sensor_model parameter is required");
    }
    sensor_configuration.sensor_model = nebula::drivers::sensor_model_from_string(sensor_model.value());
  }
  {
    auto return_mode = config_ros.param<std::string>("glim_ros", "return_mode");
    if (!return_mode) {
      spdlog::error("return_mode parameter is required");
    }
    sensor_configuration.return_mode = nebula::drivers::return_mode_from_string(return_mode.value());
  }
  {
    auto frame_id = config_ros.param<std::string>("glim_ros", "frame_id");
    if (!frame_id) {
      spdlog::error("frame_id parameter is required");
    }
    sensor_configuration.frame_id = frame_id.value();
  }
  {
    auto scan_phase = config_ros.param<double>("glim_ros", "scan_phase");
    sensor_configuration.scan_phase = scan_phase.value();
  }
  {
    auto calibration_file = config_ros.param<std::string>("glim_ros", "calibration_file");
    calibration_configuration.calibration_file = calibration_file.value();
  }
  {
    auto min_range = config_ros.param<double>("glim_ros", "min_range");
    sensor_configuration.min_range = min_range.value();
  }
  {
    auto max_range = config_ros.param<double>("glim_ros", "max_range");
    sensor_configuration.max_range = max_range.value();
  }
  double view_direction = sensor_configuration.scan_phase * M_PI / 180;
  double view_width = 360 * M_PI / 180;
  {
    auto view_direction_param = config_ros.param<double>("glim_ros", "view_direction");
    view_width = view_direction_param.value() * M_PI / 180;
  }

  if (sensor_configuration.sensor_model != nebula::drivers::SensorModel::VELODYNE_HDL64) {
    {
      auto cloud_min_angle = config_ros.param<int>("glim_ros", "cloud_min_angle");
      sensor_configuration.cloud_min_angle = cloud_min_angle.value();
    }
    {
      auto cloud_max_angle = config_ros.param<int>("glim_ros", "cloud_max_angle");
      sensor_configuration.cloud_max_angle = cloud_max_angle.value();
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

VelodyneDecoder::VelodyneDecoder(const glim::Config& config_ros) {
  drivers::VelodyneCalibrationConfiguration calibration_configuration;
  drivers::VelodyneSensorConfiguration sensor_configuration;

  Status wrapper_status_ = get_parameters(config_ros, sensor_configuration, calibration_configuration);
  if (nebula::Status::OK != wrapper_status_) {
    throw std::runtime_error("Failed to get Velodyne parameters");
  }

  auto calibration_cfg_ptr_ = std::make_shared<const drivers::VelodyneCalibrationConfiguration>(calibration_configuration);
  auto sensor_cfg_ptr_ = std::make_shared<const drivers::VelodyneSensorConfiguration>(sensor_configuration);

  wrapper_status_ = initialize_driver(sensor_cfg_ptr_, calibration_cfg_ptr_);

  if (nebula::Status::OK != wrapper_status_) {
    throw std::runtime_error("Failed to initialize Velodyne driver");
  }
}

void VelodyneDecoder::convert_velodyne_packet_to_pointcloud2(const velodyne_msgs::msg::VelodyneScan& packet_msg, sensor_msgs::msg::PointCloud2& points_msg) {
  for (auto& pkt : packet_msg.packets) {
    auto pointcloud_ts = driver_ptr_->parse_cloud_packet(std::vector<uint8_t>(pkt.data.begin(), std::next(pkt.data.begin(), pkt.data.size())), pkt.stamp.sec);
    auto pointcloud = std::get<0>(pointcloud_ts);

    if (!pointcloud) {
      continue;
    }
    pcl::toROSMsg(*pointcloud, points_msg);
    break;
  }
}
}  // namespace nebula