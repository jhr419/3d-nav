#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace
{

std::string expandUserPath(const std::string & path)
{
  if (path.empty() || path[0] != '~') {
    return path;
  }

  if (path.size() > 1 && path[1] != '/') {
    return path;
  }

  const char * home = std::getenv("HOME");
  if (home == nullptr || std::string(home).empty()) {
    return path;
  }

  if (path.size() <= 2) {
    return home;
  }

  return (std::filesystem::path(home) / path.substr(2)).string();
}

std::string toLower(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
    return static_cast<char>(std::tolower(character));
  });
  return value;
}

bool hasPointCloudContainerExtension(const std::string & input_path)
{
  const std::string extension = toLower(std::filesystem::path(input_path).extension().string());
  return extension == ".ply" || extension == ".pcd";
}

std::string makeDefaultOutputPath(
  const std::string & input_path,
  const std::string & output_dir)
{
  std::filesystem::path input(input_path);
  std::filesystem::path output_name = input.filename();
  output_name.replace_extension(".pcd");

  if (!output_dir.empty()) {
    return (std::filesystem::path(output_dir) / output_name).string();
  }

  std::filesystem::path output = input;
  output.replace_extension(".pcd");
  return output.string();
}

bool isFinitePoint(float x, float y, float z, float intensity)
{
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && std::isfinite(intensity);
}

}  // namespace

class Bin2PcdNode : public rclcpp::Node
{
public:
  Bin2PcdNode()
  : Node("bin2pcd_node")
  {
    input_bin_path_ = expandUserPath(declare_parameter<std::string>("input_bin_path", ""));
    output_pcd_path_ = expandUserPath(declare_parameter<std::string>("output_pcd_path", ""));
    output_dir_ = expandUserPath(declare_parameter<std::string>("output_dir", ""));
    fields_per_point_ = declare_parameter<int>("fields_per_point", 4);
    intensity_field_index_ = declare_parameter<int>("intensity_field_index", 3);
    save_binary_ = declare_parameter<bool>("save_binary", true);
    overwrite_ = declare_parameter<bool>("overwrite", true);
    skip_invalid_points_ = declare_parameter<bool>("skip_invalid_points", true);
    exit_after_save_ = declare_parameter<bool>("exit_after_save", true);

    using namespace std::chrono_literals;
    timer_ = create_wall_timer(100ms, [this]() {
      timer_->cancel();
      succeeded_ = convertAndSave();

      if (exit_after_save_) {
        rclcpp::shutdown();
      }
    });
  }

  bool succeeded() const
  {
    return succeeded_;
  }

private:
  bool convertAndSave()
  {
    if (input_bin_path_.empty()) {
      RCLCPP_ERROR(
        get_logger(),
        "Parameter input_bin_path is empty. Example: ros2 run map_mannger bin2pcd_node "
        "--ros-args -p input_bin_path:=/path/map.bin");
      return false;
    }

    if (hasPointCloudContainerExtension(input_bin_path_)) {
      RCLCPP_ERROR(
        get_logger(),
        "input_bin_path points to a PLY/PCD file, but bin2pcd_node expects raw float32 .bin "
        "data. Pass the original .bin map, or convert PLY to PCD directly with "
        "`pcl_ply2pcd input.ply output.pcd`.");
      return false;
    }

    if (fields_per_point_ < 3) {
      RCLCPP_ERROR(get_logger(), "fields_per_point must be >= 3, got %d", fields_per_point_);
      return false;
    }

    if (intensity_field_index_ >= fields_per_point_) {
      RCLCPP_WARN(
        get_logger(),
        "intensity_field_index=%d is outside fields_per_point=%d. Intensity will be set to 0.",
        intensity_field_index_, fields_per_point_);
    }

    std::ifstream input(input_bin_path_, std::ios::binary | std::ios::ate);
    if (!input.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open input bin file: %s", input_bin_path_.c_str());
      return false;
    }

    const std::streamoff byte_size = input.tellg();
    if (byte_size <= 0) {
      RCLCPP_ERROR(get_logger(), "Input bin file is empty: %s", input_bin_path_.c_str());
      return false;
    }

    const auto record_bytes =
      static_cast<std::streamoff>(fields_per_point_ * static_cast<int>(sizeof(float)));
    if (byte_size % record_bytes != 0) {
      RCLCPP_ERROR(
        get_logger(),
        "Input file size (%ld bytes) is not divisible by fields_per_point * sizeof(float) "
        "(%ld bytes). Check fields_per_point.",
        static_cast<long>(byte_size), static_cast<long>(record_bytes));
      return false;
    }

    const std::size_t point_count = static_cast<std::size_t>(byte_size / record_bytes);
    const std::size_t float_count = point_count * static_cast<std::size_t>(fields_per_point_);
    std::vector<float> raw_values(float_count);

    input.seekg(0, std::ios::beg);
    input.read(
      reinterpret_cast<char *>(raw_values.data()),
      static_cast<std::streamsize>(float_count * sizeof(float)));
    if (!input) {
      RCLCPP_ERROR(get_logger(), "Failed to read input bin file: %s", input_bin_path_.c_str());
      return false;
    }

    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.reserve(point_count);

    std::size_t skipped_points = 0;
    for (std::size_t i = 0; i < point_count; ++i) {
      const std::size_t offset = i * static_cast<std::size_t>(fields_per_point_);
      const float x = raw_values[offset];
      const float y = raw_values[offset + 1];
      const float z = raw_values[offset + 2];
      const float intensity =
        intensity_field_index_ >= 0 && intensity_field_index_ < fields_per_point_
        ? raw_values[offset + static_cast<std::size_t>(intensity_field_index_)]
        : 0.0F;

      if (skip_invalid_points_ && !isFinitePoint(x, y, z, intensity)) {
        ++skipped_points;
        continue;
      }

      pcl::PointXYZI point;
      point.x = x;
      point.y = y;
      point.z = z;
      point.intensity = intensity;
      cloud.push_back(point);
    }

    cloud.width = static_cast<std::uint32_t>(cloud.size());
    cloud.height = 1;
    cloud.is_dense = skip_invalid_points_;

    const std::string output_path = output_pcd_path_.empty()
      ? makeDefaultOutputPath(input_bin_path_, output_dir_)
      : output_pcd_path_;
    const std::filesystem::path output_fs_path(output_path);
    const std::filesystem::path parent_path = output_fs_path.parent_path();

    if (!parent_path.empty()) {
      std::filesystem::create_directories(parent_path);
    }

    if (!overwrite_ && std::filesystem::exists(output_fs_path)) {
      RCLCPP_ERROR(get_logger(), "Output PCD already exists and overwrite is false: %s", output_path.c_str());
      return false;
    }

    const int save_result = save_binary_
      ? pcl::io::savePCDFileBinary(output_path, cloud)
      : pcl::io::savePCDFileASCII(output_path, cloud);
    if (save_result != 0) {
      RCLCPP_ERROR(get_logger(), "Failed to save output PCD file: %s", output_path.c_str());
      return false;
    }

    const std::string skipped_message = skipped_points > 0
      ? "; skipped " + std::to_string(skipped_points) + " invalid points"
      : "";
    RCLCPP_INFO(
      get_logger(),
      "Converted %zu points from '%s' to '%s'%s.",
      cloud.size(), input_bin_path_.c_str(), output_path.c_str(), skipped_message.c_str());
    return true;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::string input_bin_path_;
  std::string output_pcd_path_;
  std::string output_dir_;
  int fields_per_point_{4};
  int intensity_field_index_{3};
  bool save_binary_{true};
  bool overwrite_{true};
  bool skip_invalid_points_{true};
  bool exit_after_save_{true};
  bool succeeded_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Bin2PcdNode>();
  rclcpp::spin(node);
  return node->succeeded() ? 0 : 1;
}
