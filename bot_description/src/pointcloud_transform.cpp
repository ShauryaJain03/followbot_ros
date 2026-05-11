#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

class PointCloudTransform : public rclcpp::Node
{
public:
  PointCloudTransform()
  : Node("pointcloud_transform")
  {
    declare_parameter("target_frame", "laser_link");
    declare_parameter("input_topic", "/points");
    declare_parameter("output_topic", "/velodyne_points");
    declare_parameter("scan_rate", 9.3);
    declare_parameter("num_scan_lines", 32);
    declare_parameter("vertical_fov_min", -15.0);
    declare_parameter("vertical_fov_max", 15.0);

    target_frame_ = get_parameter("target_frame").as_string();
    const auto input_topic = get_parameter("input_topic").as_string();
    const auto output_topic = get_parameter("output_topic").as_string();
    scan_rate_ = get_parameter("scan_rate").as_double();
    num_scan_lines_ = get_parameter("num_scan_lines").as_int();
    vert_fov_min_rad_ = get_parameter("vertical_fov_min").as_double() * M_PI / 180.0;
    vert_fov_max_rad_ = get_parameter("vertical_fov_max").as_double() * M_PI / 180.0;
    scan_period_ = 1.0 / scan_rate_;

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::QoS input_qos(10);
    input_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    input_qos.durability(rclcpp::DurabilityPolicy::Volatile);

    rclcpp::QoS output_qos(10);
    output_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    output_qos.durability(rclcpp::DurabilityPolicy::Volatile);

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, output_qos);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic,
      input_qos,
      std::bind(&PointCloudTransform::pointcloud_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "PointCloud Transform: %s -> %s (target: %s, scan_rate: %.1f Hz, lines: %d)",
      input_topic.c_str(),
      output_topic.c_str(),
      target_frame_.c_str(),
      scan_rate_,
      num_scan_lines_);
  }

private:
  static bool has_field(const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
  {
    for (const auto & field : cloud.fields) {
      if (field.name == name) {
        return true;
      }
    }
    return false;
  }

  uint16_t compute_ring(float x, float y, float z) const
  {
    const float range_xy = std::sqrt(x * x + y * y);
    float elevation = std::atan2(z, range_xy);
    elevation = std::max(
      static_cast<float>(vert_fov_min_rad_),
      std::min(static_cast<float>(vert_fov_max_rad_), elevation));

    const float ratio =
      (elevation - static_cast<float>(vert_fov_min_rad_)) /
      static_cast<float>(vert_fov_max_rad_ - vert_fov_min_rad_);
    const int ring = static_cast<int>(std::round(ratio * (num_scan_lines_ - 1)));
    return static_cast<uint16_t>(std::max(0, std::min(num_scan_lines_ - 1, ring)));
  }

  static float compute_time_offset(float x, float y, double scan_period)
  {
    float azimuth = std::atan2(y, x);
    if (azimuth < 0.0f) {
      azimuth += 2.0f * static_cast<float>(M_PI);
    }
    return static_cast<float>((azimuth / (2.0 * M_PI)) * scan_period);
  }

  sensor_msgs::msg::PointCloud2 add_velodyne_fields(
    const sensor_msgs::msg::PointCloud2 & cloud_in) const
  {
    const bool need_time = !has_field(cloud_in, "time");
    const bool need_ring = !has_field(cloud_in, "ring");
    const bool need_intensity = !has_field(cloud_in, "intensity");

    if (!need_time && !need_ring && !need_intensity) {
      auto cloud_out = cloud_in;
      // Gazebo clouds often mark is_dense=false even when finite enough for LIO-SAM.
      cloud_out.is_dense = true;
      return cloud_out;
    }

    const uint32_t num_points = cloud_in.width * cloud_in.height;

    sensor_msgs::PointCloud2ConstIterator<float> in_x(cloud_in, "x");
    sensor_msgs::PointCloud2ConstIterator<float> in_y(cloud_in, "y");
    sensor_msgs::PointCloud2ConstIterator<float> in_z(cloud_in, "z");

    sensor_msgs::msg::PointCloud2 cloud_out;
    cloud_out.header = cloud_in.header;
    cloud_out.height = 1;
    cloud_out.width = num_points;
    // LIO-SAM rejects non-dense clouds before it even removes NaNs.
    // We synthesize a finite Velodyne-style cloud here, so advertise it as dense.
    cloud_out.is_dense = true;
    cloud_out.is_bigendian = cloud_in.is_bigendian;

    sensor_msgs::PointCloud2Modifier modifier(cloud_out);
    modifier.setPointCloud2Fields(
      6,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
      "ring", 1, sensor_msgs::msg::PointField::UINT16,
      "time", 1, sensor_msgs::msg::PointField::FLOAT32);

    sensor_msgs::PointCloud2Iterator<float> out_x(cloud_out, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(cloud_out, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(cloud_out, "z");
    sensor_msgs::PointCloud2Iterator<float> out_intensity(cloud_out, "intensity");
    sensor_msgs::PointCloud2Iterator<uint16_t> out_ring(cloud_out, "ring");
    sensor_msgs::PointCloud2Iterator<float> out_time(cloud_out, "time");

    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> in_intensity_ptr;
    if (!need_intensity) {
      in_intensity_ptr = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(
        cloud_in, "intensity");
    }

    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<uint16_t>> in_ring_ptr;
    if (!need_ring) {
      in_ring_ptr = std::make_unique<sensor_msgs::PointCloud2ConstIterator<uint16_t>>(
        cloud_in, "ring");
    }

    for (uint32_t i = 0; i < num_points;
      ++i, ++in_x, ++in_y, ++in_z,
      ++out_x, ++out_y, ++out_z, ++out_intensity, ++out_ring, ++out_time)
    {
      const float px = *in_x;
      const float py = *in_y;
      const float pz = *in_z;

      *out_x = px;
      *out_y = py;
      *out_z = pz;

      if (in_intensity_ptr) {
        *out_intensity = **in_intensity_ptr;
        ++(*in_intensity_ptr);
      } else {
        *out_intensity = 0.0f;
      }

      if (in_ring_ptr) {
        *out_ring = **in_ring_ptr;
        ++(*in_ring_ptr);
      } else {
        *out_ring = compute_ring(px, py, pz);
      }

      *out_time = compute_time_offset(px, py, scan_period_);
    }

    return cloud_out;
  }

  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const auto source_frame = msg->header.frame_id;
    if (source_frame.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Received PointCloud2 with empty frame_id");
      return;
    }

    try {
      const auto cloud_augmented = add_velodyne_fields(*msg);

      if (source_frame != target_frame_) {
        const auto transform_stamped = tf_buffer_->lookupTransform(
          target_frame_, source_frame, msg->header.stamp,
          rclcpp::Duration::from_seconds(0.1));

        sensor_msgs::msg::PointCloud2 cloud_out;
        tf2::doTransform(cloud_augmented, cloud_out, transform_stamped);
        cloud_out.is_dense = true;
        publisher_->publish(cloud_out);
      } else {
        auto cloud_out = cloud_augmented;
        cloud_out.is_dense = true;
        publisher_->publish(cloud_out);
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Transform error: %s", ex.what());
    }
  }

  std::string target_frame_;
  double scan_rate_;
  double scan_period_;
  int num_scan_lines_;
  double vert_fov_min_rad_;
  double vert_fov_max_rad_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudTransform>());
  rclcpp::shutdown();
  return 0;
}
