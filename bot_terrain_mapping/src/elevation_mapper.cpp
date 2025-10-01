#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <cmath>
#include <memory>

class SimpleElevationMapper : public rclcpp::Node
{
public:
    SimpleElevationMapper() : Node("simple_elevation_mapper")
    {
        // Parameters
        this->declare_parameter("map_frame", "odom");
        this->declare_parameter("robot_frame", "base_footprint");
        this->declare_parameter("map_resolution", 0.1);
        this->declare_parameter("map_length", 20.0);
        this->declare_parameter("min_height", -2.0);
        this->declare_parameter("max_height", 2.0);

        map_frame_ = this->get_parameter("map_frame").as_string();
        robot_frame_ = this->get_parameter("robot_frame").as_string();
        resolution_ = this->get_parameter("map_resolution").as_double();
        map_length_ = this->get_parameter("map_length").as_double();
        min_height_ = this->get_parameter("min_height").as_double();
        max_height_ = this->get_parameter("max_height").as_double();

        // Initialize grid map
        grid_map_.setFrameId(map_frame_);
        grid_map_.setGeometry(grid_map::Length(map_length_, map_length_), resolution_);
        grid_map_.add("elevation", 0.0);
        grid_map_.add("variance", 0.01);
        grid_map_.add("traversability", 1.0);

        // TF2
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Subscribers
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lio_sam/mapping/cloud_registered", 10,
            std::bind(&SimpleElevationMapper::pointCloudCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/imu_raw", 10,
            std::bind(&SimpleElevationMapper::odometryCallback, this, std::placeholders::_1));

        // Publishers
        grid_map_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
            "/elevation_map", 10);

        // Timer for publishing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&SimpleElevationMapper::publishGridMap, this));

        RCLCPP_INFO(this->get_logger(), "Simple Elevation Mapper initialized");
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert to PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        if (cloud->empty()) return;

        // Get transform from sensor to map frame
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform(
                map_frame_, msg->header.frame_id,
                tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
            return;
        }

        // Transform point cloud to map frame
        Eigen::Isometry3d eigen_transform = tf2::transformToEigen(transform.transform);
        Eigen::Matrix4f tf_mat = eigen_transform.matrix().cast<float>();
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloud, *transformed_cloud, tf_mat);

        // Update grid map with robot position
        updateRobotPosition();

        // Process points
        for (const auto& point : transformed_cloud->points) {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
                continue;
            if (point.z < min_height_ || point.z > max_height_)
                continue;

            grid_map::Position position(point.x, point.y);
            if (!grid_map_.isInside(position))
                continue;

            grid_map::Index index;
            grid_map_.getIndex(position, index);

            try {
                if (!grid_map_.isValid(index, "elevation")) {
                    grid_map_.at("elevation", index) = point.z;
                    grid_map_.at("variance", index) = 0.01;
                } else {
                    double alpha = 0.1;
                    double old_val = grid_map_.at("elevation", index);
                    grid_map_.at("elevation", index) =
                        alpha * point.z + (1.0 - alpha) * old_val;
                }
            } catch (const std::out_of_range& e) {
                RCLCPP_WARN(this->get_logger(), "Index out of range: %s", e.what());
                continue;
            }
        }

        // Calculate traversability
        calculateTraversability();
    }

    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        robot_position_.x() = msg->pose.pose.position.x;
        robot_position_.y() = msg->pose.pose.position.y;

        // Move map to keep robot centered
        grid_map_.move(grid_map::Position(robot_position_.x(), robot_position_.y()));
    }

    void updateRobotPosition()
    {
        try {
            auto transform = tf_buffer_->lookupTransform(
                map_frame_, robot_frame_,
                tf2::TimePointZero);

            robot_position_.x() = transform.transform.translation.x;
            robot_position_.y() = transform.transform.translation.y;
        } catch (tf2::TransformException &ex) {
            // Use last known position
            (void)ex;
        }
    }

    void calculateTraversability()
    {
        double slope_threshold = 0.5; // radians (~28.6 degrees)

        for (grid_map::GridMapIterator iterator(grid_map_); !iterator.isPastEnd(); ++iterator) {
            const grid_map::Index index(*iterator);

            if (!grid_map_.isValid(index, "elevation")) {
                grid_map_.at("traversability", index) = 0.0;
                continue;
            }

            double center_height = grid_map_.at("elevation", index);
            double max_slope = 0.0;

            // Check 8 neighbors
            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    if (dx == 0 && dy == 0) continue;
                    grid_map::Index neighbor_index(index(0) + dx, index(1) + dy);
                    if (grid_map_.isValid(neighbor_index, "elevation")) {
                        double neighbor_height = grid_map_.at("elevation", neighbor_index);
                        double height_diff = std::abs(neighbor_height - center_height);
                        double distance = resolution_ * std::sqrt(static_cast<double>(dx*dx + dy*dy));
                        double slope = std::atan2(height_diff, distance);
                        if (slope > max_slope) max_slope = slope;
                    }
                }
            }
            grid_map_.at("traversability", index) =
                std::max(0.0, 1.0 - max_slope / slope_threshold);
        }
    }

    void publishGridMap()
    {
        auto msg_ptr = grid_map::GridMapRosConverter::toMessage(grid_map_);
        msg_ptr->header.stamp = this->now();
        if (msg_ptr->header.frame_id.empty()) {
            msg_ptr->header.frame_id = map_frame_;
        }
        grid_map_pub_->publish(std::move(*msg_ptr));
    }

    // Member variables
    grid_map::GridMap grid_map_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string map_frame_, robot_frame_;
    double resolution_, map_length_, min_height_, max_height_;
    grid_map::Position robot_position_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleElevationMapper>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}