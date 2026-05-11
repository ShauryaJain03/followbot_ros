#include <iostream>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cv_bridge/cv_bridge.h>
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../include/traversabilityGrid.h"

class TraversabilityNode : public rclcpp::Node {
  public:
    TraversabilityNode() : Node("minimal_subscriber") {
        auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());

        // Create subscriber
        std::string pc_topic;
        this->declare_parameter("pc_topic", rclcpp::ParameterValue("/point_cloud"));
        this->get_parameter("pc_topic", pc_topic);
        subscription_   = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pc_topic,
            sensor_qos,
            std::bind(&TraversabilityNode::pointcloud_callback, this, std::placeholders::_1));

        pubTraversability_ =
            this->create_publisher<grid_map_msgs::msg::GridMap>("RTQuadtree_struct", rclcpp::QoS(1).transient_local());

        pubImage_ =
            this->create_publisher<sensor_msgs::msg::Image>("RTQuadtree_image", rclcpp::QoS(1).transient_local());

        pubOccupancy_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("RTQuadtree_occupancyGrid",
                                                                             rclcpp::QoS(1).transient_local());
        puboc         = this->create_publisher<nav_msgs::msg::OccupancyGrid>("local", rclcpp::QoS(1).transient_local());

        this->declare_parameter("half_size", rclcpp::ParameterValue(7.5));
        this->get_parameter("half_size", half_size_);
        this->declare_parameter("resolution", rclcpp::ParameterValue(0.25));
        this->get_parameter("resolution", resolution_);

        this->declare_parameter("security_distance", rclcpp::ParameterValue(0.6));
        this->get_parameter("security_distance", security_distance_);
        this->declare_parameter("ground_clearance", rclcpp::ParameterValue(0.2));
        this->get_parameter("ground_clearance", ground_clearance_);
        this->declare_parameter("robot_height", rclcpp::ParameterValue(0.5));
        this->get_parameter("robot_height", robot_height_);
        this->declare_parameter("max_slope", rclcpp::ParameterValue(0.4));
        this->get_parameter("max_slope", max_slope_);
        this->declare_parameter("frame_id", rclcpp::ParameterValue("base_footprint"));
        this->get_parameter("frame_id", frame_id_);

        this->declare_parameter("robot_width", rclcpp::ParameterValue(0.8));
        this->get_parameter("robot_width", robot_width_);
        this->declare_parameter("robot_length", rclcpp::ParameterValue(1.1));
        this->get_parameter("robot_length", robot_length_);
        this->declare_parameter("draw_isodistance_each", rclcpp::ParameterValue(1.));
        this->get_parameter("draw_isodistance_each", draw_isodistance_each_);

        this->declare_parameter("global_mapping", rclcpp::ParameterValue(false));
        this->get_parameter("global_mapping", global_mapping_);


        traversabilityMap = std::make_shared<traversabilityGrid>(
            resolution_, Eigen::Vector2d(half_size_, half_size_), max_slope_, security_distance_, ground_clearance_);
        traversabilityGlobal = std::make_shared<traversabilityGrid>(
            resolution_, Eigen::Vector2d(half_size_, half_size_), max_slope_, security_distance_, ground_clearance_);
    }

  private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud) {

        point_cloud_ = point_cloud;
        publishtraversabilityMap();
    }

    void publishtraversabilityMap() {

        traversabilityMap->reset();

        // Fill traversability map 
        for (sensor_msgs::PointCloud2ConstIterator<float> it(*point_cloud_, "x"); it != it.end(); ++it) {
            Eigen::Vector3d pt3(it[0], it[1], it[2]);

            if (it[2] < 1.0)
                traversabilityMap->insertPoint(pt3);
        }

        // Compute the hazard grid
        traversabilityMap->computeHazGrid();

        // Fuse the local grid with the local map
        if (global_mapping_)
            traversabilityGlobal->fuseWithGrid(traversabilityMap);
        else
            traversabilityGlobal = traversabilityMap;

        // Compute gridmap
        grid_map::GridMap map({"roughness_haz", "pitch_haz", "step_haz", "hazard", "mean_elevation"});
        map.setFrameId(frame_id_);
        map.setGeometry(grid_map::Length(2. * half_size_, 2. * half_size_), resolution_);
        for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
            grid_map::Position position;
            map.getPosition(*it, position);
            Eigen::VectorXd haz = traversabilityGlobal->getHazMeters(Eigen::Vector2d(position.x(), position.y()));

            if (haz(0) < 0.)
                continue;


            map.at("hazard", *it)         = haz(0);
            map.at("step_haz", *it)       = haz(1);
            map.at("roughness_haz", *it)  = haz(2);
            map.at("pitch_haz", *it)      = haz(3);
            map.at("mean_elevation", *it) = haz(4);
        }

        auto message = grid_map::GridMapRosConverter::toMessage(map);
        pubTraversability_->publish(std::move(message));

        // convert grid map to CV image.
        cv::Mat originalImage, destImage;
        bool useTransparency = false;
        if (useTransparency) {
            // Note: The template parameters have to be set based on your encoding
            // of the image. For 8-bit images use `unsigned char`.
            grid_map::GridMapCvConverter::toImage<unsigned char, 3>(map, "hazard", CV_8UC3, 0.0, 1.0, originalImage);
        } else {
            grid_map::GridMapCvConverter::toImage<unsigned char, 1>(map, "hazard", CV_8UC1, 0.0, 1.0, originalImage);
        }

        // Transform iamge for visualization
        double scaling = 2;
        cv::resize(originalImage, originalImage, cv::Size(), scaling, scaling, cv::INTER_LINEAR);
        cv::applyColorMap(originalImage, originalImage, cv::COLORMAP_JET);

        // Draw robot footprint
        cv::Point2i center         = originalImage.size() / 2;
        cv::Point2i half_rect_size = cv::Point2i(ceil(scaling * 0.5 * robot_width_ / resolution_),
                                                 ceil(scaling * 0.5 * robot_length_ / resolution_));
        cv::rectangle(originalImage, center - half_rect_size, center + half_rect_size, cv::Scalar(0, 0, 255), 1);

        // Draw isodistances
        if (draw_isodistance_each_ > 0) {
            uint N = half_size_ * draw_isodistance_each_;
            for (uint i = 1; i < N; ++i)
                cv::circle(
                    originalImage, center, i * scaling * draw_isodistance_each_ / resolution_, cv::Scalar(0, 0, 0));
        }

        auto msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", originalImage).toImageMsg();
        pubImage_->publish(*msg_.get());

        nav_msgs::msg::OccupancyGrid occupancyGrid_msg;
        grid_map::GridMapRosConverter::toOccupancyGrid(map, "hazard", 0., 1., occupancyGrid_msg);
        pubOccupancy_->publish(occupancyGrid_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pubTraversability_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pubImage_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pubOccupancy_, puboc;

    sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_;
    std::shared_ptr<traversabilityGrid> traversabilityMap, traversabilityGlobal;
    double half_size_;
    double resolution_;
    double security_distance_;
    double ground_clearance_;
    double max_slope_;
    double robot_height_;
    double robot_length_;
    double robot_width_;
    double draw_isodistance_each_;
    bool global_mapping_;
    std::string frame_id_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TraversabilityNode>());
    rclcpp::shutdown();
    return 0;
}