/**
 *  @file sdf_cbf.cpp
 *  @brief Creates a local Signed Distance Field GridMap that can be used as a Control Barrier Function.
 *  @author Nicholas Sutton
 *  @date 2026-07-20
 * 
 *  Copyright 2026 Nicholas Sutton
 * 
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 * 
 *  http://www.apache.org/licenses/LICENSE-2.0
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_core/Polygon.hpp>
#include <grid_map_core/iterators/PolygonIterator.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_core/iterators/LineIterator.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

#include <opencv2/opencv.hpp>

/**
 *  @class sdfCbf
 *  @brief A Signed Distance based Control Barrier Function that publishes a GridMap.
 */
class SdfCbf : public rclcpp::Node {
public:
  SdfCbf() : Node("sdf_cbf")
  {
    setup_params();

    // Setup subscribers and publishers
    point_cloud_sub_.subscribe(this, point_cloud_topic_);
    odom_sub_.subscribe(this, odom_topic_);

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), point_cloud_sub_, odom_sub_);

    sync_->registerCallback(&SdfCbf::point_cloud_callback, this);

    grid_map_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("/acre/sdf_cbf", 10);

    // create map
    // map_.add("elevation");
    map_.add("obstacle");
    map_.add("obstacle_inflated");
    map_.add("observed", 0.0f);   // 0 = not observed, 1 = observed
    map_.add("sdf");
    map_.setGeometry(
        grid_map::Length(size_x_, size_y_),
        resolution_,
        grid_map::Position(0.0, 0.0));
    map_.setFrameId("camera_init"); // !TODO: Give the sdf its own frame
  }

private:
  /**
   * @brief Registers and Validates Node input parameters.
   * @throw std::runtime_error if a parameter cannot be validated.
   */
  void setup_params() {
    // Register topic names
    this->declare_parameter("point_cloud_topic", "/cloud_registered");
    this->get_parameter("point_cloud_topic", point_cloud_topic_);

    this->declare_parameter("odom_topic", "/Odometry");
    this->get_parameter("odom_topic", odom_topic_);

    // Register and validate obstacle bounds
    this->declare_parameter("min_obstacle_height", 0.1);
    this->get_parameter("min_obstacle_height", min_obstacle_height_);
    if (min_obstacle_height_< 0.0) {
      RCLCPP_ERROR(this->get_logger(), "min_obstacle_height must be >= 0, got %f.", min_obstacle_height_);
      throw std::runtime_error("Invalid min_obstacle_height parameter");
    }
    this->declare_parameter("max_obstacle_height", 0.80);
    this->get_parameter("max_obstacle_height", max_obstacle_height_);
    if (max_obstacle_height_< 0.0) {
      RCLCPP_ERROR(this->get_logger(), "max_obstacle_height must be >= 0, got %f.", max_obstacle_height_);
      throw std::runtime_error("Invalid max_obstacle_height parameter");
    }
    if (max_obstacle_height_ <= min_obstacle_height_) {
      RCLCPP_ERROR(this->get_logger(),
          "max_obstacle_height (%f) must be greater than min_obstacle_height (%f).",
          max_obstacle_height_, min_obstacle_height_);
      throw std::runtime_error("Invalid obstacle height range");
    }

    // Register and validate map params
    this->declare_parameter("resolution", 0.05);
    this->get_parameter("resolution", resolution_);
    if (resolution_ <= 0.0) {
      RCLCPP_ERROR(this->get_logger(), "resolution must be > 0, got %f.", resolution_);
      throw std::runtime_error("Invalid resolution parameter");
    }
    this->declare_parameter("size_x", 3.0);
    this->get_parameter("size_x", size_x_);
    if (size_x_ <= 0.0) {
      RCLCPP_ERROR(this->get_logger(), "size_x must be > 0, got %f.", size_x_);
      throw std::runtime_error("Invalid size_x parameter");
    }
    this->declare_parameter("size_y", 3.0);
    this->get_parameter("size_y", size_y_);
    if (size_y_ <= 0.0) {
      RCLCPP_ERROR(this->get_logger(), "size_y must be > 0, got %f.", size_y_);
      throw std::runtime_error("Invalid size_y parameter");
    }

    // Register and Validate sdf/cbf params
    this->declare_parameter("sigma", 0.05);
    this->get_parameter("sigma", sigma_);
    if (sigma_ < 0.0) {
      RCLCPP_ERROR(this->get_logger(), "sigma must be > 0.");
      throw std::runtime_error("Invalid sigma parameter");
    }

    this->declare_parameter("obstacle_inflation", 0.05);
    this->get_parameter("obstacle_inflation", obstacle_inflation_);
    if (obstacle_inflation_ < 0.0) {
      RCLCPP_ERROR(this->get_logger(), "obstacle_inflation must be >= 0, got %f.", obstacle_inflation_);
      throw std::runtime_error("Invalid obstacle_inflation parameter");
    }
    if (obstacle_inflation_ == 0.0) {
      RCLCPP_INFO(this->get_logger(), "obstacle_inflation set to 0. This disables obstacle inflation.");
    }

    // Register and validate robot params
    this->declare_parameter("robot_length", 0.7);
    this->get_parameter("robot_length", robot_length_);
    if (robot_length_ <= 0.0) {
      RCLCPP_ERROR(this->get_logger(), "robot_length must be > 0, got %f.", robot_length_);
      throw std::runtime_error("Invalid robot_length parameter");
    }

    this->declare_parameter("robot_width", 0.35);
    this->get_parameter("robot_width", robot_width_);
    if (robot_width_ <= 0.0) {
      RCLCPP_ERROR(this->get_logger(), "robot_width must be > 0, got %f.", robot_width_);
      throw std::runtime_error("Invalid rrobot_width parameter");
    }
  }

  /**
   * @brief Clear grid_map cells that intersect with the robots body.
   * @param robot_pos the robots current position.
   * @param orientation the robots current orientation.
   */
  void clear_robot_footprint(const grid_map::Position& robot_pos,
                            const geometry_msgs::msg::Quaternion& orientation)
  {
    double yaw = tf2::getYaw(orientation);
    double cos_yaw = std::cos(yaw);
    double sin_yaw = std::sin(yaw);

    double half_length = robot_length_ / 2.0;
    double half_width = robot_width_ / 2.0;

    std::vector<Eigen::Vector2d> local_corners = {
      {half_length, half_width}, {half_length, -half_width},
      {-half_length, -half_width}, {-half_length, half_width}
    };

    grid_map::Polygon footprint;
    footprint.setFrameId(map_.getFrameId());
    for (const auto& c : local_corners) {
      double wx = robot_pos.x() + c.x() * cos_yaw - c.y() * sin_yaw;
      double wy = robot_pos.y() + c.x() * sin_yaw + c.y() * cos_yaw;
      footprint.addVertex(grid_map::Position(wx, wy));
    }

    for (grid_map::PolygonIterator it(map_, footprint); !it.isPastEnd(); ++it) {
      map_.at("observed", *it) = OBSERVED;
      map_.at("obstacle", *it) = FREE_VALUE;
    }
  }

  /**
   * @brief Traces a path from the robots current position to each point in the point cloud and marks visited cells as free.
   * @param robot_pos the robots current position.
   * @param pc Point Cloud to raytrace.
   */
  void raytrace_points(grid_map::Position& robot_pos, const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc) {
    grid_map::Index robot_index;
    bool have_robot_index = map_.getIndex(robot_pos, robot_index);

    // classify every point
    std::vector<Endpoint> endpoints;
    endpoints.reserve(pc->points.size());

    for (const auto& pt : pc->points) {
      grid_map::Index end_index;
      if (!map_.getIndex(grid_map::Position(pt.x, pt.y), end_index)) continue;

      double height_above_ground = pt.z - GROUND_PLANE_;
      bool occupied = (height_above_ground > min_obstacle_height_ &&
                        height_above_ground < max_obstacle_height_);
      endpoints.push_back({end_index, static_cast<float>(height_above_ground), occupied});
    }

    // Raytrace from robots position to every point in the cropped map. We mark every cell along the path as free
    if (have_robot_index) {
      for (const auto& ep : endpoints) {
        for (grid_map::LineIterator line(map_, robot_index, ep.index); !line.isPastEnd(); ++line) {
          grid_map::Index idx(*line);
          map_.at("observed", idx) = OBSERVED;
          map_.at("obstacle", idx) = FREE_VALUE;
        }
      }
    }

    // Mark cells that contain points from the PC as occupied
    for (const auto& ep : endpoints) {
      // map_.at("elevation", ep.index) = ep.elevation;
      map_.at("observed", ep.index) = OBSERVED; // mark the cell as observed
      map_.at("obstacle", ep.index) = ep.occupied ? OCCUPIED_VALUE : FREE_VALUE;
    }
  }

  /**
   * @brief Infaltes obstacles in the GridMap using dialation.
   * @param not_free_u8 occupied and unknown space Matrix.
   * @param buffer_start Map start index.
   * @param buffer_size Map size.
   * @param rows Number of rows in the map.
   * @param cols Number of cols in the map.
   * @return A matrix of infalted obstacles
   */
  cv::Mat inflate_obstacles(const cv::Mat& not_free_u8, const grid_map::Index buffer_start, 
                          const grid_map::Size buffer_size, const int rows, const int cols)
  {
    int inflation_px = static_cast<int>(std::round(obstacle_inflation_ / map_.getResolution()));
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                cv::Size(2 * inflation_px + 1, 2 * inflation_px + 1));
    cv::Mat not_free_inflated;
    cv::dilate(not_free_u8, not_free_inflated, kernel);

    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
        grid_map::Index unwrapped_index(i, j);
        grid_map::Index buffer_index =
            grid_map::getBufferIndexFromIndex(unwrapped_index, buffer_size, buffer_start);
        map_.at("obstacle_inflated", buffer_index) =
            static_cast<float>(not_free_inflated.at<uint8_t>(i, j));
      }
    }

    return not_free_inflated;
  }

  /**
   * @brief Applies Guassian Blur to a discrete SDF.
   * @param dist_to_obstacle Matrix of distances to obstacles in free space.
   * @param dist_to_free Matrix of distances to free space from inside obstacle regions.
   * @return A smooth sdf matrix
   */
  cv::Mat apply_guassian_blur(const cv::Mat& dist_to_obstacle, const cv::Mat& dist_to_free)
  {
    cv::Mat sdf_cells = dist_to_obstacle - dist_to_free;
    cv::Mat sdf_meters = sdf_cells * map_.getResolution();

    // Use Gaussian blur to create a smooth sdf from a grid
    double sigma_px = sigma_ / map_.getResolution();
    int ksize = 2 * static_cast<int>(std::ceil(3.0 * sigma_px)) + 1; // kernal size needs to be odd
    cv::Mat sdf_smooth;
    cv::GaussianBlur(sdf_meters, sdf_smooth, cv::Size(ksize, ksize), sigma_px);

    return sdf_smooth;
  }

  /**
   * @brief Publishes SDF GridMap by computing a smooth SDF from a point cloud
   * @param cloud_msg The point cloud message
   * @param odom_msg The robots odometry.
   */
  void point_cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
                            const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg)
  {
    // TODO: We might need to do a coordiante frame transform here
    grid_map::Position robot_pos(odom_msg->pose.pose.position.x,
                                  odom_msg->pose.pose.position.y);
    map_.move(robot_pos);

    // convert and crop point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *pcl_cloud);

    grid_map::Position center = map_.getPosition();
    double half_x = map_.getLength().x() / 2.0;
    double half_y = map_.getLength().y() / 2.0;

    pcl::CropBox<pcl::PointXYZ> crop;
    crop.setInputCloud(pcl_cloud);
    crop.setMin(Eigen::Vector4f(center.x() - half_x, center.y() - half_y, -std::numeric_limits<float>::max(), 1.0f));
    crop.setMax(Eigen::Vector4f(center.x() + half_x, center.y() + half_y,  std::numeric_limits<float>::max(), 1.0f));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    crop.filter(*cropped_cloud);

    // Raytrace each PC point
    raytrace_points(robot_pos, cropped_cloud);

    // Clear the area directly around the robot as free
    clear_robot_footprint(robot_pos, odom_msg->pose.pose.orientation);

    // compute SDF
    const int rows = map_.getSize()(0);
    const int cols = map_.getSize()(1);
    const grid_map::Size buffer_size = map_.getSize();
    const grid_map::Index buffer_start = map_.getStartIndex();

    // we need to store the cell info in u8 CV matrices
    cv::Mat not_free_u8(rows, cols, CV_8UC1, cv::Scalar(0));
    cv::Mat free_u8(rows, cols, CV_8UC1, cv::Scalar(0));

    for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it) {
      const grid_map::Index buffer_index(*it);
      const grid_map::Index unwrapped_index =
          grid_map::getIndexFromBufferIndex(buffer_index, buffer_size, buffer_start);

      bool observed = map_.at("observed", buffer_index) > 0.5f;
      bool occupied = observed && (map_.at("obstacle", buffer_index) > 0.5f);

      not_free_u8.at<uint8_t>(unwrapped_index(0), unwrapped_index(1)) =
          (occupied || !observed) ? OCCUPIED_VALUE : FREE_VALUE;
      free_u8.at<uint8_t>(unwrapped_index(0), unwrapped_index(1)) =
          (observed && !occupied) ? OCCUPIED_VALUE : FREE_VALUE;
    }

    // Inflate obstacles using dilation
    cv::Mat not_free_inflated = inflate_obstacles(not_free_u8, buffer_start, buffer_size, rows, cols);

    // Computer the distance transform for free and occupied space (this is a discrete sdf)
    cv::Mat dist_to_obstacle, dist_to_free;
    cv::distanceTransform(OCCUPIED_VALUE - not_free_inflated, dist_to_obstacle, cv::DIST_L2, cv::DIST_MASK_PRECISE); // zero at not_free
    cv::distanceTransform(OCCUPIED_VALUE - free_u8,           dist_to_free,     cv::DIST_L2, cv::DIST_MASK_PRECISE); // zero at free

    // Use Gaussian blur to create a smooth sdf from a grid
    cv::Mat sdf_smooth = apply_guassian_blur(dist_to_obstacle, dist_to_free);

    // Add sdf layer into the gridmap
    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
        grid_map::Index unwrapped_index(i, j);
        grid_map::Index buffer_index =
            grid_map::getBufferIndexFromIndex(unwrapped_index, buffer_size, buffer_start);
        map_.at("sdf", buffer_index) = sdf_smooth.at<float>(i, j);
      }
    }

    // Publish map
    auto out_msg = grid_map::GridMapRosConverter::toMessage(map_);
    grid_map_pub_->publish(std::move(out_msg));
  }

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
          sensor_msgs::msg::PointCloud2, 
          nav_msgs::msg::Odometry
  >;

  struct Endpoint {
    grid_map::Index index;
    float elevation;
    bool occupied;
  };

  static constexpr int OCCUPIED_VALUE = 255;
  static constexpr int FREE_VALUE = 0;
  static constexpr int OBSERVED = 1;
  static constexpr double GROUND_PLANE_ = -0.37;

  std::string point_cloud_topic_;
  std::string odom_topic_;
  double min_obstacle_height_;
  double max_obstacle_height_;
  double resolution_;
  double size_x_;
  double size_y_;
  double sigma_;
  double obstacle_inflation_;
  double robot_length_;
  double robot_width_;
  grid_map::GridMap map_;

  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_pub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> point_cloud_sub_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_; 
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SdfCbf>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}