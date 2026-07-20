#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
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
#include <opencv2/core/eigen.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>;

class SdfCbfNode : public rclcpp::Node {
public:
  SdfCbfNode() : Node("sdf_cbf_node")
  {
    this->declare_parameter("point_cloud_topic", "/cloud_registered");
    this->get_parameter("point_cloud_topic", point_cloud_topic_);

    this->declare_parameter("odom_topic", "/Odometry");
    this->get_parameter("odom_topic", odom_topic_);

    this->declare_parameter("pcl_config_path", "/workspace/src/acre_cbf/config/pcl.yaml");
    this->get_parameter("pcl_config_path", config_path_);

    this->declare_parameter("min_obstacle_height", 0.1);
    this->get_parameter("min_obstacle_height", min_obstacle_height_);

    this->declare_parameter("max_obstacle_height", 0.80);
    this->get_parameter("max_obstacle_height", max_obstacle_height_);

    if (max_obstacle_height_ <= min_obstacle_height_) {
      RCLCPP_ERROR(this->get_logger(),
          "max_obstacle_height (%f) must be greater than min_obstacle_height (%f).",
          max_obstacle_height_, min_obstacle_height_);
      throw std::runtime_error("Invalid obstacle height range");
    }

    this->declare_parameter("resolution", 0.05);
    this->get_parameter("resolution", resolution_);

    this->declare_parameter("size_x", 3.0);
    this->get_parameter("size_x", size_x_);

    this->declare_parameter("size_y", 3.0);
    this->get_parameter("size_y", size_y_);

    this->declare_parameter("sigma", 0.05);
    this->get_parameter("sigma", sigma_);

    this->declare_parameter("obstacle_inflation", 0.05);
    this->get_parameter("obstacle_inflation", obstacle_inflation_);

    this->declare_parameter("robot_length", 0.7);
    this->get_parameter("robot_length", robot_length_);

    this->declare_parameter("robot_width", 0.35);
    this->get_parameter("robot_width", robot_width_);

    if (obstacle_inflation_ < 0.0) {
      RCLCPP_ERROR(this->get_logger(), "obstacle_inflation must be >= 0, got %f.", obstacle_inflation_);
      throw std::runtime_error("Invalid obstacle_inflation parameter");
    }
    if (obstacle_inflation_ == 0.0) {
      RCLCPP_INFO(this->get_logger(), "obstacle_inflation set to 0. This disables obstacle inflation.");
    }
    if (sigma_ < 0.0) {
      RCLCPP_ERROR(this->get_logger(), "sigma must be > 0.");
      throw std::runtime_error("Invalid sigma parameter");
    }
    if (resolution_ <= 0.0) {
      RCLCPP_ERROR(this->get_logger(), "resolution must be > 0, got %f.", resolution_);
      throw std::runtime_error("Invalid resolution parameter");
    }

    point_cloud_sub_.subscribe(this, point_cloud_topic_);
    odom_sub_.subscribe(this, odom_topic_);

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), point_cloud_sub_, odom_sub_);
    sync_->registerCallback(&SdfCbfNode::point_cloud_callback, this);

    grid_map_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("/acre_gridmap", 10);

    map_.add("elevation");
    map_.add("obstacle");
    map_.add("observed", 0.0f);   // 0 = not observed, 1 = observed
    map_.setGeometry(
        grid_map::Length(size_x_, size_y_),
        resolution_,
        grid_map::Position(0.0, 0.0));
    map_.setFrameId("camera_init");
  }

private:
  void clearRobotFootprint(const grid_map::Position& robot_pos,
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
      map_.at("observed", *it) = 1.0f;
      map_.at("obstacle", *it) = FREE_VALUE;
    }
  }

  void point_cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
                            const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg)
  {
    grid_map::Position robot_pos(odom_msg->pose.pose.position.x,
                                  odom_msg->pose.pose.position.y);
    map_.move(robot_pos);

    // convert + crop
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

    grid_map::Index robot_index;
    bool have_robot_index = map_.getIndex(robot_pos, robot_index);

    // --- Pass 1: classify every point, without writing yet ---
    struct EndpointResult {
      grid_map::Index index;
      float elevation;
      bool occupied;
    };
    std::vector<EndpointResult> endpoint_results;
    endpoint_results.reserve(cropped_cloud->points.size());

    for (const auto& pt : cropped_cloud->points) {
      grid_map::Index end_index;
      if (!map_.getIndex(grid_map::Position(pt.x, pt.y), end_index)) continue;

      double height_above_ground = pt.z - GROUND_PLANE_;
      bool occupied = (height_above_ground > min_obstacle_height_ &&
                        height_above_ground < max_obstacle_height_);
      endpoint_results.push_back({end_index, static_cast<float>(height_above_ground), occupied});
    }

    // --- Pass 2: raytrace-clear every ray (nav2's raytraceFreespace equivalent) ---
    if (have_robot_index) {
      for (const auto& pt : cropped_cloud->points) {
        grid_map::Index end_index;
        if (!map_.getIndex(grid_map::Position(pt.x, pt.y), end_index)) continue;

        for (grid_map::LineIterator line(map_, robot_index, end_index); !line.isPastEnd(); ++line) {
          grid_map::Index idx(*line);
          map_.at("observed", idx) = 1.0f;
          map_.at("obstacle", idx) = FREE_VALUE;
        }
      }
    }

    // --- Pass 3: apply endpoint marks last (nav2's marking loop equivalent) ---
    for (const auto& res : endpoint_results) {
      map_.at("elevation", res.index) = res.elevation;
      map_.at("observed", res.index) = 1.0f;
      map_.at("obstacle", res.index) = res.occupied ? OCCUPIED_VALUE : FREE_VALUE;
    }

    // Footprint clearing last, same as nav2's updateFootprint + setConvexPolygonCost
    clearRobotFootprint(robot_pos, odom_msg->pose.pose.orientation);

    // compute SDF
    const int rows = map_.getSize()(0);
    const int cols = map_.getSize()(1);
    const grid_map::Size buffer_size = map_.getSize();
    const grid_map::Index buffer_start = map_.getStartIndex();
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
    int inflation_px = static_cast<int>(std::round(obstacle_inflation_ / map_.getResolution()));
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                cv::Size(2 * inflation_px + 1, 2 * inflation_px + 1));
    cv::Mat not_free_inflated;
    cv::dilate(not_free_u8, not_free_inflated, kernel);

    if (!map_.exists("obstacle_inflated")) {
      map_.add("obstacle_inflated");
    }
    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
        grid_map::Index unwrapped_index(i, j);
        grid_map::Index buffer_index =
            grid_map::getBufferIndexFromIndex(unwrapped_index, buffer_size, buffer_start);
        map_.at("obstacle_inflated", buffer_index) =
            static_cast<float>(not_free_inflated.at<uint8_t>(i, j));
      }
    }

    // Do a double pass of the free and obstacle cells
    cv::Mat dist_to_obstacle, dist_to_free;
    cv::distanceTransform(OCCUPIED_VALUE - not_free_inflated, dist_to_obstacle, cv::DIST_L2, cv::DIST_MASK_PRECISE); // zero at not_free
    cv::distanceTransform(OCCUPIED_VALUE - free_u8,           dist_to_free,     cv::DIST_L2, cv::DIST_MASK_PRECISE); // zero at free

    // Convert sdf in cells to meters
    cv::Mat sdf_cells = dist_to_obstacle - dist_to_free;
    cv::Mat sdf_meters = sdf_cells * map_.getResolution();

    // Use Gaussian blur to create a smooth sdf from a grid
    double sigma_px = sigma_ / map_.getResolution();
    int ksize = 2 * static_cast<int>(std::ceil(3.0 * sigma_px)) + 1; // kernal size needs to be odd
    cv::Mat sdf_smooth;
    cv::GaussianBlur(sdf_meters, sdf_smooth, cv::Size(ksize, ksize), sigma_px);

    // Add sdf layer into the gridmap
    if (!map_.exists("sdf")) {
      map_.add("sdf");
    }
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

  static constexpr int OCCUPIED_VALUE = 255;
  static constexpr int FREE_VALUE = 0;
  static constexpr double GROUND_PLANE_ = -0.37;// 0.360;

  std::string point_cloud_topic_;
  std::string odom_topic_;
  std::string config_path_;
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
  auto node = std::make_shared<SdfCbfNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}