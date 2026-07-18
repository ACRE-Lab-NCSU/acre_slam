#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/msg/odometry.hpp>

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

    this->declare_parameter("obstacle_height_threshold", 0.15);
    this->get_parameter("obstacle_height_threshold", obstacle_threshold_);

    this->declare_parameter("resolution", 0.05);
    this->get_parameter("resolution", resolution_);

    this->declare_parameter("size_x", 3.0);
    this->get_parameter("size_x", size_x_);

    this->declare_parameter("size_y", 3.0);
    this->get_parameter("size_y", size_y_);

    this->declare_parameter("sigma", 0.2);
    this->get_parameter("sigma", sigma_);

    this->declare_parameter("obstacle_inflation", 0.2);
    this->get_parameter("obstacle_inflation", obstacle_inflation_);

    if (obstacle_inflation_ < 0.0) {
      RCLCPP_ERROR(this->get_logger(), "obstacle_inflation must be >= 0, got %f.", obstacle_inflation_);
      throw std::runtime_error("Invalid obstacle_inflation parameter");
    }
    if (obstacle_inflation_ == 0.0) {
      RCLCPP_INFO(this->get_logger(), "obstacle_inflation set to 0. This disables obstacle inflation.", obstacle_inflation_);
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
  void point_cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
                            const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg)
  {
    // Get the robots position in the maps frame
    grid_map::Position robot_pos(odom_msg->pose.pose.position.x,
                                  odom_msg->pose.pose.position.y);
    map_.move(robot_pos);

    // Clear these layers. This is kind of hacky. We should write proper raytracing
    map_["elevation"].setConstant(std::numeric_limits<float>::quiet_NaN());
    map_["obstacle"].setConstant(FREE_VALUE);
    map_["observed"].setConstant(0.0f);

    // Convert PointCloud2 to ANYbiotics GridMap (PC2->PCL->GridMap)
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *pcl_cloud);

    // Add a bounding box to the map that matches the input size of the map
    // This avoids processing points outside the bounds of the local map
    grid_map::Position center = map_.getPosition();
    double half_x = map_.getLength().x() / 2.0;
    double half_y = map_.getLength().y() / 2.0;

    pcl::CropBox<pcl::PointXYZ> crop;
    crop.setInputCloud(pcl_cloud);
    crop.setMin(Eigen::Vector4f(center.x() - half_x, center.y() - half_y, -std::numeric_limits<float>::max(), 1.0f));
    crop.setMax(Eigen::Vector4f(center.x() + half_x, center.y() + half_y,  std::numeric_limits<float>::max(), 1.0f));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    crop.filter(*cropped_cloud);


    // Transform pcl_cloud into grid_map frame
    // ..

    // Update current map with new points
    for (const auto& pt : cropped_cloud->points) {
      grid_map::Position p(pt.x, pt.y);
      grid_map::Index index;
      if (!map_.getIndex(p, index)) continue;  // outside local window

      map_.at("elevation", index) = pt.z;
      map_.at("observed", index) = 1.0f;
      map_.at("obstacle", index) =
          (pt.z > obstacle_threshold_) ? OCCUPIED_VALUE : FREE_VALUE; // Mark occupied cells (does this convert to float)
    }

    // compute SDF
    const int rows = map_.getSize()(0);
    const int cols = map_.getSize()(1);
    cv::Mat not_free_u8(rows, cols, CV_8UC1, cv::Scalar(0));   // obstacle and unknown cells
    cv::Mat free_u8(rows, cols, CV_8UC1, cv::Scalar(0));       // free cells

    for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it) {
      const grid_map::Index index(*it);
      bool observed = map_.at("observed", *it) > 0.5f; // Check if the cell has been observed
      bool occupied = observed && (map_.at("obstacle", *it) > 0.5f);

      // Cells are only free if they have been observed and are not occupied
      not_free_u8.at<uint8_t>(index(0), index(1)) = (occupied || !observed) ? OCCUPIED_VALUE : FREE_VALUE;
      free_u8.at<uint8_t>(index(0), index(1))     = (observed && !occupied) ? OCCUPIED_VALUE : FREE_VALUE;
    }

    // Inflate obstacles using dilation
    int inflation_px = static_cast<int>(std::round(obstacle_inflation_ / map_.getResolution()));
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                cv::Size(2 * inflation_px + 1, 2 * inflation_px + 1));
    cv::Mat not_free_inflated;
    cv::dilate(not_free_u8, not_free_inflated, kernel);

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
    Eigen::MatrixXf sdf_eigen;
    cv::cv2eigen(sdf_smooth, sdf_eigen);
    map_.add("sdf", sdf_eigen);

    // Publish map
    auto out_msg = grid_map::GridMapRosConverter::toMessage(map_);
    grid_map_pub_->publish(std::move(out_msg));
  }

  static constexpr int OCCUPIED_VALUE = 255;
  static constexpr int FREE_VALUE = 0;

  std::string point_cloud_topic_;
  std::string odom_topic_;
  std::string config_path_;
  double obstacle_threshold_;
  double resolution_;
  size_t size_x_;
  size_t size_y_;
  double sigma_;
  double obstacle_inflation_;
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