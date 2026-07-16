#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <grid_map_cv/grid_map_cv.hpp>

using std::placeholders::_1;

class SdfCbfNode : public rclcpp::Node {
public:
  SdfCbfNode() : Node("sdf_cbf_node")
  {
    this->declare_parameter("point_cloud_topic", "/cloud_registered");
    this->get_parameter("point_cloud_topic", point_cloud_topic_);

    this->declare_parameter("pcl_config_path", "/workspace/src/acre_cbf/config/pcl.yaml");
    this->get_parameter("pcl_config_path", config_path_);

    this->declare_parameter("obstacle_height_threshold", 0.15);
    this->get_parameter("obstacle_height_threshold", obstacle_threshold_);

    this->declare_parameter("resolution", 0.05);
    this->get_parameter("resolution", resolution_);

    this->declare_parameter("size_x", 10);
    this->get_parameter("size_x", size_x_);

    this->declare_parameter("size_y", 10);
    this->get_parameter("size_y", size_y_);

    point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        point_cloud_topic_, 10,
        std::bind(&SdfCbfNode::point_cloud_callback, this, _1)
    );

    grid_map_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("/acre_gridmap", 10);

    map_.add("elevation");
    map_.add("obstacle");
    map_.add("observed", 0.0f);   // 0 = never observed, 1 = observed
    map_.setGeometry(
        grid_map::Length(size_x_, size_y_),
        resolution_,
        grid_map::Position(0.0, 0.0));
    map_.setFrameId("odom");
  }

private:
  void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Get the robots position in the maps frame
    grid_map::Position robot_pos = // ,,,
    map_.move(robot_pos);

    // Mark new NaN cells with the correct value in the observed layer
    for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it) {
      if (std::isnan(map_.at("observed", *it))) {
        map_.at("observed", *it) = 0.0f;
      }
    }

    // Convert PointCloud2 to ANYbiotics GridMap (PC2->PCL->GridMap)
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pcl_cloud);

    // Transform pcl_cloud into grid_map frame
    // ..

    // Update current map with new points
    for (const auto& pt : pcl_cloud->points) {
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

    // Do a double pass of the free and obstacle cells
    cv::Mat dist_to_obstacle, dist_to_free;
    cv::distanceTransform(OCCUPIED_VALUE - not_free_u8, dist_to_obstacle, cv::DIST_L2, cv::DIST_MASK_PRECISE); // zero at not_free
    cv::distanceTransform(OCCUPIED_VALUE - free_u8,     dist_to_free,     cv::DIST_L2, cv::DIST_MASK_PRECISE); // zero at free

    // Convert sdf in cells to meters
    cv::Mat sdf_cells = dist_to_obstacle - dist_to_free;
    cv::Mat sdf_meters = sdf_cells * map_.getResolution();

    // Use Gaussian Smoothing to create a smooth sdf from a grid
    double sigma_meters = 0.2;
    double sigma_px = sigma_meters / map_.getResolution();
    int ksize = 2 * static_cast<int>(std::ceil(3.0 * sigma_px)) + 1; // odd, ~3 sigma
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
  std::string config_path_;
  double obstacle_threshold_;
  double resolution_;
  double length_x_;
  double length_y_;
  grid_map::GridMap map_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SdfCbfNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}