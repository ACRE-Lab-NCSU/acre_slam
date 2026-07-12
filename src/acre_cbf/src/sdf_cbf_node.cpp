#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <grid_map_pcl/GridMapPclLoader.hpp>
#include <grid_map_sdf/SignedDistanceField.hpp>

using std::placeholders::_1;

class SdfCbfNode : public rclcpp::Node {
public:
  SdfCbfNode() : Node("sdf_cbf_node")
  {
    this->declare_parameter("point_cloud_topic", "/unitree/slam_lidar/points");
    this->get_parameter("point_cloud_topic", point_cloud_topic_);

    this->declare_parameter("pcl_config_path", "/workspace/src/acre_cbf/config/pcl.yaml");
    this->get_parameter("pcl_config_path", config_path_);

    this->declare_parameter("obstacle_height_threshold", 0.15);
    this->get_parameter("obstacle_height_threshold", obstacle_threshold_);

    this->declare_parameter("sdf_height_clearance", 0.1);
    this->get_parameter("sdf_height_clearance", sdf_height_clearance_);

    point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        point_cloud_topic_, 10,
        std::bind(&SdfCbfNode::point_cloud_callback, this, _1)
    );

    grid_map_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("/acre_gridmap", 10);
  }

private:
  void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Convert PointCloud2 to ANYbiotics GridMap (PC2->PCL->GridMap)
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pcl_cloud);

    grid_map::GridMapPclLoader loader(this->get_logger());
    loader.setInputCloud(pcl_cloud);
    loader.loadParameters(config_path_);
    loader.preProcessInputCloud();
    loader.initializeGridMapGeometryFromInputCloud();
    loader.addLayerFromInputCloud("elevation");

    grid_map::GridMap grid_map = loader.getGridMap();

    // Derive a binary obstacle layer from elevation.
    // Anything above the threshold height is treated as an obstacle.
    grid_map.add("obstacle", 0.0); // The z-height of these layers are set to "0.0"
    grid_map.add("flat_elevation", 0.0);
    for (grid_map::GridMapIterator it(grid_map); !it.isPastEnd(); ++it) {
      const float elevation = grid_map.at("elevation", *it);
      bool occupied = std::isfinite(elevation) && (elevation > obstacle_threshold_);
      grid_map.at("obstacle", *it) = occupied ? 1.0 : 0.0;
      grid_map.at("flat_elevation", *it) = occupied ? 1.0 : 0.0;
    }

    // Compute the SDF
    grid_map::SignedDistanceField sdf;
    sdf.calculateSignedDistanceField(grid_map, "flat_elevation", sdf_height_clearance_);

    // Populate sdf and gradient layer
    grid_map.add("sdf");
    grid_map.add("sdf_grad_x");
    grid_map.add("sdf_grad_y");
    for (grid_map::GridMapIterator it(grid_map); !it.isPastEnd(); ++it) {
      grid_map::Position pos;
      grid_map.getPosition(*it, pos);
      grid_map::Position3 pos3(pos.x(), pos.y(), 0.0);

      grid_map.at("sdf", *it) = sdf.getDistanceAt(pos3);
      grid_map::Vector3 grad = sdf.getDistanceGradientAt(pos3);
      grid_map.at("sdf_grad_x", *it) = grad.x();
      grid_map.at("sdf_grad_y", *it) = grad.y();
    }

    // Publish map
    grid_map.setFrameId(msg->header.frame_id);
    auto out_msg = grid_map::GridMapRosConverter::toMessage(grid_map);
    grid_map_pub_->publish(std::move(out_msg));
  }

  std::string point_cloud_topic_;
  std::string config_path_;
  double obstacle_threshold_;
  double sdf_height_clearance_;

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