#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp> // New include for LaserScan
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include "gpu_lidar_handler.hpp"
#ifdef WITH_NITROS
#include <isaac_ros_nitros/managed_nitros_publisher.hpp>
#include <isaac_ros_nitros_point_cloud_type/nitros_point_cloud2.hpp>
#endif
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Struct to hold a single ROI filter configuration
struct RoiFilterConfig
{
  std::string name;
  std::string type; // "positive" or "negative"
  float min_x, max_x, min_y, max_y, min_z, max_z;
};

class MultiLidarNode : public rclcpp::Node
{
public:
  MultiLidarNode(const rclcpp::NodeOptions& options);

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr statistics_timer_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr tf_check_timer_;

  void loadParameters();
  void loadFilterParameters();
  void loadFlatScanParameters();
  void runInitialCalibration();
  void mergeAndPublish();
  void printCurrentParameters();
  void checkTfUpdates();
  void logStatistics();
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

  std::vector<std::shared_ptr<GPULidarHandler>> lidar_handlers_;
  std::vector<std::string> lidar_frame_ids_;
  std::vector<size_t> last_tf_hashes_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_pub_;
  bool publish_3d_pcd_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr flatscan_pub_;
#ifdef WITH_NITROS
  std::shared_ptr<nvidia::isaac_ros::nitros::ManagedNitrosPublisher<nvidia::isaac_ros::nitros::NitrosPointCloud2>> nitros_pub_;
#endif
  std::string base_frame_id_;

  // Filter parameters
  bool enable_roi_filter_ = false;
  std::vector<RoiFilterConfig> roi_filters_;
  bool enable_voxel_filter_ = false;
  float voxel_leaf_size_ = 0.1f;

  // FlatScan parameters
  bool publish_flatscan_ = false;
  std::string flatscan_topic_name_;
  float flatscan_min_height_, flatscan_max_height_;
  float flatscan_angle_min_, flatscan_angle_max_, flatscan_angle_increment_;
  float flatscan_range_min_, flatscan_range_max_;

  // Statistics
  std::atomic<size_t> last_total_points_merged_{0};
  std::atomic<size_t> last_points_after_roi_{0};
  std::atomic<size_t> last_points_after_voxel_{0};
  std::atomic<uint64_t> merge_publish_count_{0};
  std::atomic<double> last_processing_latency_{0.0};
  rclcpp::Time last_statistics_log_time_;
};