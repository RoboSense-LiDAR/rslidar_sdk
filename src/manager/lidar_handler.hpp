#pragma once

#include <rclcpp/rclcpp.hpp> // For logging
#include <rs_driver/api/lidar_driver.hpp>
#include <rs_driver/driver/driver_param.hpp>
#include <rs_driver/msg/point_cloud_msg.hpp>
#include <Eigen/Dense>
#include <mutex>

using namespace robosense::lidar;

using PointCloudMsg = PointCloudT<PointXYZI>;
using RSDriver = LidarDriver<PointCloudMsg>;

class LidarHandler
{
public:
  LidarHandler(const RSDriverParam& driver_param, const Eigen::Matrix4f& transform)
    : transform_(transform)
  {
    driver_.regPointCloudCallback(
        std::bind(&LidarHandler::getPointCloudForDriver, this),
        std::bind(&LidarHandler::pointCloudCallback, this, std::placeholders::_1));
    driver_.init(driver_param);
    driver_.start();
  }

  virtual ~LidarHandler() = default;

  std::shared_ptr<PointCloudMsg> getPointCloudForDriver()
  {
      return std::make_shared<PointCloudMsg>();
  }

  std::shared_ptr<const PointCloudMsg> getPointCloud()
  {
    std::lock_guard<std::mutex> lock(pointcloud_mutex_);
    return pointcloud_;
  }

  const Eigen::Matrix4f& getTransform() const
  {
    std::lock_guard<std::mutex> lock(transform_mutex_);
    return transform_;
  }

  void setTransform(const Eigen::Matrix4f& transform)
  {
    std::lock_guard<std::mutex> lock(transform_mutex_);
    transform_ = transform;
  }

private:
  void pointCloudCallback(std::shared_ptr<PointCloudMsg> pointcloud_msg)
  {
    // Defense code: Check if the point cloud is valid before storing it.
    // The driver may send point clouds full of zeros during initialization.
    if (pointcloud_msg && !pointcloud_msg->points.empty())
    {
      const auto& first_point = pointcloud_msg->points[0];
      if (first_point.x == 0.0f && first_point.y == 0.0f && first_point.z == 0.0f)
      {
        RCLCPP_WARN(rclcpp::get_logger("LidarHandler"), 
                    "Discarding an invalid (all zeros) point cloud frame. This is normal during driver startup.");
        return;
      }
    }

    std::lock_guard<std::mutex> lock(pointcloud_mutex_);
    pointcloud_ = pointcloud_msg;
  }

  RSDriver driver_;
  std::shared_ptr<PointCloudMsg> pointcloud_;
  Eigen::Matrix4f transform_;
  mutable std::mutex pointcloud_mutex_;
  mutable std::mutex transform_mutex_;
};