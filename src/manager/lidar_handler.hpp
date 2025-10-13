#pragma once

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
    driver_.regPointCloudCallback(std::bind(&LidarHandler::pointCloudCallback, this, std::placeholders::_1));
    driver_.init(driver_param);
    driver_.start();
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
  void pointCloudCallback(const std::shared_ptr<const PointCloudMsg>& pointcloud_msg)
  {
    std::lock_guard<std::mutex> lock(pointcloud_mutex_);
    pointcloud_ = pointcloud_msg;
  }

  RSDriver driver_;
  std::shared_ptr<const PointCloudMsg> pointcloud_;
  Eigen::Matrix4f transform_;
  mutable std::mutex pointcloud_mutex_;
  mutable std::mutex transform_mutex_;
};