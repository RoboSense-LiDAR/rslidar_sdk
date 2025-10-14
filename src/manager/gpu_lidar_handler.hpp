#pragma once

#include <rs_driver/api/lidar_driver.hpp>
#include <rs_driver/driver/driver_param.hpp>
#include <rs_driver/msg/point_cloud_msg.hpp>
#include <Eigen/Dense>
#include <mutex>
#include <memory>
#include <core/cuda_point_types.cuh>

// CUDA runtime API
#include <cuda_runtime.h>

using namespace robosense::lidar;

using PointCloudMsg = PointCloudT<PointXYZI>;
using RSDriver = LidarDriver<PointCloudMsg>;

// Custom deleter for unique_ptr to manage CUDA device memory
struct CudaFreeDeleter
{
    void operator()(CudaPointXYZI* ptr)
    {
        if (ptr)
        {
            cudaFree(ptr);
        }
    }
};

class GPULidarHandler
{
public:
  GPULidarHandler(const RSDriverParam& driver_param, const Eigen::Matrix4f& transform)
    : transform_(transform)
  {
    driver_.regPointCloudCallback(
        std::bind(&GPULidarHandler::getPointCloud, this),
        std::bind(&GPULidarHandler::pointCloudCallback, this, std::placeholders::_1));
    driver_.init(driver_param);
    driver_.start();
    initGPU();
  }

  ~GPULidarHandler()
  {
    // cudaFree is handled by CudaFreeDeleter in unique_ptr
  }

  std::shared_ptr<PointCloudMsg> getPointCloud()
  {
      return std::make_shared<PointCloudMsg>();
  }

  // Returns a shared_ptr to a struct containing device pointer and count
  struct GPUPointCloudData
  {
      std::unique_ptr<CudaPointXYZI, CudaFreeDeleter> d_points_ptr;
      size_t num_points;
  };

  std::shared_ptr<GPUPointCloudData> getGPUPointCloud()
  {
    std::lock_guard<std::mutex> lock(gpu_cloud_mutex_);
    return gpu_pointcloud_data_;
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

  void initGPU()
  {
    if (initGPU_())
    {
      is_gpu_ready_ = true;
      RCLCPP_INFO(rclcpp::get_logger("GPULidarHandler"), "GPU resources initialized successfully.");
    }
    else
    {
      is_gpu_ready_ = false;
      RCLCPP_ERROR(rclcpp::get_logger("GPULidarHandler"), "Failed to initialize GPU resources.");
    }
  }

  bool isGPUReady() const
  {
    return is_gpu_ready_;
  }

private:
  bool is_gpu_ready_ = false;
  void pointCloudCallback(std::shared_ptr<PointCloudMsg> pointcloud_msg)
  {
    if (!pointcloud_msg || pointcloud_msg->points.empty())
    {
      return;
    }

    size_t num_points = pointcloud_msg->points.size();
    CudaPointXYZI* d_points = nullptr;

    // Allocate GPU memory
    cudaError_t err = cudaMalloc((void**)&d_points, num_points * sizeof(CudaPointXYZI));
    if (err != cudaSuccess)
    {
        // Handle error, e.g., log it
        fprintf(stderr, "cudaMalloc failed: %s\n", cudaGetErrorString(err));
        return;
    }

    // Create a temporary CPU buffer for copying
    std::vector<CudaPointXYZI> h_points(num_points);
    for (size_t i = 0; i < num_points; ++i)
    {
        h_points[i].x = pointcloud_msg->points[i].x;
        h_points[i].y = pointcloud_msg->points[i].y;
        h_points[i].z = pointcloud_msg->points[i].z;
        h_points[i].intensity = pointcloud_msg->points[i].intensity;
    }

    // Copy data from CPU to GPU
    err = cudaMemcpy(d_points, h_points.data(), num_points * sizeof(CudaPointXYZI), cudaMemcpyHostToDevice);
    if (err != cudaSuccess)
    {
        fprintf(stderr, "cudaMemcpy failed: %s\n", cudaGetErrorString(err));
        cudaFree(d_points); // Free allocated GPU memory on failure
        return;
    }

    std::lock_guard<std::mutex> lock(gpu_cloud_mutex_);
    gpu_pointcloud_data_ = std::make_shared<GPUPointCloudData>();
    gpu_pointcloud_data_->d_points_ptr.reset(d_points);
    gpu_pointcloud_data_->num_points = num_points;
  }

  RSDriver driver_;
  std::shared_ptr<GPUPointCloudData> gpu_pointcloud_data_;
  Eigen::Matrix4f transform_;
  mutable std::mutex gpu_cloud_mutex_;
  mutable std::mutex transform_mutex_;
};
