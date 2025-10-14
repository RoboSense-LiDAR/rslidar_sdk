#pragma once

#include "lidar_handler.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cuda_runtime.h>
#include "../core/cuda_point_types.cuh"

// A struct to hold GPU point cloud data
struct GPUPointCloudData
{
  std::shared_ptr<CudaPointXYZI> d_points_ptr;
  size_t num_points;
};

class GPULidarHandler : public LidarHandler
{
public:
  GPULidarHandler(const RSDriverParam& param, const Eigen::Matrix4f& transform)
    : LidarHandler(param, transform)
  {
    initGPUResources();
  }

  ~GPULidarHandler()
  {
    if (d_point_cloud_buffer_)
    {
      cudaFree(d_point_cloud_buffer_);
      d_point_cloud_buffer_ = nullptr;
    }
  }

  std::shared_ptr<GPUPointCloudData> getGPUPointCloud()
  {
    auto cpu_cloud_msg = LidarHandler::getPointCloud();
    if (!cpu_cloud_msg || cpu_cloud_msg->points.empty())
    {
      return nullptr;
    }

    const auto& cpu_points = cpu_cloud_msg->points;
    size_t num_points = cpu_points.size();

    if (num_points > max_points_)
    {
      RCLCPP_WARN(rclcpp::get_logger("GPULidarHandler"), 
                  "Point cloud size (%zu) exceeds buffer capacity (%zu). Truncating.", 
                  num_points, max_points_);
      num_points = max_points_;
    }

    cudaError_t err = cudaMemcpy(d_point_cloud_buffer_, cpu_points.data(), 
                                 num_points * sizeof(PointXYZI), cudaMemcpyHostToDevice);
    if (err != cudaSuccess)
    {
      RCLCPP_ERROR(rclcpp::get_logger("GPULidarHandler"), "cudaMemcpy (Host to Device) failed: %s", cudaGetErrorString(err));
      return nullptr;
    }

    auto gpu_data = std::make_shared<GPUPointCloudData>();
    // Use a custom deleter to ensure the shared_ptr does not try to delete the buffer
    gpu_data->d_points_ptr = std::shared_ptr<CudaPointXYZI>(reinterpret_cast<CudaPointXYZI*>(d_point_cloud_buffer_), [](CudaPointXYZI*){});
    gpu_data->num_points = num_points;

    return gpu_data;
  }

  bool isGPUReady() const
  {
    return is_gpu_ready_;
  }

private:
  void initGPUResources()
  {
    max_points_ = 250000; // Allocate for a generous number of points
    cudaError_t err = cudaMalloc((void**)&d_point_cloud_buffer_, max_points_ * sizeof(PointXYZI));
    if (err != cudaSuccess)
    {
      RCLCPP_ERROR(rclcpp::get_logger("GPULidarHandler"), "cudaMalloc failed: %s", cudaGetErrorString(err));
      is_gpu_ready_ = false;
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger("GPULidarHandler"), "GPU buffer allocated successfully for %zu points.", max_points_);
      is_gpu_ready_ = true;
    }
  }

  bool is_gpu_ready_ = false;
  PointXYZI* d_point_cloud_buffer_ = nullptr;
  size_t max_points_ = 0;
};