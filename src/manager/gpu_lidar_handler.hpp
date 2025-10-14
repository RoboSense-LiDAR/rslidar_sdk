#pragma once

#include "lidar_handler.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cuda_runtime.h>
#include "../core/cuda_point_types.cuh"

// A struct to hold GPU point cloud data.
// The shared_ptr will manage the lifetime of the GPU memory.
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
    // The constructor is now simpler. We just check if the GPU is generally available.
    // The main check is now in the node's constructor.
    int device_count = 0;
    cudaError_t err = cudaGetDeviceCount(&device_count);
    if (err == cudaSuccess && device_count > 0)
    {
      is_gpu_ready_ = true;
    }
    else
    {
      is_gpu_ready_ = false;
      RCLCPP_ERROR(rclcpp::get_logger("GPULidarHandler"), "Failed to find a CUDA-enabled GPU during handler initialization.");
    }
  }

  // No destructor needed, shared_ptr will handle memory deallocation.

  std::shared_ptr<GPUPointCloudData> getGPUPointCloud()
  {
    if (!is_gpu_ready_) return nullptr;

    auto cpu_cloud_msg = LidarHandler::getPointCloud();
    if (!cpu_cloud_msg || cpu_cloud_msg->points.empty())
    {
      return nullptr;
    }

    const auto& cpu_points = cpu_cloud_msg->points;
    size_t num_points = cpu_points.size();

    // 1. Allocate a new GPU buffer for this specific point cloud
    CudaPointXYZI* d_new_buffer = nullptr;
    cudaError_t err = cudaMalloc((void**)&d_new_buffer, num_points * sizeof(CudaPointXYZI));
    if (err != cudaSuccess)
    {
      RCLCPP_ERROR(rclcpp::get_logger("GPULidarHandler"), "cudaMalloc for %zu points failed: %s", num_points, cudaGetErrorString(err));
      return nullptr;
    }

    // 2. Copy data from CPU to the newly allocated GPU buffer
    err = cudaMemcpy(d_new_buffer, cpu_points.data(), 
                     num_points * sizeof(PointXYZI), cudaMemcpyHostToDevice);
    if (err != cudaSuccess)
    {
      RCLCPP_ERROR(rclcpp::get_logger("GPULidarHandler"), "cudaMemcpy (Host to Device) for %zu points failed: %s", num_points, cudaGetErrorString(err));
      cudaFree(d_new_buffer); // Clean up the failed allocation
      return nullptr;
    }

    // 3. Create the data structure to return
    auto gpu_data = std::make_shared<GPUPointCloudData>();
    gpu_data->num_points = num_points;

    // 4. Wrap the raw pointer in a shared_ptr with a custom deleter that calls cudaFree
    gpu_data->d_points_ptr = std::shared_ptr<CudaPointXYZI>(d_new_buffer, [](CudaPointXYZI* ptr) {
      cudaFree(ptr);
    });

    return gpu_data;
  }

  bool isGPUReady() const
  {
    return is_gpu_ready_;
  }

private:
  bool is_gpu_ready_ = false;
};
