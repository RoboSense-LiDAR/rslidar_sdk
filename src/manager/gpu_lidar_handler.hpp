#pragma once

#include "lidar_handler.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cuda_runtime.h>
#include "../core/cuda_point_types.cuh"
#include <thread>
#include <sstream>

// A struct to hold GPU point cloud data.
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
    std::stringstream ss;
    ss << std::this_thread::get_id();
    RCLCPP_INFO(rclcpp::get_logger("GPULidarHandler"), "Constructor called in thread %s.", ss.str().c_str());

    // Check for any pre-existing CUDA errors before we do anything.
    cudaError_t last_err = cudaGetLastError();
    if (last_err != cudaSuccess)
    {
        RCLCPP_ERROR(rclcpp::get_logger("GPULidarHandler"), "A CUDA error existed before constructor: %s", cudaGetErrorString(last_err));
    }

    int device_count = 0;
    cudaError_t err = cudaGetDeviceCount(&device_count);
    if (err == cudaSuccess && device_count > 0)
    {
      is_gpu_ready_ = true;
      RCLCPP_INFO(rclcpp::get_logger("GPULidarHandler"), "Constructor confirmed GPU is ready.");
    }
    else
    {
      is_gpu_ready_ = false;
      RCLCPP_ERROR(rclcpp::get_logger("GPULidarHandler"), "Constructor failed to find a CUDA-enabled GPU. Error: %s", cudaGetErrorString(err));
    }
  }

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

    std::stringstream ss;
    ss << std::this_thread::get_id();
    RCLCPP_INFO(rclcpp::get_logger("GPULidarHandler"), "getGPUPointCloud called in thread %s for %zu points.", ss.str().c_str(), num_points);

    // Check for errors from previous CUDA operations before allocating new memory
    cudaError_t last_err = cudaGetLastError();
    if (last_err != cudaSuccess)
    {
        RCLCPP_ERROR(rclcpp::get_logger("GPULidarHandler"), "CUDA error detected before cudaMalloc: %s", cudaGetErrorString(last_err));
    }

    CudaPointXYZI* d_new_buffer = nullptr;
    RCLCPP_INFO(rclcpp::get_logger("GPULidarHandler"), "Attempting cudaMalloc...");
    cudaError_t err = cudaMalloc((void**)&d_new_buffer, num_points * sizeof(CudaPointXYZI));
    if (err != cudaSuccess)
    {
      RCLCPP_ERROR(rclcpp::get_logger("GPULidarHandler"), "cudaMalloc for %zu points failed: %s", num_points, cudaGetErrorString(err));
      return nullptr;
    }
    RCLCPP_INFO(rclcpp::get_logger("GPULidarHandler"), "cudaMalloc successful. Pointer: %p", d_new_buffer);

    RCLCPP_INFO(rclcpp::get_logger("GPULidarHandler"), "Attempting cudaMemcpy...");
    err = cudaMemcpy(d_new_buffer, cpu_points.data(), 
                     num_points * sizeof(PointXYZI), cudaMemcpyHostToDevice);
    if (err != cudaSuccess)
    {
      RCLCPP_ERROR(rclcpp::get_logger("GPULidarHandler"), "cudaMemcpy (Host to Device) for %zu points failed: %s", num_points, cudaGetErrorString(err));
      cudaFree(d_new_buffer);
      return nullptr;
    }
    RCLCPP_INFO(rclcpp::get_logger("GPULidarHandler"), "cudaMemcpy successful.");

    auto gpu_data = std::make_shared<GPUPointCloudData>();
    gpu_data->num_points = num_points;
    gpu_data->d_points_ptr = std::shared_ptr<CudaPointXYZI>(d_new_buffer, [](CudaPointXYZI* ptr) {
      RCLCPP_INFO(rclcpp::get_logger("GPULidarHandler"), "Custom deleter freeing GPU pointer: %p", ptr);
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