#pragma once

#include "lidar_handler.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cuda_runtime.h>
#include "../core/cuda_point_types.cuh"
#include <thread>
#include <sstream>

// Macro for debug-only logging
#ifndef NDEBUG
  #define DEBUG_LOG(logger, ...) RCLCPP_INFO(logger, __VA_ARGS__)
#else
  #define DEBUG_LOG(logger, ...)
#endif

// A struct to hold GPU point cloud data.
struct GPUPointCloudData
{
  std::shared_ptr<CudaPointXYZI> d_points_ptr;
  size_t num_points;
};

class GPULidarHandler : public LidarHandler
{
public:
  GPULidarHandler(const RSDriverParam& param, const Eigen::Matrix4f& transform, rclcpp::Clock::SharedPtr clock)
    : LidarHandler(param, transform, clock)
  {
    auto logger = rclcpp::get_logger("GPULidarHandler");
    std::stringstream ss;
    ss << std::this_thread::get_id();
    DEBUG_LOG(logger, "Constructor called in thread %s.", ss.str().c_str());

    cudaError_t last_err = cudaGetLastError();
    if (last_err != cudaSuccess)
    {
        RCLCPP_ERROR(logger, "A CUDA error existed before constructor: %s", cudaGetErrorString(last_err));
    }

    int device_count = 0;
    cudaError_t err = cudaGetDeviceCount(&device_count);
    if (err == cudaSuccess && device_count > 0)
    {
      is_gpu_ready_ = true;
      DEBUG_LOG(logger, "Constructor confirmed GPU is ready.");
    }
    else
    {
      is_gpu_ready_ = false;
      RCLCPP_ERROR(logger, "Constructor failed to find a CUDA-enabled GPU. Error: %s", cudaGetErrorString(err));
    }
  }

  virtual ~GPULidarHandler() override
  {
    stop();
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
    auto logger = rclcpp::get_logger("GPULidarHandler");

    std::stringstream ss;
    ss << std::this_thread::get_id();
    DEBUG_LOG(logger, "getGPUPointCloud called in thread %s for %zu points.", ss.str().c_str(), num_points);

    cudaError_t last_err = cudaGetLastError();
    if (last_err != cudaSuccess)
    {
        RCLCPP_ERROR(logger, "CUDA error detected before cudaMalloc: %s", cudaGetErrorString(last_err));
    }

    CudaPointXYZI* d_new_buffer = nullptr;
    DEBUG_LOG(logger, "Attempting cudaMalloc...");
    cudaError_t err = cudaMalloc((void**)&d_new_buffer, num_points * sizeof(CudaPointXYZI));
    if (err != cudaSuccess)
    {
      RCLCPP_ERROR(logger, "cudaMalloc for %zu points failed: %s", num_points, cudaGetErrorString(err));
      return nullptr;
    }
    DEBUG_LOG(logger, "cudaMalloc successful. Pointer: %p", d_new_buffer);

    DEBUG_LOG(logger, "Attempting cudaMemcpy...");
    err = cudaMemcpy(d_new_buffer, cpu_points.data(), 
                     num_points * sizeof(PointXYZI), cudaMemcpyHostToDevice);
    if (err != cudaSuccess)
    {
      RCLCPP_ERROR(logger, "cudaMemcpy (Host to Device) for %zu points failed: %s", num_points, cudaGetErrorString(err));
      cudaFree(d_new_buffer);
      return nullptr;
    }
    DEBUG_LOG(logger, "cudaMemcpy successful.");

    auto gpu_data = std::make_shared<GPUPointCloudData>();
    gpu_data->num_points = num_points;
    gpu_data->d_points_ptr = std::shared_ptr<CudaPointXYZI>(d_new_buffer, [logger](CudaPointXYZI* ptr) {
      DEBUG_LOG(logger, "Custom deleter freeing GPU pointer: %p", ptr);
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
