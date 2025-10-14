#include "multi_lidar_node.hpp"
#include <utility/yaml_reader.hpp>
#include <yaml-cpp/emitter.h> // Required for YAML::Emitter
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <chrono>
#include <thread>
#include <algorithm>
#include "../core/cuda_transform_merge.cuh"
#include "../core/cuda_roi_filter.cuh"
#include "../core/cuda_voxel_grid.cuh"
#include "../core/cuda_flatscan.cuh"
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/exceptions.h>
#include <cuda_runtime.h> // For CUDA device check

using namespace robosense::lidar;

MultiLidarNode::MultiLidarNode(const rclcpp::NodeOptions& options)
  : Node("multi_lidar_node", options)
{
  // Explicitly check for CUDA device availability before doing anything else
  int device_count = 0;
  cudaError_t err = cudaGetDeviceCount(&device_count);
  if (err != cudaSuccess)
  {
    RCLCPP_FATAL(this->get_logger(), "Failed to get CUDA device count: %s", cudaGetErrorString(err));
    RCLCPP_FATAL(this->get_logger(), "Please ensure NVIDIA drivers are installed and the GPU is accessible.");
    rclcpp::shutdown();
    return;
  }
  if (device_count == 0)
  {
    RCLCPP_FATAL(this->get_logger(), "No CUDA-enabled GPU found. Aborting.");
    rclcpp::shutdown();
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Found %d CUDA-enabled GPU(s).", device_count);

  // Force initialization of the primary CUDA context on the main thread before any other threads (like the driver's) are created.
  err = cudaFree(0);
  if (err != cudaSuccess)
  {
    RCLCPP_FATAL(this->get_logger(), "Failed to initialize CUDA context: %s", cudaGetErrorString(err));
    rclcpp::shutdown();
    return;
  }
  RCLCPP_INFO(this->get_logger(), "CUDA context initialized successfully on the main thread.");

  bool enable_icp_calibration = this->declare_parameter("enable_icp_calibration", true);
  loadParameters();
  if (enable_icp_calibration)
  {
    runInitialCalibration();
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "[ICP Calibration] ICP calibration is disabled by parameter.");
  }
  loadFilterParameters();
  loadFlatScanParameters(); // New call

  printCurrentParameters(); // Print all loaded parameters

  double publish_frequency = this->declare_parameter("publish_frequency", 10.0);
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / publish_frequency)),
      std::bind(&MultiLidarNode::mergeAndPublish, this));

  param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&MultiLidarNode::parametersCallback, this, std::placeholders::_1));

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  tf_check_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), // 10Hz
      std::bind(&MultiLidarNode::checkTfUpdates, this));

  statistics_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MultiLidarNode::logStatistics, this));
  last_statistics_log_time_ = this->get_clock()->now();
}

void MultiLidarNode::loadParameters()
{
  base_frame_id_ = this->declare_parameter("base_frame_id", "base_link");
  std::string merged_topic_name = this->declare_parameter("merged_topic_name", "/points_merged");
  publish_3d_pcd_ = this->declare_parameter("publish_3d_pcd", true);
  if (publish_3d_pcd_)
  {
    merged_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(merged_topic_name, 10);
  }

#ifdef WITH_NITROS
  bool publish_nitros = this->declare_parameter("publish_nitros", false);
  if (publish_nitros)
  {
    nitros_pub_ = std::make_shared<nvidia::isaac_ros::nitros::ManagedNitrosPublisher<nvidia::isaac_ros::nitros::NitrosPointCloud2>>(
      this,
      "nitros_points_merged",
      nvidia::isaac_ros::nitros::NitrosPointCloud2::supported_type_names);
  }
#endif

  auto lidar_params = this->declare_parameter<std::vector<std::string>>("lidars", std::vector<std::string>());
  for (size_t i = 0; i < lidar_params.size(); ++i) 
  {
      std::string lidar_prefix = "lidars." + std::to_string(i) + ".";
      bool enabled = this->declare_parameter(lidar_prefix + "enabled", false);
      if (!enabled) continue;

      std::string lidar_name = this->declare_parameter(lidar_prefix + "name", "");
      std::string frame_id = this->declare_parameter(lidar_prefix + "pointcloud.frame_id", lidar_name);

      RSDriverParam driver_param;
      std::string lidar_type_str = this->declare_parameter(lidar_prefix + "driver.lidar_type", "RS16");
      driver_param.lidar_type = strToLidarType(lidar_type_str);
      std::string input_type_str = this->declare_parameter(lidar_prefix + "driver.input_type", "online");
      if (input_type_str == "online")
      {
        driver_param.input_type = InputType::ONLINE_LIDAR;
      }
      else if (input_type_str == "pcap")
      {
        driver_param.input_type = InputType::PCAP_FILE;
      }
      else if (input_type_str == "rosbag")
      {
        driver_param.input_type = InputType::RAW_PACKET;
      }
      else
      {
        driver_param.input_type = InputType::ONLINE_LIDAR; // Default
        RCLCPP_WARN(this->get_logger(), "Invalid input_type '%s' for %s. Defaulting to 'online'.", input_type_str.c_str(), lidar_name.c_str());
      }
      driver_param.input_param.msop_port = this->declare_parameter(lidar_prefix + "driver.msop_port", 6699);
      driver_param.input_param.difop_port = this->declare_parameter(lidar_prefix + "driver.difop_port", 7788);

      double x = this->declare_parameter(lidar_prefix + "tf.x", 0.0);
      double y = this->declare_parameter(lidar_prefix + "tf.y", 0.0);
      double z = this->declare_parameter(lidar_prefix + "tf.z", 0.0);
      double roll = this->declare_parameter(lidar_prefix + "tf.roll", 0.0);
      double pitch = this->declare_parameter(lidar_prefix + "tf.pitch", 0.0);
      double yaw = this->declare_parameter(lidar_prefix + "tf.yaw", 0.0);

      Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
      Eigen::Translation3f translation(x, y, z);
      Eigen::AngleAxisf rot_x(roll, Eigen::Vector3f::UnitX());
      Eigen::AngleAxisf rot_y(pitch, Eigen::Vector3f::UnitY());
      Eigen::AngleAxisf rot_z(yaw, Eigen::Vector3f::UnitZ());
      transform = (translation * rot_x * rot_y * rot_z).matrix();

#ifndef NDEBUG
      // Diagnostic logging to check CUDA state around handler creation
      cudaError_t err_before = cudaGetLastError();
      RCLCPP_INFO(this->get_logger(), "CUDA state before creating handler for '%s': %s", lidar_name.c_str(), cudaGetErrorString(err_before));
#endif

      LidarInfo info;
      info.handler = std::make_shared<GPULidarHandler>(driver_param, transform, this->get_clock());
      info.frame_id = frame_id;
      info.original_index = i;
      info.last_tf_hash = 0;
      info.icp_correction = Eigen::Matrix4f::Identity();
      lidar_info_.push_back(info);

      // Get a reference to the new element to modify it in place
      LidarInfo& new_info_ref = lidar_info_.back();

      // Check if raw pointcloud publishing is enabled
      bool publish_raw = this->declare_parameter(lidar_prefix + "pointcloud.publish_raw", false);
      if (publish_raw)
      {
        std::string default_raw_topic = "/" + lidar_name + "/points_raw";
        std::string raw_topic = this->declare_parameter(lidar_prefix + "pointcloud.raw_topic", default_raw_topic);
        new_info_ref.raw_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(raw_topic, 10);
        RCLCPP_INFO(this->get_logger(), "Raw pointcloud publishing enabled for '%s' on topic '%s'", lidar_name.c_str(), raw_topic.c_str());
      }
      
#ifndef NDEBUG
      cudaError_t err_after = cudaGetLastError();
      RCLCPP_INFO(this->get_logger(), "CUDA state after creating handler for '%s': %s", lidar_name.c_str(), cudaGetErrorString(err_after));
#endif

      RCLCPP_INFO(this->get_logger(), "Initialized lidar: %s", lidar_name.c_str());
  }
}

void MultiLidarNode::loadFilterParameters()
{
  enable_roi_filter_ = this->declare_parameter("enable_roi_filter", false);
  if (enable_roi_filter_)
  {
    RCLCPP_INFO(this->get_logger(), "[Filter] ROI filter enabled.");
    auto roi_filters_param = this->declare_parameter<std::vector<std::string>>("roi_filters", std::vector<std::string>());

    for (size_t i = 0; i < roi_filters_param.size(); ++i)
    {
      std::string prefix = "roi_filters." + std::to_string(i) + ".";
      RoiFilterConfig config;
      config.name = this->declare_parameter(prefix + "name", "unnamed_roi");
      config.type = this->declare_parameter(prefix + "type", "positive");
      config.min_x = this->declare_parameter(prefix + "min_x", -100.0f);
      config.max_x = this->declare_parameter(prefix + "max_x", 100.0f);
      config.min_y = this->declare_parameter(prefix + "min_y", -100.0f);
      config.max_y = this->declare_parameter(prefix + "max_y", 100.0f);
      config.min_z = this->declare_parameter(prefix + "min_z", -100.0f);
      config.max_z = this->declare_parameter(prefix + "max_z", 100.0f);
      roi_filters_.push_back(config);

      RCLCPP_INFO(this->get_logger(), "  - ROI Filter '%s' (type: %s): X=[%f, %f], Y=[%f, %f], Z=[%f, %f]",
                  config.name.c_str(), config.type.c_str(),
                  config.min_x, config.max_x, config.min_y, config.max_y, config.min_z, config.max_z);
    }
  }

  enable_voxel_filter_ = this->declare_parameter("enable_voxel_filter", false);
  if (enable_voxel_filter_)
  {
    voxel_leaf_size_ = this->declare_parameter("voxel_leaf_size", 0.1f);
    RCLCPP_INFO(this->get_logger(), "[Filter] Voxel filter enabled: leaf_size=%f", voxel_leaf_size_);
  }
}

void MultiLidarNode::loadFlatScanParameters()
{
  publish_flatscan_ = this->declare_parameter("publish_flatscan", false);
  if (publish_flatscan_)
  {
    flatscan_topic_name_ = this->declare_parameter("flatscan_topic_name", "/flatscan");
    flatscan_min_height_ = this->declare_parameter("flatscan_min_height", -0.1f);
    flatscan_max_height_ = this->declare_parameter("flatscan_max_height", 0.1f);
    flatscan_angle_min_ = this->declare_parameter("flatscan_angle_min", -M_PI);
    flatscan_angle_max_ = this->declare_parameter("flatscan_angle_max", M_PI);
    flatscan_angle_increment_ = this->declare_parameter("flatscan_angle_increment", 0.0087f); // approx 0.5 deg
    flatscan_range_min_ = this->declare_parameter("flatscan_range_min", 0.1f);
    flatscan_range_max_ = this->declare_parameter("flatscan_range_max", 100.0f);

    flatscan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(flatscan_topic_name_, 10);
    RCLCPP_INFO(this->get_logger(), "[FlatScan] LaserScan publishing enabled on topic '%s'. Height range: [%f, %f]",
                flatscan_topic_name_.c_str(), flatscan_min_height_, flatscan_max_height_);
  }
}

void MultiLidarNode::runInitialCalibration()
{
  if (lidar_info_.size() < 2)
  {
    RCLCPP_INFO(this->get_logger(), "[ICP Calibration] Not enough LiDARs (%zu) for ICP calibration. Skipping.", lidar_info_.size());
    return;
  }

  RCLCPP_INFO(this->get_logger(), "[ICP Calibration] Starting initial ICP calibration...");

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> initial_cpu_clouds(lidar_info_.size());
  std::vector<bool> cloud_received(lidar_info_.size(), false);
  auto start_time = std::chrono::high_resolution_clock::now();
  const std::chrono::seconds timeout(15); // 15 seconds to collect initial clouds

  // 1. Collect initial point clouds from all LiDARs (copy from GPU to CPU for PCL ICP)
  while (std::any_of(cloud_received.begin(), cloud_received.end(), [](bool b){ return !b; }))
  {
    auto current_time = std::chrono::high_resolution_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time) > timeout)
    {
      RCLCPP_ERROR(this->get_logger(), "[ICP Calibration] Timeout while collecting initial point clouds. Calibration aborted.");
      return;
    }

    for (size_t i = 0; i < lidar_info_.size(); ++i)
    {
      if (!cloud_received[i])
      {
        auto gpu_cloud_data = lidar_info_[i].handler->getGPUPointCloud();
        if (gpu_cloud_data && gpu_cloud_data->d_points_ptr && gpu_cloud_data->num_points > 0)
        {
          pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
          pcl_cloud->points.resize(gpu_cloud_data->num_points);

          // Copy from GPU to a temporary CPU buffer first, as PCL and CUDA structs may have different padding.
          std::vector<CudaPointXYZI> temp_cpu_buffer(gpu_cloud_data->num_points);
          cudaError_t err = cudaMemcpy(temp_cpu_buffer.data(), gpu_cloud_data->d_points_ptr.get(), 
                                       gpu_cloud_data->num_points * sizeof(CudaPointXYZI), cudaMemcpyDeviceToHost);
          if (err != cudaSuccess)
          {
              RCLCPP_ERROR(this->get_logger(), "cudaMemcpy (GPU to CPU for ICP) failed: %s", cudaGetErrorString(err));
              continue;
          }

          // Manually copy from the temporary buffer to the PCL cloud to handle struct differences.
          for (size_t j = 0; j < gpu_cloud_data->num_points; ++j)
          {
            pcl_cloud->points[j].x = temp_cpu_buffer[j].x;
            pcl_cloud->points[j].y = temp_cpu_buffer[j].y;
            pcl_cloud->points[j].z = temp_cpu_buffer[j].z;
            pcl_cloud->points[j].intensity = temp_cpu_buffer[j].intensity;
          }

          pcl_cloud->width = pcl_cloud->points.size();
          pcl_cloud->height = 1;
          pcl_cloud->is_dense = false; // Assume it might have NaNs

          // The driver can produce NaN points. PCL's ICP asserts on these.
          // We must remove them before proceeding.
          pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
          std::vector<int> indices;
          pcl::removeNaNFromPointCloud(*pcl_cloud, *filtered_cloud, indices);
          filtered_cloud->is_dense = true; // Now it's actually dense

          initial_cpu_clouds[i] = filtered_cloud; // Use the filtered cloud
          cloud_received[i] = true;
          RCLCPP_INFO(this->get_logger(), "[ICP Calibration] Received initial cloud from LiDAR %zu (%zu points, %zu after NaN filter).", 
            i, pcl_cloud->points.size(), filtered_cloud->points.size());
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Wait a bit before checking again
  }

  RCLCPP_INFO(this->get_logger(), "[ICP Calibration] All initial point clouds collected. Proceeding with ICP.");

  // 2. Perform ICP for each source LiDAR against the first LiDAR (target)
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud = initial_cpu_clouds[0];
  // The transform of the target LiDAR (lidar_info_[0].handler) relative to base_link
  Eigen::Matrix4f base_to_target_tf = lidar_info_[0].handler->getTransform();

  for (size_t i = 1; i < lidar_info_.size(); ++i)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud = initial_cpu_clouds[i];
    std::shared_ptr<GPULidarHandler> source_handler = lidar_info_[i].handler;

    // Get the initial guess for source_lidar_link to base_link
    Eigen::Matrix4f initial_source_to_base_tf = source_handler->getTransform();

    // Calculate the initial guess for source_lidar_link to target_lidar_link
    // This is: (base_link -> target_lidar_link)^-1 * (base_link -> source_lidar_link)
    Eigen::Matrix4f initial_guess_source_to_target = base_to_target_tf.inverse() * initial_source_to_base_tf;

    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    icp.setMaxCorrespondenceDistance(0.5); // Max distance for a point to be considered a correspondence
    icp.setMaximumIterations(100);         // Max number of ICP iterations
    icp.setTransformationEpsilon(1e-8);    // Minimum transformation difference between iterations
    icp.setEuclideanFitnessEpsilon(1e-5);  // Minimum fitness score difference between iterations

    icp.setInputSource(source_cloud);
    icp.setInputTarget(target_cloud);

    pcl::PointCloud<pcl::PointXYZI> unused_aligned_cloud; // To store the aligned cloud, not directly used after ICP
    icp.align(unused_aligned_cloud, initial_guess_source_to_target);

    if (icp.hasConverged())
    {
      Eigen::Matrix4f final_source_to_target_tf = icp.getFinalTransformation();
      
      // Calculate the new transform for source_lidar_link to base_link
      Eigen::Matrix4f calibrated_source_to_base_tf = base_to_target_tf * final_source_to_target_tf;

      // Calculate and store the correction
      Eigen::Matrix4f correction = initial_source_to_base_tf.inverse() * calibrated_source_to_base_tf;
      lidar_info_[i].icp_correction = correction;

      RCLCPP_INFO(this->get_logger(), "[ICP Calibration] LiDAR %zu calibrated successfully. Fitness score: %f", i, icp.getFitnessScore());
      
      // Decompose and print the new transform
      Eigen::Vector3f translation = calibrated_source_to_base_tf.block<3,1>(0,3);
      Eigen::Matrix3f rotation = calibrated_source_to_base_tf.block<3,3>(0,0);
      Eigen::Vector3f euler = rotation.eulerAngles(0, 1, 2); // XYZ order for roll, pitch, yaw from Eigen
      
      double calibrated_x = translation.x();
      double calibrated_y = translation.y();
      double calibrated_z = translation.z();
      double calibrated_roll = euler[0];
      double calibrated_pitch = euler[1];
      double calibrated_yaw = euler[2];

      RCLCPP_INFO(this->get_logger(), "[ICP Calibration] New TF for LiDAR %zu: [x: %.4f, y: %.4f, z: %.4f, roll: %.4f, pitch: %.4f, yaw: %.4f]",
        lidar_info_[i].original_index, calibrated_x, calibrated_y, calibrated_z, calibrated_roll, calibrated_pitch, calibrated_yaw);

      // Update the ROS parameters with the new TF values
      size_t original_index = lidar_info_[i].original_index;
      std::string tf_prefix = "lidars." + std::to_string(original_index) + ".tf.";
      this->set_parameters({
        rclcpp::Parameter(tf_prefix + "x", calibrated_x),
        rclcpp::Parameter(tf_prefix + "y", calibrated_y),
        rclcpp::Parameter(tf_prefix + "z", calibrated_z),
        rclcpp::Parameter(tf_prefix + "roll", calibrated_roll),
        rclcpp::Parameter(tf_prefix + "pitch", calibrated_pitch),
        rclcpp::Parameter(tf_prefix + "yaw", calibrated_yaw)
      });
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "[ICP Calibration] ICP did not converge for LiDAR %zu. Using initial guess.", i);
    }
  }
  RCLCPP_INFO(this->get_logger(), "[ICP Calibration] Initial ICP calibration finished.");
}

void MultiLidarNode::mergeAndPublish()

{

  // --- Early Exit if no publishing is enabled ---

  if (!publish_3d_pcd_ && !publish_flatscan_)

  {

    return;

  }



  std::vector<CudaPointXYZI*> d_input_clouds;

  std::vector<size_t> h_input_counts;

    std::vector<size_t> h_prefix_sums;

    std::vector<CudaMatrix4f> h_transforms;

    std::vector<rclcpp::Time> timestamps;

    size_t total_points_to_merge = 0;

    std::vector<std::shared_ptr<GPUPointCloudData>> alive_gpu_clouds; // Keep pointers alive

  

    for (const auto& info : lidar_info_)

    {

      if (!info.handler->isGPUReady())

      {

        continue; 

      }

          auto gpu_cloud_data = info.handler->getGPUPointCloud();

          if (gpu_cloud_data && gpu_cloud_data->d_points_ptr && gpu_cloud_data->num_points > 100) // Basic check for a reasonable number of points

          {

            // Publish raw pointcloud if enabled

            if (info.raw_pub)

            {

              pcl::PointCloud<pcl::PointXYZI>::Ptr cpu_cloud(new pcl::PointCloud<pcl::PointXYZI>);

              cpu_cloud->points.resize(gpu_cloud_data->num_points);

              cudaError_t err = cudaMemcpy(cpu_cloud->points.data(), gpu_cloud_data->d_points_ptr.get(),

                                           gpu_cloud_data->num_points * sizeof(CudaPointXYZI), cudaMemcpyDeviceToHost);

              if (err != cudaSuccess)

              {

                RCLCPP_ERROR(this->get_logger(), "cudaMemcpy (GPU to CPU for raw publish) failed: %s", cudaGetErrorString(err));

              }

              else

              {

                sensor_msgs::msg::PointCloud2 output_msg;

                pcl::toROSMsg(*cpu_cloud, output_msg);

                output_msg.header.stamp = gpu_cloud_data->timestamp;

                output_msg.header.frame_id = info.frame_id;

                info.raw_pub->publish(output_msg);

              }

            }

      

            // Simple validity check: if all points are clustered at the origin, the driver might not be ready.

            // This is a simplified check. A proper one would be a kernel to calculate the bounding box.

            // For now, we assume that if we have points, they are valid enough to proceed.

            // A more robust check could be added here if needed.

            

            alive_gpu_clouds.push_back(gpu_cloud_data); // Keep the shared_ptr alive

            d_input_clouds.push_back(gpu_cloud_data->d_points_ptr.get());

            h_input_counts.push_back(gpu_cloud_data->num_points);

            total_points_to_merge += gpu_cloud_data->num_points;

            timestamps.push_back(gpu_cloud_data->timestamp);

      

            CudaMatrix4f cuda_transform;

            cuda_transform.fromEigen(info.handler->getTransform().data());

            h_transforms.push_back(cuda_transform);

          }

    }



  if (total_points_to_merge == 0 || timestamps.empty())

  {

    return;

  }



  // Use the earliest timestamp from all LiDARs for this merged frame

  const auto earliest_timestamp = *std::min_element(timestamps.begin(), timestamps.end());



  // Calculate prefix sums (exclusive scan) to determine the starting offset for each cloud in the merged buffer

  h_prefix_sums.resize(h_input_counts.size(), 0);

  size_t running_total = 0;

  for (size_t i = 0; i < h_input_counts.size(); ++i)

  {

    h_prefix_sums[i] = running_total;

    running_total += h_input_counts[i];

  }



  CudaPointXYZI* d_merged_cloud = nullptr;

  cudaError_t err = cudaMalloc((void**)&d_merged_cloud, total_points_to_merge * sizeof(CudaPointXYZI));

  if (err != cudaSuccess)

  {

      RCLCPP_ERROR(this->get_logger(), "cudaMalloc for merged cloud failed: %s", cudaGetErrorString(err));

      return;

  }





#ifndef NDEBUG

  // --- Sanity Check Logging ---

  RCLCPP_DEBUG(this->get_logger(), "--- Pre-Kernel Sanity Check ---");

  RCLCPP_DEBUG(this->get_logger(), "Total points to merge: %zu", total_points_to_merge);

  RCLCPP_DEBUG(this->get_logger(), "Number of LiDARs to merge: %zu", d_input_clouds.size());

  for (size_t i = 0; i < d_input_clouds.size(); ++i)

  {

    std::stringstream ss;

    for(int j=0; j<16; ++j) ss << h_transforms[i].data[j] << " ";

    RCLCPP_DEBUG(this->get_logger(), "Lidar %zu: Points=%zu, Offset=%zu, Transform=[%s]", 

      i, h_input_counts[i], h_prefix_sums[i], ss.str().c_str());

  }

  RCLCPP_DEBUG(this->get_logger(), "-----------------------------");

#endif



  // Sequentially launch a simple kernel for each LiDAR. This is more robust than a single complex kernel.

  for (size_t i = 0; i < d_input_clouds.size(); ++i)

  {

    err = transformAndCopyToOffsetGPU(d_input_clouds[i], h_input_counts[i], h_transforms[i], d_merged_cloud, h_prefix_sums[i]);

    if (err != cudaSuccess)

    {

      RCLCPP_ERROR(this->get_logger(), "transformAndCopyToOffsetGPU kernel launch for lidar %zu failed: %s", i, cudaGetErrorString(err));

      cudaFree(d_merged_cloud);

      return;

    }

  }



  // Wait for all the above asynchronous kernels to complete before proceeding to the filter stage.

  err = cudaDeviceSynchronize();

  if (err != cudaSuccess)

  {

    RCLCPP_ERROR(this->get_logger(), "cudaDeviceSynchronize after merging failed: %s", cudaGetErrorString(err));

    cudaFree(d_merged_cloud);

    return;

  }



  CudaPointXYZI* d_roi_filtered_cloud = nullptr;

  size_t num_roi_filtered_points = 0;



  if (enable_roi_filter_ && !roi_filters_.empty())

  {

    std::vector<CudaRoiFilterConfig> h_cuda_roi_filters;

    for (const auto& config : roi_filters_)

    {

      CudaRoiFilterConfig cuda_config;

      cuda_config.min_x = config.min_x; cuda_config.max_x = config.max_x;

      cuda_config.min_y = config.min_y; cuda_config.max_y = config.max_y;

      cuda_config.min_z = config.min_z; cuda_config.max_z = config.max_z;

      cuda_config.type = (config.type == "positive") ? 0 : 1;

      h_cuda_roi_filters.push_back(cuda_config);

    }



    err = roiFilterGPU(d_merged_cloud, total_points_to_merge,

                       h_cuda_roi_filters.data(), h_cuda_roi_filters.size(),

                       &d_roi_filtered_cloud, &num_roi_filtered_points);

    if (err != cudaSuccess)

    {

        RCLCPP_ERROR(this->get_logger(), "roiFilterGPU kernel launch failed: %s", cudaGetErrorString(err));

        cudaFree(d_merged_cloud);

        return;

    }

    cudaFree(d_merged_cloud);

  }

  else

  {

    d_roi_filtered_cloud = d_merged_cloud;

    num_roi_filtered_points = total_points_to_merge;

  }



  CudaPointXYZI* d_voxel_filtered_cloud = nullptr;

  size_t num_voxel_filtered_points = 0;



  if (enable_voxel_filter_ && num_roi_filtered_points > 0)

  {

    err = voxelGridDownsampleGPU(d_roi_filtered_cloud, num_roi_filtered_points,

                                 voxel_leaf_size_, &d_voxel_filtered_cloud, &num_voxel_filtered_points);

    if (err != cudaSuccess)

    {

        RCLCPP_ERROR(this->get_logger(), "voxelGridDownsampleGPU kernel launch failed: %s", cudaGetErrorString(err));

        cudaFree(d_roi_filtered_cloud);

        return;

    }

    cudaFree(d_roi_filtered_cloud);

  }

  else

  {

    d_voxel_filtered_cloud = d_roi_filtered_cloud;

    num_voxel_filtered_points = num_roi_filtered_points;

  }





#ifndef NDEBUG

  RCLCPP_DEBUG(this->get_logger(), "Point Processing: Merged: %zu -> ROI Filter: %zu -> Voxel Filter: %zu",

    total_points_to_merge, num_roi_filtered_points, num_voxel_filtered_points);

#endif



  if (num_voxel_filtered_points == 0)

  {

    if (d_voxel_filtered_cloud) cudaFree(d_voxel_filtered_cloud);

    return;

  }



  if (publish_3d_pcd_)

  {

    pcl::PointCloud<pcl::PointXYZI>::Ptr final_cpu_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    final_cpu_cloud->points.resize(num_voxel_filtered_points);

    err = cudaMemcpy(final_cpu_cloud->points.data(), d_voxel_filtered_cloud, 

                     num_voxel_filtered_points * sizeof(CudaPointXYZI), cudaMemcpyDeviceToHost);

    if (err != cudaSuccess)

    {

        RCLCPP_ERROR(this->get_logger(), "cudaMemcpy (GPU to CPU for ROS publish) failed: %s", cudaGetErrorString(err));

    }

    else

    {

        sensor_msgs::msg::PointCloud2 output_msg;

        pcl::toROSMsg(*final_cpu_cloud, output_msg);

        output_msg.header.stamp = earliest_timestamp;

        output_msg.header.frame_id = base_frame_id_;

#ifndef NDEBUG

        RCLCPP_DEBUG(this->get_logger(), "Publishing PointCloud2 on topic '%s' with %zu points.",

          merged_pub_->get_topic_name(), num_voxel_filtered_points);

#endif

        merged_pub_->publish(output_msg);



#ifdef WITH_NITROS

        if (nitros_pub_)

        {

          auto nitros_output_msg = output_msg;

          nvidia::isaac_ros::nitros::NitrosPointCloud2 nitros_msg =

            nvidia::isaac_ros::nitros::NitrosPointCloud2Builder()

            .With(

              std::move(nitros_output_msg)

            )

            .Build();

          nitros_pub_->publish(nitros_msg);

        }

#endif

    }

  }



  if (publish_flatscan_)

  {

    CudaLaserScanParams flatscan_params;

    flatscan_params.angle_min = flatscan_angle_min_;

    flatscan_params.angle_max = flatscan_angle_max_;

    flatscan_params.angle_increment = flatscan_angle_increment_;

    flatscan_params.range_min = flatscan_range_min_;

    flatscan_params.range_max = flatscan_range_max_;

    flatscan_params.min_height = flatscan_min_height_;

    flatscan_params.max_height = flatscan_max_height_;

    flatscan_params.num_beams = static_cast<size_t>((flatscan_angle_max_ - flatscan_angle_min_) / flatscan_angle_increment_) + 1;



    float* d_ranges = nullptr;

    err = generateFlatScanGPU(d_voxel_filtered_cloud, num_voxel_filtered_points, flatscan_params, &d_ranges);

    if (err != cudaSuccess)

    {

        RCLCPP_ERROR(this->get_logger(), "generateFlatScanGPU kernel launch failed: %s", cudaGetErrorString(err));

    }

    else

    {

        sensor_msgs::msg::LaserScan scan_msg;

        scan_msg.header.stamp = earliest_timestamp;

        scan_msg.header.frame_id = base_frame_id_;

        scan_msg.angle_min = flatscan_params.angle_min;

        scan_msg.angle_max = flatscan_params.angle_max;

        scan_msg.angle_increment = flatscan_params.angle_increment;

        scan_msg.range_min = flatscan_params.range_min;

        scan_msg.range_max = flatscan_params.range_max;

        scan_msg.ranges.resize(flatscan_params.num_beams);



        err = cudaMemcpy(scan_msg.ranges.data(), d_ranges, flatscan_params.num_beams * sizeof(float), cudaMemcpyDeviceToHost);

        if (err != cudaSuccess)

        {

            RCLCPP_ERROR(this->get_logger(), "cudaMemcpy (GPU to CPU for LaserScan) failed: %s", cudaGetErrorString(err));

        }

        else

        {

#ifndef NDEBUG

            RCLCPP_DEBUG(this->get_logger(), "Publishing LaserScan on topic '%s' with %zu ranges.",

              flatscan_pub_->get_topic_name(), scan_msg.ranges.size());

#endif

            flatscan_pub_->publish(scan_msg);

        }

        cudaFree(d_ranges);

    }

  }



  if (d_voxel_filtered_cloud) cudaFree(d_voxel_filtered_cloud);



  // Update statistics for the logger

  double latency_sec = (this->get_clock()->now() - earliest_timestamp).seconds();

  last_processing_latency_.store(latency_sec);

  last_total_points_merged_.store(total_points_to_merge);

  last_points_after_roi_.store(num_roi_filtered_points);

  last_points_after_voxel_.store(num_voxel_filtered_points);

  merge_publish_count_++;

}



void MultiLidarNode::printCurrentParameters()

{

  RCLCPP_INFO(this->get_logger(), "--- Current ROS 2 Parameters ---");



  auto parameter_names = this->list_parameters({}, 0).names;



  if (parameter_names.empty())

  {

    RCLCPP_INFO(this->get_logger(), "  No parameters found for this node.");

  }

  else

  {

    auto parameters = this->get_parameters(parameter_names);



    for (const auto &param : parameters)

    {

      RCLCPP_INFO(this->get_logger(), "  %s: %s",

        param.get_name().c_str(),

        param.value_to_string().c_str());

    }

  }



  RCLCPP_INFO(this->get_logger(), "--------------------------------");

}



rcl_interfaces::msg::SetParametersResult MultiLidarNode::parametersCallback(const std::vector<rclcpp::Parameter> &parameters)

{

  rcl_interfaces::msg::SetParametersResult result;

  result.successful = true;



  for (const auto &parameter : parameters)

  {

    std::string name = parameter.get_name();

    std::string prefix = "lidars.";



    if (name.rfind(prefix, 0) == 0)

    {

      size_t dot_pos = name.find(".", prefix.length());

                              if (dot_pos != std::string::npos)

                              {

                                // TF parameters are now handled exclusively by checkTfUpdates.

                                // We no longer need to handle them here to avoid conflicts.

                              }

                            }

              }



  return result;

}



namespace

{

  template <class T>

  inline void hash_combine(std::size_t& seed, const T& v)

  {

    std::hash<T> hasher;

    seed ^= hasher(v) + 0x9e3779b9 + (seed<<6) + (seed>>2);

  }



  size_t hash_transform(const geometry_msgs::msg::Transform& transform)

  {

    size_t seed = 0;

    hash_combine(seed, transform.translation.x);

    hash_combine(seed, transform.translation.y);

    hash_combine(seed, transform.translation.z);

    hash_combine(seed, transform.rotation.x);

    hash_combine(seed, transform.rotation.y);

    hash_combine(seed, transform.rotation.z);

    hash_combine(seed, transform.rotation.w);

    return seed;

  }

}



void MultiLidarNode::checkTfUpdates()



{



  for (size_t i = 0; i < lidar_info_.size(); ++i)



  {



    if (lidar_info_[i].frame_id.empty())



    {



      continue;



    }







    try



    {



      geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(



        base_frame_id_, lidar_info_[i].frame_id, tf2::TimePointZero);







      size_t new_hash = hash_transform(transform_stamped.transform);







                  if (new_hash != lidar_info_[i].last_tf_hash)







                  {







                    lidar_info_[i].last_tf_hash = new_hash;







            







                    Eigen::Affine3d affine_transform = tf2::transformToEigen(transform_stamped);







                    Eigen::Matrix4f dynamic_transform = affine_transform.matrix().cast<float>();







            







                    // Apply the ICP correction to the dynamic transform







                    Eigen::Matrix4f final_transform = dynamic_transform * lidar_info_[i].icp_correction;







            







                    lidar_info_[i].handler->setTransform(final_transform);







                    RCLCPP_INFO(this->get_logger(), "Updated TF for lidar (config index %zu) from TF tree with ICP correction.", lidar_info_[i].original_index);







                  }







          }







          catch (const tf2::TransformException &ex)







          {







          }







        }







      }



void MultiLidarNode::logStatistics()

{

    // Calculate frequency of the main processing loop

    auto now = this->get_clock()->now();

    double elapsed_sec = (now - last_statistics_log_time_).seconds();

    uint64_t current_count = merge_publish_count_.exchange(0); // Atomically get and reset counter

    double frequency = (elapsed_sec > 0) ? (current_count / elapsed_sec) : 0.0;

    last_statistics_log_time_ = now;



    RCLCPP_INFO(this->get_logger(), "-------------------- Pipeline Statistics --------------------");

    RCLCPP_INFO(this->get_logger(), "Frequency: %.2f Hz", frequency);

    RCLCPP_INFO(this->get_logger(), "End-to-End Latency: %.3f ms", last_processing_latency_.load() * 1000.0);



        // LiDAR Status



        RCLCPP_INFO(this->get_logger(), "LiDAR Status:");



        for (size_t i = 0; i < lidar_info_.size(); ++i)



        {



            auto last_cloud_time = lidar_info_[i].handler->getLastCloudTimestamp();



            // Check for a very old timestamp, indicating no cloud has ever been received



            if (last_cloud_time.seconds() == 0)



            {



                RCLCPP_INFO(this->get_logger(), "  - LiDAR %zu (Config Index): Waiting for data...", lidar_info_[i].original_index);



            }



            else



            {



                double time_since_last_cloud = (now - last_cloud_time).seconds();



                std::string status = (time_since_last_cloud < 2.0) ? "Connected" : "Disconnected"; // 2-second timeout



                RCLCPP_INFO(this->get_logger(), "  - LiDAR %zu (Config Index): %s (Last cloud: %.2f s ago)", lidar_info_[i].original_index, status.c_str(), time_since_last_cloud);



            }



        }



        // Published Topics



        RCLCPP_INFO(this->get_logger(), "Published Topics:");



        if (publish_3d_pcd_ && merged_pub_)



        {



            RCLCPP_INFO(this->get_logger(), "  - Merged PointCloud2: '%s'", merged_pub_->get_topic_name());



        }



        else



        {



            RCLCPP_INFO(this->get_logger(), "  - Merged PointCloud2: Disabled");



        }



    



        if (publish_flatscan_ && flatscan_pub_)



        {



            RCLCPP_INFO(this->get_logger(), "  - LaserScan: '%s'", flatscan_pub_->get_topic_name());



        }



        else



        {



            RCLCPP_INFO(this->get_logger(), "  - LaserScan: Disabled");



        }



        for (const auto& info : lidar_info_)



        {



            if (info.raw_pub)



            {



                RCLCPP_INFO(this->get_logger(), "  - Raw PointCloud2 (Lidar %zu): '%s'", info.original_index, info.raw_pub->get_topic_name());



            }



            else



            {



                RCLCPP_INFO(this->get_logger(), "  - Raw PointCloud2 (Lidar %zu): Disabled", info.original_index);



            }



        }



    // Point Processing Stats for the last processed frame

    RCLCPP_INFO(this->get_logger(), "Point Processing (Last Frame):");

    RCLCPP_INFO(this->get_logger(), "  - Merged Raw: %zu points", last_total_points_merged_.load());

    RCLCPP_INFO(this->get_logger(), "  - After ROI Filter: %zu points", last_points_after_roi_.load());

    RCLCPP_INFO(this->get_logger(), "  - After Voxel Filter: %zu points", last_points_after_voxel_.load());

    RCLCPP_INFO(this->get_logger(), "---------------------------------------------------------");

}
