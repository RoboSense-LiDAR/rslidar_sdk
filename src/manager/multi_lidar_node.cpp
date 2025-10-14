
#include "multi_lidar_node.hpp"
#include <utility/yaml_reader.hpp>
#include <yaml-cpp/emitter.h> // Required for YAML::Emitter
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <chrono>
#include <thread>
#include <algorithm>
#include "../core/cuda_transform_merge.cuh"
#include "../core/cuda_roi_filter.cuh"
#include "../core/cuda_voxel_grid.cuh"
#include "../core/cuda_flatscan.cuh"
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/exceptions.h>

using namespace robosense::lidar;

MultiLidarNode::MultiLidarNode(const rclcpp::NodeOptions& options)
  : Node("multi_lidar_node", options)
{
  loadParameters();
  runInitialCalibration();
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
      std::string frame_id = this->declare_parameter(lidar_prefix + "frame_id", lidar_name);
      lidar_frame_ids_.push_back(frame_id);

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
      else if (input_type_str == "raw")
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
      transform = (translation * rot_z * rot_y * rot_x).matrix();

      lidar_handlers_.emplace_back(std::make_shared<GPULidarHandler>(driver_param, transform));
      RCLCPP_INFO(this->get_logger(), "Initialized lidar: %s", this->declare_parameter(lidar_prefix + "name", "").c_str());
  }
  last_tf_hashes_.resize(lidar_handlers_.size());
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
  if (lidar_handlers_.size() < 2)
  {
    RCLCPP_INFO(this->get_logger(), "[ICP Calibration] Not enough LiDARs (%zu) for ICP calibration. Skipping.", lidar_handlers_.size());
    return;
  }

  RCLCPP_INFO(this->get_logger(), "[ICP Calibration] Starting initial ICP calibration...");

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> initial_cpu_clouds(lidar_handlers_.size());
  std::vector<bool> cloud_received(lidar_handlers_.size(), false);
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

    for (size_t i = 0; i < lidar_handlers_.size(); ++i)
    {
      if (!cloud_received[i])
      {
        auto gpu_cloud_data = lidar_handlers_[i]->getGPUPointCloud();
        if (gpu_cloud_data && gpu_cloud_data->d_points_ptr && gpu_cloud_data->num_points > 0)
        {
          pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
          pcl_cloud->points.resize(gpu_cloud_data->num_points);

          // Copy from GPU to CPU for PCL ICP
          cudaError_t err = cudaMemcpy(pcl_cloud->points.data(), gpu_cloud_data->d_points_ptr.get(), 
                                       gpu_cloud_data->num_points * sizeof(CudaPointXYZI), cudaMemcpyDeviceToHost);
          if (err != cudaSuccess)
          {
              RCLCPP_ERROR(this->get_logger(), "cudaMemcpy (GPU to CPU for ICP) failed: %s", cudaGetErrorString(err));
              continue;
          }

          pcl_cloud->width = pcl_cloud->points.size();
          pcl_cloud->height = 1;
          pcl_cloud->is_dense = true;

          initial_cpu_clouds[i] = pcl_cloud;
          cloud_received[i] = true;
          RCLCPP_INFO(this->get_logger(), "[ICP Calibration] Received initial cloud from LiDAR %zu.", i);
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Wait a bit before checking again
  }

  RCLCPP_INFO(this->get_logger(), "[ICP Calibration] All initial point clouds collected. Proceeding with ICP.");

  // 2. Perform ICP for each source LiDAR against the first LiDAR (target)
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud = initial_cpu_clouds[0];
  // The transform of the target LiDAR (lidar_handlers_[0]) relative to base_link
  Eigen::Matrix4f base_to_target_tf = lidar_handlers_[0]->getTransform();

  for (size_t i = 1; i < lidar_handlers_.size(); ++i)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud = initial_cpu_clouds[i];
    std::shared_ptr<GPULidarHandler> source_handler = lidar_handlers_[i];

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
      // This is: (base_link -> target_lidar_link) * (target_lidar_link -> source_lidar_link)
      Eigen::Matrix4f calibrated_source_to_base_tf = base_to_target_tf * final_source_to_target_tf;

      source_handler->setTransform(calibrated_source_to_base_tf);
      RCLCPP_INFO(this->get_logger(), "[ICP Calibration] LiDAR %zu calibrated successfully. Fitness score: %f", i, icp.getFitnessScore());
      // Optionally print the new transform matrix
      // RCLCPP_INFO(this->get_logger(), "New transform for LiDAR %zu:\n%s", i, Eigen::Matrix4f(calibrated_source_to_base_tf).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ";\n", "[", "]", "", "")).c_str());
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
    RCLCPP_DEBUG(this->get_logger(), "Neither 3D PCD nor FlatScan publishing enabled. Skipping all processing.");
    return;
  }

  std::vector<CudaPointXYZI*> d_input_clouds;
  std::vector<size_t> h_input_counts;
  std::vector<size_t> h_prefix_sums; // New: prefix sums
  std::vector<CudaMatrix4f> h_transforms;
  size_t total_points_to_merge = 0;

  for (const auto& handler : lidar_handlers_)
  {
    auto gpu_cloud_data = handler->getGPUPointCloud();
    if (gpu_cloud_data && gpu_cloud_data->d_points_ptr && gpu_cloud_data->num_points > 0)
    {
      d_input_clouds.push_back(gpu_cloud_data->d_points_ptr.get());
      h_input_counts.push_back(gpu_cloud_data->num_points);
      total_points_to_merge += gpu_cloud_data->num_points;

      CudaMatrix4f cuda_transform;
      cuda_transform.fromEigen(handler->getTransform().data());
      h_transforms.push_back(cuda_transform);
    }
  }

  if (total_points_to_merge == 0)
  {
    RCLCPP_WARN(this->get_logger(), "No points to merge. Not publishing.");
    return;
  }

  // Calculate prefix sums
  h_prefix_sums.resize(h_input_counts.size());
  if (!h_input_counts.empty())
  {
    h_prefix_sums[0] = h_input_counts[0];
    for (size_t i = 1; i < h_input_counts.size(); ++i)
    {
      h_prefix_sums[i] = h_prefix_sums[i-1] + h_input_counts[i];
    }
  }

  CudaPointXYZI* d_merged_cloud = nullptr;
  cudaError_t err = cudaMalloc((void**)&d_merged_cloud, total_points_to_merge * sizeof(CudaPointXYZI));
  if (err != cudaSuccess)
  {
      RCLCPP_ERROR(this->get_logger(), "cudaMalloc for merged cloud failed: %s", cudaGetErrorString(err));
      return;
  }

  // Launch CUDA kernel for transform and merge
  err = transformAndMergeGPU(d_input_clouds, h_input_counts, h_prefix_sums, h_transforms, d_merged_cloud, total_points_to_merge);
  if (err != cudaSuccess)
  {
      RCLCPP_ERROR(this->get_logger(), "transformAndMergeGPU kernel launch failed: %s", cudaGetErrorString(err));
      cudaFree(d_merged_cloud);
      return;
  }

  CudaPointXYZI* d_roi_filtered_cloud = nullptr;
  size_t num_roi_filtered_points = 0;

  // 1. Apply ROI Filters (multiple positive/negative with priority) on GPU
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
    cudaFree(d_merged_cloud); // Free merged cloud after filtering
  }
  else
  {
    d_roi_filtered_cloud = d_merged_cloud; // No ROI filter, pass through
    num_roi_filtered_points = total_points_to_merge;
  }

  CudaPointXYZI* d_voxel_filtered_cloud = nullptr;
  size_t num_voxel_filtered_points = 0;

  // 2. Apply Voxel Grid Filter on GPU
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
    cudaFree(d_roi_filtered_cloud); // Free ROI filtered cloud after voxel filtering
  }
  else
  {
    d_voxel_filtered_cloud = d_roi_filtered_cloud; // No Voxel filter, pass through
    num_voxel_filtered_points = num_roi_filtered_points;
  }

  if (num_voxel_filtered_points == 0)
  {
    RCLCPP_WARN(this->get_logger(), "Final filtered cloud is empty. Not publishing.");
    if (d_voxel_filtered_cloud) cudaFree(d_voxel_filtered_cloud);
    return;
  }

  // --- Conditional Publishing --- 

  // 1. Publish 3D PointCloud2 (if enabled)
  if (publish_3d_pcd_)
  {
    // Copy final GPU cloud back to CPU for ROS publishing
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
        output_msg.header.stamp = this->get_clock()->now();
        output_msg.header.frame_id = base_frame_id_;
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

  // 2. Publish FlatScan (LaserScan) (if enabled)
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
        // Continue without publishing flatscan
    }
    else
    {
        sensor_msgs::msg::LaserScan scan_msg;
        scan_msg.header.stamp = this->get_clock()->now();
        scan_msg.header.frame_id = base_frame_id_;
        scan_msg.angle_min = flatscan_params.angle_min;
        scan_msg.angle_max = flatscan_params.angle_max;
        scan_msg.angle_increment = flatscan_params.angle_increment;
        scan_msg.range_min = flatscan_params.range_min;
        scan_msg.range_max = flatscan_params.range_max;
        scan_msg.ranges.resize(flatscan_params.num_beams);

        // Copy ranges from GPU to CPU
        err = cudaMemcpy(scan_msg.ranges.data(), d_ranges, flatscan_params.num_beams * sizeof(float), cudaMemcpyDeviceToHost);
        if (err != cudaSuccess)
        {
            RCLCPP_ERROR(this->get_logger(), "cudaMemcpy (GPU to CPU for LaserScan) failed: %s", cudaGetErrorString(err));
        }
        else
        {
            flatscan_pub_->publish(scan_msg);
        }
        cudaFree(d_ranges);
    }
  }

  // Free the final GPU cloud after all potential uses
  if (d_voxel_filtered_cloud) cudaFree(d_voxel_filtered_cloud);
}

void MultiLidarNode::printCurrentParameters()
{
  RCLCPP_INFO(this->get_logger(), "--- Current ROS 2 Parameters ---");

  // 1. 노드에 선언된 모든 파라미터의 '이름' 목록을 가져옵니다.
  // 첫 번째 인자: 접두사 목록 (비어 있으면 모든 최상위 파라미터를 의미)
  // 두 번째 인자: 탐색 깊이 (0은 모든 깊이를 의미)
  auto parameter_names = this->list_parameters({}, 0).names;

  if (parameter_names.empty())
  {
    RCLCPP_INFO(this->get_logger(), "  No parameters found for this node.");
  }
  else
  {
    // 2. 이름 목록을 사용하여 실제 파라미터 객체(값 포함)들을 가져옵니다.
    auto parameters = this->get_parameters(parameter_names);

    // 3. 각 파라미터를 순회하며 이름과 값을 출력합니다.
    for (const auto &param : parameters)
    {
      RCLCPP_INFO(this->get_logger(), "  %s: %s",
        param.get_name().c_str(),
        param.value_to_string().c_str()); // 이 방식에서는 .value_to_string()이 잘 동작합니다.
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
        std::string index_str = name.substr(prefix.length(), dot_pos - prefix.length());
        try
        {
          size_t lidar_index = std::stoul(index_str);
          if (lidar_index < lidar_handlers_.size())
          {
            std::string tf_prefix = prefix + index_str + ".tf.";
            if (name.rfind(tf_prefix, 0) == 0)
            {
              double x = this->get_parameter(tf_prefix + "x").as_double();
              double y = this->get_parameter(tf_prefix + "y").as_double();
              double z = this->get_parameter(tf_prefix + "z").as_double();
              double roll = this->get_parameter(tf_prefix + "roll").as_double();
              double pitch = this->get_parameter(tf_prefix + "pitch").as_double();
              double yaw = this->get_parameter(tf_prefix + "yaw").as_double();

              if (name == tf_prefix + "x") x = parameter.as_double();
              if (name == tf_prefix + "y") y = parameter.as_double();
              if (name == tf_prefix + "z") z = parameter.as_double();
              if (name == tf_prefix + "roll") roll = parameter.as_double();
              if (name == tf_prefix + "pitch") pitch = parameter.as_double();
              if (name == tf_prefix + "yaw") yaw = parameter.as_double();

              Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
              Eigen::Translation3f translation(x, y, z);
              Eigen::AngleAxisf rot_x(roll, Eigen::Vector3f::UnitX());
              Eigen::AngleAxisf rot_y(pitch, Eigen::Vector3f::UnitY());
              Eigen::AngleAxisf rot_z(yaw, Eigen::Vector3f::UnitZ());
              transform = (translation * rot_z * rot_y * rot_x).matrix();

              lidar_handlers_[lidar_index]->setTransform(transform);
              RCLCPP_INFO(this->get_logger(), "Updated TF for lidar %zu", lidar_index);
            }
          }
        }
        catch (const std::exception &e)
        {
          RCLCPP_WARN(this->get_logger(), "Could not parse lidar index from parameter name: %s", name.c_str());
        }
      }
    }
  }

  return result;
}

namespace
{
  // Helper function to combine hashes
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
  for (size_t i = 0; i < lidar_frame_ids_.size(); ++i)
  {
    if (lidar_frame_ids_[i].empty())
    {
      continue;
    }

    try
    {
      geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
        base_frame_id_, lidar_frame_ids_[i], tf2::TimePointZero);

      size_t new_hash = hash_transform(transform_stamped.transform);

      if (new_hash != last_tf_hashes_[i])
      {
        last_tf_hashes_[i] = new_hash;

        // Convert transform to roll, pitch, yaw and x, y, z
        tf2::Quaternion q(transform_stamped.transform.rotation.x,
                          transform_stamped.transform.rotation.y,
                          transform_stamped.transform.rotation.z,
                          transform_stamped.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        double x = transform_stamped.transform.translation.x;
        double y = transform_stamped.transform.translation.y;
        double z = transform_stamped.transform.translation.z;

        // Set parameters
        std::string tf_prefix = "lidars." + std::to_string(i) + ".tf.";
        this->set_parameters({
          rclcpp::Parameter(tf_prefix + "x", x),
          rclcpp::Parameter(tf_prefix + "y", y),
          rclcpp::Parameter(tf_prefix + "z", z),
          rclcpp::Parameter(tf_prefix + "roll", roll),
          rclcpp::Parameter(tf_prefix + "pitch", pitch),
          rclcpp::Parameter(tf_prefix + "yaw", yaw)
        });
        RCLCPP_INFO(this->get_logger(), "Updated parameters for lidar %zu from TF.", i);
      }
    }
    catch (const tf2::TransformException &ex)
    {
      //RCLCPP_WARN(this->get_logger(), "Could not get transform for %s: %s", lidar_frame_ids_[i].c_str(), ex.what());
    }
  }
}
