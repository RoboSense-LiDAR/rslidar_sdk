
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

    // LiDAR Status
    RCLCPP_INFO(this->get_logger(), "LiDAR Status:");
    for (size_t i = 0; i < lidar_handlers_.size(); ++i)
    {
        auto last_cloud_time = lidar_handlers_[i]->getLastCloudTimestamp();
        // Check for a very old timestamp, indicating no cloud has ever been received
        if (last_cloud_time.seconds() == 0)
        {
            RCLCPP_INFO(this->get_logger(), "  - LiDAR %zu: Waiting for data...", i);
        }
        else
        {
            double time_since_last_cloud = (now - last_cloud_time).seconds();
            std::string status = (time_since_last_cloud < 2.0) ? "Connected" : "Disconnected"; // 2-second timeout
            RCLCPP_INFO(this->get_logger(), "  - LiDAR %zu: %s (Last cloud: %.2f s ago)", i, status.c_str(), time_since_last_cloud);
        }
    }

    // Published Topics
    RCLCPP_INFO(this->get_logger(), "Published Topics:");
    if (publish_3d_pcd_ && merged_pub_)
    {
        RCLCPP_INFO(this->get_logger(), "  - PointCloud2: '%s'", merged_pub_->get_topic_name());
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "  - PointCloud2: Disabled");
    }

    if (publish_flatscan_ && flatscan_pub_)
    {
        RCLCPP_INFO(this->get_logger(), "  - LaserScan: '%s'", flatscan_pub_->get_topic_name());
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "  - LaserScan: Disabled");
    }

    // Point Processing Stats for the last processed frame
    RCLCPP_INFO(this->get_logger(), "Point Processing (Last Frame):");
    RCLCPP_INFO(this->get_logger(), "  - Merged Raw: %zu points", last_total_points_merged_.load());
    RCLCPP_INFO(this->get_logger(), "  - After ROI Filter: %zu points", last_points_after_roi_.load());
    RCLCPP_INFO(this->get_logger(), "  - After Voxel Filter: %zu points", last_points_after_voxel_.load());
    RCLCPP_INFO(this->get_logger(), "---------------------------------------------------------");
}
