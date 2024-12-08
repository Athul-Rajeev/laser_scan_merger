/*
Name: LaserScanMerger
Author: Athul Rajeev
Date: 2024-12-07
Version: 1.0
Description: Merges two LaserScan topics into a combined LaserScan and PointCloud2 output.
*/

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <dynamic_reconfigure/server.h>
#include <laser_scan_merger/ScanMergerConfig.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


class LaserScanMerger {
public:
    LaserScanMerger();
    void spin();

private:
    // ROS node handle, subscribers, publishers
    ros::NodeHandle m_nh;
    ros::Publisher m_mergedPointcloudPub;
    ros::Publisher m_mergedScanPub;

    // Laser Projection and TF
    tf2_ros::Buffer m_tfBuffer;
    tf2_ros::TransformListener m_tfListener;
    sensor_msgs::PointCloud2 cloud;

    // Dynamic Reconfigure Server
    dynamic_reconfigure::Server<laser_scan_merger::ScanMergerConfig> m_dynServer;

    // Parameters
    std::string m_frameId;
    std::string m_scanTopic1;
    std::string m_scanTopic2;
    double m_rangeLimit;
    double m_angleMin;
    double m_angleMax;

    // Combined Point Cloud
    pcl::PointCloud<pcl::PointXYZ> m_combinedCloud;

    // Message Filter Synchronizer
    message_filters::Subscriber<sensor_msgs::LaserScan> m_scanSub1;
    message_filters::Subscriber<sensor_msgs::LaserScan> m_scanSub2;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> *m_sync;

    // Callbacks
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scanMsg1, const sensor_msgs::LaserScan::ConstPtr& scanMsg2);
    void dynamicReconfigCallback(laser_scan_merger::ScanMergerConfig& config, uint32_t level);

    // Helper functions
    void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scanMsg);
    void publishMergedData();
    void convertPointCloudToLaserScan(const pcl::PointCloud<pcl::PointXYZ>& cloud, sensor_msgs::LaserScan& scan);
};

