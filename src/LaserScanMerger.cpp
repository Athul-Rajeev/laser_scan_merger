/*
Name: LaserScanMerger
Author: Athul Rajeev
Date: 2024-12-07
Version: 1.0
Description: Implementation file for LaserScanMerger class.
*/

#include "LaserScanMerger.h"

/**
 * @brief Constructor to initialize parameters, subscribers, publishers, and dynamic reconfigure.
 * @param None
 * @return None
 */

LaserScanMerger::LaserScanMerger() 
    : m_nh("~"), m_tfListener(m_tfBuffer) {
    // Load parameters from YAML
    m_nh.param<std::string>("frame_id", m_frameId, "map");
    m_nh.param<std::string>("scan_topic1", m_scanTopic1, "/scan1");
    m_nh.param<std::string>("scan_topic2", m_scanTopic2, "/scan2");
    m_nh.param<double>("range_limit", m_rangeLimit, 10.0);
    m_nh.param<double>("angle_min", m_angleMin, -M_PI);
    m_nh.param<double>("angle_max", m_angleMax, M_PI);

    // Initialize subscribers and publishers
    m_scanSub1 = m_nh.subscribe(m_scanTopic1, 1, &LaserScanMerger::scanCallback, this);
    m_scanSub2 = m_nh.subscribe(m_scanTopic2, 1, &LaserScanMerger::scanCallback, this);
    m_mergedPointcloudPub = m_nh.advertise<sensor_msgs::PointCloud2>("merged_pointcloud", 1);
    m_mergedScanPub = m_nh.advertise<sensor_msgs::LaserScan>("merged_scan", 1);

    // Setup dynamic reconfigure
    dynamic_reconfigure::Server<laser_scan_merger::ScanMergerConfig>::CallbackType dynCb;
    dynCb = boost::bind(&LaserScanMerger::dynamicReconfigCallback, this, _1, _2);
    m_dynServer.setCallback(dynCb);
}

/**
 * @brief Callback to process LaserScan data from the first/second topic.
 * @param scan_msg LaserScan message received from the respective topic.
 * @return None
 */

void LaserScanMerger::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
    processLaserScan(scan_msg);
}

/**
 * @brief Processes a LaserScan message, transforms it to the target frame, and merges it into a combined cloud.
 * @param scan_msg LaserScan message to process.
 * @return None
 */

void LaserScanMerger::processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scanMsg) {
    try {
        sensor_msgs::PointCloud2 cloud;
        sensor_msgs::PointCloud2 tempCloud;
        m_projector.projectLaser(*scanMsg, tempCloud);

        geometry_msgs::TransformStamped transform = 
            m_tfBuffer.lookupTransform(m_frameId, scanMsg->header.frame_id, ros::Time(0));
        tf2::doTransform(tempCloud, cloud, transform);

        pcl::PointCloud<pcl::PointXYZ> pclCloud;
        pcl::fromROSMsg(cloud, pclCloud);

        m_combinedCloud += pclCloud;
    } catch (tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(1.0, "Transform error: %s", ex.what());
    }
}

/**
 * @brief Publishes the merged point cloud and laser scan to respective topics.
 * @param None
 * @return None
 */

void LaserScanMerger::publishMergedData() {
    if (m_combinedCloud.empty()) return;

    sensor_msgs::PointCloud2 outputCloud;
    pcl::toROSMsg(m_combinedCloud, outputCloud);
    outputCloud.header.frame_id = m_frameId;
    outputCloud.header.stamp = ros::Time::now();
    m_mergedPointcloudPub.publish(outputCloud);

    sensor_msgs::LaserScan mergedScan;
    convertPointCloudToLaserScan(m_combinedCloud, mergedScan);
    m_mergedScanPub.publish(mergedScan);

    m_combinedCloud.clear();
}

/**
 * @brief Converts a point cloud into a LaserScan message and fills the ranges based on scan angles.
 * @param cloud Input point cloud to convert.
 * @param scan Output LaserScan message.
 * @return None
 */

void LaserScanMerger::convertPointCloudToLaserScan(
    const pcl::PointCloud<pcl::PointXYZ>& cloud, sensor_msgs::LaserScan& scan) {
    scan.header.frame_id = m_frameId;
    scan.header.stamp = ros::Time::now();
    scan.angle_min = m_angleMin;
    scan.angle_max = m_angleMax;
    scan.angle_increment = (m_angleMax - m_angleMin) / 360.0;
    scan.range_min = 0.1;
    scan.range_max = m_rangeLimit;
    scan.ranges.resize(360, std::numeric_limits<float>::infinity());

    for (const auto& point : cloud) {
        double angle = atan2(point.y, point.x);
        if (angle < m_angleMin || angle > m_angleMax) continue;

        double range = hypot(point.x, point.y);
        if (range < scan.range_min || range > scan.range_max) continue;

        int index = static_cast<int>((angle - m_angleMin) / scan.angle_increment);
        if (index >= 0 && index < 360) {
            scan.ranges[index] = std::min(scan.ranges[index], static_cast<float>(range));
        }
    }
}

/**
 * @brief Handles dynamic reconfigure updates for the node's parameters.
 * @param config Configuration values from dynamic reconfigure.
 * @param level Dynamic reconfigure level (unused).
 * @return None
 */

void LaserScanMerger::dynamicReconfigCallback(laser_scan_merger::ScanMergerConfig& config, uint32_t level) {
    m_frameId = config.frame_id;
    m_rangeLimit = config.range_limit;
    m_angleMin = config.angle_min;
    m_angleMax = config.angle_max;
}

/**
 * @brief Main loop to process and publish data at a fixed rate.
 * @param None
 * @return None
 */

void LaserScanMerger::spin() {
    ros::Rate rate(10);
    while (ros::ok()) {
        publishMergedData();
        ros::spinOnce();
        rate.sleep();
    }
}

/**
 * @brief Main entry point of the node.
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return None
 */

int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_scan_merger");
    LaserScanMerger merger;
    merger.spin();
    return 0;
}
