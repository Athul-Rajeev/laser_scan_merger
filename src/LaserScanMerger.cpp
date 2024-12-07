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

    // Initialize subscribers
    m_scanSub1.subscribe(m_nh, m_scanTopic1, 1);
    m_scanSub2.subscribe(m_nh, m_scanTopic2, 1);

    // Synchronize the messages
    m_sync = new message_filters::TimeSynchronizer<sensor_msgs::LaserScan, sensor_msgs::LaserScan>(m_scanSub1, m_scanSub2, 10);
    m_sync->registerCallback(boost::bind(&LaserScanMerger::scanCallback, this, _1, _2));

    // Initialize publishers
    m_mergedPointcloudPub = m_nh.advertise<sensor_msgs::PointCloud2>("merged_pointcloud", 1);
    m_mergedScanPub = m_nh.advertise<sensor_msgs::LaserScan>("merged_scan", 1);

    // Setup dynamic reconfigure
    dynamic_reconfigure::Server<laser_scan_merger::ScanMergerConfig>::CallbackType dynCb;
    dynCb = boost::bind(&LaserScanMerger::dynamicReconfigCallback, this, _1, _2);
    m_dynServer.setCallback(dynCb);
}

/**
 * @brief Callback to process LaserScan data from the synchronized topics.
 * @param scanMsg1 LaserScan message from the first topic.
 * @param scanMsg2 LaserScan message from the second topic.
 * @return None
 */

void LaserScanMerger::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scanMsg1, const sensor_msgs::LaserScan::ConstPtr& scanMsg2) {
    // Process both LaserScan messages
    processLaserScan(scanMsg1);
    processLaserScan(scanMsg2);
}

/**
 * @brief Processes a LaserScan message, transforms it to the target frame, and merges it into a combined cloud.
 * @param scan_msg LaserScan message to process.
 * @return None
 */

void LaserScanMerger::processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scanMsg) {
    try {
        cloud.header.frame_id = scanMsg->header.frame_id;
        cloud.header.stamp = scanMsg->header.stamp;
        
        // Prepare PointCloud2 structure
        cloud.height = 1;
        cloud.is_dense = true;
        cloud.is_bigendian = false;

        // Define PointCloud2 fields
        cloud.fields.resize(4); // Add 1 more field for intensity

        cloud.fields[0].name = "x";
        cloud.fields[0].offset = 0;
        cloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
        cloud.fields[0].count = 1;

        cloud.fields[1].name = "y";
        cloud.fields[1].offset = 4;
        cloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
        cloud.fields[1].count = 1;

        cloud.fields[2].name = "z";
        cloud.fields[2].offset = 8;
        cloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
        cloud.fields[2].count = 1;

        cloud.fields[3].name = "intensity";
        cloud.fields[3].offset = 12;
        cloud.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
        cloud.fields[3].count = 1;

        cloud.point_step = 16;  // 4 bytes for x, 4 for y, 4 for z, 4 for intensity (3 * sizeof(float) + 1 * sizeof(float))
        cloud.row_step = cloud.point_step * scanMsg->ranges.size();
        cloud.width = scanMsg->ranges.size();
        cloud.height = 1;
        cloud.data.resize(cloud.row_step);

        // Populate PointCloud2 with data from LaserScan
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud, "intensity"); // New iterator for intensity

        for (size_t i = 0; i < scanMsg->ranges.size(); ++i) {
        float range = scanMsg->ranges[i];
        if (range >= scanMsg->range_min && range <= scanMsg->range_max) {
            float angle = scanMsg->angle_min + i * scanMsg->angle_increment;
            
            // Calculate the Cartesian coordinates
            *iter_x = range * cos(angle);
            *iter_y = range * sin(angle);
            *iter_z = 0.0;  // LaserScan is 2D
            
            // Add the intensity (it comes directly from the scanMsg)
            *iter_intensity = scanMsg->intensities[i];
        } else {
            // If the range is invalid, set x, y, z, and intensity to NaN
            *iter_x = std::numeric_limits<float>::quiet_NaN();
            *iter_y = std::numeric_limits<float>::quiet_NaN();
            *iter_z = std::numeric_limits<float>::quiet_NaN();
            *iter_intensity = std::numeric_limits<float>::quiet_NaN();
        }

        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_intensity;  // Increment the intensity iterator
        }

        // Transform to desired frame
        sensor_msgs::PointCloud2 transformedCloud;
        geometry_msgs::TransformStamped transform = 
            m_tfBuffer.lookupTransform(m_frameId, scanMsg->header.frame_id, ros::Time(0));
        tf2::doTransform(cloud, transformedCloud, transform);

        // Convert to PCL for further processing
        pcl::PointCloud<pcl::PointXYZ> pclCloud;
        pcl::fromROSMsg(transformedCloud, pclCloud);

        // Merge with the combined cloud
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
