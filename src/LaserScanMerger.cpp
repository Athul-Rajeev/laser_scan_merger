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
    m_nh.param<std::string>("frame_id", m_frameId, "base_link");
    m_nh.param<std::string>("scan_topic1", m_scanTopic1, "/scan1");
    m_nh.param<std::string>("scan_topic2", m_scanTopic2, "/scan2");
    m_nh.param<double>("range_limit", m_rangeLimit, 10.0);
    m_nh.param<double>("angle_min", m_angleMin, -M_PI);
    m_nh.param<double>("angle_max", m_angleMax, M_PI);

    // Initialize subscribers
    // m_scanSub1.subscribe(m_nh, m_scanTopic1, 1);
    // m_scanSub2.subscribe(m_nh, m_scanTopic2, 1);

    m_scanSub1 = m_nh.subscribe(m_scanTopic1, 1, &LaserScanMerger::scanCallback1, this);
    m_scanSub2 = m_nh.subscribe(m_scanTopic2, 1, &LaserScanMerger::scanCallback2, this);

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

void LaserScanMerger::scanCallback1(const sensor_msgs::LaserScan::ConstPtr& scanMsg1) {
    std::lock_guard<std::mutex> lock(dataMutex);

    // Store the latest scan from topic1
    lastScan1 = scanMsg1;

    // Try to synchronize with the latest scan from topic2
    if (lastScan2) {
        double timeDiff = std::abs((lastScan1->header.stamp - lastScan2->header.stamp).toSec());
        const double syncThreshold = 0.1; // Adjust threshold as needed

        if (timeDiff <= syncThreshold) {
            // Messages are synchronized; process them
            processLaserScan(lastScan1);
            processLaserScan(lastScan2);

            // Reset both messages after processing
            lastScan1.reset();
            lastScan2.reset();
        }
    }
}

void LaserScanMerger::scanCallback2(const sensor_msgs::LaserScan::ConstPtr& scanMsg2) {
    std::lock_guard<std::mutex> lock(dataMutex);

    // Store the latest scan from topic2
    lastScan2 = scanMsg2;

    // Try to synchronize with the latest scan from topic1
    if (lastScan1) {
        double timeDiff = std::abs((lastScan1->header.stamp - lastScan2->header.stamp).toSec());
        const double syncThreshold = 0.1; // Adjust threshold as needed

        if (timeDiff <= syncThreshold) {
            // Messages are synchronized; process them
            processLaserScan(lastScan1);
            processLaserScan(lastScan2);

            // Reset both messages after processing
            lastScan1.reset();
            lastScan2.reset();
        }
    }
}

/**        // Prepare a PCL PointCloud directly
 * @brief Processes a LaserScan message, transforms it to the target frame, and merges it into a combined cloud.
 * @param scan_msg LaserScan message to process.
 * @return None
 */

void LaserScanMerger::processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scanMsg) {
    // Prepare a point cloud
    pcl::PointCloud<pcl::PointXYZI> pclCloud;
    pclCloud.header.frame_id = scanMsg->header.frame_id;
    pclCloud.header.stamp = pcl_conversions::toPCL(scanMsg->header.stamp);
    pclCloud.is_dense = false;

    // Reserve space for points (reduces dynamic resizing during population)
    pclCloud.points.reserve(scanMsg->ranges.size());

    // Precompute angle values for each index to avoid repetitive calculations
    std::vector<float> precomputed_angles(scanMsg->ranges.size());
    for (size_t i = 0; i < scanMsg->ranges.size(); ++i) {
        precomputed_angles[i] = scanMsg->angle_min + i * scanMsg->angle_increment;
    }

    // Populate the PCL PointCloud
    for (size_t i = 0; i < scanMsg->ranges.size(); ++i) {
        float range = scanMsg->ranges[i];
        float intensity = (i < scanMsg->intensities.size()) ? scanMsg->intensities[i] : 0.0f;

        // Skip invalid ranges early to avoid unnecessary computation
        if (range < scanMsg->range_min || range > scanMsg->range_max) {
            continue;
        }

        // Compute Cartesian coordinates
        float angle = precomputed_angles[i];
        float x = range * cos(angle);
        float y = range * sin(angle);

        // Add valid points to the cloud
        pcl::PointXYZI point;
        point.x = x;
        point.y = y;
        point.z = 0.0;  // LaserScan is 2D
        point.intensity = intensity;

        pclCloud.points.push_back(point);
    }

    // Transform the cloud to the desired frame
        geometry_msgs::TransformStamped transform =
            m_tfBuffer.lookupTransform(m_frameId, scanMsg->header.frame_id, scanMsg->header.stamp);
        
        // Use Eigen for transformation matrix
        Eigen::Matrix4f transformMatrix;
        pcl_ros::transformAsMatrix(transform.transform, transformMatrix);

        // Apply transformation
        pcl::PointCloud<pcl::PointXYZI> transformedCloud;
        pcl::transformPointCloud(pclCloud, transformedCloud, transformMatrix);

        // Merge with the combined cloud
        m_combinedCloud += transformedCloud;
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
    outputCloud.header.stamp = cloud.header.stamp;
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
    const pcl::PointCloud<pcl::PointXYZI>& cloud, sensor_msgs::LaserScan& scan) {

    // Initialize LaserScan parameters
    scan.header.frame_id = m_frameId;
    scan.header.stamp = ros::Time(cloud.header.stamp);
    scan.angle_min = m_angleMin;
    scan.angle_max = m_angleMax;
    scan.angle_increment = (m_angleMax - m_angleMin) / static_cast<float>(cloud.width);
    scan.range_min = 0.1;  // Minimum valid range
    scan.range_max = m_rangeLimit;  // Maximum valid range

    // Pre-allocate memory for ranges
    uint32_t ranges_size = std::ceil((scan.angle_max - scan.angle_min) / scan.angle_increment);
    scan.ranges.assign(ranges_size, std::numeric_limits<float>::infinity());

    // Use Eigen for efficient matrix/vector operations
    Eigen::VectorXf x_vals(cloud.size()), y_vals(cloud.size());
    for (size_t i = 0; i < cloud.size(); ++i) {
        x_vals(i) = cloud[i].x;
        y_vals(i) = cloud[i].y;
    }

    // Vectorized range and angle computations
    Eigen::VectorXf ranges = (x_vals.array().square() + y_vals.array().square()).sqrt();
    Eigen::VectorXf angles = y_vals.array().binaryExpr(x_vals.array(), [](float y, float x) {
        return std::atan2(y, x);
    });

    for (int i = 0; i < angles.size(); ++i) {
        float range = ranges(i);
        float angle = angles(i);

        // Skip invalid points
        if (std::isnan(range) || range < scan.range_min || range > scan.range_max) {
            continue;
        }

        // Angle-based filtering
        if (angle < scan.angle_min || angle > scan.angle_max) {
            continue;
        }

        // Compute index based on angle
        int index = static_cast<int>((angle - scan.angle_min) / scan.angle_increment);

        // Update range if this point is closer
        if (index >= 0 && index < scan.ranges.size() && range < scan.ranges[index]) {
            scan.ranges[index] = range;
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
    ros::Rate rate(120);
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
