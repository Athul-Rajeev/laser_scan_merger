# Laser Scan Merger

The `laser_scan_merger` is a ROS package designed to merge multiple laser scan topics into a single scan topic, providing a unified representation of the environment. This is useful for robots equipped with multiple laser sensors or for combining overlapping scans in simulation or real-world scenarios.

---

## Features

- Combines multiple laser scan topics into a single `sensor_msgs/LaserScan` topic.
- Configurable parameters to adjust the scan ranges, angles, and frame transformations.
- Compatible with both real and simulated robots in ROS Noetic.

---

## Prerequisites

Ensure the following dependencies are installed in your ROS workspace:

- ROS Noetic
- `sensor_msgs`
- `tf2`
- `tf2_ros`
- `tf2_sensor_msgs`

---

## Installation

1. Clone this repository into your catkin workspace:
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/Athul-Rajeev/laser_scan_merger.git
   ```

2. Navigate back to your workspace and build the package:
   ```bash
   cd ~/catkin_ws
   catkin build laser_scan_merger
   source devel/setup.bash
   ```

---

## Usage

1. **Launch the Node:**
   ```bash
   roslaunch laser_scan_merger laser_scan_merger.launch
   ```

2. **Parameters:**
   The following parameters can be set in the `laser_scan_merger.launch` file or through the parameter server:
   - `input_scans`: List of input laser scan topics to merge.
   - `output_scan`: Topic name for the merged scan output.
   - `target_frame`: The frame into which all scans will be transformed.
   - `range_min` and `range_max`: Minimum and maximum range limits for the merged scan.
   - `angle_min` and `angle_max`: Angular bounds for the output scan.

3. **Dynamic Reconfigure:**
   Use the `dynamic_reconfigure` interface to modify parameters at runtime:
   ```bash
   rosrun rqt_reconfigure rqt_reconfigure
   ```

---
</launch>
```

---
