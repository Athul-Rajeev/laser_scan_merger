```markdown
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

To install missing dependencies:
```bash
sudo apt-get update
sudo apt-get install ros-noetic-tf2-sensor-msgs
```

---

## Installation

1. Clone this repository into your catkin workspace:
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/your_username/laser_scan_merger.git
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

## Example Configuration

Here's an example configuration for the `laser_scan_merger.launch` file:

```xml
<launch>
    <node pkg="laser_scan_merger" type="laser_scan_merger_node" name="laser_scan_merger">
        <param name="input_scans" value="['/scan1', '/scan2']" />
        <param name="output_scan" value="/merged_scan" />
        <param name="target_frame" value="base_link" />
        <param name="range_min" value="0.1" />
        <param name="range_max" value="10.0" />
        <param name="angle_min" value="-1.57" />
        <param name="angle_max" value="1.57" />
    </node>
</launch>
```

---
