# FINS: Fused Inertial Navigation System

## ⏰ Wait to be updated ...

A LiDAR-Inertial SLAM system pipline that integrates **FAST-LIO2** as the high-frequency frontend, **LIO-SAM-style** factor graph backend for global optimization and a wrapper to manage the **map data structure**.

## 🧩 Contributions

* A SLAM system that integrates FAST-LIO2 with a LIO-SAM-style factor graph backend.
  
* A map wrapper to manage the map data structure 🗺️

* High-frequency odometry via IMU propagation between LiDAR scans 📍

* Manual initial pose setting for relocalization

* Support for extended LiDARs (e.g., RoboSense Airy, Unilidar L2)

* ROS2 adaptation

## 🛠️ Prerequisites

### Build

```bash
mkdir li_pipline_ws
cd li_pipline_ws

mkdir src && cd src
git clone https://github.com/RightTr/LI-pipline.git

#If you are using ROS2 Humble, please clone the repository below for message generation.
git clone https://github.com/RightTr/fast_lio_interfaces.git

# ROS1 build
cd src/LI-pipline
./build.sh ROS1

# ROS2 build
cd src/LI-pipline
./build.sh humble
```

## ✨ Features

### LIO-SAM-style backend

```bash
cd li_pipline_ws 
source devel/setup.bash
# e.g.
roslaunch fins sam_airy.launch
```

### Map wrapper

* ikd-Tree [[FAST-LIO2]](doc/Fast_LIO_2.pdf)
  
* OctVoxMap + HKNN Search [[Super-LIO]](https://github.com/Liansheng-Wang/Super-LIO.git)

### Relocalization

The modified system supports relocalization using manually set odometry poses. Once odometry poses are published to the */reloc_topic* (according to the following .yaml file), the system will reset the system and the initial pose according to your input.

Run relocalization mode:

```bash
# e.g.
# ROS1
cd fastlio_ws 
source devel/setup.bash
roslaunch fins reloc_mid360.launch
# Publish geometry_msgs::PoseStamped to the /reloc_topic

# ROS2
cd fastlio_ws 
source install/setup.bash
ros2 launch fins reloc_mid360.launch.py
# Publish geometry_msgs::msg::PoseStamped to the /reloc_topic
```

### High frequency odometry via IMU propagation between LiDAR scans 📍

Subscribe the topic */OdometryHighFreq* to receive high frequency odometry output via IMU propagation between LiDAR scans.

### Extended LiDAR support

Now, FAST-LIO supports tracking and mapping using the RoboSense LiDARs (e.g., RoboSense Airy) and the Unilidar LiDARs (e.g., Unilidar L2). Check the related files in ./config and ./launch folder.

```bash
# e.g.
roslaunch fins mapping_airy.launch
```

## 📝 TODO List

- [ ] ROS2 fully Adaption

## 📚 Related Works

[FAST-LIO2](doc/Fast_LIO_2.pdf), [FAST-LIO](https://arxiv.org/abs/2010.08196), [FAST-LIO official repository](https://github.com/hku-mars/FAST_LIO.git)

[LIO-SAM](https://github.com/TixiaoShan/LIO-SAM.git)

[FAST_LIO_SAM](https://github.com/kahowang/FAST_LIO_SAM.git)

[robosense_fast_lio](https://github.com/RuanJY/robosense_fast_lio.git)

[Super-LIO](https://github.com/Liansheng-Wang/Super-LIO.git)

[point_lio_unilidar](https://github.com/unitreerobotics/point_lio_unilidar.git)

[MINS](https://github.com/rpng/MINS.git)