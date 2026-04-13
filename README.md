# LiWINS: LiDAR-Wheel-Inertial Navigation System

## ⏰ Wait to be updated ...

A **LiDAR-Wheel-Inertial** SLAM and calibration system based on IKFoM, integrating **LiDAR mapping**, **IMU propagation**, **wheel motion constraints** and **wheel extrinsic / scale offline calibration** with an optional **LIO-SAM-style backend**.

## 🧩 Contributions

* A LiDAR-Wheel-Inertial SLAM framework based on IKFoM, combining LiDAR mapping, IMU propagation, and wheel motion constraints.

* A wheel-aided calibration pipeline for estimating planar IMU-wheel extrinsics and wheel scale parameters.

* An optional LIO-SAM-style factor graph backend for joint optimization.

* A unified map wrapper supporting multiple map backends, including ikd-Tree and OctVoxMap. 🗺️

* High-frequency odometry output via IMU propagation between LiDAR scans. 📍

* Manual initial pose setting support for relocalization.

* Extended LiDAR support, including RoboSense Airy and Unilidar L2.

* ROS2 adaptation and launch support.

## 🛠️ Prerequisites

* [gtsam](https://gtsam.org/get_started/) (Georgia Tech Smoothing and Mapping library)

```bash
# Ubuntu 20.04
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
# Ubuntu 22.04
sudo add-apt-repository ppa:borglab/gtsam-release-4.1

sudo apt install libgtsam-dev libgtsam-unstable-dev
```

### Build

```bash
mkdir -p liwins_ws/src
cd liwins_ws/src

git clone https://github.com/RightTr/LiWINS.git
cd LiWINS

# ROS1 build
./build.sh ROS1

# ROS2 build
./build.sh humble

```

## ✨ Features

### 🗺️ Mapping mode

```bash
# For ROS2
ros2 launch liwins mapping.launch.py
```

This mode runs the standard LiDAR-inertial mapping frontend / backend pipeline.

### 🔗 LIO-SAM-style backend

```bash
# For ROS2
ros2 launch liwins sam.launch.py
```

This mode enables the optional LIO-SAM-style factor graph backend for joint optimization.

### ⚙️ Wheel calibration

```bash
# For ROS2
ros2 launch liwins calib.launch.py
```

This mode runs LiDAR-IMU-wheel calibration, including planar IMU-wheel extrinsic estimation and wheel scale estimation.

### 🧱 Map wrapper

* ikd-Tree [[FAST-LIO2]](doc/Fast_LIO_2.pdf)
* OctVoxMap + HKNN Search [[Super-LIO]](https://github.com/Liansheng-Wang/Super-LIO.git)

### 📍 Relocalization

The modified system supports relocalization using manually set odometry poses. Once odometry poses are published to the */reloc_topic* (according to the following .yaml file), the system will reset the system and the initial pose according to your input.

## 📚 Related Works

[FAST-LIO2](doc/Fast_LIO_2.pdf), [FAST-LIO](https://arxiv.org/abs/2010.08196), [FAST-LIO official repository](https://github.com/hku-mars/FAST_LIO.git)

[LIO-SAM](https://github.com/TixiaoShan/LIO-SAM.git)

[FAST_LIO_SAM](https://github.com/kahowang/FAST_LIO_SAM.git)

[robosense_fast_lio](https://github.com/RuanJY/robosense_fast_lio.git)

[Super-LIO](https://github.com/Liansheng-Wang/Super-LIO.git)

[point_lio_unilidar](https://github.com/unitreerobotics/point_lio_unilidar.git)

[MINS](https://github.com/rpng/MINS.git)
