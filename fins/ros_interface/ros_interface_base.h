#ifndef ROS_INTERFACE_BASE_H
#define ROS_INTERFACE_BASE_H

#include <string>
#include <vector>
#include <memory>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <atomic>

#include "common_lib.h"
#include "sensors/preprocess.h"
#include "sensors/IMU_Processing.h"
#include "posebuffer.h"
#include <reloc.h>

struct OdomData {
    double x = 0, y = 0, z = 0;
    double qx = 0, qy = 0, qz = 0, qw = 1;
    double timestamp = 0;
    double covariance[36] = {};
};

struct PoseData {
    double x = 0, y = 0, z = 0;
    double qx = 0, qy = 0, qz = 0, qw = 1;
    double timestamp = 0;
};

#ifdef USE_ROS1
  #include <sensor_msgs/Imu.h>
  using ImuConstPtr = sensor_msgs::Imu::ConstPtr;
#elif defined(USE_ROS2)
  #include <sensor_msgs/msg/imu.hpp>
  using ImuConstPtr = sensor_msgs::msg::Imu::ConstPtr;
#endif

class ROSInterfaceBase
{
public:
  virtual ~ROSInterfaceBase() = default;

  virtual void load_config()      = 0;
  virtual void register_pub_sub() = 0;
  virtual void spin_once()        = 0;
  virtual bool ros_ok()           = 0;

  virtual void publish_odometry(const OdomData& odom, double lidar_end_time) = 0;
  virtual void publish_path(const PoseData& pose, double lidar_end_time) = 0;
  virtual void publish_world_cloud(const PointCloudXYZI::Ptr& cloud, double lidar_end_time,
                                   const std::string& frame_id) = 0;
  virtual void publish_body_cloud(const PointCloudXYZI::Ptr& cloud, double lidar_end_time,
                                  const std::string& frame_id) = 0;
  virtual void publish_effect_cloud(const PointCloudXYZI::Ptr& cloud, double lidar_end_time,
                                    const std::string& frame_id) = 0;
  virtual void publish_map_cloud(const PointCloudXYZI::Ptr& cloud, double lidar_end_time,
                                 const std::string& frame_id) = 0;
  virtual void publish_odometryhighfreq(const Pose& pose) = 0;

  std::mutex              mtx_buffer;
  std::condition_variable sig_buffer;
  std::deque<double>               time_buffer;
  std::deque<PointCloudXYZI::Ptr>  lidar_buffer;
  std::deque<ImuConstPtr>   imu_buffer;  
  double  last_timestamp_lidar = 0.0;
  double  last_timestamp_imu   = -1.0;
  int     scan_count           = 0;
  int     publish_count        = 0;

  std::mutex         mtx_reloc;
  RelocState         reloc_state;
  std::atomic<bool>  relocalize_flag{false};

  bool   sam_enable           = false;
  bool   path_en              = true;
  bool   scan_pub_en          = false;
  bool   dense_pub_en         = false;
  bool   scan_body_pub_en     = false;
  bool   feature_pub_en       = false;
  bool   effect_pub_en        = false;
  bool   reloc_en             = false;
  bool   time_sync_en         = false;
  bool   runtime_pos_log      = false;
  bool   extrinsic_est_en     = true;
  bool   pcd_save_en          = false;

  int    NUM_MAX_ITERATIONS   = 4;
  int    pcd_save_interval    = -1;
  int    odom_imu_frequency   = 100;
  int    lidar_type           = AVIA;

  float  DET_RANGE            = 300.f;

  double time_diff_lidar_to_imu   = 0.0;
  double filter_size_corner_min   = 0.5;
  double filter_size_surf_min     = 0.5;
  double filter_size_map_min      = 0.5;
  double cube_len                 = 200.0;
  double fov_deg                  = 180.0;
  double gyr_cov                  = 0.1;
  double acc_cov                  = 0.1;
  double b_gyr_cov                = 0.0001;
  double b_acc_cov                = 0.0001;

  std::string map_file_path;
  std::string lid_topic       = "/livox/lidar";
  std::string imu_topic       = "/livox/imu";
  std::string reloc_topic     = "/reloc/manual";

  std::vector<double> extrinT;
  std::vector<double> extrinR;

  std::shared_ptr<Preprocess> p_pre = std::make_shared<Preprocess>();
  std::shared_ptr<ImuProcess> p_imu = std::make_shared<ImuProcess>();
};

#endif // ROS_INTERFACE_BASE_H