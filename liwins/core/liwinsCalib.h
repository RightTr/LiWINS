#pragma once

#include <deque>
#include <memory>
#include <vector>

#include <pcl/filters/voxel_grid.h>

#include <gtsam/inference/Symbol.h>

#include "calib/lidar_imu_wheel_init.h"
#include "map/ikd-Tree/ikd_Tree.hpp"
#include "map/pointMapBase.hpp"
#include "ros_interface/ros_interface.h"
#include "utils/trans_utils.h"

class LIWINSCalib
{
 public:
  LIWINSCalib() = default;
  ~LIWINSCalib() = default;

  void init();
  void run();
  void set_exit() { flg_exit_ = true; }

  bool has_result() const { return !result_.values.empty(); }
  const InitGraphResult &result() const { return result_; }

 private:
  bool sync_packages(MeasureGroup &meas);
  void init_state();
  OdomData make_odom_data() const;
  PoseData make_pose_data() const;
  void build_lidarObs(std::vector<optimizeLidarObs> &observations);
  void append_keyframe(
      const std::vector<optimizeLidarObs> &observations,
      const std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> &imu_preintegration = nullptr);
  void publish_integrated_poses(double integration_beg_time);
  void map_incremental();
  void optimize();

  MeasureGroup Measures_;
  state_ikfom state_curr_;
  std::vector<Pose6D> imu_pose_traj_;

  PointCloudXYZI::Ptr feats_undistort_{new PointCloudXYZI()};
  PointCloudXYZI::Ptr feats_down_body_{new PointCloudXYZI()};
  PointCloudXYZI::Ptr feats_down_world_{new PointCloudXYZI()};

  PointMapBase<PointType>::Ptr point_map_{std::make_shared<KD_TREE<PointType>>()};
  pcl::VoxelGrid<PointType> downSizeFilterSurf_;
  std::vector<PointVector> nearest_points_;

  bool flg_first_scan_ = true;
  bool flg_exit_ = false;
  bool lidar_pushed_ = false;

  double lidar_end_time_ = 0.0;
  double first_lidar_time_ = 0.0;
  double lidar_mean_scantime_ = 0.0;
  double wheel_last_lidar_time_ = -1.0;
  double wheel_integrated_x_ = 0.0;
  double wheel_integrated_y_ = 0.0;
  int scan_num_ = 0;

  V3D Lidar_T_wrt_IMU_{Zero3d};
  M3D Lidar_R_wrt_IMU_{Eye3d};
  V3D Wheel_T_wrt_IMU_{Zero3d};
  M3D Wheel_R_wrt_IMU_{Eye3d};

  InitGraphConfig graph_config_;
  std::deque<LWIKeyframe> keyframes_;
  InitGraphResult result_;
  int window_size_ = 25;
  int optimize_every_n_ = 1;
  int frames_since_last_optimize_ = 0;
  int optimize_max_iterations_ = 30;
  double lidar_point_cov_ = 0.001;
};
