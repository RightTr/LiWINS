#pragma once

#include <deque>
#include <vector>

#include <Eigen/Eigen>

#include "common_lib.h"

enum WHEEL_TYPE{Wheel2DAng = 0, Wheel2DLin, Wheel2DCen, Wheel3DAng, Wheel3DLin, Wheel3DCen};

struct WheelPreintegration
{
  double start_time = -1.0;
  double end_time = -1.0;
  double th_2D = 0.0;
  double x_2D = 0.0;
  double y_2D = 0.0;
  M3D R_3D = Eye3d;
  V3D p_3D = Zero3d;
  Eigen::Matrix3d Cov_2D = Eigen::Matrix3d::Zero();
  Eigen::Matrix<double, 6, 6> Cov_3D = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 1, 3> dth_di_2D = Eigen::Matrix<double, 1, 3>::Zero();
  Eigen::Matrix<double, 1, 3> dx_di_2D = Eigen::Matrix<double, 1, 3>::Zero();
  Eigen::Matrix<double, 1, 3> dy_di_2D = Eigen::Matrix<double, 1, 3>::Zero();
  Eigen::Matrix3d dR_di_3D = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d dp_di_3D = Eigen::Matrix3d::Zero();
};

struct WheelPoseState
{
  M3D rot = Eye3d;
  V3D pos = Zero3d;
  M3D rot_fej = Eye3d;
  V3D pos_fej = Zero3d;
  V3D ang_vel = Zero3d;
  V3D lin_vel = Zero3d;
};

struct WheelLinearizationResult
{
  Eigen::MatrixXd H;
  Eigen::VectorXd res;
};

class WheelProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  WheelProcess();
  ~WheelProcess();

  void Reset();

  void set_intrinsic(double rl, double rr, double b);
  void set_noise(double noise_w, double noise_v, double noise_p);
  void set_history_time(double max_history_time);
  void enable_calib_ext(bool enable);
  void enable_calib_dt(bool enable);
  void enable_calib_int(bool enable);

  void feed_measurement(const WheelMsg &data);

  bool Process(double time0, double time1, WheelPreintegration *result = nullptr);
  bool select_wheel_data(double time0, double time1, std::vector<WheelMsg> &data_vec) const;

  bool ComputeLinearSystem(const WheelPoseState &pose0,
                           const WheelPoseState &pose1,
                           const M3D &extrinsic_rot,
                           const V3D &extrinsic_pos,
                           WheelLinearizationResult &result) const;

  const WheelPreintegration &latest_result() const { return preintegration_; }
  const std::vector<WheelMsg> &measurements() const { return data_stack_; }

  WHEEL_TYPE type = Wheel2DAng;
  double rl = 1.0;
  double rr = 1.0;
  double b = 1.0;
  std::deque<double> t_hist;

 private:
  static WheelMsg interpolate_data(const WheelMsg &data1, const WheelMsg &data2, double timestamp);
  void preintegration_intrinsics_2D(double dt, const WheelMsg &data);
  void preintegration_intrinsics_3D(double dt, const WheelMsg &data);
  void preintegration_2D(double dt, const WheelMsg &data1, const WheelMsg &data2);
  void preintegration_3D(double dt, const WheelMsg &data1, const WheelMsg &data2);
  void compute_linear_system_2D(const WheelPoseState &pose0,
                                const WheelPoseState &pose1,
                                const M3D &extrinsic_rot,
                                const V3D &extrinsic_pos,
                                WheelLinearizationResult &result) const;
  void compute_linear_system_3D(const WheelPoseState &pose0,
                                const WheelPoseState &pose1,
                                const M3D &extrinsic_rot,
                                const V3D &extrinsic_pos,
                                WheelLinearizationResult &result) const;

  bool do_calib_ext = false;
  bool do_calib_dt = false;
  bool do_calib_int = false;
  double noise_w = 0.02;
  double noise_v = 0.02;
  double noise_p = 0.02;
  double max_history_time = 100.0;
  std::vector<WheelMsg> data_stack_;
  WheelPreintegration preintegration_;
};
