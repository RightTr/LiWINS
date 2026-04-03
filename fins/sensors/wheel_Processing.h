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
};

struct WheelPoseState
{
  M3D rot = Eye3d;
  V3D pos = Zero3d;
  M3D rot_fej = Eye3d;
  V3D pos_fej = Zero3d;
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
  void set_extrinsic(const V3D &transl, const M3D &rot);
  void set_extrinsic(const V3D &transl);
  void set_extrinsic(const MD(4,4) &T);

  bool Process(const std::deque<WheelMsgConstPtr> &wheel,
               double time0,
               double time1);
  bool select_wheel_data(double time0, double time1, std::vector<WheelMsg> &data_vec) const;

  // output residual and Jacobian
  void ComputeLinearSystem(const WheelPoseState &pose0,
                           const WheelPoseState &pose1,
                           Eigen::MatrixXd &H,
                           Eigen::VectorXd &res) const;

  const WheelPreintegration &latest_result() const { return preintegration_; }

  int wheel_type;
  double rl, rr, b;

 private:
  static WheelMsg interpolate_data(const WheelMsg &data1, const WheelMsg &data2, double timestamp);
  void preintegration_2D(double dt, const WheelMsg &data1, const WheelMsg &data2);
  void preintegration_3D(double dt, const WheelMsg &data1, const WheelMsg &data2);
  void compute_linear_system_2D(const WheelPoseState &pose0,
                                const WheelPoseState &pose1,
                                Eigen::MatrixXd &H,
                                Eigen::VectorXd &res) const;
  void compute_linear_system_3D(const WheelPoseState &pose0,
                                const WheelPoseState &pose1,
                                Eigen::MatrixXd &H,
                                Eigen::VectorXd &res) const;

  double noise_w, noise_v, noise_p;
  double max_history_time;
  std::vector<WheelMsg> data_stack_;
  WheelPreintegration preintegration_;
  M3D wheel_R_wrt_IMU;
  V3D wheel_T_wrt_IMU;
};
