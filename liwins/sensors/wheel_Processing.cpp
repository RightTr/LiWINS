#include "sensors/wheel_Processing.h"

#include <algorithm>
#include <cmath>
#include <Eigen/Geometry>

namespace
{
constexpr double kWheelEps = 1e-9;
} // namespace

WheelProcess::WheelProcess()
{
  Reset();
}

WheelProcess::~WheelProcess() {}

void WheelProcess::Reset()
{
  data_stack_.clear();
  preintegration_ = WheelPreintegration();
}

void WheelProcess::set_intrinsic(double sr, double sl)
{
  this->sr = sr;
  this->sl = sl;
}

void WheelProcess::set_noise(double noise_x, double noise_y)
{
  this->noise_x = noise_x;
  this->noise_y = noise_y;
}

void WheelProcess::set_history_time(double max_history_time)
{
  this->max_history_time = max_history_time;
}

void WheelProcess::set_extrinsic(const MD(4,4) &T)
{
  wheel_T_wrt_IMU = T.block<3,1>(0,3);
  wheel_R_wrt_IMU = T.block<3,3>(0,0);
}

void WheelProcess::set_extrinsic(const V3D &transl)
{
  wheel_T_wrt_IMU = transl;
  wheel_R_wrt_IMU.setIdentity();
}

void WheelProcess::set_extrinsic(const V3D &transl, const M3D &rot)
{
  wheel_T_wrt_IMU = transl;
  wheel_R_wrt_IMU = rot;
}

bool WheelProcess::Process(const std::deque<WheelMsgConstPtr> &wheel,
                           double time0,
                           double time1)
{
  for (const auto &msg : wheel)
  {
    if (!data_stack_.empty() && msg->timestamp <= data_stack_.back().timestamp)
      continue;
    data_stack_.push_back(*msg);
  }

  if (!data_stack_.empty())
  {
    const double latest_time = data_stack_.back().timestamp;
    for (auto it = data_stack_.begin(); it != data_stack_.end();)
    {
      if (latest_time - it->timestamp > max_history_time)
        it = data_stack_.erase(it);
      else
        ++it;
    }
  }

  std::vector<WheelMsg> data_vec;
  if (!select_wheel_data(time0, time1, data_vec))
    return false;

  preintegration_ = WheelPreintegration();
  preintegration_.start_time = time0;
  preintegration_.end_time = time1;

  for (size_t i = 0; i + 1 < data_vec.size(); ++i)
  {
    const double dt = data_vec[i + 1].timestamp - data_vec[i].timestamp;
    preintegration_2D(dt, data_vec[i], data_vec[i + 1]);
  }
  return true;
}

bool WheelProcess::select_wheel_data(double time0, double time1, std::vector<WheelMsg> &data_vec) const
{
  data_vec.clear();
  if (data_stack_.empty())
    return false;

  if (data_stack_.back().timestamp <= time1 || data_stack_.front().timestamp > time0)
    return false;

  for (size_t i = 0; i + 1 < data_stack_.size(); ++i)
  {
    if (data_stack_[i + 1].timestamp > time0 && data_stack_[i].timestamp < time0)
    {
      data_vec.push_back(interpolate_data(data_stack_[i], data_stack_[i + 1], time0));
      continue;
    }

    if (data_stack_[i].timestamp >= time0 && data_stack_[i + 1].timestamp <= time1)
    {
      data_vec.push_back(data_stack_[i]);
      continue;
    }

    if (data_stack_[i + 1].timestamp > time1)
    {
      if (data_stack_[i].timestamp > time1)
      {
        if (i == 0)
          break;
        data_vec.push_back(interpolate_data(data_stack_[i - 1], data_stack_[i], time1));
      }
      else
      {
        data_vec.push_back(data_stack_[i]);
      }

      if (data_vec.empty() || std::abs(data_vec.back().timestamp - time1) >= kWheelEps)
        data_vec.push_back(interpolate_data(data_stack_[i], data_stack_[i + 1], time1));
      break;
    }
  }

  if (data_vec.size() < 2)
    return false;

  for (size_t i = 0; i + 1 < data_vec.size();)
  {
    if (std::abs(data_vec[i + 1].timestamp - data_vec[i].timestamp) < 1e-12)
      data_vec.erase(data_vec.begin() + static_cast<long>(i));
    else
      ++i;
  }
  return data_vec.size() >= 2;
}

void WheelProcess::ComputeLinearSystem(const WheelPoseState &pose0,
                                       const WheelPoseState &pose1,
                                       Eigen::MatrixXd &H,
                                       Eigen::VectorXd &res) const
{
  compute_linear_system_2D(pose0, pose1, H, res);
}

WheelMsg WheelProcess::interpolate_data(const WheelMsg &data1, const WheelMsg &data2, double timestamp)
{
  const double lambda = (timestamp - data1.timestamp) / (data2.timestamp - data1.timestamp);
  WheelMsg data;
  data.timestamp = timestamp;
  data.encoder1 = (1.0 - lambda) * data1.encoder1 + lambda * data2.encoder1;
  data.encoder2 = (1.0 - lambda) * data1.encoder2 + lambda * data2.encoder2;
  return data;
}

void WheelProcess::preintegration_2D(double dt, const WheelMsg &data1, const WheelMsg &data2)
{
  const double vx1 = sr * data1.encoder1;
  const double vy1 = sl * data1.encoder2;
  const double vx2 = sr * data2.encoder1;
  const double vy2 = sl * data2.encoder2;

  preintegration_.x_2D += 0.5 * (vx1 + vx2) * dt;
  preintegration_.y_2D += 0.5 * (vy1 + vy2) * dt;

  Eigen::Matrix2d step_cov = Eigen::Matrix2d::Zero();
  step_cov(0, 0) = std::pow(sr * noise_x, 2) * dt;
  step_cov(1, 1) = std::pow(sl * noise_y, 2) * dt;
  preintegration_.Cov_2D += step_cov;
  preintegration_.Cov_2D = 0.5 * (preintegration_.Cov_2D + preintegration_.Cov_2D.transpose());
}

void WheelProcess::compute_linear_system_2D(const WheelPoseState &pose0,
                                            const WheelPoseState &pose1,
                                            Eigen::MatrixXd &H,
                                            Eigen::VectorXd &res) const
{
  const Eigen::Vector3d pI0inG = pose0.pos;
  const Eigen::Vector3d pI1inG = pose1.pos;
  const Eigen::Matrix3d RGtoI0 = pose0.rot;
  const Eigen::Matrix3d RGtoI1 = pose1.rot;
  const Eigen::Vector3d pIinO = wheel_T_wrt_IMU;
  const Eigen::Matrix3d RItoO = wheel_R_wrt_IMU;
  const Eigen::Vector3d pOinI = -RItoO.transpose() * pIinO;

  Eigen::Matrix<double, 2, 3> Lambda = Eigen::Matrix<double, 2, 3>::Zero();
  Lambda.block<2, 2>(0, 0) = Eigen::Matrix2d::Identity();

  res = Eigen::Vector2d::Zero();
  const Eigen::Vector2d d_int(preintegration_.x_2D, preintegration_.y_2D);
  const Eigen::Vector2d d_est =
      Lambda * RItoO * RGtoI0 * (pI1inG + RGtoI1.transpose() * pOinI - pI0inG - RGtoI0.transpose() * pOinI);
  res = d_int - d_est;

  H = Eigen::MatrixXd::Zero(2, 12);

  const Eigen::Vector3d pI0inG_fej = pose0.pos_fej;
  const Eigen::Vector3d pI1inG_fej = pose1.pos_fej;
  const Eigen::Matrix3d RGtoI0_fej = pose0.rot_fej;
  const Eigen::Matrix3d RGtoI1_fej = pose1.rot_fej;

  const Eigen::Vector3d dzp_dth0_arg = RGtoI0_fej * (pI1inG_fej + RGtoI1_fej.transpose() * pOinI - pI0inG_fej);
  const Eigen::Matrix<double, 2, 3> dzp_dth0 = Lambda * RItoO * skew_sym_mat(dzp_dth0_arg);
  const Eigen::Matrix<double, 2, 3> dzp_dp0 = -Lambda * RItoO * RGtoI0_fej;
  const Eigen::Matrix<double, 2, 3> dzp_dth1 = -Lambda * RItoO * RGtoI0_fej * RGtoI1_fej.transpose() * skew_sym_mat(pOinI);
  const Eigen::Matrix<double, 2, 3> dzp_dp1 = Lambda * RItoO * RGtoI0_fej;

  H.block<2, 3>(0, 0) = dzp_dth0;
  H.block<2, 3>(0, 3) = dzp_dp0;
  H.block<2, 3>(0, 6) = dzp_dth1;
  H.block<2, 3>(0, 9) = dzp_dp1;
}
