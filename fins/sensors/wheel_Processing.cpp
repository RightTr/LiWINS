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
  t_hist.clear();
  preintegration_ = WheelPreintegration();
}

void WheelProcess::set_intrinsic(double rl, double rr, double b)
{
  this->rl = rl;
  this->rr = rr;
  this->b = b;
}

void WheelProcess::set_noise(double noise_w, double noise_v, double noise_p)
{
  this->noise_w = noise_w;
  this->noise_v = noise_v;
  this->noise_p = noise_p;
}

void WheelProcess::set_history_time(double max_history_time)
{
  this->max_history_time = max_history_time;
}

void WheelProcess::enable_calib_ext(bool enable)
{
  do_calib_ext = enable;
}

void WheelProcess::enable_calib_dt(bool enable)
{
  do_calib_dt = enable;
}

void WheelProcess::enable_calib_int(bool enable)
{
  do_calib_int = enable;
}

void WheelProcess::feed_measurement(const WheelMsg &data)
{
  data_stack_.push_back(data);
  std::sort(data_stack_.begin(), data_stack_.end(),
            [](const WheelMsg &lhs, const WheelMsg &rhs) { return lhs.timestamp < rhs.timestamp; });

  for (auto it = data_stack_.begin(); it != data_stack_.end();)
  {
    if (data.timestamp - it->timestamp > max_history_time)
      it = data_stack_.erase(it);
    else
      ++it;
  }

  if (t_hist.size() > 100)
    t_hist.pop_front();
  t_hist.push_back(data.timestamp);
}

bool WheelProcess::Process(double time0, double time1, WheelPreintegration *result)
{
  std::vector<WheelMsg> data_vec;
  if (!select_wheel_data(time0, time1, data_vec))
    return false;

  preintegration_ = WheelPreintegration();
  preintegration_.start_time = time0;
  preintegration_.end_time = time1;

  for (size_t i = 0; i + 1 < data_vec.size(); ++i)
  {
    const double dt = data_vec[i + 1].timestamp - data_vec[i].timestamp;
    if (dt <= 0.0)
      continue;

    if (type == Wheel3DAng || type == Wheel3DLin || type == Wheel3DCen)
    {
      if (do_calib_int)
        preintegration_intrinsics_3D(dt, data_vec[i]);
      preintegration_3D(dt, data_vec[i], data_vec[i + 1]);
    }
    else
    {
      if (do_calib_int)
        preintegration_intrinsics_2D(dt, data_vec[i]);
      preintegration_2D(dt, data_vec[i], data_vec[i + 1]);
    }
  }

  if (result != nullptr)
    *result = preintegration_;
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

bool WheelProcess::ComputeLinearSystem(const WheelPoseState &pose0,
                                       const WheelPoseState &pose1,
                                       const M3D &extrinsic_rot,
                                       const V3D &extrinsic_pos,
                                       WheelLinearizationResult &result) const
{
  if (type == Wheel3DAng || type == Wheel3DLin || type == Wheel3DCen)
    compute_linear_system_3D(pose0, pose1, extrinsic_rot, extrinsic_pos, result);
  else
    compute_linear_system_2D(pose0, pose1, extrinsic_rot, extrinsic_pos, result);
  return result.H.size() > 0 && result.res.size() > 0;
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

void WheelProcess::preintegration_intrinsics_2D(double dt, const WheelMsg &data)
{
  const double w_l = data.encoder1;
  const double w_r = data.encoder2;
  const double rl = this->rl;
  const double rr = this->rr;
  const double b = this->b;

  const double w = (w_r * rr - w_l * rl) / b;
  const double v = (w_r * rr + w_l * rl) / 2.0;

  Eigen::Matrix<double, 1, 3> Hwx = Eigen::Matrix<double, 1, 3>::Zero();
  Hwx(0, 0) = -w_l / b;
  Hwx(0, 1) = w_r / b;
  Hwx(0, 2) = -(w_r * rr - w_l * rl) / (b * b);

  Eigen::Matrix<double, 1, 3> Hvx = Eigen::Matrix<double, 1, 3>::Zero();
  Hvx(0, 0) = w_l / 2.0;
  Hvx(0, 1) = w_r / 2.0;

  double h_thw = dt;
  double h_xth = (v * (std::cos(preintegration_.th_2D - w * dt) - std::cos(preintegration_.th_2D))) / w;
  double h_yth = -(v * (std::sin(preintegration_.th_2D - w * dt) - std::sin(preintegration_.th_2D))) / w;
  double h_xw = (v * (std::sin(preintegration_.th_2D - w * dt) - std::sin(preintegration_.th_2D))) / (w * w) +
                (v * std::cos(preintegration_.th_2D - w * dt) * dt) / w;
  double h_yw = (v * (std::cos(preintegration_.th_2D - w * dt) - std::cos(preintegration_.th_2D))) / (w * w) -
                (v * std::sin(preintegration_.th_2D - w * dt) * dt) / w;
  double h_xv = -(std::sin(preintegration_.th_2D - w * dt) - std::sin(preintegration_.th_2D)) / w;
  double h_yv = -(std::cos(preintegration_.th_2D - w * dt) - std::cos(preintegration_.th_2D)) / w;

  if (std::abs(w) < 1e-4)
  {
    h_xth = v * std::sin(preintegration_.th_2D) * dt;
    h_yth = v * std::cos(preintegration_.th_2D) * dt;
    h_xw = v * std::sin(preintegration_.th_2D) * dt * dt / 2.0;
    h_yw = v * std::cos(preintegration_.th_2D) * dt * dt / 2.0;
    h_xv = std::cos(preintegration_.th_2D) * dt;
    h_yv = -std::sin(preintegration_.th_2D) * dt;
  }

  preintegration_.dx_di_2D = preintegration_.dx_di_2D + h_xth * preintegration_.dth_di_2D + h_xw * Hwx + h_xv * Hvx;
  preintegration_.dy_di_2D = preintegration_.dy_di_2D + h_yth * preintegration_.dth_di_2D + h_yw * Hwx + h_yv * Hvx;
  preintegration_.dth_di_2D = preintegration_.dth_di_2D + h_thw * Hwx;
}

void WheelProcess::preintegration_intrinsics_3D(double dt, const WheelMsg &data)
{
  const double w_l = data.encoder1;
  const double w_r = data.encoder2;
  const double rl = this->rl;
  const double rr = this->rr;
  const double b = this->b;

  Eigen::Vector3d w(0.0, 0.0, (w_r * rr - w_l * rl) / b);
  Eigen::Vector3d v((w_r * rr + w_l * rl) / 2.0, 0.0, 0.0);

  Eigen::Matrix3d Hwx = Eigen::Matrix3d::Zero();
  Hwx(2, 0) = -w_l / b;
  Hwx(2, 1) = w_r / b;
  Hwx(2, 2) = -(w_r * rr - w_l * rl) / (b * b);

  Eigen::Matrix3d Hvx = Eigen::Matrix3d::Zero();
  Hvx(0, 0) = w_l / 2.0;
  Hvx(0, 1) = w_r / 2.0;

  const Eigen::Vector3d w_dt_exp = -w * dt;
  const Eigen::Matrix3d R = Exp(Eigen::Vector3d(w_dt_exp));
  Eigen::Matrix3d Hth = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d w_dt = -w * dt;
  const double theta = w_dt.norm();
  if (theta >= 1e-6)
  {
    const Eigen::Vector3d axis = w_dt / theta;
    Hth = std::sin(theta) / theta * Eigen::Matrix3d::Identity() +
          (1.0 - std::sin(theta) / theta) * axis * axis.transpose() +
          ((1.0 - std::cos(theta)) / theta) * skew_sym_mat(axis);
  }
  Hth *= dt;

  preintegration_.dp_di_3D =
      preintegration_.dp_di_3D - preintegration_.R_3D.transpose() * skew_sym_mat(Eigen::Vector3d(v * dt)) * preintegration_.dR_di_3D +
      preintegration_.R_3D.transpose() * Hvx * dt;
  preintegration_.dR_di_3D = R * preintegration_.dR_di_3D + Hth * Hwx;
}

void WheelProcess::preintegration_2D(double dt, const WheelMsg &data1, const WheelMsg &data2)
{
  const double rl = this->rl;
  const double rr = this->rr;
  const double b = this->b;

  double w1 = 0.0, w2 = 0.0, v1 = 0.0, v2 = 0.0;
  if (type == Wheel2DAng)
  {
    w1 = (data1.encoder2 * rr - data1.encoder1 * rl) / b;
    v1 = (data1.encoder2 * rr + data1.encoder1 * rl) / 2.0;
    w2 = (data2.encoder2 * rr - data2.encoder1 * rl) / b;
    v2 = (data2.encoder2 * rr + data2.encoder1 * rl) / 2.0;
  }
  else if (type == Wheel2DLin)
  {
    w1 = (data1.encoder2 - data1.encoder1) / b;
    v1 = (data1.encoder2 + data1.encoder1) / 2.0;
    w2 = (data2.encoder2 - data2.encoder1) / b;
    v2 = (data2.encoder2 + data2.encoder1) / 2.0;
  }
  else
  {
    w1 = data1.encoder1;
    v1 = data1.encoder2;
    w2 = data2.encoder1;
    v2 = data2.encoder2;
  }

  const double w_alpha = (w2 - w1) / dt;
  const double v_jerk = (v2 - v1) / dt;

  double w = w1;
  double v = v1;
  const double k1_th = -w * dt;
  const double k1_x = v * dt;
  const double k1_y = 0.0;

  const double th2 = 0.5 * k1_th;
  w += 0.5 * w_alpha * dt;
  v += 0.5 * v_jerk * dt;
  const double k2_th = -w * dt;
  const double k2_x = v * std::cos(th2) * dt;
  const double k2_y = -v * std::sin(th2) * dt;

  const double th3 = 0.5 * k2_th;
  const double k3_th = -w * dt;
  const double k3_x = v * std::cos(th3) * dt;
  const double k3_y = -v * std::sin(th3) * dt;

  const double th4 = k3_th;
  w += 0.5 * w_alpha * dt;
  v += 0.5 * v_jerk * dt;
  const double k4_th = -w * dt;
  const double k4_x = v * std::cos(th4) * dt;
  const double k4_y = -v * std::sin(th4) * dt;

  const double th_next = preintegration_.th_2D + (k1_th + 2.0 * k2_th + 2.0 * k3_th + k4_th) / 6.0;
  const double x_next = preintegration_.x_2D + (k1_x + 2.0 * k2_x + 2.0 * k3_x + k4_x) / 6.0;
  double y_next = preintegration_.y_2D + (k1_y + 2.0 * k2_y + 2.0 * k3_y + k4_y) / 6.0;
  if (std::abs(w1) < 1e-4)
    y_next = preintegration_.y_2D - v1 * std::sin(preintegration_.th_2D - w1 * dt) * dt;
  else
    y_next = preintegration_.y_2D - (v1 * (std::cos(preintegration_.th_2D - w1 * dt) - std::cos(preintegration_.th_2D))) / w1;

  Eigen::Matrix<double, 1, 2> Hwn = Eigen::Matrix<double, 1, 2>::Zero();
  Eigen::Matrix<double, 1, 2> Hvn = Eigen::Matrix<double, 1, 2>::Zero();
  if (type == Wheel2DAng)
  {
    Hwn << rl / b, -rr / b;
    Hvn << -rl / 2.0, -rr / 2.0;
  }
  else if (type == Wheel2DLin)
  {
    Hwn << 1.0 / b, -1.0 / b;
    Hvn << -0.5, -0.5;
  }
  else
  {
    Hwn << 1.0, 0.0;
    Hvn << 0.0, 1.0;
  }

  double h_thw = dt;
  double h_xth = (v1 * (std::cos(preintegration_.th_2D - w1 * dt) - std::cos(preintegration_.th_2D))) / w1;
  double h_yth = -(v1 * (std::sin(preintegration_.th_2D - w1 * dt) - std::sin(preintegration_.th_2D))) / w1;
  double h_xw = (v1 * (std::sin(preintegration_.th_2D - w1 * dt) - std::sin(preintegration_.th_2D))) / (w1 * w1) +
                (v1 * std::cos(preintegration_.th_2D - w1 * dt) * dt) / w1;
  double h_yw = (v1 * (std::cos(preintegration_.th_2D - w1 * dt) - std::cos(preintegration_.th_2D))) / (w1 * w1) -
                (v1 * std::sin(preintegration_.th_2D - w1 * dt) * dt) / w1;
  double h_xv = -(std::sin(preintegration_.th_2D - w1 * dt) - std::sin(preintegration_.th_2D)) / w1;
  double h_yv = -(std::cos(preintegration_.th_2D - w1 * dt) - std::cos(preintegration_.th_2D)) / w1;

  if (std::abs(w1) < 1e-4)
  {
    h_xth = v1 * std::sin(preintegration_.th_2D) * dt;
    h_yth = v1 * std::cos(preintegration_.th_2D) * dt;
    h_xw = v1 * std::sin(preintegration_.th_2D) * dt * dt / 2.0;
    h_yw = v1 * std::cos(preintegration_.th_2D) * dt * dt / 2.0;
    h_xv = std::cos(preintegration_.th_2D) * dt;
    h_yv = -std::sin(preintegration_.th_2D) * dt;
  }

  Eigen::Matrix3d Phi_tr = Eigen::Matrix3d::Identity();
  Phi_tr(1, 0) = h_xth;
  Phi_tr(2, 0) = h_yth;

  Eigen::Matrix<double, 3, 2> Phi_ns = Eigen::Matrix<double, 3, 2>::Zero();
  Phi_ns.block<1, 2>(0, 0) = h_thw * Hwn;
  Phi_ns.block<1, 2>(1, 0) = h_xw * Hwn + h_xv * Hvn;
  Phi_ns.block<1, 2>(2, 0) = h_yw * Hwn + h_yv * Hvn;

  Eigen::Matrix2d Q = Eigen::Matrix2d::Zero();
  if (type == Wheel2DAng)
  {
    Q = std::pow(noise_w, 2) / dt * Eigen::Matrix2d::Identity();
  }
  else if (type == Wheel2DLin)
  {
    Q = std::pow(noise_v, 2) / dt * Eigen::Matrix2d::Identity();
  }
  else
  {
    Q(0, 0) = std::pow(noise_w, 2) / dt;
    Q(1, 1) = std::pow(noise_v, 2) / dt;
  }

  preintegration_.Cov_2D = Phi_tr * preintegration_.Cov_2D * Phi_tr.transpose() + Phi_ns * Q * Phi_ns.transpose();
  preintegration_.Cov_2D = 0.5 * (preintegration_.Cov_2D + preintegration_.Cov_2D.transpose());
  preintegration_.th_2D = th_next;
  preintegration_.x_2D = x_next;
  preintegration_.y_2D = y_next;
}

void WheelProcess::preintegration_3D(double dt, const WheelMsg &data1, const WheelMsg &data2)
{
  const double rl = this->rl;
  const double rr = this->rr;
  const double b = this->b;

  Eigen::Vector3d w_hat1 = Zero3d;
  Eigen::Vector3d v_hat1 = Zero3d;
  Eigen::Vector3d w_hat2 = Zero3d;
  Eigen::Vector3d v_hat2 = Zero3d;

  if (type == Wheel3DAng)
  {
    w_hat1 << 0.0, 0.0, (data1.encoder2 * rr - data1.encoder1 * rl) / b;
    v_hat1 << (data1.encoder2 * rr + data1.encoder1 * rl) / 2.0, 0.0, 0.0;
    w_hat2 << 0.0, 0.0, (data2.encoder2 * rr - data2.encoder1 * rl) / b;
    v_hat2 << (data2.encoder2 * rr + data2.encoder1 * rl) / 2.0, 0.0, 0.0;
  }
  else if (type == Wheel3DLin)
  {
    w_hat1 << 0.0, 0.0, (data1.encoder2 - data1.encoder1) / b;
    v_hat1 << (data1.encoder2 + data1.encoder1) / 2.0, 0.0, 0.0;
    w_hat2 << 0.0, 0.0, (data2.encoder2 - data2.encoder1) / b;
    v_hat2 << (data2.encoder2 + data2.encoder1) / 2.0, 0.0, 0.0;
  }
  else
  {
    w_hat1 << 0.0, 0.0, data1.encoder1;
    v_hat1 << data1.encoder2, 0.0, 0.0;
    w_hat2 << 0.0, 0.0, data2.encoder1;
    v_hat2 << data2.encoder2, 0.0, 0.0;
  }

  Eigen::Vector3d w_hat = w_hat1;
  Eigen::Vector3d v_hat = v_hat1;
  const Eigen::Vector3d w_alpha = (w_hat2 - w_hat1) / dt;
  const Eigen::Vector3d v_jerk = (v_hat2 - v_hat1) / dt;
  Eigen::Quaterniond q_local_q(preintegration_.R_3D);
  q_local_q.normalize();
  Eigen::Vector4d q_local = q_local_q.coeffs();
  if (q_local(3) < 0.0)
    q_local *= -1.0;

  const Eigen::Vector4d dq_0(0.0, 0.0, 0.0, 1.0);
  Eigen::Matrix4d Omega_mat = Eigen::Matrix4d::Zero();
  Omega_mat.block<3, 3>(0, 0) = -skew_sym_mat(w_hat);
  Omega_mat.block<3, 1>(0, 3) = w_hat;
  Omega_mat.block<1, 3>(3, 0) = -w_hat.transpose();
  const Eigen::Vector4d q0_dot = 0.5 * Omega_mat * dq_0;
  Eigen::Quaterniond dq0_q(dq_0(3), dq_0(0), dq_0(1), dq_0(2));
  Eigen::Quaterniond q_local_eigen(q_local(3), q_local(0), q_local(1), q_local(2));
  Eigen::Quaterniond q_mul = (dq0_q * q_local_eigen).normalized();
  Eigen::Vector4d q_mul_vec = q_mul.coeffs();
  if (q_mul_vec(3) < 0.0)
    q_mul_vec *= -1.0;
  const Eigen::Matrix3d R_Gto0 = Eigen::Quaterniond(q_mul_vec(3), q_mul_vec(0), q_mul_vec(1), q_mul_vec(2)).toRotationMatrix();
  const Eigen::Vector3d p0_dot = R_Gto0.transpose() * v_hat;
  const Eigen::Vector4d k1_q = q0_dot * dt;
  const Eigen::Vector3d k1_p = p0_dot * dt;

  w_hat += 0.5 * w_alpha * dt;
  v_hat += 0.5 * v_jerk * dt;
  Eigen::Vector4d dq_1 = (dq_0 + 0.5 * k1_q);
  Eigen::Quaterniond dq1_q(dq_1(3), dq_1(0), dq_1(1), dq_1(2));
  dq1_q.normalize();
  dq_1 = dq1_q.coeffs();
  if (dq_1(3) < 0.0)
    dq_1 *= -1.0;
  Omega_mat.setZero();
  Omega_mat.block<3, 3>(0, 0) = -skew_sym_mat(w_hat);
  Omega_mat.block<3, 1>(0, 3) = w_hat;
  Omega_mat.block<1, 3>(3, 0) = -w_hat.transpose();
  const Eigen::Vector4d q1_dot = 0.5 * Omega_mat * dq_1;
  dq1_q = Eigen::Quaterniond(dq_1(3), dq_1(0), dq_1(1), dq_1(2));
  q_mul = (dq1_q * q_local_eigen).normalized();
  q_mul_vec = q_mul.coeffs();
  if (q_mul_vec(3) < 0.0)
    q_mul_vec *= -1.0;
  const Eigen::Matrix3d R_Gto1 = Eigen::Quaterniond(q_mul_vec(3), q_mul_vec(0), q_mul_vec(1), q_mul_vec(2)).toRotationMatrix();
  const Eigen::Vector3d p1_dot = R_Gto1.transpose() * v_hat;
  const Eigen::Vector4d k2_q = q1_dot * dt;
  const Eigen::Vector3d k2_p = p1_dot * dt;

  Eigen::Vector4d dq_2 = (dq_0 + 0.5 * k2_q);
  Eigen::Quaterniond dq2_q(dq_2(3), dq_2(0), dq_2(1), dq_2(2));
  dq2_q.normalize();
  dq_2 = dq2_q.coeffs();
  if (dq_2(3) < 0.0)
    dq_2 *= -1.0;
  const Eigen::Vector4d q2_dot = 0.5 * Omega_mat * dq_2;
  q_mul = (dq2_q * q_local_eigen).normalized();
  q_mul_vec = q_mul.coeffs();
  if (q_mul_vec(3) < 0.0)
    q_mul_vec *= -1.0;
  const Eigen::Matrix3d R_Gto2 = Eigen::Quaterniond(q_mul_vec(3), q_mul_vec(0), q_mul_vec(1), q_mul_vec(2)).toRotationMatrix();
  const Eigen::Vector3d p2_dot = R_Gto2.transpose() * v_hat;
  const Eigen::Vector4d k3_q = q2_dot * dt;
  const Eigen::Vector3d k3_p = p2_dot * dt;

  w_hat += 0.5 * w_alpha * dt;
  v_hat += 0.5 * v_jerk * dt;
  Eigen::Vector4d dq_3 = (dq_0 + k3_q);
  Eigen::Quaterniond dq3_q(dq_3(3), dq_3(0), dq_3(1), dq_3(2));
  dq3_q.normalize();
  dq_3 = dq3_q.coeffs();
  if (dq_3(3) < 0.0)
    dq_3 *= -1.0;
  Omega_mat.setZero();
  Omega_mat.block<3, 3>(0, 0) = -skew_sym_mat(w_hat);
  Omega_mat.block<3, 1>(0, 3) = w_hat;
  Omega_mat.block<1, 3>(3, 0) = -w_hat.transpose();
  const Eigen::Vector4d q3_dot = 0.5 * Omega_mat * dq_3;
  q_mul = (dq3_q * q_local_eigen).normalized();
  q_mul_vec = q_mul.coeffs();
  if (q_mul_vec(3) < 0.0)
    q_mul_vec *= -1.0;
  const Eigen::Matrix3d R_Gto3 = Eigen::Quaterniond(q_mul_vec(3), q_mul_vec(0), q_mul_vec(1), q_mul_vec(2)).toRotationMatrix();
  const Eigen::Vector3d p3_dot = R_Gto3.transpose() * v_hat;
  const Eigen::Vector4d k4_q = q3_dot * dt;
  const Eigen::Vector3d k4_p = p3_dot * dt;

  Eigen::Vector4d dq = dq_0 + k1_q / 6.0 + k2_q / 3.0 + k3_q / 3.0 + k4_q / 6.0;
  Eigen::Quaterniond dq_q(dq(3), dq(0), dq(1), dq(2));
  dq_q.normalize();
  dq = dq_q.coeffs();
  if (dq(3) < 0.0)
    dq *= -1.0;
  q_mul = (dq_q * q_local_eigen).normalized();
  q_mul_vec = q_mul.coeffs();
  if (q_mul_vec(3) < 0.0)
    q_mul_vec *= -1.0;
  const Eigen::Matrix3d R_new = Eigen::Quaterniond(q_mul_vec(3), q_mul_vec(0), q_mul_vec(1), q_mul_vec(2)).toRotationMatrix();
  const Eigen::Vector3d p_new = preintegration_.p_3D + k1_p / 6.0 + k2_p / 3.0 + k3_p / 3.0 + k4_p / 6.0;

  Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
  if (type == Wheel3DAng)
  {
    Q(0, 0) = std::pow(noise_w, 2) / dt;
    Q(1, 1) = std::pow(noise_p, 2) / dt;
    Q(2, 2) = std::pow(noise_p, 2) / dt;
    Q(3, 3) = std::pow(noise_w, 2) / dt;
    Q(4, 4) = std::pow(noise_p, 2) / dt;
    Q(5, 5) = std::pow(noise_p, 2) / dt;
  }
  else if (type == Wheel3DLin)
  {
    Q(0, 0) = std::pow(noise_v, 2) / (b * b * dt);
    Q(1, 1) = std::pow(noise_p, 2) / dt;
    Q(2, 2) = std::pow(noise_p, 2) / dt;
    Q(3, 3) = std::pow(noise_v, 2) / (4.0 * dt);
    Q(4, 4) = std::pow(noise_p, 2) / dt;
    Q(5, 5) = std::pow(noise_p, 2) / dt;
  }
  else
  {
    Q(0, 0) = std::pow(noise_w, 2) / dt;
    Q(1, 1) = std::pow(noise_p, 2) / dt;
    Q(2, 2) = std::pow(noise_p, 2) / dt;
    Q(3, 3) = std::pow(noise_v, 2) / dt;
    Q(4, 4) = std::pow(noise_p, 2) / dt;
    Q(5, 5) = std::pow(noise_p, 2) / dt;
  }

  Eigen::Matrix<double, 6, 6> Phi_tr = Eigen::Matrix<double, 6, 6>::Zero();
  Phi_tr.block<3, 3>(0, 0) = R_new * preintegration_.R_3D.transpose();
  const Eigen::Vector3d delta_p_body = preintegration_.R_3D.transpose() * (p_new - preintegration_.p_3D);
  Phi_tr.block<3, 3>(3, 0) = -preintegration_.R_3D.transpose() * skew_sym_mat(delta_p_body);
  Phi_tr.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();

  Eigen::Matrix<double, 6, 6> Phi_ns = Eigen::Matrix<double, 6, 6>::Zero();
  Phi_ns.block<3, 3>(0, 0) = dt * Eigen::Matrix3d::Identity();
  Phi_ns.block<3, 3>(3, 3) = preintegration_.R_3D.transpose() * dt;

  preintegration_.Cov_3D = Phi_tr * preintegration_.Cov_3D * Phi_tr.transpose() + Phi_ns * Q * Phi_ns.transpose();
  preintegration_.Cov_3D = 0.5 * (preintegration_.Cov_3D + preintegration_.Cov_3D.transpose());
  preintegration_.R_3D = R_new;
  preintegration_.p_3D = p_new;
}

void WheelProcess::compute_linear_system_2D(const WheelPoseState &pose0,
                                            const WheelPoseState &pose1,
                                            const M3D &extrinsic_rot,
                                            const V3D &extrinsic_pos,
                                            WheelLinearizationResult &result) const
{
  const Eigen::Vector3d pI0inG = pose0.pos;
  const Eigen::Vector3d pI1inG = pose1.pos;
  const Eigen::Matrix3d RGtoI0 = pose0.rot;
  const Eigen::Matrix3d RGtoI1 = pose1.rot;
  const Eigen::Vector3d pIinO = extrinsic_pos;
  const Eigen::Matrix3d RItoO = extrinsic_rot;
  const Eigen::Vector3d pOinI = -RItoO.transpose() * pIinO;
  Eigen::Matrix3d RO0toO1 = RItoO * RGtoI1 * RGtoI0.transpose() * RItoO.transpose();
  Eigen::Matrix3d RO1toO0 = RO0toO1.transpose();

  const Eigen::Vector3d e3(0.0, 0.0, 1.0);
  Eigen::Matrix<double, 2, 3> Lambda = Eigen::Matrix<double, 2, 3>::Zero();
  Lambda.block<2, 2>(0, 0) = Eigen::Matrix2d::Identity();

  result.res = Eigen::Vector3d::Zero();
  const Eigen::Matrix3d theta_est_rot = RItoO * RGtoI1 * RGtoI0.transpose() * RItoO.transpose();
  const double theta_est = e3.transpose() * Log(theta_est_rot);
  result.res(0) = theta_est - preintegration_.th_2D;
  const Eigen::Vector2d d_int(preintegration_.x_2D, preintegration_.y_2D);
  const Eigen::Vector2d d_est =
      Lambda * RItoO * RGtoI0 * (pI1inG + RGtoI1.transpose() * pOinI - pI0inG - RGtoI0.transpose() * pOinI);
  result.res.segment<2>(1) = d_int - d_est;

  int H_size = 12;
  int H_count = 12;
  H_size += do_calib_ext ? 6 : 0;
  H_size += do_calib_dt ? 1 : 0;
  H_size += do_calib_int ? 3 : 0;
  result.H = Eigen::MatrixXd::Zero(3, H_size);

  const Eigen::Vector3d pI0inG_fej = pose0.pos_fej;
  const Eigen::Vector3d pI1inG_fej = pose1.pos_fej;
  const Eigen::Matrix3d RGtoI0_fej = pose0.rot_fej;
  const Eigen::Matrix3d RGtoI1_fej = pose1.rot_fej;
  RO0toO1 = RItoO * RGtoI1_fej * RGtoI0_fej.transpose() * RItoO.transpose();
  RO1toO0 = RO0toO1.transpose();

  const Eigen::Matrix<double, 1, 3> dzr_dth0 = -e3.transpose() * RItoO * RGtoI1_fej * RGtoI0_fej.transpose();
  const Eigen::Matrix<double, 1, 3> dzr_dth1 = e3.transpose() * RItoO;
  const Eigen::Vector3d dzp_dth0_arg = RGtoI0_fej * (pI1inG_fej + RGtoI1_fej.transpose() * pOinI - pI0inG_fej);
  const Eigen::Matrix<double, 2, 3> dzp_dth0 = Lambda * RItoO * skew_sym_mat(dzp_dth0_arg);
  const Eigen::Matrix<double, 2, 3> dzp_dp0 = -Lambda * RItoO * RGtoI0_fej;
  const Eigen::Matrix<double, 2, 3> dzp_dth1 = -Lambda * RItoO * RGtoI0_fej * RGtoI1_fej.transpose() * skew_sym_mat(pOinI);
  const Eigen::Matrix<double, 2, 3> dzp_dp1 = Lambda * RItoO * RGtoI0_fej;

  result.H.block<1, 3>(0, 0) = dzr_dth0;
  result.H.block<1, 3>(0, 6) = dzr_dth1;
  result.H.block<2, 3>(1, 0) = dzp_dth0;
  result.H.block<2, 3>(1, 3) = dzp_dp0;
  result.H.block<2, 3>(1, 6) = dzp_dth1;
  result.H.block<2, 3>(1, 9) = dzp_dp1;

  if (do_calib_ext)
  {
    const Eigen::Matrix<double, 1, 3> dzr_dthcalib = e3.transpose() * (Eigen::Matrix3d::Identity() - RO0toO1);
    const Eigen::Vector3d dzp_dthcalib_arg = RItoO * RGtoI0_fej * (pI1inG_fej - pI0inG_fej) - RO1toO0 * pIinO;
    const Eigen::Matrix<double, 2, 3> dzp_dthcalib =
        Lambda * (skew_sym_mat(dzp_dthcalib_arg) + RO1toO0 * skew_sym_mat(pIinO));
    const Eigen::Matrix<double, 2, 3> dzp_dpcalib = Lambda * (-RO1toO0 + Eigen::Matrix3d::Identity());
    result.H.block<1, 3>(0, H_count) = dzr_dthcalib;
    result.H.block<2, 3>(1, H_count) = dzp_dthcalib;
    result.H.block<2, 3>(1, H_count + 3) = dzp_dpcalib;
    H_count += 6;
  }

  if (do_calib_dt)
  {
    result.H(0, H_count) = (dzr_dth0 * pose0.ang_vel + dzr_dth1 * pose1.ang_vel)(0, 0);
    result.H.block<2, 1>(1, H_count) = dzp_dth0 * pose0.ang_vel + dzp_dp0 * pose0.lin_vel +
                                       dzp_dth1 * pose1.ang_vel + dzp_dp1 * pose1.lin_vel;
    ++H_count;
  }

  if (do_calib_int)
  {
    result.H.block<1, 3>(0, H_count) = -preintegration_.dth_di_2D;
    result.H.block<1, 3>(1, H_count) = -preintegration_.dx_di_2D;
    result.H.block<1, 3>(2, H_count) = -preintegration_.dy_di_2D;
  }
}

void WheelProcess::compute_linear_system_3D(const WheelPoseState &pose0,
                                            const WheelPoseState &pose1,
                                            const M3D &extrinsic_rot,
                                            const V3D &extrinsic_pos,
                                            WheelLinearizationResult &result) const
{
  const Eigen::Vector3d pI0inG = pose0.pos;
  const Eigen::Vector3d pI1inG = pose1.pos;
  const Eigen::Matrix3d RGtoI0 = pose0.rot;
  const Eigen::Matrix3d RGtoI1 = pose1.rot;
  const Eigen::Vector3d pIinO = extrinsic_pos;
  const Eigen::Matrix3d RItoO = extrinsic_rot;
  const Eigen::Vector3d pOinI = -RItoO.transpose() * pIinO;
  Eigen::Matrix3d RO0toO1 = RItoO * RGtoI1 * RGtoI0.transpose() * RItoO.transpose();
  Eigen::Matrix3d RO1toO0 = RO0toO1.transpose();

  result.res = Eigen::Matrix<double, 6, 1>::Zero();
  const Eigen::Matrix3d rot_residual = preintegration_.R_3D * RO0toO1.transpose();
  result.res.segment<3>(0) = -Log(rot_residual);
  const Eigen::Vector3d p_est =
      RItoO * RGtoI0 * (pI1inG + RGtoI1.transpose() * pOinI - pI0inG - RGtoI0.transpose() * pOinI);
  result.res.segment<3>(3) = preintegration_.p_3D - p_est;

  int H_size = 12;
  int H_count = 12;
  H_size += do_calib_ext ? 6 : 0;
  H_size += do_calib_dt ? 1 : 0;
  H_size += do_calib_int ? 3 : 0;
  result.H = Eigen::MatrixXd::Zero(6, H_size);

  const Eigen::Vector3d pI0inG_fej = pose0.pos_fej;
  const Eigen::Vector3d pI1inG_fej = pose1.pos_fej;
  const Eigen::Matrix3d RGtoI0_fej = pose0.rot_fej;
  const Eigen::Matrix3d RGtoI1_fej = pose1.rot_fej;
  RO0toO1 = RItoO * RGtoI1_fej * RGtoI0_fej.transpose() * RItoO.transpose();
  RO1toO0 = RO0toO1.transpose();

  const Eigen::Matrix3d dzr_dth0 = -RItoO * RGtoI1_fej * RGtoI0_fej.transpose();
  const Eigen::Matrix3d dzr_dth1 = RItoO;
  const Eigen::Vector3d dzp_dth0_arg =
      RGtoI0_fej * pI1inG_fej + RGtoI0_fej * RGtoI1_fej.transpose() * pOinI - RGtoI0_fej * pI0inG_fej;
  const Eigen::Matrix3d dzp_dth0 = RItoO * skew_sym_mat(dzp_dth0_arg);
  const Eigen::Matrix3d dzp_dp0 = -RItoO * RGtoI0_fej;
  const Eigen::Matrix3d dzp_dth1 = -RItoO * RGtoI0_fej * RGtoI1_fej.transpose() * skew_sym_mat(pOinI);
  const Eigen::Matrix3d dzp_dp1 = RItoO * RGtoI0_fej;

  result.H.block<3, 3>(0, 0) = dzr_dth0;
  result.H.block<3, 3>(0, 6) = dzr_dth1;
  result.H.block<3, 3>(3, 0) = dzp_dth0;
  result.H.block<3, 3>(3, 3) = dzp_dp0;
  result.H.block<3, 3>(3, 6) = dzp_dth1;
  result.H.block<3, 3>(3, 9) = dzp_dp1;

  if (do_calib_ext)
  {
    const Eigen::Matrix3d dzr_dthcalib = Eigen::Matrix3d::Identity() - RO0toO1;
    const Eigen::Matrix3d dzp_dpcalib = -RO1toO0 + Eigen::Matrix3d::Identity();
    const Eigen::Vector3d dzp_dthcalib_arg = RItoO * RGtoI0_fej * (pI1inG_fej - pI0inG_fej) - RO1toO0 * pIinO;
    const Eigen::Matrix3d dzp_dthcalib = skew_sym_mat(dzp_dthcalib_arg) + RO1toO0 * skew_sym_mat(pIinO);
    result.H.block<3, 3>(0, H_count) = dzr_dthcalib;
    result.H.block<3, 3>(3, H_count) = dzp_dthcalib;
    result.H.block<3, 3>(3, H_count + 3) = dzp_dpcalib;
    H_count += 6;
  }

  if (do_calib_dt)
  {
    result.H.block<3, 1>(0, H_count) = dzr_dth0 * pose0.ang_vel + dzr_dth1 * pose1.ang_vel;
    result.H.block<3, 1>(3, H_count) = dzp_dth0 * pose0.ang_vel + dzp_dp0 * pose0.lin_vel +
                                       dzp_dth1 * pose1.ang_vel + dzp_dp1 * pose1.lin_vel;
    ++H_count;
  }

  if (do_calib_int)
  {
    result.H.block<3, 3>(0, H_count) = -preintegration_.dR_di_3D;
    result.H.block<3, 3>(3, H_count) = -preintegration_.dp_di_3D;
  }
}
