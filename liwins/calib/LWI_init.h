#pragma once

#include <cstddef>
#include <deque>
#include <vector>

#include <boost/optional.hpp>

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "common_lib.h"

struct optimizeLidarObs
{
  V3D point_in_lidar = Zero3d;
  V3D plane_unit_normal_in_world = V3D(0.0, 0.0, 1.0);
  double plane_offset = 0.0;
};

struct LWIKeyframe
{
  double timestamp = 0.0;
  double wheel_interval_start = 0.0;
  double wheel_interval_end = 0.0;
  gtsam::Pose3 initial_pose;
  gtsam::Vector3 initial_velocity = gtsam::Vector3::Zero();
  gtsam::imuBias::ConstantBias initial_bias;
  std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> imu_preintegration;
  std::deque<WheelMsgConstPtr> wheel_msgs;
  std::vector<optimizeLidarObs> lidar_obs;
};

struct InitGraphConfig
{
  gtsam::Pose3 lidar_pose_in_imu;
  gtsam::Pose2 initial_wheel_pose_in_imu = gtsam::Pose2();
  gtsam::Point2 initial_wheel_scales = gtsam::Point2(1.0, 1.0);

  double lidar_factor_sigma = 0.05;

  gtsam::Vector6 first_pose_prior_sigma =
      (gtsam::Vector6() << 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3).finished();
  gtsam::Vector3 first_velocity_prior_sigma =
      (gtsam::Vector3() << 1e-2, 1e-2, 1e-2).finished();
  gtsam::Vector6 first_bias_prior_sigma =
      (gtsam::Vector6() << 1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-2).finished();
  gtsam::Vector3 wheel_extrinsic_prior_sigma =
      (gtsam::Vector3() << 0.1, 0.1, 0.2).finished();
  gtsam::Vector2 wheel_scale_prior_sigma =
      (gtsam::Vector2() << 0.2, 0.2).finished();
  gtsam::Vector2 wheel_factor_sigma =
      (gtsam::Vector2() << 0.05, 0.05).finished();
};

struct InitGraphResult
{
  gtsam::Values values;
  gtsam::Pose2 wheel_pose_in_imu;
  gtsam::Point2 wheel_scales;
};

void integrate_wheel_segment(
    const WheelMsgConstPtr &wheel_msg0,
    const WheelMsgConstPtr &wheel_msg1,
    double interval_start,
    double interval_end,
    double sr,
    double sl,
    Eigen::Vector2d &delta,
    double &mid_time);

Eigen::Vector2d integrate_wheel_delta(
    const std::deque<WheelMsgConstPtr> &wheel_msgs,
    double interval_start,
    double interval_end,
    double sr,
    double sl);

class WheelFactor
    : public gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Pose3, gtsam::Pose2, gtsam::Point2>
{
 public:
  using Base = gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Pose3, gtsam::Pose2, gtsam::Point2>;

  WheelFactor(gtsam::Key pose0_key,
              gtsam::Key pose1_key,
              gtsam::Key wheel_extrinsic_key,
              gtsam::Key wheel_scale_key,
              const std::deque<WheelMsgConstPtr> &wheel_msgs,
              double wheel_interval_start,
              double wheel_interval_end,
              const gtsam::SharedNoiseModel &noise_model);

  gtsam::Vector evaluateError(
      const gtsam::Pose3 &pose0,
      const gtsam::Pose3 &pose1,
      const gtsam::Pose2 &wheel_pose_in_imu,
      const gtsam::Point2 &wheel_scales,
      boost::optional<gtsam::Matrix &> H1 = boost::none,
      boost::optional<gtsam::Matrix &> H2 = boost::none,
      boost::optional<gtsam::Matrix &> H3 = boost::none,
      boost::optional<gtsam::Matrix &> H4 = boost::none) const override;

 private:
  gtsam::Vector2 evaluateResidual(const gtsam::Pose3 &pose0,
                                  const gtsam::Pose3 &pose1,
                                  const gtsam::Pose2 &wheel_pose_in_imu,
                                  const gtsam::Point2 &wheel_scales) const;

  std::deque<WheelMsgConstPtr> wheel_msgs_;
  double wheel_interval_start_ = 0.0;
  double wheel_interval_end_ = 0.0;
};

class LidarFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
{
 public:
  using Base = gtsam::NoiseModelFactor1<gtsam::Pose3>;

  LidarFactor(gtsam::Key pose_key,
              const optimizeLidarObs &observation,
              const gtsam::Pose3 &lidar_pose_in_imu,
              const gtsam::SharedNoiseModel &noise_model);

  gtsam::Vector evaluateError(
      const gtsam::Pose3 &imu_pose_in_world,
      boost::optional<gtsam::Matrix &> H = boost::none) const override;

 private:
  gtsam::Vector1 evaluateResidual(const gtsam::Pose3 &imu_pose_in_world) const;

  optimizeLidarObs observation_;
  gtsam::Pose3 lidar_pose_in_imu_;
};

class LidarImuWheelInitGraph
{
 public:
  explicit LidarImuWheelInitGraph(const InitGraphConfig &config);

  void Reset();
  void AddKeyframe(const LWIKeyframe &keyframe);

  InitGraphResult Optimize(const gtsam::LevenbergMarquardtParams &params = gtsam::LevenbergMarquardtParams());

  gtsam::Key wheel_extrinsic_key() const { return wheel_extrinsic_key_; }
  gtsam::Key wheel_scale_key() const { return wheel_scale_key_; }

 private:
  void addCalibrationPriors();
  void addFirstStatePriors(std::size_t frame_idx, const LWIKeyframe &keyframe);
  void addImuFactor(std::size_t frame_idx, const LWIKeyframe &keyframe);
  void addWheelFactor(std::size_t frame_idx, const LWIKeyframe &keyframe);
  void addLidarFactor(std::size_t frame_idx, const LWIKeyframe &keyframe);

  InitGraphConfig config_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_values_;
  std::size_t keyframe_count_ = 0;
  bool calibration_priors_added_ = false;
  gtsam::Key wheel_extrinsic_key_;
  gtsam::Key wheel_scale_key_;
};
