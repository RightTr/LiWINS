#include "calib/LWI_init.h"

#include <stdexcept>

#include <boost/make_shared.hpp>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/PriorFactor.h>

Eigen::Vector2d integrate_wheel_delta(
    const std::deque<WheelMsgConstPtr> &wheel_msgs,
    double sr,
    double sl)
{
  Eigen::Vector2d delta = Eigen::Vector2d::Zero();
  if (wheel_msgs.size() < 2)
    return delta;

  for (std::size_t i = 0; i + 1 < wheel_msgs.size(); ++i)
  {
    const double dt = wheel_msgs[i + 1]->timestamp - wheel_msgs[i]->timestamp;
    if (dt <= 0.0)
      continue;

    const double vx0 = sr * wheel_msgs[i]->encoder1;
    const double vy0 = sl * wheel_msgs[i]->encoder2;
    const double vx1 = sr * wheel_msgs[i + 1]->encoder1;
    const double vy1 = sl * wheel_msgs[i + 1]->encoder2;
    delta.x() += 0.5 * (vx0 + vx1) * dt;
    delta.y() += 0.5 * (vy0 + vy1) * dt;
  }

  return delta;
}

WheelFactor::WheelFactor(gtsam::Key pose0_key,
                         gtsam::Key pose1_key,
                         gtsam::Key wheel_extrinsic_key,
                         gtsam::Key wheel_scale_key,
                         const std::deque<WheelMsgConstPtr> &wheel_msgs,
                         const gtsam::SharedNoiseModel &noise_model)
    : Base(noise_model, pose0_key, pose1_key, wheel_extrinsic_key, wheel_scale_key),
      wheel_msgs_(wheel_msgs)
{
}

gtsam::Vector2 WheelFactor::evaluateResidual(const gtsam::Pose3 &pose0,
                                             const gtsam::Pose3 &pose1,
                                             const gtsam::Pose2 &wheel_pose_in_imu,
                                             const gtsam::Point2 &wheel_scales) const
{
  const gtsam::Vector2 wheel_delta =
      integrate_wheel_delta(wheel_msgs_, wheel_scales.x(), wheel_scales.y());

  const Eigen::Matrix3d R_WI0 = pose0.rotation().matrix();
  const Eigen::Matrix3d R_WI1 = pose1.rotation().matrix();
  const Eigen::Vector3d p_I0inW = pose0.translation();
  const Eigen::Vector3d p_I1inW = pose1.translation();

  const double theta = wheel_pose_in_imu.theta();
  const Eigen::Matrix3d R_ItoO =
      Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d p_IinO(wheel_pose_in_imu.x(), wheel_pose_in_imu.y(), 0.0);
  const Eigen::Vector3d p_OinI = -R_ItoO.transpose() * p_IinO;

  Eigen::Matrix<double, 2, 3> Lambda = Eigen::Matrix<double, 2, 3>::Zero();
  Lambda.block<2, 2>(0, 0) = Eigen::Matrix2d::Identity();

  const gtsam::Vector2 predicted_delta =
      Lambda * R_ItoO * R_WI0.transpose() *
      (p_I1inW + R_WI1 * p_OinI - p_I0inW - R_WI0 * p_OinI);
  return wheel_delta - predicted_delta;
}

gtsam::Vector WheelFactor::evaluateError(const gtsam::Pose3 &pose0,
                                         const gtsam::Pose3 &pose1,
                                         const gtsam::Pose2 &wheel_pose_in_imu,
                                         const gtsam::Point2 &wheel_scales,
                                         boost::optional<gtsam::Matrix &> H1,
                                         boost::optional<gtsam::Matrix &> H2,
                                         boost::optional<gtsam::Matrix &> H3,
                                         boost::optional<gtsam::Matrix &> H4) const
{
  if (H1)
  {
    *H1 = gtsam::numericalDerivative41<gtsam::Vector2, gtsam::Pose3, gtsam::Pose3, gtsam::Pose2, gtsam::Point2>(
        [this](const gtsam::Pose3 &p0, const gtsam::Pose3 &p1, const gtsam::Pose2 &ext, const gtsam::Point2 &scale)
        { return evaluateResidual(p0, p1, ext, scale); },
        pose0, pose1, wheel_pose_in_imu, wheel_scales);
  }
  if (H2)
  {
    *H2 = gtsam::numericalDerivative42<gtsam::Vector2, gtsam::Pose3, gtsam::Pose3, gtsam::Pose2, gtsam::Point2>(
        [this](const gtsam::Pose3 &p0, const gtsam::Pose3 &p1, const gtsam::Pose2 &ext, const gtsam::Point2 &scale)
        { return evaluateResidual(p0, p1, ext, scale); },
        pose0, pose1, wheel_pose_in_imu, wheel_scales);
  }
  if (H3)
  {
    *H3 = gtsam::numericalDerivative43<gtsam::Vector2, gtsam::Pose3, gtsam::Pose3, gtsam::Pose2, gtsam::Point2>(
        [this](const gtsam::Pose3 &p0, const gtsam::Pose3 &p1, const gtsam::Pose2 &ext, const gtsam::Point2 &scale)
        { return evaluateResidual(p0, p1, ext, scale); },
        pose0, pose1, wheel_pose_in_imu, wheel_scales);
  }
  if (H4)
  {
    *H4 = gtsam::numericalDerivative44<gtsam::Vector2, gtsam::Pose3, gtsam::Pose3, gtsam::Pose2, gtsam::Point2>(
        [this](const gtsam::Pose3 &p0, const gtsam::Pose3 &p1, const gtsam::Pose2 &ext, const gtsam::Point2 &scale)
        { return evaluateResidual(p0, p1, ext, scale); },
        pose0, pose1, wheel_pose_in_imu, wheel_scales);
  }

  return evaluateResidual(pose0, pose1, wheel_pose_in_imu, wheel_scales);
}

LidarFactor::LidarFactor(gtsam::Key pose_key,
                         const optimizeLidarObs &observation,
                         const gtsam::Pose3 &lidar_pose_in_imu,
                         const gtsam::SharedNoiseModel &noise_model)
    : Base(noise_model, pose_key),
      observation_(observation),
      lidar_pose_in_imu_(lidar_pose_in_imu)
{
}

gtsam::Vector1 LidarFactor::evaluateResidual(const gtsam::Pose3 &imu_pose_in_world) const
{
  const gtsam::Point3 point_in_lidar(observation_.point_in_lidar.x(),
                                     observation_.point_in_lidar.y(),
                                     observation_.point_in_lidar.z());
  const gtsam::Point3 point_in_imu = lidar_pose_in_imu_.transformFrom(point_in_lidar);
  const gtsam::Point3 point_in_world = imu_pose_in_world.transformFrom(point_in_imu);

  const double residual = observation_.plane_unit_normal_in_world.dot(point_in_world) +
                          observation_.plane_offset;
  return (gtsam::Vector1() << residual).finished();
}

gtsam::Vector LidarFactor::evaluateError(const gtsam::Pose3 &imu_pose_in_world,
                                         boost::optional<gtsam::Matrix &> H) const
{
  if (H)
  {
    *H = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(
        [this](const gtsam::Pose3 &pose)
        { return evaluateResidual(pose); },
        imu_pose_in_world);
  }
  return evaluateResidual(imu_pose_in_world);
}

LidarImuWheelInitGraph::LidarImuWheelInitGraph(const InitGraphConfig &config)
    : config_(config),
      wheel_extrinsic_key_(gtsam::Symbol('e', 0)),
      wheel_scale_key_(gtsam::Symbol('s', 0))
{
}

void LidarImuWheelInitGraph::Reset()
{
  graph_.resize(0);
  initial_values_.clear();
  keyframe_count_ = 0;
  calibration_priors_added_ = false;
}

void LidarImuWheelInitGraph::addCalibrationPriors()
{
  if (calibration_priors_added_)
    return;

  if (!initial_values_.exists(wheel_extrinsic_key_))
    initial_values_.insert(wheel_extrinsic_key_, config_.initial_wheel_pose_in_imu);
  if (!initial_values_.exists(wheel_scale_key_))
    initial_values_.insert(wheel_scale_key_, config_.initial_wheel_scales);

  graph_.add(gtsam::PriorFactor<gtsam::Pose2>(
      wheel_extrinsic_key_,
      config_.initial_wheel_pose_in_imu,
      gtsam::noiseModel::Diagonal::Sigmas(config_.wheel_extrinsic_prior_sigma)));
  graph_.add(gtsam::PriorFactor<gtsam::Point2>(
      wheel_scale_key_,
      config_.initial_wheel_scales,
      gtsam::noiseModel::Diagonal::Sigmas(config_.wheel_scale_prior_sigma)));

  calibration_priors_added_ = true;
}

void LidarImuWheelInitGraph::addFirstStatePriors(std::size_t frame_idx,
                                                 const LWIKeyframe &keyframe)
{
  const gtsam::Key pose_key = gtsam::Symbol('x', frame_idx);
  const gtsam::Key vel_key = gtsam::Symbol('v', frame_idx);
  const gtsam::Key bias_key = gtsam::Symbol('b', frame_idx);

  graph_.add(gtsam::PriorFactor<gtsam::Pose3>(
      pose_key, keyframe.initial_pose,
      gtsam::noiseModel::Diagonal::Sigmas(config_.first_pose_prior_sigma)));
  graph_.add(gtsam::PriorFactor<gtsam::Vector3>(
      vel_key, keyframe.initial_velocity,
      gtsam::noiseModel::Diagonal::Sigmas(config_.first_velocity_prior_sigma)));
  graph_.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
      bias_key, keyframe.initial_bias,
      gtsam::noiseModel::Diagonal::Sigmas(config_.first_bias_prior_sigma)));
}

void LidarImuWheelInitGraph::addImuFactor(std::size_t frame_idx,
                                          const LWIKeyframe &keyframe)
{
  if (frame_idx == 0 || !keyframe.imu_preintegration)
    return;

  graph_.add(gtsam::CombinedImuFactor(
      gtsam::Symbol('x', frame_idx - 1),
      gtsam::Symbol('v', frame_idx - 1),
      gtsam::Symbol('x', frame_idx),
      gtsam::Symbol('v', frame_idx),
      gtsam::Symbol('b', frame_idx - 1),
      gtsam::Symbol('b', frame_idx),
      *keyframe.imu_preintegration));
}

void LidarImuWheelInitGraph::addWheelFactor(std::size_t frame_idx,
                                            const LWIKeyframe &keyframe)
{
  if (frame_idx == 0 || keyframe.wheel_msgs.size() < 2)
    return;

  graph_.add(boost::make_shared<WheelFactor>(
      gtsam::Symbol('x', frame_idx - 1),
      gtsam::Symbol('x', frame_idx),
      wheel_extrinsic_key_,
      wheel_scale_key_,
      keyframe.wheel_msgs,
      gtsam::noiseModel::Diagonal::Sigmas(config_.wheel_factor_sigma)));
}

void LidarImuWheelInitGraph::addLidarFactor(std::size_t frame_idx,
                                            const LWIKeyframe &keyframe)
{
  const gtsam::Key pose_key = gtsam::Symbol('x', frame_idx);
  for (const auto &observation : keyframe.lidar_obs)
  {
    graph_.add(boost::make_shared<LidarFactor>(
        pose_key,
        observation,
        config_.lidar_pose_in_imu,
        gtsam::noiseModel::Isotropic::Sigma(1, config_.lidar_factor_sigma)));
  }
}

void LidarImuWheelInitGraph::AddKeyframe(const LWIKeyframe &keyframe)
{
  addCalibrationPriors();

  const std::size_t frame_idx = keyframe_count_;
  const gtsam::Key pose_key = gtsam::Symbol('x', frame_idx);
  const gtsam::Key vel_key = gtsam::Symbol('v', frame_idx);
  const gtsam::Key bias_key = gtsam::Symbol('b', frame_idx);

  initial_values_.insert(pose_key, keyframe.initial_pose);
  initial_values_.insert(vel_key, keyframe.initial_velocity);
  initial_values_.insert(bias_key, keyframe.initial_bias);

  if (frame_idx == 0)
    addFirstStatePriors(frame_idx, keyframe);
  else
    addImuFactor(frame_idx, keyframe);

  addWheelFactor(frame_idx, keyframe);
  addLidarFactor(frame_idx, keyframe);
  ++keyframe_count_;
}

InitGraphResult LidarImuWheelInitGraph::Optimize(const gtsam::LevenbergMarquardtParams &params)
{
  gtsam::LevenbergMarquardtOptimizer optimizer(graph_, initial_values_, params);
  InitGraphResult result;
  result.values = optimizer.optimize();
  initial_values_ = result.values;
  result.wheel_pose_in_imu = result.values.at<gtsam::Pose2>(wheel_extrinsic_key_);
  result.wheel_scales = result.values.at<gtsam::Point2>(wheel_scale_key_);
  return result;
}
