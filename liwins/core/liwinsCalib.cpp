#include "core/liwinsCalib.h"

#include <cmath>
#include <vector>

#include <gtsam/geometry/Rot3.h>

namespace
{

constexpr double kFixedWheelTranslationSigma = 1e-9;

gtsam::Pose3 state_to_gtsam_pose3(const state_ikfom &state)
{
  return gtsam::Pose3(
      gtsam::Rot3(state.rot.toRotationMatrix()),
      gtsam::Point3(state.pos[0], state.pos[1], state.pos[2]));
}

gtsam::Vector3 state_to_gtsam_velocity(const state_ikfom &state)
{
  return gtsam::Vector3(state.vel[0], state.vel[1], state.vel[2]);
}

gtsam::imuBias::ConstantBias state_to_gtsam_bias(const state_ikfom &state)
{
  return gtsam::imuBias::ConstantBias(
      gtsam::Vector3(state.ba[0], state.ba[1], state.ba[2]),
      gtsam::Vector3(state.bg[0], state.bg[1], state.bg[2]));
}

} // namespace

void LIWINSCalib::init()
{
  std::vector<double> first_pose_prior_sigma;
  std::vector<double> first_velocity_prior_sigma;
  std::vector<double> first_bias_prior_sigma;
  std::vector<double> wheel_extrinsic_prior_sigma;
  std::vector<double> wheel_scale_prior_sigma;
  std::vector<double> wheel_factor_sigma;

  rosparam_get("calib/min_init_frames", min_init_frames_, 10);
  rosparam_get("calib/optimize_max_iterations", optimize_max_iterations_, 30);
  rosparam_get("calib/lidar_point_cov", lidar_point_cov_, 0.001);
  rosparam_get("calib/first_pose_prior_sigma", first_pose_prior_sigma, std::vector<double>{});
  rosparam_get("calib/first_velocity_prior_sigma", first_velocity_prior_sigma, std::vector<double>{});
  rosparam_get("calib/first_bias_prior_sigma", first_bias_prior_sigma, std::vector<double>{});
  rosparam_get("calib/wheel_extrinsic_prior_sigma", wheel_extrinsic_prior_sigma, std::vector<double>{});
  rosparam_get("calib/wheel_scale_prior_sigma", wheel_scale_prior_sigma, std::vector<double>{});
  rosparam_get("calib/wheel_factor_sigma", wheel_factor_sigma, std::vector<double>{});

  downSizeFilterSurf_.setLeafSize(
      filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);

  Lidar_T_wrt_IMU_ << VEC_FROM_ARRAY(Lidar_extrinT);
  Lidar_R_wrt_IMU_ << MAT_FROM_ARRAY(Lidar_extrinR);
  Wheel_T_wrt_IMU_ << wheel_extrinT[0], wheel_extrinT[1], 0.0;
  Wheel_R_wrt_IMU_ =
      Eigen::AngleAxisd(wheel_extrinTheta, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  if (imu_flip_en)
  {
    Lidar_T_wrt_IMU_ = IMU_FLIP_R * Lidar_T_wrt_IMU_;
    Lidar_R_wrt_IMU_ = IMU_FLIP_R * Lidar_R_wrt_IMU_;
    Wheel_T_wrt_IMU_ = IMU_FLIP_R * Wheel_T_wrt_IMU_;
    Wheel_R_wrt_IMU_ = IMU_FLIP_R * Wheel_R_wrt_IMU_;
  }

  p_imu->set_extrinsic(Lidar_T_wrt_IMU_, Lidar_R_wrt_IMU_);
  p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
  p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
  p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
  p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));
  p_imu->lidar_type = lidar_type;
  p_imu->Reset();

  init_state();

  graph_config_.lidar_pose_in_imu =
      gtsam::Pose3(gtsam::Rot3(Lidar_R_wrt_IMU_),
                   gtsam::Point3(Lidar_T_wrt_IMU_.x(), Lidar_T_wrt_IMU_.y(), Lidar_T_wrt_IMU_.z()));
  const double wheel_theta_in_imu = std::atan2(Wheel_R_wrt_IMU_(1, 0), Wheel_R_wrt_IMU_(0, 0));
  graph_config_.initial_wheel_pose_in_imu =
      gtsam::Pose2(Wheel_T_wrt_IMU_.x(), Wheel_T_wrt_IMU_.y(), wheel_theta_in_imu);
  graph_config_.initial_wheel_scales = gtsam::Point2(wheel_sr, wheel_sl);
  graph_config_.gyro_noise_sigma = std::sqrt(gyr_cov);
  graph_config_.accel_noise_sigma = std::sqrt(acc_cov);
  graph_config_.gyro_bias_rw_sigma = std::sqrt(b_gyr_cov);
  graph_config_.accel_bias_rw_sigma = std::sqrt(b_acc_cov);
  graph_config_.first_pose_prior_sigma << first_pose_prior_sigma[0], first_pose_prior_sigma[1],
      first_pose_prior_sigma[2], first_pose_prior_sigma[3], first_pose_prior_sigma[4],
      first_pose_prior_sigma[5];
  graph_config_.first_velocity_prior_sigma << first_velocity_prior_sigma[0],
      first_velocity_prior_sigma[1], first_velocity_prior_sigma[2];
  graph_config_.first_bias_prior_sigma << first_bias_prior_sigma[0], first_bias_prior_sigma[1],
      first_bias_prior_sigma[2], first_bias_prior_sigma[3], first_bias_prior_sigma[4],
      first_bias_prior_sigma[5];
  graph_config_.wheel_extrinsic_prior_sigma << kFixedWheelTranslationSigma,
      kFixedWheelTranslationSigma, wheel_extrinsic_prior_sigma[2];
  graph_config_.wheel_scale_prior_sigma << wheel_scale_prior_sigma[0], wheel_scale_prior_sigma[1];
  graph_config_.wheel_factor_sigma << wheel_factor_sigma[0], wheel_factor_sigma[1];

  ROS_PRINT_INFO(
      "LIW calib config: min_init_frames=%d max_iterations=%d lidar_point_cov=%.6f",
      min_init_frames_,
      optimize_max_iterations_,
      lidar_point_cov_);
  ROS_PRINT_INFO(
      "LIW calib sigma: wheel_ext=[%.2e %.2e %.4f] wheel_scale=[%.4f %.4f] wheel_factor=[%.4f %.4f]",
      graph_config_.wheel_extrinsic_prior_sigma[0],
      graph_config_.wheel_extrinsic_prior_sigma[1],
      graph_config_.wheel_extrinsic_prior_sigma[2],
      graph_config_.wheel_scale_prior_sigma[0],
      graph_config_.wheel_scale_prior_sigma[1],
      graph_config_.wheel_factor_sigma[0],
      graph_config_.wheel_factor_sigma[1]);

  init_graph_ = std::make_unique<LidarImuWheelInitGraph>(graph_config_);
}

void LIWINSCalib::init_state()
{
  state_curr_ = state_ikfom();
  state_curr_.pos = Zero3d;
  state_curr_.vel = Zero3d;
  state_curr_.rot = Eigen::Quaterniond::Identity();
  state_curr_.bg = Zero3d;
  state_curr_.ba = Zero3d;
  state_curr_.grav = S2(V3D(0.0, 0.0, -G_m_s2));
  state_curr_.offset_T_L_I = Lidar_T_wrt_IMU_;
  state_curr_.offset_R_L_I = Lidar_R_wrt_IMU_;
}

OdomData LIWINSCalib::make_odom_data() const
{
  OdomData odom;
  odom.x = state_curr_.pos.x();
  odom.y = state_curr_.pos.y();
  odom.z = state_curr_.pos.z();
  odom.qx = state_curr_.rot.coeffs()[0];
  odom.qy = state_curr_.rot.coeffs()[1];
  odom.qz = state_curr_.rot.coeffs()[2];
  odom.qw = state_curr_.rot.coeffs()[3];
  odom.timestamp = lidar_end_time_;
  return odom;
}

PoseData LIWINSCalib::make_pose_data() const
{
  PoseData pose;
  pose.x = state_curr_.pos.x();
  pose.y = state_curr_.pos.y();
  pose.z = state_curr_.pos.z();
  pose.qx = state_curr_.rot.coeffs()[0];
  pose.qy = state_curr_.rot.coeffs()[1];
  pose.qz = state_curr_.rot.coeffs()[2];
  pose.qw = state_curr_.rot.coeffs()[3];
  pose.timestamp = lidar_end_time_;
  return pose;
}

void LIWINSCalib::run()
{
  while (ros_ok())
  {
    if (flg_exit_)
      break;

    spin_once();
    if (!sync_packages(Measures_))
      continue;

    if (flg_first_scan_)
    {
      first_lidar_time_ = Measures_.lidar_beg_time;
      p_imu->first_lidar_time = first_lidar_time_;
      flg_first_scan_ = false;
      continue;
    }

    const double integration_beg_time =
        last_integration_end_time_ > 0.0 ? last_integration_end_time_ : Measures_.lidar_beg_time;
    state_ikfom end_state = state_curr_;
    std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> imu_preintegration;
    p_imu->ProcessPureIntegration(
        Measures_, state_curr_, end_state, feats_undistort_, &imu_pose_traj_, &imu_preintegration);
    state_curr_ = end_state;

    downSizeFilterSurf_.setInputCloud(feats_undistort_);
    downSizeFilterSurf_.filter(*feats_down_body_);

    std::vector<optimizeLidarObs> obs;
    build_lidarObs(obs);
    append_keyframe(obs, imu_preintegration);

    optimize();
    map_incremental();

    publish_odometry(make_odom_data(), lidar_end_time_);
    publish_path(make_pose_data(), lidar_end_time_);
    publish_world_cloud(dense_pub_en ? feats_undistort_ : feats_down_body_, lidar_end_time_, "camera_init", state_curr_);
    if (scan_body_pub_en) publish_body_cloud(feats_undistort_, lidar_end_time_, "body", state_curr_);
    last_integration_end_time_ = lidar_end_time_;
  }
}

bool LIWINSCalib::sync_packages(MeasureGroup &meas)
{
  if (lidar_buffer.empty() || imu_buffer.empty())
    return false;

  if (!lidar_pushed_)
  {
    meas.lidar = lidar_buffer.front();
    meas.lidar_beg_time = time_buffer.front();

    if (meas.lidar->points.size() <= 1)
    {
      lidar_end_time_ = meas.lidar_beg_time + lidar_mean_scantime_;
    }
    else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime_)
    {
      lidar_end_time_ = meas.lidar_beg_time + lidar_mean_scantime_;
    }
    else
    {
      scan_num_++;
      lidar_end_time_ = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
      lidar_mean_scantime_ +=
          (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime_) / scan_num_;
    }

    meas.lidar_end_time = lidar_end_time_;
    lidar_pushed_ = true;
  }

  if (last_timestamp_imu < lidar_end_time_)
    return false;

  double imu_time = get_ros_time_sec(imu_buffer.front()->header.stamp);
  meas.imu.clear();
  meas.wheel.clear();

  while (!imu_buffer.empty() && imu_time < lidar_end_time_)
  {
    imu_time = get_ros_time_sec(imu_buffer.front()->header.stamp);
    if (imu_time > lidar_end_time_)
      break;
    meas.imu.push_back(imu_buffer.front());
    imu_buffer.pop_front();
  }

  while (!wheel_buffer.empty() && wheel_buffer.front()->timestamp <= lidar_end_time_)
  {
    meas.wheel.push_back(wheel_buffer.front());
    wheel_buffer.pop_front();
  }

  if (wheel_en && !wheel_buffer.empty())
  {
    meas.wheel.push_back(wheel_buffer.front());
    wheel_buffer.pop_front();
  }

  lidar_buffer.pop_front();
  time_buffer.pop_front();
  lidar_pushed_ = false;
  return true;
}

void LIWINSCalib::build_lidarObs(
    std::vector<optimizeLidarObs> &obs)
{
  obs.clear();
  if (point_map_->Size() < NUM_MATCH_POINTS)
    return;

  feats_down_world_->clear();
  feats_down_world_->resize(feats_down_body_->size());
  nearest_points_.assign(feats_down_body_->size(), PointVector());

  for (std::size_t i = 0; i < feats_down_body_->size(); ++i)
  {
    pointBodyToWorld(&feats_down_body_->points[i], &feats_down_world_->points[i], state_curr_);

    std::vector<float> point_search_sq_dis(NUM_MATCH_POINTS);
    auto &points_near = nearest_points_[i];
    point_map_->Nearest_Search(
        feats_down_world_->points[i], NUM_MATCH_POINTS, points_near, point_search_sq_dis);

    if (points_near.size() < NUM_MATCH_POINTS || point_search_sq_dis[NUM_MATCH_POINTS - 1] > 5.0f)
      continue;

    VF(4) plane_coeff;
    if (!esti_plane(plane_coeff, points_near, 0.1f))
      continue;

    const PointType &point_world = feats_down_world_->points[i];
    const PointType &point_body = feats_down_body_->points[i];
    const float point_plane_distance =
        plane_coeff(0) * point_world.x +
        plane_coeff(1) * point_world.y +
        plane_coeff(2) * point_world.z +
        plane_coeff(3);
    const float score = 1.0f - 0.9f * std::fabs(point_plane_distance) /
                                   std::sqrt(std::max(1e-6f, point_body.getVector3fMap().norm()));
    if (score <= 0.9f)
      continue;

    optimizeLidarObs observation;
    observation.point_in_lidar = V3D(point_body.x, point_body.y, point_body.z);
    observation.plane_unit_normal_in_world =
        V3D(plane_coeff(0), plane_coeff(1), plane_coeff(2));
    observation.plane_offset = plane_coeff(3);
    observation.sigma = lidar_point_cov_;
    obs.push_back(observation);
  }
}

void LIWINSCalib::append_keyframe(
    const std::vector<optimizeLidarObs> &obs,
    const std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> &imu_preintegration)
{
  InitKeyframe keyframe;
  keyframe.timestamp = lidar_end_time_;
  keyframe.initial_pose = state_to_gtsam_pose3(state_curr_);
  keyframe.initial_velocity = state_to_gtsam_velocity(state_curr_);
  keyframe.initial_bias = state_to_gtsam_bias(state_curr_);
  keyframe.imu_preintegration = imu_preintegration;
  keyframe.lidar_obs = obs;

  keyframe.wheel_msgs.assign(Measures_.wheel.begin(), Measures_.wheel.end());

  init_graph_->AddKeyframe(keyframe);
}

void LIWINSCalib::apply_latest_optimized_state()
{
  if (!has_result_ || init_graph_->keyframe_count() == 0)
    return;

  const std::size_t frame_idx = init_graph_->keyframe_count() - 1;
  const gtsam::Pose3 latest_pose =
      result_.values.at<gtsam::Pose3>(gtsam::Symbol('x', frame_idx));
  const gtsam::Vector3 latest_vel =
      result_.values.at<gtsam::Vector3>(gtsam::Symbol('v', frame_idx));
  const gtsam::imuBias::ConstantBias latest_bias =
      result_.values.at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b', frame_idx));

  state_curr_.pos = V3D(
      latest_pose.translation().x(),
      latest_pose.translation().y(),
      latest_pose.translation().z());
  state_curr_.rot = Eigen::Quaterniond(latest_pose.rotation().matrix());
  state_curr_.rot.normalize();
  state_curr_.vel = V3D(latest_vel.x(), latest_vel.y(), latest_vel.z());
  state_curr_.ba = V3D(
      latest_bias.accelerometer().x(),
      latest_bias.accelerometer().y(),
      latest_bias.accelerometer().z());
  state_curr_.bg = V3D(
      latest_bias.gyroscope().x(),
      latest_bias.gyroscope().y(),
      latest_bias.gyroscope().z());
}

void LIWINSCalib::map_incremental()
{
  if (feats_down_body_->empty())
    return;

  PointVector points_to_add;
  points_to_add.reserve(feats_down_body_->size());
  feats_down_world_->resize(feats_down_body_->size());
  for (std::size_t i = 0; i < feats_down_body_->size(); ++i)
  {
    pointBodyToWorld(&feats_down_body_->points[i], &feats_down_world_->points[i], state_curr_);
    points_to_add.push_back(feats_down_world_->points[i]);
  }

  if (!point_map_->IsInitialized())
  {
    PointMapBase<PointType>::MapConfig map_config;
    point_map_->SetConfig(map_config);
    point_map_->Build(points_to_add);
    return;
  }

  point_map_->InsertPoints(points_to_add, true);
}

void LIWINSCalib::optimize()
{
  if (init_graph_->keyframe_count() == 0)
    return;

  auto lm_params = gtsam::LevenbergMarquardtParams();
  lm_params.maxIterations = optimize_max_iterations_;
  const bool first_optimization = !has_result_;
  result_ = init_graph_->Optimize(lm_params);
  has_result_ = true;
  apply_latest_optimized_state();
  if (first_optimization)
  {
    ROS_PRINT_INFO(
        "LIW calib initialized: tx=%.6f ty=%.6f theta=%.6f sr=%.6f sl=%.6f",
        result_.wheel_pose_in_imu.x(),
        result_.wheel_pose_in_imu.y(),
        result_.wheel_pose_in_imu.theta(),
        result_.wheel_scales.x(),
        result_.wheel_scales.y());
    return;
  }

  ROS_PRINT_INFO(
      "LIW calib updated: tx=%.6f ty=%.6f theta=%.6f sr=%.6f sl=%.6f",
      result_.wheel_pose_in_imu.x(),
      result_.wheel_pose_in_imu.y(),
      result_.wheel_pose_in_imu.theta(),
      result_.wheel_scales.x(),
      result_.wheel_scales.y());
}
