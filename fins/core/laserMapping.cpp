#include "laser_mapping.h"
#include <math.h>
#include <thread>

// File-scope pointer so the EKF's C-style callback can reach the class instance
static LaserMapping* g_laser_mapping = nullptr;

static void h_share_model_cb(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data)
{
    g_laser_mapping->h_share_model_impl(s, ekfom_data);
}

// ---------------------------------------------------------------------------
// init
// ---------------------------------------------------------------------------
void LaserMapping::init()
{
    auto* ri = ros_interface_;
    auto* p_imu = ri->p_imu.get();

    FOV_DEG_      = (ri->fov_deg + 10.0) > 179.9 ? 179.9 : (ri->fov_deg + 10.0);
    HALF_FOV_COS_ = cos(FOV_DEG_ * 0.5 * PI_M / 180.0);

    _featsArray_.reset(new PointCloudXYZI());

    memset(point_selected_surf_, true,    sizeof(point_selected_surf_));
    memset(res_last_,            -1000.0f, sizeof(res_last_));

    downSizeFilterSurf_.setLeafSize(
        ri->filter_size_surf_min, ri->filter_size_surf_min, ri->filter_size_surf_min);
    
    Lidar_T_wrt_IMU_ << VEC_FROM_ARRAY(ri->extrinT);
    Lidar_R_wrt_IMU_ << MAT_FROM_ARRAY(ri->extrinR);
    if (ri->imu_flip_en)
    {
        // IMU frame inversion: coordinates in IMU frame should be transformed consistently.
        Lidar_T_wrt_IMU_ = -Lidar_T_wrt_IMU_;
        Lidar_R_wrt_IMU_ = -Lidar_R_wrt_IMU_;
    }

    p_imu->set_extrinsic(Lidar_T_wrt_IMU_, Lidar_R_wrt_IMU_);
    p_imu->set_gyr_cov(V3D(ri->gyr_cov,   ri->gyr_cov,   ri->gyr_cov));
    p_imu->set_acc_cov(V3D(ri->acc_cov,   ri->acc_cov,   ri->acc_cov));
    p_imu->set_gyr_bias_cov(V3D(ri->b_gyr_cov, ri->b_gyr_cov, ri->b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(ri->b_acc_cov, ri->b_acc_cov, ri->b_acc_cov));
    p_imu->lidar_type = ri->lidar_type;
    p_imu->set_use_zupt(ri->use_zupt);
    p_imu->set_zupt_thresholds(ri->zupt_acc_norm_threshold, ri->zupt_gyro_threshold);

    g_laser_mapping = this;
    double epsi[23] = {0.001};
    fill(epsi, epsi + 23, 0.001);
    // Always bind one callback; choose lidar/zupt dynamically in h_share_model_impl.
    kf_.init_dyn_share(get_f, df_dx, df_dw, h_share_model_cb, ri->NUM_MAX_ITERATIONS, epsi);

    if (ri->sam_enable)
        MapOptimizationInit();
}

// ---------------------------------------------------------------------------
// run
// ---------------------------------------------------------------------------
void LaserMapping::run()
{
    auto* ri = ros_interface_;

    // High-frequency odometry publishing thread (driven by IMU rate)
    std::thread odomhighthread([&]() {
        while (!flg_exit_ && ri->ros_ok()) {
            Pose pose = ri->p_imu->pbuffer.Pop();
            if (flg_exit_ || !ri->ros_ok()) break;
            ri->publish_odometryhighfreq(pose);
        }
    });

    if (ri->sam_enable) {
        std::thread loopthread(&loopClosureThread);
        std::thread globalthread(&visualizeGlobalMapThread);
        loopthread.detach();
        globalthread.detach();
    }

    while (ri->ros_ok())
    {
        if (flg_exit_) break;
        ri->spin_once();

        // ---- Relocalization -----------------------------------------------
        if (ri->reloc_en && ri->relocalize_flag.load())
        {
            feats_down_world_->clear();
            ri->p_imu->Reset();
            state_ikfom state_point_reloc;
            {
                std::lock_guard<std::mutex> lock(ri->mtx_reloc);
                state_point_reloc.pos = Eigen::Vector3d(
                    ri->reloc_state.x_, ri->reloc_state.y_, ri->reloc_state.z_);
                state_point_reloc.rot = Eigen::Quaterniond(
                    ri->reloc_state.qw_, ri->reloc_state.qx_,
                    ri->reloc_state.qy_, ri->reloc_state.qz_);
            }
            state_point_reloc.rot.normalize();
            kf_.reset(state_point_reloc);
            point_map_->Clear();
            ri->relocalize_flag.store(false);
            flg_first_scan_ = true;
            continue;
        }

        if (!sync_packages(Measures_)) continue;

        // ---- First scan init ----------------------------------------------
        if (flg_first_scan_)
        {
            first_lidar_time_              = Measures_.lidar_beg_time;
            ri->p_imu->first_lidar_time   = first_lidar_time_;
            flg_first_scan_                = false;
            continue;
        }

        // ---- IMU undistortion ---------------------------------------------
        ri->p_imu->Process(Measures_, kf_, feats_undistort_);
        state_point_ = kf_.get_x();
        pos_lid_     = state_point_.pos + state_point_.rot * state_point_.offset_T_L_I;

        if (feats_undistort_->empty() || feats_undistort_ == nullptr) continue;

        flg_EKF_inited_ = (Measures_.lidar_beg_time - first_lidar_time_) < INIT_TIME ? false : true;

        // ---- FOV map management -------------------------------------------
        lasermap_fov_segment();

        // ---- Downsampling -------------------------------------------------
        downSizeFilterSurf_.setInputCloud(feats_undistort_);
        downSizeFilterSurf_.filter(*feats_down_body_);
        feats_down_size_ = feats_down_body_->points.size();

        // ---- Map initialization -------------------------------------------
        if (!point_map_->IsInitialized())
        {
            if (feats_down_size_ > 5)
            {
                PointMapBase<PointType>::MapConfig mapConfig;
                point_map_->SetConfig(mapConfig);
                feats_down_world_->resize(feats_down_size_);
                for (int i = 0; i < feats_down_size_; i++)
                    pointBodyToWorld(&feats_down_body_->points[i], &feats_down_world_->points[i]);
                point_map_->Build(feats_down_world_->points);
            }
            continue;
        }

        if (feats_down_size_ < 5) continue;

        normvec_->resize(feats_down_size_);
        feats_down_world_->resize(feats_down_size_);

        if (ri->feature_pub_en)
        {
            PointVector().swap(point_map_->PCL_Storage);
            point_map_->GetMap(point_map_->PCL_Storage);
            featsFromMap_->clear();
            featsFromMap_->points = point_map_->PCL_Storage;
        }

        pointSearchInd_surf_.resize(feats_down_size_);
        Nearest_Points_.resize(feats_down_size_);

        // ---- Iterated EKF update ------------------------------------------
        double solve_H_time = 0;
        kf_.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
        state_point_ = kf_.get_x();
        euler_cur_   = SO3ToEuler(state_point_.rot);
        pos_lid_     = state_point_.pos + state_point_.rot * state_point_.offset_T_L_I;

        // ---- LIO-SAM backend (optional) -----------------------------------
        if (ri->sam_enable)
        {
            getCurrPose(state_point_);
            getCurrOffset(state_point_);
            saveKeyFramesAndFactor(feats_undistort_);
            update_state_ikfom();
            correctPoses();
            publishSamMsg();
        }

        // ---- Publish odometry ---------------------------------------------
        ri->publish_odometry(make_odom_data(), lidar_end_time_);

        // ---- Map update ---------------------------------------------------
        map_incremental();

        // ---- Publish path -------------------------------------------------
        if (ri->path_en)
            ri->publish_path(make_pose_data(), lidar_end_time_);

        // ---- Publish world frame cloud ------------------------------------
        if (ri->scan_pub_en || ri->pcd_save_en)
        {
            PointCloudXYZI::Ptr src = ri->dense_pub_en ? feats_undistort_ : feats_down_body_;
            int sz = src->points.size();
            PointCloudXYZI::Ptr cloud_world(new PointCloudXYZI(sz, 1));
            for (int i = 0; i < sz; i++)
                RGBpointBodyToWorld(&src->points[i], &cloud_world->points[i]);
            ri->publish_world_cloud(cloud_world, lidar_end_time_, "camera_init");
        }

        if (ri->scan_pub_en && ri->scan_body_pub_en)
        {
            int sz = feats_undistort_->points.size();
            PointCloudXYZI::Ptr cloud_body(new PointCloudXYZI(sz, 1));
            for (int i = 0; i < sz; i++)
                RGBpointBodyLidarToIMU(&feats_undistort_->points[i], &cloud_body->points[i]);
            ri->publish_body_cloud(cloud_body, lidar_end_time_, "body");
        }

        if (ri->effect_pub_en)
        {
            PointCloudXYZI::Ptr cloud_eff(new PointCloudXYZI(effct_feat_num_, 1));
            for (int i = 0; i < effct_feat_num_; i++)
                RGBpointBodyToWorld(&laserCloudOri_->points[i], &cloud_eff->points[i]);
            ri->publish_effect_cloud(cloud_eff, lidar_end_time_, "camera_init");
        }

        if (ri->feature_pub_en)
            ri->publish_map_cloud(featsFromMap_, lidar_end_time_, "camera_init");
    }

    // Wake up the blocked Pop() so odomhighthread can exit cleanly
    ri->p_imu->pbuffer.Push(Pose{});
    odomhighthread.join();
}

// ---------------------------------------------------------------------------
// sync_packages
// ---------------------------------------------------------------------------
bool LaserMapping::sync_packages(MeasureGroup& meas)
{
    auto* ri = ros_interface_;

    if (ri->lidar_buffer.empty() || ri->imu_buffer.empty()) return false;

    if (!lidar_pushed_)
    {
        meas.lidar          = ri->lidar_buffer.front();
        meas.lidar_beg_time = ri->time_buffer.front();

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
            lidar_end_time_   = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime_ += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime_) / scan_num_;
        }

        if (ri->lidar_type == MARSIM)
            lidar_end_time_ = meas.lidar_beg_time;

        meas.lidar_end_time = lidar_end_time_;
        lidar_pushed_       = true;
    }

    if (ri->last_timestamp_imu < lidar_end_time_) return false;

    double imu_time = get_ros_time(ri->imu_buffer.front());

    meas.imu.clear();
    while (!ri->imu_buffer.empty() && (imu_time < lidar_end_time_))
    {
        imu_time = get_ros_time(ri->imu_buffer.front());
        if (imu_time > lidar_end_time_) break;
        meas.imu.push_back(ri->imu_buffer.front());
        ri->imu_buffer.pop_front();
    }

    ri->lidar_buffer.pop_front();
    ri->time_buffer.pop_front();
    lidar_pushed_ = false;
    setLaserCurTime(lidar_end_time_);
    return true;
}

// ---------------------------------------------------------------------------
// lasermap_fov_segment
// ---------------------------------------------------------------------------
void LaserMapping::lasermap_fov_segment()
{
    auto* ri = ros_interface_;
    constexpr float MOV_THRESHOLD = 1.5f;

    cub_needrm_.clear();
    pointBodyToWorld(XAxisPoint_body_, XAxisPoint_world_);
    V3D pos_LiD = pos_lid_;

    if (!Localmap_Initialized_)
    {
        for (int i = 0; i < 3; i++)
        {
            LocalMap_Points_.vertex_min[i] = pos_LiD(i) - ri->cube_len / 2.0;
            LocalMap_Points_.vertex_max[i] = pos_LiD(i) + ri->cube_len / 2.0;
        }
        Localmap_Initialized_ = true;
        return;
    }

    float dist_to_map_edge[3][2];
    bool  need_move = false;
    for (int i = 0; i < 3; i++)
    {
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points_.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points_.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * ri->DET_RANGE ||
            dist_to_map_edge[i][1] <= MOV_THRESHOLD * ri->DET_RANGE)
            need_move = true;
    }
    if (!need_move) return;

    BoxPointType New_LocalMap_Points = LocalMap_Points_;
    float mov_dist = std::max(
        (ri->cube_len - 2.0 * MOV_THRESHOLD * ri->DET_RANGE) * 0.5 * 0.9,
        double(ri->DET_RANGE * (MOV_THRESHOLD - 1)));

    for (int i = 0; i < 3; i++)
    {
        BoxPointType tmp_boxpoints = LocalMap_Points_;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * ri->DET_RANGE)
        {
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i]        = LocalMap_Points_.vertex_max[i] - mov_dist;
            cub_needrm_.push_back(tmp_boxpoints);
        }
        else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * ri->DET_RANGE)
        {
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i]        = LocalMap_Points_.vertex_min[i] + mov_dist;
            cub_needrm_.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points_ = New_LocalMap_Points;

    if (!cub_needrm_.empty())
        point_map_->Delete_Point_Boxes(cub_needrm_);
}

// ---------------------------------------------------------------------------
// map_incremental
// ---------------------------------------------------------------------------
void LaserMapping::map_incremental()
{
    const double fsz = ros_interface_->filter_size_map_min;

    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size_);
    PointNoNeedDownsample.reserve(feats_down_size_);

    for (int i = 0; i < feats_down_size_; i++)
    {
        pointBodyToWorld(&feats_down_body_->points[i], &feats_down_world_->points[i]);

        if (!Nearest_Points_[i].empty() && flg_EKF_inited_)
        {
            const PointVector& points_near = Nearest_Points_[i];
            PointType mid_point;
            mid_point.x = floor(feats_down_world_->points[i].x / fsz) * fsz + 0.5 * fsz;
            mid_point.y = floor(feats_down_world_->points[i].y / fsz) * fsz + 0.5 * fsz;
            mid_point.z = floor(feats_down_world_->points[i].z / fsz) * fsz + 0.5 * fsz;
            float dist  = calc_dist(feats_down_world_->points[i], mid_point);

            if (fabs(points_near[0].x - mid_point.x) > 0.5 * fsz &&
                fabs(points_near[0].y - mid_point.y) > 0.5 * fsz &&
                fabs(points_near[0].z - mid_point.z) > 0.5 * fsz)
            {
                PointNoNeedDownsample.push_back(feats_down_world_->points[i]);
                continue;
            }

            bool need_add = true;
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world_->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world_->points[i]);
        }
    }

    point_map_->InsertPoints(PointToAdd, true);
    point_map_->InsertPoints(PointNoNeedDownsample, false);
}

// ---------------------------------------------------------------------------
// h_share_model_impl  (EKF measurement model)
// ---------------------------------------------------------------------------
void LaserMapping::h_share_model_impl(state_ikfom& s,
                                      esekfom::dyn_share_datastruct<double>& ekfom_data)
{
    laserCloudOri_->clear();
    corr_normvect_->clear();
    total_residual_ = 0.0;

#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
    #pragma omp parallel for
#endif
    for (int i = 0; i < feats_down_size_; i++)
    {
        PointType& point_body  = feats_down_body_->points[i];
        PointType& point_world = feats_down_world_->points[i];

        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);
        point_world.x         = p_global(0);
        point_world.y         = p_global(1);
        point_world.z         = p_global(2);
        point_world.intensity = point_body.intensity;

        std::vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
        auto& points_near = Nearest_Points_[i];

        if (ekfom_data.converge)
        {
            point_map_->Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf_[i] =
                (points_near.size() < NUM_MATCH_POINTS) ? false :
                (pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5) ? false : true;
        }

        if (!point_selected_surf_[i]) continue;

        VF(4) pabcd;
        point_selected_surf_[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            float pd2   = pabcd(0) * point_world.x + pabcd(1) * point_world.y +
                          pabcd(2) * point_world.z + pabcd(3);
            float s_val = 1 - 0.9f * fabs(pd2) / sqrt(p_body.norm());
            if (s_val > 0.9)
            {
                point_selected_surf_[i]       = true;
                normvec_->points[i].x         = pabcd(0);
                normvec_->points[i].y         = pabcd(1);
                normvec_->points[i].z         = pabcd(2);
                normvec_->points[i].intensity = pd2;
                res_last_[i]                  = fabs(pd2);
            }
        }
    }

    effct_feat_num_ = 0;
    for (int i = 0; i < feats_down_size_; i++)
    {
        if (point_selected_surf_[i])
        {
            laserCloudOri_->points[effct_feat_num_]  = feats_down_body_->points[i];
            corr_normvect_->points[effct_feat_num_]  = normvec_->points[i];
            total_residual_ += res_last_[i];
            effct_feat_num_++;
        }
    }

    if (effct_feat_num_ < 1)
    {
        ekfom_data.valid = false;
        return;
    }

    res_mean_last_ = total_residual_ / effct_feat_num_;

    lidar_updater::build_measurement_model(
        s, laserCloudOri_, corr_normvect_, effct_feat_num_,
        ros_interface_->extrinsic_est_en, ekfom_data);
}

// ---------------------------------------------------------------------------
// Point transforms
// ---------------------------------------------------------------------------
void LaserMapping::pointBodyToWorld(PointType const* pi, PointType* po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point_.rot * (state_point_.offset_R_L_I * p_body +
                                     state_point_.offset_T_L_I) + state_point_.pos);
    po->x         = p_global(0);
    po->y         = p_global(1);
    po->z         = p_global(2);
    po->intensity = pi->intensity;
}

template<typename T>
void LaserMapping::pointBodyToWorld(const Eigen::Matrix<T, 3, 1>& pi,
                                    Eigen::Matrix<T, 3, 1>&       po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point_.rot * (state_point_.offset_R_L_I * p_body +
                                     state_point_.offset_T_L_I) + state_point_.pos);
    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void LaserMapping::pointBodyToWorld_ikfom(PointType const* pi, PointType* po, state_ikfom& s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);
    po->x         = p_global(0);
    po->y         = p_global(1);
    po->z         = p_global(2);
    po->intensity = pi->intensity;
}

void LaserMapping::RGBpointBodyToWorld(PointType const* pi, PointType* po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point_.rot * (state_point_.offset_R_L_I * p_body +
                                     state_point_.offset_T_L_I) + state_point_.pos);
    po->x         = p_global(0);
    po->y         = p_global(1);
    po->z         = p_global(2);
    po->intensity = pi->intensity;
}

void LaserMapping::RGBpointBodyLidarToIMU(PointType const* pi, PointType* po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point_.offset_R_L_I * p_body_lidar + state_point_.offset_T_L_I);
    po->x         = p_body_imu(0);
    po->y         = p_body_imu(1);
    po->z         = p_body_imu(2);
    po->intensity = pi->intensity;
}

// ---------------------------------------------------------------------------
// Build ROS-agnostic output structs
// ---------------------------------------------------------------------------
OdomData LaserMapping::make_odom_data()
{
    OdomData odom;
    odom.x         = state_point_.pos(0);
    odom.y         = state_point_.pos(1);
    odom.z         = state_point_.pos(2);
    odom.qx        = state_point_.rot.coeffs()[0];
    odom.qy        = state_point_.rot.coeffs()[1];
    odom.qz        = state_point_.rot.coeffs()[2];
    odom.qw        = state_point_.rot.coeffs()[3];
    odom.timestamp = lidar_end_time_;

    auto P = kf_.get_P();
    for (int i = 0; i < 6; i++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odom.covariance[i*6 + 0] = P(k, 3);
        odom.covariance[i*6 + 1] = P(k, 4);
        odom.covariance[i*6 + 2] = P(k, 5);
        odom.covariance[i*6 + 3] = P(k, 0);
        odom.covariance[i*6 + 4] = P(k, 1);
        odom.covariance[i*6 + 5] = P(k, 2);
    }
    return odom;
}

PoseData LaserMapping::make_pose_data()
{
    PoseData pose;
    pose.x         = state_point_.pos(0);
    pose.y         = state_point_.pos(1);
    pose.z         = state_point_.pos(2);
    pose.qx        = state_point_.rot.coeffs()[0];
    pose.qy        = state_point_.rot.coeffs()[1];
    pose.qz        = state_point_.rot.coeffs()[2];
    pose.qw        = state_point_.rot.coeffs()[3];
    pose.timestamp = lidar_end_time_;
    return pose;
}

// ---------------------------------------------------------------------------
// LIO-SAM helpers
// ---------------------------------------------------------------------------
void LaserMapping::getCurrPose(const state_ikfom& s)
{
    Eigen::Vector3d eulerAngle = s.rot.matrix().eulerAngles(2, 1, 0);
    transformTobeMapped[0] = eulerAngle(2);
    transformTobeMapped[1] = eulerAngle(1);
    transformTobeMapped[2] = eulerAngle(0);
    transformTobeMapped[3] = s.pos(0);
    transformTobeMapped[4] = s.pos(1);
    transformTobeMapped[5] = s.pos(2);
}

void LaserMapping::getCurrOffset(const state_ikfom& s)
{
    translationLidarToIMU = s.offset_T_L_I;
    rotationLidarToIMU    = s.offset_R_L_I.toRotationMatrix();
}

void LaserMapping::update_state_ikfom()
{
    state_ikfom state_updated = kf_.get_x();
    Eigen::Vector3d pos(transformTobeMapped[3],
                        transformTobeMapped[4],
                        transformTobeMapped[5]);
    Eigen::Quaterniond q =
        Eigen::Quaterniond(
            Eigen::AngleAxisd(transformTobeMapped[2], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(transformTobeMapped[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(transformTobeMapped[0], Eigen::Vector3d::UnitX()));

    state_updated.pos = pos;
    state_updated.rot = q;
    state_point_      = state_updated;
    kf_.change_x(state_updated);
}
