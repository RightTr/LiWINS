#pragma once

#include <omp.h>
#include <mutex>
#include <csignal>
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include "ros_interface/ros_interface_base.h"
#include "filter/filter.h"
#include "map/pointMapBase.hpp"
#include "map/octvox/octvox.hpp"
#include "map/ikd-Tree/ikd_Tree.hpp"
#include "lio_sam/map_optimization.h"
#include "lio_sam/utility.h"

template<typename MsgPtr>
inline double get_ros_time(const MsgPtr& msg)
{
#ifdef USE_ROS1
    return msg->header.stamp.toSec();
#elif defined(USE_ROS2)
    return msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
#else
    return 0.0;
#endif
}

#define INIT_TIME       (0.1)
#define LASER_POINT_COV (0.001)
#define PUBFRAME_PERIOD (20)

class LaserMapping
{
public:
    LaserMapping() = default;
    ~LaserMapping() = default;

    void set_ros_interface(ROSInterfaceBase* ptr) { ros_interface_ = ptr; }
    void init();
    void run();
    void set_exit() { flg_exit_ = true; }

    // Called by file-scope EKF callback
    void h_share_model_impl(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data);

private:
    ROSInterfaceBase* ros_interface_ = nullptr;

    // EKF
    MeasureGroup Measures_;
    esekfom::esekf<state_ikfom, 12, input_ikfom> kf_;
    state_ikfom state_point_;
    vect3 pos_lid_;

    // Point clouds
    PointCloudXYZI::Ptr featsFromMap_{new PointCloudXYZI()};
    PointCloudXYZI::Ptr feats_undistort_{new PointCloudXYZI()};
    PointCloudXYZI::Ptr feats_down_body_{new PointCloudXYZI()};
    PointCloudXYZI::Ptr feats_down_world_{new PointCloudXYZI()};
    PointCloudXYZI::Ptr normvec_{new PointCloudXYZI(100000, 1)};
    PointCloudXYZI::Ptr laserCloudOri_{new PointCloudXYZI(100000, 1)};
    PointCloudXYZI::Ptr corr_normvect_{new PointCloudXYZI(100000, 1)};
    PointCloudXYZI::Ptr _featsArray_;

    // Map
    PointMapBase<PointType>::Ptr point_map_{std::make_shared<OctVoxMap<PointType, float>>()};
    pcl::VoxelGrid<PointType> downSizeFilterSurf_;

    // Algorithm state flags
    bool flg_first_scan_      = true;
    bool flg_exit_            = false;
    bool flg_EKF_inited_      = false;
    bool lidar_pushed_        = false;
    bool Localmap_Initialized_= false;

    // Timing & counting
    double lidar_end_time_    = 0.0;
    double first_lidar_time_  = 0.0;
    double lidar_mean_scantime_ = 0.0;
    int    scan_num_          = 0;
    int    feats_down_size_   = 0;
    int    effct_feat_num_    = 0;

    // FOV
    double HALF_FOV_COS_      = 0.0;
    double FOV_DEG_           = 0.0;

    // Extrinsics
    V3D Lidar_T_wrt_IMU_{Zero3d};
    M3D Lidar_R_wrt_IMU_{Eye3d};
    V3D euler_cur_;
    V3D position_last_{Zero3d};
    V3D XAxisPoint_body_{LIDAR_SP_LEN, 0.0, 0.0};
    V3D XAxisPoint_world_{LIDAR_SP_LEN, 0.0, 0.0};

    // Local map box
    BoxPointType LocalMap_Points_;

    // Per-point arrays
    bool  point_selected_surf_[100000] = {0};
    float res_last_[100000]            = {0.0};
    std::vector<std::vector<int>> pointSearchInd_surf_;
    std::vector<BoxPointType>     cub_needrm_;
    std::vector<PointVector>      Nearest_Points_;

    // Residual stats
    double res_mean_last_  = 0.05;
    double total_residual_ = 0.0;

    // Core algorithm methods
    bool sync_packages(MeasureGroup& meas);
    void lasermap_fov_segment();
    void map_incremental();

    // Point transforms (depend on current state_point_)
    void pointBodyToWorld(PointType const* pi, PointType* po);
    template<typename T>
    void pointBodyToWorld(const Eigen::Matrix<T,3,1>& pi, Eigen::Matrix<T,3,1>& po);
    void pointBodyToWorld_ikfom(PointType const* pi, PointType* po, state_ikfom& s);
    void RGBpointBodyToWorld(PointType const* pi, PointType* po);
    void RGBpointBodyLidarToIMU(PointType const* pi, PointType* po);

    // Helpers to build ROS-agnostic data structs
    OdomData make_odom_data();
    PoseData make_pose_data();

    // SAM helpers
    void getCurrPose(const state_ikfom& s);
    void getCurrOffset(const state_ikfom& s);
    void update_state_ikfom();

};
