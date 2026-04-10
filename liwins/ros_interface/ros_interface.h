#ifndef ROS_INTERFACE_H
#define ROS_INTERFACE_H

#include "ros_utils.h"
#include "sensors/IMU_Processing.h"
#include "sensors/wheel_Processing.h"
#include "utils/trans_utils.h"
#include "utils/reloc.h"
#include "utils/common_lib.h"

extern bool   sam_enable;
extern bool   path_en;
extern bool   scan_pub_en;
extern bool   dense_pub_en;
extern bool   scan_body_pub_en;
extern bool   feature_pub_en;
extern bool   effect_pub_en;
extern bool   reloc_en;
extern bool   time_sync_en;
extern bool   runtime_pos_log;
extern bool   extrinsic_est_en;
extern bool   pcd_save_en;
extern bool   imu_flip_en;
extern bool   wheel_en;

extern int    NUM_MAX_ITERATIONS;
extern int    pcd_save_interval;
extern int    lidar_type;

extern float  DET_RANGE;

extern double time_diff_lidar_to_imu;
extern double filter_size_corner_min;
extern double filter_size_surf_min;
extern double filter_size_map_min;
extern double cube_len;
extern double fov_deg;
extern double gyr_cov;
extern double acc_cov;
extern double b_gyr_cov;
extern double b_acc_cov;
extern double zupt_gyro_threshold;
extern double zupt_acc_norm_threshold;

extern std::vector<double> Lidar_extrinT;
extern std::vector<double> Lidar_extrinR;
extern std::vector<double> wheel_extrinT;
extern double wheel_extrinTheta;

extern std::shared_ptr<Preprocess> p_pre;
extern std::shared_ptr<ImuProcess> p_imu;
extern std::shared_ptr<WheelProcess> p_wheel;

extern double wheel_sr;
extern double wheel_sl;
extern double wheel_noise_x;
extern double wheel_noise_y;
extern double wheel_max_history_time;

extern std::atomic<bool>  relocalize_flag;

extern std::mutex         mtx_reloc;
extern RelocState         reloc_state;

extern std::mutex              mtx_buffer;
extern std::condition_variable sig_buffer;
extern std::deque<double>               time_buffer;
extern std::deque<PointCloudXYZI::Ptr>  lidar_buffer;
extern std::deque<ImuMsgConstPtr>       imu_buffer;
extern std::deque<WheelMsgConstPtr>     wheel_buffer;

extern double  last_timestamp_lidar;
extern double  last_timestamp_imu;
extern double  last_timestamp_wheel;

extern M3D IMU_FLIP_R;

void load_config();
void register_pub_sub();

void publish_odometry(const OdomData& odom, double lidar_end_time);
void publish_path(const PoseData& pose, double lidar_end_time);
void publish_world_cloud(const PointCloudXYZI::Ptr& cloud, double lidar_end_time,
							const std::string& frame_id, const state_ikfom& s);
void publish_body_cloud(const PointCloudXYZI::Ptr& cloud, double lidar_end_time,
						const std::string& frame_id, const state_ikfom& s);
void publish_effect_cloud(const PointCloudXYZI::Ptr& cloud, double lidar_end_time,
							const std::string& frame_id, const state_ikfom& s);
void publish_map_cloud(const PointCloudXYZI::Ptr& cloud, double lidar_end_time,
						const std::string& frame_id);
void publish_odometryhighfreq(const Pose& pose);
void publish_wheel_integration(const WheelPreintegration& wheel_preintegration);
void publish_wheel_path(double x, double y, double z, double timestamp);

#endif // ROS_INTERFACE_H
