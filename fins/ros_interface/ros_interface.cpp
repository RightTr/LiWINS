#include "ros_interface.h"  

std::mutex              mtx_buffer;
std::condition_variable sig_buffer;
std::deque<double>               time_buffer;
std::deque<PointCloudXYZI::Ptr>  lidar_buffer;
std::deque<ImuMsgConstPtr>       imu_buffer;
std::deque<WheelMsgConstPtr>     wheel_buffer;
double  last_timestamp_lidar;
double  last_timestamp_imu = -1.0;
double  last_timestamp_wheel = -1.0;
int     scan_count;
int     publish_count;

std::mutex         mtx_reloc;
RelocState         reloc_state;
std::atomic<bool>  relocalize_flag;

bool   sam_enable;
bool   path_en;
bool   scan_pub_en;
bool   dense_pub_en;
bool   scan_body_pub_en;
bool   feature_pub_en;
bool   effect_pub_en;
bool   reloc_en;
bool   time_sync_en;
bool   runtime_pos_log;
bool   extrinsic_est_en;
bool   pcd_save_en;
bool   imu_flip_en;
bool   wheel_en;

int    NUM_MAX_ITERATIONS;
int    pcd_save_interval;
int    odom_imu_frequency = 100;
int    lidar_type;

float  DET_RANGE;

double time_diff_lidar_to_imu;
double filter_size_corner_min;
double filter_size_surf_min;
double filter_size_map_min;
double cube_len;
double fov_deg;
double gyr_cov;
double acc_cov;
double b_gyr_cov;
double b_acc_cov;
double zupt_gyro_threshold     = 0.05;
double zupt_acc_norm_threshold = 0.30;

std::string map_file_path;
std::string lid_topic;
std::string imu_topic;
std::string wheel_topic;
std::string reloc_topic;

std::vector<double> extrinT;
std::vector<double> extrinR;
std::vector<double> wheel_extrinT;
std::vector<double> wheel_extrinR;

std::shared_ptr<Preprocess> p_pre = std::make_shared<Preprocess>();
std::shared_ptr<ImuProcess> p_imu = std::make_shared<ImuProcess>();
std::shared_ptr<WheelProcess> p_wheel = std::make_shared<WheelProcess>();

double wheel_sr = 1.0;
double wheel_sl = 1.0;
double wheel_noise_x = 0.02;
double wheel_noise_y = 0.02;
double wheel_max_history_time = 100.0;

Pcl2Publisher pubLaserCloudFull;
Pcl2Publisher pubLaserCloudFull_body;
Pcl2Publisher pubLaserCloudEffect;
Pcl2Publisher pubLaserCloudMap;
OdomPublisher pubOdomAftMapped;
PathPublisher pubPath;
OdomPublisher pubOdomHighFreq;

Pcl2Subscriber sub_pcl_standard;
LivoxSubscriber sub_pcl_livox;
PoseStampedSubscriber sub_reloc;
ImuSubscriber sub_imu;
WheelSubscriber sub_wheel;

OdometryMsg odomAftMapped;
PathMsg path;
PoseStampedMsg msgBodyPose;

double timediff_lidar_wrt_imu = 0.0;
bool timediff_set_flg = false;

M3D IMU_FLIP_R = (M3D() <<
    1.0,  0.0,  0.0,
    0.0, -1.0,  0.0,
    0.0,  0.0, -1.0).finished();

void load_config()
{
	rosparam_get("sam_enable", sam_enable, false);
    rosparam_get("publish/path_en", path_en, true);
    rosparam_get("publish/scan_publish_en", scan_pub_en, true);
    rosparam_get("publish/dense_publish_en", dense_pub_en, true);
    rosparam_get("publish/scan_bodyframe_pub_en", scan_body_pub_en, true);
    rosparam_get("publish/feature_pub_en", feature_pub_en, false);
    rosparam_get("publish/effect_pub_en", effect_pub_en, false);
    rosparam_get("reloc/reloc_en", reloc_en, false);
    rosparam_get("max_iteration", NUM_MAX_ITERATIONS, 4);
    rosparam_get("map_file_path", map_file_path, std::string(""));
    rosparam_get("common/lid_topic", lid_topic, std::string("/livox/lidar"));
    rosparam_get("common/imu_topic", imu_topic, std::string("/livox/imu"));
    rosparam_get("common/wheel_topic", wheel_topic, std::string("/wheel"));
    rosparam_get("reloc/reloc_topic", reloc_topic, std::string("/reloc/manual"));
    rosparam_get("common/time_sync_en", time_sync_en, false);
    rosparam_get("common/time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
    rosparam_get("common/imu_flip_en", imu_flip_en, false);
    rosparam_get("filter_size_corner", filter_size_corner_min, 0.5);
    rosparam_get("filter_size_surf", filter_size_surf_min, 0.5);
    rosparam_get("filter_size_map", filter_size_map_min, 0.5);
    rosparam_get("cube_side_length", cube_len, 200.0);
    rosparam_get("mapping/det_range", DET_RANGE, 300.f);
    rosparam_get("mapping/fov_degree", fov_deg, 180.0);
    rosparam_get("mapping/gyr_cov", gyr_cov, 0.1);
    rosparam_get("mapping/acc_cov", acc_cov, 0.1);
    rosparam_get("mapping/b_gyr_cov", b_gyr_cov, 0.0001);
    rosparam_get("mapping/b_acc_cov", b_acc_cov, 0.0001);
    rosparam_get("preprocess/blind", p_pre->blind, 0.01);
    rosparam_get("preprocess/lidar_type", lidar_type, (int)AVIA);
    rosparam_get("preprocess/scan_line", p_pre->N_SCANS, 16);
    rosparam_get("preprocess/timestamp_unit", p_pre->time_unit, (int)US);
    rosparam_get("preprocess/scan_rate", p_pre->SCAN_RATE, 10);
    rosparam_get("point_filter_num", p_pre->point_filter_num, 2);
    rosparam_get("feature_extract_enable", p_pre->feature_enabled, false);
    rosparam_get("runtime_pos_log_enable", runtime_pos_log, false);
    rosparam_get("mapping/extrinsic_est_en", extrinsic_est_en, true);
    rosparam_get("pcd_save/pcd_save_en", pcd_save_en, false);
    rosparam_get("pcd_save/interval", pcd_save_interval, -1);
    rosparam_get("mapping/extrinsic_T", extrinT, std::vector<double>());
    rosparam_get("mapping/extrinsic_R", extrinR, std::vector<double>());
    
    rosparam_get("wheel/enable", wheel_en, false);
    rosparam_get("wheel/sr", wheel_sr, 1.0);
    rosparam_get("wheel/sl", wheel_sl, 1.0);
    rosparam_get("wheel/noise_x", wheel_noise_x, 0.02);
    rosparam_get("wheel/noise_y", wheel_noise_y, 0.02);
    rosparam_get("wheel/max_history_time", wheel_max_history_time, 100.0);
    rosparam_get("wheel/extrinsic_T", wheel_extrinT, std::vector<double>{0.0, 0.0, 0.0});
    rosparam_get("wheel/extrinsic_R", wheel_extrinR, std::vector<double>{1.0, 0.0, 0.0,
                                                                          0.0, 1.0, 0.0,
                                                                          0.0, 0.0, 1.0});

    p_pre->lidar_type = lidar_type;
}

void livox_pcl_cbk(const LivoxCustomMsgConstPtr msg);
void standard_pcl_cbk(const Pcl2MsgConstPtr msg);
void reloc_cbk(const PoseStampedMsgConstPtr msg_in);
void imu_cbk(const ImuMsgConstPtr msg_in);
void wheel_cbk(const WheelMsgConstPtr msg_in);

void register_pub_sub()
{
	/*** ROS subscribe initialization ***/
    if (p_pre->lidar_type == AVIA) {
        sub_pcl_livox = create_subscriber<LivoxMsg>(lid_topic, 200000, livox_pcl_cbk);
    } else {
        sub_pcl_standard = create_subscriber<PointCloud2Msg>(lid_topic, 200000, standard_pcl_cbk);
    }
    
    sub_reloc = create_subscriber<PoseStampedMsg>(reloc_topic, 10, reloc_cbk);
    sub_imu = create_subscriber<ImuMsg>(imu_topic, 200000, imu_cbk);
    if (wheel_en)
        sub_wheel = create_subscriber<WheelMsg>(wheel_topic, 200000, wheel_cbk);
    pubLaserCloudFull = create_publisher<PointCloud2Msg>("/cloud_registered", 100000);
    pubLaserCloudFull_body = create_publisher<PointCloud2Msg>("/cloud_registered_body", 100000);
    pubLaserCloudEffect = create_publisher<PointCloud2Msg>("/cloud_effected", 100000);
    pubLaserCloudMap = create_publisher<PointCloud2Msg>("/Laser_map", 100000);
    #ifdef USE_ROS1
    int odom_qos = 0;  // ROS1 ignores this parameter
    #elif defined(USE_ROS2)
    auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(50)).reliable();
    #endif
    pubOdomAftMapped = create_publisher_qos<OdometryMsg>("/Odometry", odom_qos);
    pubPath = create_publisher_qos<PathMsg>("/path", odom_qos);
    pubOdomHighFreq = create_publisher_qos<OdometryMsg>("/OdometryHighFreq", odom_qos);
}

void standard_pcl_cbk(const Pcl2MsgConstPtr msg)
{
    mtx_buffer.lock();
    scan_count ++;
    double preprocess_start_time = omp_get_wtime();
    const double stamp_sec = get_ros_time_sec(msg->header.stamp);
    if (stamp_sec < last_timestamp_lidar)
    {
        ROS_PRINT_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(stamp_sec);
    last_timestamp_lidar = stamp_sec;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void livox_pcl_cbk(const LivoxCustomMsgConstPtr msg)
{
	mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count ++;
    const double stamp_sec = get_ros_time_sec(msg->header.stamp);
    if (stamp_sec < last_timestamp_lidar)
    {
        ROS_PRINT_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    last_timestamp_lidar = stamp_sec;
    
    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty() )
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);
    
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void imu_cbk(const ImuMsgConstPtr msg_in)
{
	publish_count ++;
    // cout<<"IMU got at: "<<get_ros_time_sec(msg_in->header.stamp)<<endl;
    ImuMsgPtr msg(new ImuMsg(*msg_in));

    if (imu_flip_en)
    {
        // Use a proper rotation (det=+1) instead of a reflection.
        const V3D gyr_raw(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
        const V3D acc_raw(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
        const V3D gyr_flip = IMU_FLIP_R * gyr_raw;
        const V3D acc_flip = IMU_FLIP_R * acc_raw;

        msg->angular_velocity.x = gyr_flip.x();
        msg->angular_velocity.y = gyr_flip.y();
        msg->angular_velocity.z = gyr_flip.z();
        msg->linear_acceleration.x = acc_flip.x();
        msg->linear_acceleration.y = acc_flip.y();
        msg->linear_acceleration.z = acc_flip.z();
    }

    const double msg_in_stamp_sec = get_ros_time_sec(msg_in->header.stamp);
    msg->header.stamp = get_ros_time(msg_in_stamp_sec - time_diff_lidar_to_imu);
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp = get_ros_time(timediff_lidar_wrt_imu + msg_in_stamp_sec);
    }
    double timestamp = get_ros_time_sec(msg->header.stamp);

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        ROS_PRINT_WARN("imu loop back, clear buffer");
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void wheel_cbk(const WheelMsgConstPtr msg_in)
{
    const double timestamp = msg_in->timestamp;

    mtx_buffer.lock();

    if (timestamp < last_timestamp_wheel)
    {
        ROS_PRINT_WARN("wheel loop back, clear buffer");
        wheel_buffer.clear();
    }

    last_timestamp_wheel = timestamp;
    wheel_buffer.push_back(msg_in);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void reloc_cbk(const PoseStampedMsgConstPtr msg_in)
{
	double timestamp = get_ros_time_sec(msg_in->header.stamp);
    double x = msg_in->pose.position.x;
    double y = msg_in->pose.position.y;
    double z = msg_in->pose.position.z;

    double qx = msg_in->pose.orientation.x;
    double qy = msg_in->pose.orientation.y;
    double qz = msg_in->pose.orientation.z;
    double qw = msg_in->pose.orientation.w;
    
    std::lock_guard<std::mutex> lock(mtx_reloc);
    reloc_state = RelocState(x, y, z,
                    qx, qy, qz, qw, timestamp);
    relocalize_flag.store(true); 
    ROS_PRINT_INFO("Reloc received: (%.3f, %.3f, %.3f), quat=(%.3f, %.3f, %.3f, %.3f)",
        x, y, z, qx, qy, qz, qw);
}

void publish_world_cloud(const PointCloudXYZI::Ptr& cloud, double lidar_end_time,
                         const std::string& frame_id, const state_ikfom& s)
{
    int size = cloud->points.size();
    PointCloudXYZI::Ptr cloud_world(new PointCloudXYZI(size, 1));
    for (int i = 0; i < size; i++)
        RGBpointBodyToWorld(&cloud->points[i], &cloud_world->points[i], s);

    Pcl2Msg msg;
    pcl::toROSMsg(*cloud_world, msg);
    msg.header.stamp = get_ros_time(lidar_end_time);
    msg.header.frame_id = frame_id;
    ros_publish(pubLaserCloudFull, msg);
}

void publish_body_cloud(const PointCloudXYZI::Ptr& cloud, double lidar_end_time,
                        const std::string& frame_id, const state_ikfom& s)
{
    int size = cloud->points.size();
    PointCloudXYZI::Ptr cloud_body(new PointCloudXYZI(size, 1));
    for (int i = 0; i < size; i++)
        RGBpointBodyLidarToIMU(&cloud->points[i], &cloud_body->points[i], s);

    Pcl2Msg msg;
    pcl::toROSMsg(*cloud_body, msg);
    msg.header.stamp = get_ros_time(lidar_end_time);
    msg.header.frame_id = frame_id;
    ros_publish(pubLaserCloudFull_body, msg);
}

void publish_effect_cloud(const PointCloudXYZI::Ptr& cloud, double lidar_end_time,
                          const std::string& frame_id, const state_ikfom& s)
{
    int size = cloud->points.size();
    PointCloudXYZI::Ptr cloud_world(new PointCloudXYZI(size, 1));
    for (int i = 0; i < size; i++)
        RGBpointBodyToWorld(&cloud->points[i], &cloud_world->points[i], s);

    Pcl2Msg msg;
    pcl::toROSMsg(*cloud_world, msg);
    msg.header.stamp = get_ros_time(lidar_end_time);
    msg.header.frame_id = frame_id;
    ros_publish(pubLaserCloudEffect, msg);
}

void publish_map_cloud(const PointCloudXYZI::Ptr& cloud, double lidar_end_time,
                       const std::string& frame_id)
{
    Pcl2Msg msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = get_ros_time(lidar_end_time);
    msg.header.frame_id = frame_id;
    ros_publish(pubLaserCloudMap, msg);
}

void publish_odometry(const OdomData& odom, double lidar_end_time)
{
	odomAftMapped.header.frame_id = "camera_init";
	odomAftMapped.child_frame_id = "body";
	odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
	odomAftMapped.pose.pose.position.x = odom.x;
	odomAftMapped.pose.pose.position.y = odom.y;
	odomAftMapped.pose.pose.position.z = odom.z;
	odomAftMapped.pose.pose.orientation.x = odom.qx;
	odomAftMapped.pose.pose.orientation.y = odom.qy;
	odomAftMapped.pose.pose.orientation.z = odom.qz;
	odomAftMapped.pose.pose.orientation.w = odom.qw;
    std::copy(std::begin(odom.covariance), std::end(odom.covariance), odomAftMapped.pose.covariance.begin());
    ros_publish(pubOdomAftMapped, odomAftMapped);

    TransformStampedMsg tf_msg;
    tf_msg.header.stamp = get_ros_time(lidar_end_time);
    tf_msg.header.frame_id = "camera_init";
    tf_msg.child_frame_id = "body";
    tf_msg.transform.translation.x = odom.x;
    tf_msg.transform.translation.y = odom.y;
    tf_msg.transform.translation.z = odom.z;
    tf_msg.transform.rotation.x = odom.qx;
    tf_msg.transform.rotation.y = odom.qy;
    tf_msg.transform.rotation.z = odom.qz;
    tf_msg.transform.rotation.w = odom.qw;
#ifdef USE_ROS1
    static tf::TransformBroadcaster br;
#elif defined(USE_ROS2)
    static tf2_ros::TransformBroadcaster br(get_ros_node());
#endif
    br.sendTransform(tf_msg);
}

void publish_path(const PoseData& pose, double lidar_end_time)
{
	msgBodyPose.header.stamp = get_ros_time(lidar_end_time);
	msgBodyPose.header.frame_id = "camera_init";
	msgBodyPose.pose.position.x = pose.x;
	msgBodyPose.pose.position.y = pose.y;
	msgBodyPose.pose.position.z = pose.z;
	msgBodyPose.pose.orientation.x = pose.qx;
	msgBodyPose.pose.orientation.y = pose.qy;
	msgBodyPose.pose.orientation.z = pose.qz;
	msgBodyPose.pose.orientation.w = pose.qw;

	static int jjj = 0;
	if (++jjj % 10 == 0)
	{
		path.poses.push_back(msgBodyPose);
		path.header.stamp = msgBodyPose.header.stamp;
		path.header.frame_id = "camera_init";
        ros_publish(pubPath, path);
	}
}

void publish_odometryhighfreq(const Pose& pose)
{
    OdometryMsg msg;
    msg.header.stamp = get_ros_time(pose._timestamp);
    msg.header.frame_id = "camera_init";
    msg.child_frame_id = "body";
    msg.pose.pose.position.x = pose._x;
    msg.pose.pose.position.y = pose._y;
    msg.pose.pose.position.z = pose._z;
    msg.pose.pose.orientation.x = pose._qx;
    msg.pose.pose.orientation.y = pose._qy;
    msg.pose.pose.orientation.z = pose._qz;
    msg.pose.pose.orientation.w = pose._qw;
    ros_publish(pubOdomHighFreq, msg);

    TransformStampedMsg tf_msg;
    tf_msg.header.stamp = get_ros_time(pose._timestamp);
    tf_msg.header.frame_id = "camera_init";
    tf_msg.child_frame_id = "body_hf";
    tf_msg.transform.translation.x = pose._x;
    tf_msg.transform.translation.y = pose._y;
    tf_msg.transform.translation.z = pose._z;
    tf_msg.transform.rotation.x = pose._qx;
    tf_msg.transform.rotation.y = pose._qy;
    tf_msg.transform.rotation.z = pose._qz;
    tf_msg.transform.rotation.w = pose._qw;
#ifdef USE_ROS1
    static tf::TransformBroadcaster br_hf;
#elif defined(USE_ROS2)
    static tf2_ros::TransformBroadcaster br_hf(get_ros_node());
#endif
    br_hf.sendTransform(tf_msg);
}
