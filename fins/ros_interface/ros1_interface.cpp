#include "ros1_interdace.h"

void ROS1_Interface::load_config()
{
    nh_.param<bool>("sam_enable",                        sam_enable,             false);
    nh_.param<bool>("publish/path_en",                   path_en,                true);
    nh_.param<bool>("publish/scan_publish_en",           scan_pub_en,            true);
    nh_.param<bool>("publish/dense_publish_en",          dense_pub_en,           true);
    nh_.param<bool>("publish/scan_bodyframe_pub_en",     scan_body_pub_en,       true);
    nh_.param<bool>("publish/feature_pub_en",            feature_pub_en,         false);
    nh_.param<bool>("publish/effect_pub_en",             effect_pub_en,          false);
    nh_.param<bool>("reloc/reloc_en",                    reloc_en,               false);
    nh_.param<int>("max_iteration",                      NUM_MAX_ITERATIONS,     4);
    nh_.param<std::string>("map_file_path",              map_file_path,          "");
    nh_.param<std::string>("common/lid_topic",           lid_topic,              "/livox/lidar");
    nh_.param<std::string>("common/imu_topic",           imu_topic,              "/livox/imu");
    nh_.param<std::string>("reloc/reloc_topic",          reloc_topic,            "/reloc/manual");
    nh_.param<bool>("common/time_sync_en",               time_sync_en,           false);
    nh_.param<double>("common/time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
    nh_.param<double>("filter_size_corner",              filter_size_corner_min,  0.5);
    nh_.param<double>("filter_size_surf",                filter_size_surf_min,    0.5);
    nh_.param<double>("filter_size_map",                 filter_size_map_min,     0.5);
    nh_.param<double>("cube_side_length",                cube_len,               200);
    nh_.param<float>("mapping/det_range",                DET_RANGE,              300.f);
    nh_.param<double>("mapping/fov_degree",              fov_deg,                180);
    nh_.param<double>("mapping/gyr_cov",                 gyr_cov,                0.1);
    nh_.param<double>("mapping/acc_cov",                 acc_cov,                0.1);
    nh_.param<double>("mapping/b_gyr_cov",               b_gyr_cov,              0.0001);
    nh_.param<double>("mapping/b_acc_cov",               b_acc_cov,              0.0001);
    nh_.param<double>("mapping/zupt_gyro_threshold",     zupt_gyro_threshold,     0.05);
    nh_.param<double>("mapping/zupt_acc_norm_threshold", zupt_acc_norm_threshold, 0.30);
    nh_.param<double>("preprocess/blind",                p_pre->blind,           0.01);
    nh_.param<int>("preprocess/lidar_type",              lidar_type,             AVIA);
    nh_.param<int>("preprocess/scan_line",               p_pre->N_SCANS,         16);
    nh_.param<int>("preprocess/timestamp_unit",          p_pre->time_unit,       US);
    nh_.param<int>("preprocess/scan_rate",               p_pre->SCAN_RATE,       10);
    nh_.param<int>("point_filter_num",                   p_pre->point_filter_num, 2);
    nh_.param<bool>("feature_extract_enable",            p_pre->feature_enabled, false);
    nh_.param<bool>("runtime_pos_log_enable",            runtime_pos_log,        0);
    nh_.param<bool>("mapping/extrinsic_est_en",          extrinsic_est_en,       true);
    nh_.param<bool>("mapping/use_zupt", use_zupt, false);
    nh_.param<bool>("mapping/use_known_initial_attitude", use_known_initial_attitude, false);
    nh_.param<bool>("pcd_save/pcd_save_en",              pcd_save_en,            false);
    nh_.param<int>("pcd_save/interval",                  pcd_save_interval,      -1);
    nh_.param<std::vector<double>>("mapping/extrinsic_T", extrinT,               std::vector<double>());
    nh_.param<std::vector<double>>("mapping/extrinsic_R", extrinR,               std::vector<double>());
    nh_.param<std::vector<double>>("mapping/initial_attitude", initial_attitude, std::vector<double>());
    nh_.param<int>("publish/odom_imu_frequency",         odom_imu_frequency,     100);
    p_pre->lidar_type = lidar_type;
}

void ROS1_Interface::register_pub_sub()
{
    sub_pcl_ = p_pre->lidar_type == AVIA ?
        nh_.subscribe(lid_topic,   200000, &ROS1_Interface::livox_pcl_cbk,    this) :
        nh_.subscribe(lid_topic,   200000, &ROS1_Interface::standard_pcl_cbk, this);
    sub_reloc_ = nh_.subscribe(reloc_topic, 10,     &ROS1_Interface::reloc_cbk, this);
    sub_imu_   = nh_.subscribe(imu_topic,   200000, &ROS1_Interface::imu_cbk,   this);

    pubLaserCloudFull_      = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_registered",      100000);
    pubLaserCloudFull_body_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 100000);
    pubLaserCloudEffect_    = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_effected",        100000);
    pubLaserCloudMap_       = nh_.advertise<sensor_msgs::PointCloud2>("/Laser_map",             100000);
    pubOdomAftMapped_       = nh_.advertise<nav_msgs::Odometry>      ("/Odometry",              100000);
    pubPath_                = nh_.advertise<nav_msgs::Path>           ("/path",                  100000);
    pubOdomHighFreq_        = nh_.advertise<nav_msgs::Odometry>      ("/OdometryHighFreq",      100000);
}

void ROS1_Interface::standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    mtx_buffer.lock();
    scan_count++;
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);

    time_buffer.push_back(msg->header.stamp.toSec());
    last_timestamp_lidar = msg->header.stamp.toSec();

    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void ROS1_Interface::livox_pcl_cbk(const livox_ros_driver2::CustomMsg::ConstPtr& msg)
{
    mtx_buffer.lock();
    scan_count++;
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    last_timestamp_lidar = msg->header.stamp.toSec();

    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty())
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n", last_timestamp_imu, last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg_ && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg_ = true;
        timediff_lidar_wrt_imu_ = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu_);
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);

    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void ROS1_Interface::imu_cbk(const sensor_msgs::Imu::ConstPtr& msg_in)
{
    publish_count++;
    sensor_msgs::ImuPtr msg(new sensor_msgs::Imu(*msg_in));

    msg->header.stamp = ros::Time().fromSec(msg_in->header.stamp.toSec() - time_diff_lidar_to_imu);
    if (abs(timediff_lidar_wrt_imu_) > 0.1 && time_sync_en)
    {
        msg->header.stamp = ros::Time().fromSec(timediff_lidar_wrt_imu_ + msg_in->header.stamp.toSec());
    }

    double timestamp = msg->header.stamp.toSec();

    mtx_buffer.lock();
    if (timestamp < last_timestamp_imu)
    {
        ROS_WARN("imu loop back, clear buffer");
        imu_buffer.clear();
    }
    last_timestamp_imu = timestamp;
    imu_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void ROS1_Interface::reloc_cbk(const geometry_msgs::PoseStamped::ConstPtr& msg_in)
{
    double timestamp = msg_in->header.stamp.toSec();
    double x  = msg_in->pose.position.x;
    double y  = msg_in->pose.position.y;
    double z  = msg_in->pose.position.z;
    double qx = msg_in->pose.orientation.x;
    double qy = msg_in->pose.orientation.y;
    double qz = msg_in->pose.orientation.z;
    double qw = msg_in->pose.orientation.w;

    std::lock_guard<std::mutex> lock(mtx_reloc);
    reloc_state = RelocState(x, y, z, qx, qy, qz, qw, timestamp);
    relocalize_flag.store(true);
    ROS_INFO("Reloc received: (%.3f, %.3f, %.3f), quat=(%.3f, %.3f, %.3f, %.3f)",
             x, y, z, qx, qy, qz, qw);
}

void ROS1_Interface::publish_world_cloud(const PointCloudXYZI::Ptr& cloud, double lidar_end_time,
                                         const std::string& frame_id)
{
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp    = ros::Time().fromSec(lidar_end_time);
    msg.header.frame_id = frame_id;
    pubLaserCloudFull_.publish(msg);
}

void ROS1_Interface::publish_body_cloud(const PointCloudXYZI::Ptr& cloud, double lidar_end_time,
                                        const std::string& frame_id)
{
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp    = ros::Time().fromSec(lidar_end_time);
    msg.header.frame_id = frame_id;
    pubLaserCloudFull_body_.publish(msg);
}

void ROS1_Interface::publish_effect_cloud(const PointCloudXYZI::Ptr& cloud, double lidar_end_time,
                                          const std::string& frame_id)
{
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp    = ros::Time().fromSec(lidar_end_time);
    msg.header.frame_id = frame_id;
    pubLaserCloudEffect_.publish(msg);
}

void ROS1_Interface::publish_map_cloud(const PointCloudXYZI::Ptr& cloud, double lidar_end_time,
                                       const std::string& frame_id)
{
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp    = ros::Time().fromSec(lidar_end_time);
    msg.header.frame_id = frame_id;
    pubLaserCloudMap_.publish(msg);
}

void ROS1_Interface::publish_odometry(const OdomData& odom, double lidar_end_time)
{
    
    odomAftMapped_.header.frame_id         = "camera_init";
    odomAftMapped_.child_frame_id          = "body";
    odomAftMapped_.header.stamp            = ros::Time().fromSec(lidar_end_time);
    odomAftMapped_.pose.pose.position.x    = odom.x;
    odomAftMapped_.pose.pose.position.y    = odom.y;
    odomAftMapped_.pose.pose.position.z    = odom.z;
    odomAftMapped_.pose.pose.orientation.x = odom.qx;
    odomAftMapped_.pose.pose.orientation.y = odom.qy;
    odomAftMapped_.pose.pose.orientation.z = odom.qz;
    odomAftMapped_.pose.pose.orientation.w = odom.qw;
    std::copy(std::begin(odom.covariance), std::end(odom.covariance),
              odomAftMapped_.pose.covariance.begin());
    pubOdomAftMapped_.publish(odomAftMapped_);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(odom.x, odom.y, odom.z));
    q.setW(odom.qw); q.setX(odom.qx); q.setY(odom.qy); q.setZ(odom.qz);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, odomAftMapped_.header.stamp,
                                          "camera_init", "body"));
}

void ROS1_Interface::publish_path(const PoseData& pose, double lidar_end_time)
{
    
    msgBodyPose_.header.stamp            = ros::Time().fromSec(lidar_end_time);
    msgBodyPose_.header.frame_id         = "camera_init";
    msgBodyPose_.pose.position.x         = pose.x;
    msgBodyPose_.pose.position.y         = pose.y;
    msgBodyPose_.pose.position.z         = pose.z;
    msgBodyPose_.pose.orientation.x      = pose.qx;
    msgBodyPose_.pose.orientation.y      = pose.qy;
    msgBodyPose_.pose.orientation.z      = pose.qz;
    msgBodyPose_.pose.orientation.w      = pose.qw;

    static int jjj = 0;
    if (++jjj % 10 == 0)
    {
        path_.poses.push_back(msgBodyPose_);
        path_.header.stamp    = msgBodyPose_.header.stamp;
        path_.header.frame_id = "camera_init";
        pubPath_.publish(path_);
    }
}

void ROS1_Interface::publish_odometryhighfreq(const Pose& pose)
{
    if (!ros_ok()) return;  // Prevent publishing during ROS shutdown
    
    nav_msgs::Odometry msg;
    msg.header.stamp    = ros::Time(pose._timestamp);
    msg.header.frame_id = "camera_init";
    msg.child_frame_id  = "body";

    msg.pose.pose.position.x    = pose._x;
    msg.pose.pose.position.y    = pose._y;
    msg.pose.pose.position.z    = pose._z;
    msg.pose.pose.orientation.x = pose._qx;
    msg.pose.pose.orientation.y = pose._qy;
    msg.pose.pose.orientation.z = pose._qz;
    msg.pose.pose.orientation.w = pose._qw;
    pubOdomHighFreq_.publish(msg);

    static tf::TransformBroadcaster br_hf;
    tf::Transform  transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(msg.pose.pose.position.x,
                                    msg.pose.pose.position.y,
                                    msg.pose.pose.position.z));
    q.setW(msg.pose.pose.orientation.w);
    q.setX(msg.pose.pose.orientation.x);
    q.setY(msg.pose.pose.orientation.y);
    q.setZ(msg.pose.pose.orientation.z);
    transform.setRotation(q);
    br_hf.sendTransform(tf::StampedTransform(transform, msg.header.stamp,
                                             "camera_init", "body_hf"));
}
