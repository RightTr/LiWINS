#ifndef ROS1_INTERFACE_H
#define ROS1_INTERFACE_H

#ifdef USE_ROS1
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#endif  

#include "ros_interface_base.h"  
#include "lio_sam/utility.h"

class ROS1_Interface : public ROSInterfaceBase
{
public:

    ROS1_Interface(ros::NodeHandle& nh) : nh_(nh) {
        load_config();
        register_pub_sub();

        if (sam_enable) read_liosam_params(nh);
    } 

    ~ROS1_Interface() override = default;

    void load_config()      override;
    void register_pub_sub() override;
    void spin_once()        override { ros::spinOnce(); }
    bool ros_ok()           override { return ros::ok(); }

    void publish_odometry(const OdomData& odom, double lidar_end_time) override;
    void publish_path(const PoseData& pose, double lidar_end_time) override;
    void publish_world_cloud(const PointCloudXYZI::Ptr& cloud, double lidar_end_time,
                             const std::string& frame_id) override;
    void publish_body_cloud(const PointCloudXYZI::Ptr& cloud, double lidar_end_time,
                            const std::string& frame_id) override;
    void publish_effect_cloud(const PointCloudXYZI::Ptr& cloud, double lidar_end_time,
                              const std::string& frame_id) override;
    void publish_map_cloud(const PointCloudXYZI::Ptr& cloud, double lidar_end_time,
                           const std::string& frame_id) override;
    void publish_odometryhighfreq(const Pose& pose) override;

private:

    ros::NodeHandle& nh_;
    
    // Subscribers
    ros::Subscriber sub_pcl_;
    ros::Subscriber sub_reloc_;
    ros::Subscriber sub_imu_;

    // Publishers
    ros::Publisher pubLaserCloudFull_;
    ros::Publisher pubLaserCloudFull_body_;
    ros::Publisher pubLaserCloudEffect_;
    ros::Publisher pubLaserCloudMap_;
    ros::Publisher pubOdomAftMapped_;
    ros::Publisher pubPath_;
    ros::Publisher pubOdomHighFreq_;

    nav_msgs::Odometry          odomAftMapped_;
    nav_msgs::Path              path_;
    geometry_msgs::PoseStamped  msgBodyPose_;

    double timediff_lidar_wrt_imu_ = 0.0;
    bool   timediff_set_flg_       = false;

    void livox_pcl_cbk   (const livox_ros_driver2::CustomMsg::ConstPtr& msg);
    void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void reloc_cbk       (const geometry_msgs::PoseStamped::ConstPtr& msg);
    void imu_cbk         (const sensor_msgs::Imu::ConstPtr& msg);
};

#endif // ROS1_INTERFACE_H
