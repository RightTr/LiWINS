#ifndef ROS2_INTERFACE_H
#define ROS2_INTERFACE_H

#include <memory>

#ifdef USE_ROS2
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif

#include "ros_interface_base.h"

class ROS2_Interface : public ROSInterfaceBase
{
public:
	explicit ROS2_Interface(const rclcpp::Node::SharedPtr& node);
	~ROS2_Interface() override = default;

	void load_config() override;
	void register_pub_sub() override;
	void spin_once() override { rclcpp::spin_some(node_); }
	bool ros_ok() override { return rclcpp::ok(); }

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
	rclcpp::Node::SharedPtr node_;

	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_standard_;
	rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_reloc_;
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomHighFreq_;

	nav_msgs::msg::Odometry odomAftMapped_;
	nav_msgs::msg::Path path_;
	geometry_msgs::msg::PoseStamped msgBodyPose_;

	double timediff_lidar_wrt_imu_ = 0.0;
	bool timediff_set_flg_ = false;

	static rclcpp::Time convert_to_rclcpp_time(double timestamp);

	void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
	void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg);
	void reloc_cbk(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg_in);
	void imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr msg_in);
};

#endif // ROS2_INTERFACE_H
