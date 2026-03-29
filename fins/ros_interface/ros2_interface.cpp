#include "ros2_interface.h"
#include "lio_sam/utility.h"

#include <algorithm>
#include <cmath>

#include <pcl_conversions/pcl_conversions.h>

ROS2_Interface::ROS2_Interface(const rclcpp::Node::SharedPtr& node) : node_(node)
{
	load_config();
	register_pub_sub();

	if (sam_enable) read_liosam_params(node_);
}

rclcpp::Time ROS2_Interface::convert_to_rclcpp_time(double timestamp)
{
	int32_t sec = std::floor(timestamp);
	uint32_t nanosec = static_cast<uint32_t>((timestamp - sec) * 1e9);
	return rclcpp::Time(sec, nanosec);
}

void ROS2_Interface::load_config()
{
	sam_enable             = node_->declare_parameter<bool>("sam_enable", false);
	path_en                = node_->declare_parameter<bool>("publish.path_en", true);
	scan_pub_en            = node_->declare_parameter<bool>("publish.scan_publish_en", true);
	dense_pub_en           = node_->declare_parameter<bool>("publish.dense_publish_en", true);
	scan_body_pub_en       = node_->declare_parameter<bool>("publish.scan_bodyframe_pub_en", true);
	feature_pub_en         = node_->declare_parameter<bool>("publish.feature_pub_en", false);
	effect_pub_en          = node_->declare_parameter<bool>("publish.effect_pub_en", false);
	reloc_en               = node_->declare_parameter<bool>("reloc.reloc_en", false);
	NUM_MAX_ITERATIONS     = node_->declare_parameter<int>("max_iteration", 4);
	map_file_path          = node_->declare_parameter<std::string>("map_file_path", "");
	lid_topic              = node_->declare_parameter<std::string>("common.lid_topic", "/livox/lidar");
	imu_topic              = node_->declare_parameter<std::string>("common.imu_topic", "/livox/imu");
	reloc_topic            = node_->declare_parameter<std::string>("reloc.reloc_topic", "/reloc/manual");
	time_sync_en           = node_->declare_parameter<bool>("common.time_sync_en", false);
	time_diff_lidar_to_imu = node_->declare_parameter<double>("common.time_offset_lidar_to_imu", 0.0);
	imu_flip_en            = node_->declare_parameter<bool>("common.imu_flip_en", false);

	filter_size_corner_min = node_->declare_parameter<double>("filter_size_corner", 0.5);
	filter_size_surf_min   = node_->declare_parameter<double>("filter_size_surf", 0.5);
	filter_size_map_min    = node_->declare_parameter<double>("filter_size_map", 0.5);
	cube_len               = node_->declare_parameter<double>("cube_side_length", 200.0);
	DET_RANGE              = node_->declare_parameter<double>("mapping.det_range", 300.0);
	fov_deg                = node_->declare_parameter<double>("mapping.fov_degree", 180.0);
	gyr_cov                = node_->declare_parameter<double>("mapping.gyr_cov", 0.1);
	acc_cov                = node_->declare_parameter<double>("mapping.acc_cov", 0.1);
	b_gyr_cov              = node_->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
	b_acc_cov              = node_->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
	zupt_gyro_threshold     = node_->declare_parameter<double>("mapping.zupt_gyro_threshold", 0.05);
	zupt_acc_norm_threshold = node_->declare_parameter<double>("mapping.zupt_acc_norm_threshold", 0.30);

	p_pre->blind            = node_->declare_parameter<double>("preprocess.blind", 0.01);
	lidar_type              = node_->declare_parameter<int>("preprocess.lidar_type", AVIA);
	p_pre->N_SCANS          = node_->declare_parameter<int>("preprocess.scan_line", 16);
	p_pre->time_unit        = node_->declare_parameter<int>("preprocess.timestamp_unit", US);
	p_pre->SCAN_RATE        = node_->declare_parameter<int>("preprocess.scan_rate", 10);
	p_pre->point_filter_num = node_->declare_parameter<int>("point_filter_num", 2);
	p_pre->feature_enabled  = node_->declare_parameter<bool>("feature_extract_enable", false);
	runtime_pos_log         = node_->declare_parameter<bool>("runtime_pos_log_enable", false);
	extrinsic_est_en        = node_->declare_parameter<bool>("mapping.extrinsic_est_en", true);
	use_zupt = node_->declare_parameter<bool>("mapping.use_zupt", false);
	pcd_save_en             = node_->declare_parameter<bool>("pcd_save.pcd_save_en", false);
	pcd_save_interval       = node_->declare_parameter<int>("pcd_save.interval", -1);
	odom_imu_frequency      = node_->declare_parameter<int>("publish.odom_imu_frequency", 100);

	extrinT = node_->declare_parameter<std::vector<double>>("mapping.extrinsic_T", std::vector<double>());
	extrinR = node_->declare_parameter<std::vector<double>>("mapping.extrinsic_R", std::vector<double>());

	p_pre->lidar_type = lidar_type;
}

void ROS2_Interface::register_pub_sub()
{
	const auto sensor_qos = rclcpp::SensorDataQoS();

	if (p_pre->lidar_type == AVIA)
	{
		sub_pcl_livox_ = node_->create_subscription<livox_ros_driver2::msg::CustomMsg>(
			lid_topic, sensor_qos,
			std::bind(&ROS2_Interface::livox_pcl_cbk, this, std::placeholders::_1));
	}
	else
	{
		sub_pcl_standard_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
			lid_topic, sensor_qos,
			std::bind(&ROS2_Interface::standard_pcl_cbk, this, std::placeholders::_1));
	}

	sub_reloc_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
		reloc_topic, 10,
		std::bind(&ROS2_Interface::reloc_cbk, this, std::placeholders::_1));

	sub_imu_ = node_->create_subscription<sensor_msgs::msg::Imu>(
		imu_topic, sensor_qos,
		std::bind(&ROS2_Interface::imu_cbk, this, std::placeholders::_1));

	pubLaserCloudFull_      = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 100000);
	pubLaserCloudFull_body_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", 100000);
	pubLaserCloudEffect_    = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 100000);
	pubLaserCloudMap_       = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 100000);
	pubOdomAftMapped_       = node_->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 100000);
	pubPath_                = node_->create_publisher<nav_msgs::msg::Path>("/path", 100000);
	pubOdomHighFreq_        = node_->create_publisher<nav_msgs::msg::Odometry>("/OdometryHighFreq", 100000);
}

void ROS2_Interface::standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
	std::lock_guard<std::mutex> lock(mtx_buffer);
	scan_count++;

	const double stamp_sec = rclcpp::Time(msg->header.stamp).seconds();
	if (stamp_sec < last_timestamp_lidar)
	{
		RCLCPP_ERROR(node_->get_logger(), "lidar loop back, clear buffer");
		lidar_buffer.clear();
	}

	PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
	p_pre->process(msg, ptr);
	lidar_buffer.push_back(ptr);
	time_buffer.push_back(stamp_sec);
	last_timestamp_lidar = stamp_sec;

	sig_buffer.notify_all();
}

void ROS2_Interface::livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg)
{
	std::lock_guard<std::mutex> lock(mtx_buffer);
	scan_count++;

	const double stamp_sec = rclcpp::Time(msg->header.stamp).seconds();
	if (stamp_sec < last_timestamp_lidar)
	{
		RCLCPP_ERROR(node_->get_logger(), "lidar loop back, clear buffer");
		lidar_buffer.clear();
	}
	last_timestamp_lidar = stamp_sec;

	if (!time_sync_en && std::abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 &&
		!imu_buffer.empty() && !lidar_buffer.empty())
	{
		RCLCPP_WARN(node_->get_logger(),
					"IMU and LiDAR not synced, IMU time: %.6f, lidar header time: %.6f",
					last_timestamp_imu, last_timestamp_lidar);
	}

	if (time_sync_en && !timediff_set_flg_ && std::abs(last_timestamp_lidar - last_timestamp_imu) > 1.0 &&
		!imu_buffer.empty())
	{
		timediff_set_flg_ = true;
		timediff_lidar_wrt_imu_ = last_timestamp_lidar + 0.1 - last_timestamp_imu;
		RCLCPP_INFO(node_->get_logger(), "Self sync IMU and LiDAR, time diff is %.10f",
					timediff_lidar_wrt_imu_);
	}

	PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
	p_pre->process(msg, ptr);
	lidar_buffer.push_back(ptr);
	time_buffer.push_back(last_timestamp_lidar);

	sig_buffer.notify_all();
}

void ROS2_Interface::imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr msg_in)
{
	publish_count++;

	auto msg = std::make_shared<sensor_msgs::msg::Imu>(*msg_in);
	const double imu_stamp = rclcpp::Time(msg_in->header.stamp).seconds();

	if (imu_flip_en)
    {
        msg->angular_velocity.x = -msg->angular_velocity.x;
        msg->angular_velocity.y = -msg->angular_velocity.y;
        msg->angular_velocity.z = -msg->angular_velocity.z;
        msg->linear_acceleration.x = -msg->linear_acceleration.x;
        msg->linear_acceleration.y = -msg->linear_acceleration.y;
        msg->linear_acceleration.z = -msg->linear_acceleration.z;
    }

	double corrected_stamp = imu_stamp - time_diff_lidar_to_imu;
	if (std::abs(timediff_lidar_wrt_imu_) > 0.1 && time_sync_en)
	{
		corrected_stamp = timediff_lidar_wrt_imu_ + imu_stamp;
	}
	msg->header.stamp = convert_to_rclcpp_time(corrected_stamp);

	std::lock_guard<std::mutex> lock(mtx_buffer);
	if (corrected_stamp < last_timestamp_imu)
	{
		RCLCPP_WARN(node_->get_logger(), "imu loop back, clear buffer");
		imu_buffer.clear();
	}

	last_timestamp_imu = corrected_stamp;
	imu_buffer.push_back(msg);
	sig_buffer.notify_all();
}

void ROS2_Interface::reloc_cbk(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg_in)
{
	const double timestamp = rclcpp::Time(msg_in->header.stamp).seconds();
	const double x = msg_in->pose.position.x;
	const double y = msg_in->pose.position.y;
	const double z = msg_in->pose.position.z;
	const double qx = msg_in->pose.orientation.x;
	const double qy = msg_in->pose.orientation.y;
	const double qz = msg_in->pose.orientation.z;
	const double qw = msg_in->pose.orientation.w;

	std::lock_guard<std::mutex> lock(mtx_reloc);
	reloc_state = RelocState(x, y, z, qx, qy, qz, qw, timestamp);
	relocalize_flag.store(true);

	RCLCPP_INFO(node_->get_logger(),
				"Reloc received: (%.3f, %.3f, %.3f), quat=(%.3f, %.3f, %.3f, %.3f)",
				x, y, z, qx, qy, qz, qw);
}

void ROS2_Interface::publish_world_cloud(const PointCloudXYZI::Ptr& cloud, double lidar_end_time,
										 const std::string& frame_id)
{
	sensor_msgs::msg::PointCloud2 msg;
	pcl::toROSMsg(*cloud, msg);
	msg.header.stamp = convert_to_rclcpp_time(lidar_end_time);
	msg.header.frame_id = frame_id;
	pubLaserCloudFull_->publish(msg);
}

void ROS2_Interface::publish_body_cloud(const PointCloudXYZI::Ptr& cloud, double lidar_end_time,
										const std::string& frame_id)
{
	sensor_msgs::msg::PointCloud2 msg;
	pcl::toROSMsg(*cloud, msg);
	msg.header.stamp = convert_to_rclcpp_time(lidar_end_time);
	msg.header.frame_id = frame_id;
	pubLaserCloudFull_body_->publish(msg);
}

void ROS2_Interface::publish_effect_cloud(const PointCloudXYZI::Ptr& cloud, double lidar_end_time,
										  const std::string& frame_id)
{
	sensor_msgs::msg::PointCloud2 msg;
	pcl::toROSMsg(*cloud, msg);
	msg.header.stamp = convert_to_rclcpp_time(lidar_end_time);
	msg.header.frame_id = frame_id;
	pubLaserCloudEffect_->publish(msg);
}

void ROS2_Interface::publish_map_cloud(const PointCloudXYZI::Ptr& cloud, double lidar_end_time,
									   const std::string& frame_id)
{
	sensor_msgs::msg::PointCloud2 msg;
	pcl::toROSMsg(*cloud, msg);
	msg.header.stamp = convert_to_rclcpp_time(lidar_end_time);
	msg.header.frame_id = frame_id;
	pubLaserCloudMap_->publish(msg);
}

void ROS2_Interface::publish_odometry(const OdomData& odom, double lidar_end_time)
{
	odomAftMapped_.header.frame_id = "camera_init";
	odomAftMapped_.child_frame_id = "body";
	odomAftMapped_.header.stamp = convert_to_rclcpp_time(lidar_end_time);
	odomAftMapped_.pose.pose.position.x = odom.x;
	odomAftMapped_.pose.pose.position.y = odom.y;
	odomAftMapped_.pose.pose.position.z = odom.z;
	odomAftMapped_.pose.pose.orientation.x = odom.qx;
	odomAftMapped_.pose.pose.orientation.y = odom.qy;
	odomAftMapped_.pose.pose.orientation.z = odom.qz;
	odomAftMapped_.pose.pose.orientation.w = odom.qw;
	std::copy(std::begin(odom.covariance), std::end(odom.covariance), odomAftMapped_.pose.covariance.begin());
	pubOdomAftMapped_->publish(odomAftMapped_);

	geometry_msgs::msg::TransformStamped tf_msg;
	tf_msg.header.stamp = convert_to_rclcpp_time(lidar_end_time);
	tf_msg.header.frame_id = "camera_init";
	tf_msg.child_frame_id = "body";
	tf_msg.transform.translation.x = odom.x;
	tf_msg.transform.translation.y = odom.y;
	tf_msg.transform.translation.z = odom.z;
	tf_msg.transform.rotation.x = odom.qx;
	tf_msg.transform.rotation.y = odom.qy;
	tf_msg.transform.rotation.z = odom.qz;
	tf_msg.transform.rotation.w = odom.qw;
	static tf2_ros::TransformBroadcaster br(node_);
	br.sendTransform(tf_msg);
}

void ROS2_Interface::publish_path(const PoseData& pose, double lidar_end_time)
{
	msgBodyPose_.header.stamp = convert_to_rclcpp_time(lidar_end_time);
	msgBodyPose_.header.frame_id = "camera_init";
	msgBodyPose_.pose.position.x = pose.x;
	msgBodyPose_.pose.position.y = pose.y;
	msgBodyPose_.pose.position.z = pose.z;
	msgBodyPose_.pose.orientation.x = pose.qx;
	msgBodyPose_.pose.orientation.y = pose.qy;
	msgBodyPose_.pose.orientation.z = pose.qz;
	msgBodyPose_.pose.orientation.w = pose.qw;

	static int jjj = 0;
	if (++jjj % 10 == 0)
	{
		path_.poses.push_back(msgBodyPose_);
		path_.header.stamp = msgBodyPose_.header.stamp;
		path_.header.frame_id = "camera_init";
		pubPath_->publish(path_);
	}
}

void ROS2_Interface::publish_odometryhighfreq(const Pose& pose)
{
	if (!ros_ok()) return;

	nav_msgs::msg::Odometry msg;
	msg.header.stamp = convert_to_rclcpp_time(pose._timestamp);
	msg.header.frame_id = "camera_init";
	msg.child_frame_id = "body";
	msg.pose.pose.position.x = pose._x;
	msg.pose.pose.position.y = pose._y;
	msg.pose.pose.position.z = pose._z;
	msg.pose.pose.orientation.x = pose._qx;
	msg.pose.pose.orientation.y = pose._qy;
	msg.pose.pose.orientation.z = pose._qz;
	msg.pose.pose.orientation.w = pose._qw;
	pubOdomHighFreq_->publish(msg);

	geometry_msgs::msg::TransformStamped tf_msg;
	tf_msg.header.stamp = convert_to_rclcpp_time(pose._timestamp);
	tf_msg.header.frame_id = "camera_init";
	tf_msg.child_frame_id = "body_hf";
	tf_msg.transform.translation.x = msg.pose.pose.position.x;
	tf_msg.transform.translation.y = msg.pose.pose.position.y;
	tf_msg.transform.translation.z = msg.pose.pose.position.z;
	tf_msg.transform.rotation.x = msg.pose.pose.orientation.x;
	tf_msg.transform.rotation.y = msg.pose.pose.orientation.y;
	tf_msg.transform.rotation.z = msg.pose.pose.orientation.z;
	tf_msg.transform.rotation.w = msg.pose.pose.orientation.w;
	static tf2_ros::TransformBroadcaster br_hf(node_);
	br_hf.sendTransform(tf_msg);
}
