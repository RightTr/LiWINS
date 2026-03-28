#include <csignal>

#ifdef USE_ROS1
#include <ros/ros.h>
#include "ros_interface/ros1_interdace.h"
#elif defined(USE_ROS2)
#include <rclcpp/rclcpp.hpp>
#include "ros_interface/ros2_interface.h"
#endif

#include "laser_mapping.h"

static LaserMapping* g_mapping = nullptr;

static void sig_handle(int sig)
{
    if (g_mapping) g_mapping->set_exit();
}

int main(int argc, char** argv)
{
#ifdef USE_ROS1
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;

    ROS1_Interface ri(nh);

    LaserMapping mapping;
    g_mapping = &mapping;

    signal(SIGINT, sig_handle);

    mapping.set_ros_interface(&ri);
    mapping.init();
    mapping.run();
    
    ros::shutdown();  // Ensure clean ROS shutdown
#elif defined(USE_ROS2)
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("laserMapping");

    ROS2_Interface ri(node);

    LaserMapping mapping;
    g_mapping = &mapping;

    signal(SIGINT, sig_handle);

    mapping.set_ros_interface(&ri);
    mapping.init();
    mapping.run();

    rclcpp::shutdown();
#endif

    return 0;
}
