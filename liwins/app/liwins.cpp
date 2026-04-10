#include <csignal>

#include "ros_interface/ros_utils.h"
#include "ros_interface/ros_interface.h"
#include "laser_mapping.h"

static LaserMapping* g_mapping = nullptr;

static void sig_handle(int sig)
{
    if (g_mapping) g_mapping->set_exit();
}

int main(int argc, char** argv)
{
#ifdef USE_ROS1
    ros::init(argc, argv, "liwins");
    init_ros_node();
#elif defined(USE_ROS2)
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("liwins");
    init_ros_node(node);
#endif

    load_config();
    register_pub_sub();

    LaserMapping mapping;
    g_mapping = &mapping;
    signal(SIGINT, sig_handle);

    mapping.init();
    mapping.run();

#ifdef USE_ROS1
    ros::shutdown();
#elif defined(USE_ROS2)
    rclcpp::shutdown();
#endif

    return 0;
}
