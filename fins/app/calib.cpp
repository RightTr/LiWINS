#include <csignal>

#include "core/liwinCalib.h"
#include "ros_interface/ros_interface.h"
#include "ros_interface/ros_utils.h"

static LIWINCalib *g_calib = nullptr;

static void sig_handle(int sig)
{
  (void)sig;
  if (g_calib)
    g_calib->set_exit();
}

int main(int argc, char **argv)
{
#ifdef USE_ROS1
  ros::init(argc, argv, "fins_calib");
  init_ros_node();
#elif defined(USE_ROS2)
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("fins_calib");
  init_ros_node(node);
#endif

  load_config();
  register_pub_sub();

  LIWINCalib calib;
  g_calib = &calib;
  signal(SIGINT, sig_handle);

  calib.init();
  calib.run();

#ifdef USE_ROS1
  ros::shutdown();
#elif defined(USE_ROS2)
  rclcpp::shutdown();
#endif

  return 0;
}
