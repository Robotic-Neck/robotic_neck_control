#include "pid_library/pid_library.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PidLibrary>();

  int loopFreq;
  node->declare_parameter<int>("loop_freq", 10);
  node->get_parameter("loop_freq", loopFreq);
  rclcpp::Rate loop_rate(loopFreq);
  
  while (rclcpp::ok())
  {
    node->UsePidLibrary();
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  
  rclcpp::shutdown();
  return 0;
}