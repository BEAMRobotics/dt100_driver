#include <ros/ros.h>
#include <stdlib.h>
#include <signal.h>

void shutdown(int sig) {
  system("VBoxManage controlvm \"XP_32\" acpipowerbutton");
  ros::shutdown();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "dt100_virtual_machine");
  ros::NodeHandle nh;

  ROS_INFO("dt100_virtual_machine initialize");

  system("VBoxManage startvm \"XP_32\"");
  signal(SIGINT, shutdown);

  ros::spin();
  return 0;
}