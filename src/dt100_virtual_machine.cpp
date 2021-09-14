#include <dt100_driver/global_variables.h>

#include <ros/ros.h>
#include <stdlib.h>
#include <signal.h>

void shutdown(int sig) {
  system("VBoxManage controlvm \"Windows_XP_32_DT100\" acpipowerbutton");
  ros::shutdown();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "dt100_virtual_machine");
  try {
    ros::NodeHandle nh = ros::NodeHandle{"DT100_VM"};

    // get params
    std::string bridge_adapter;
    bool headless{true}; 

    nh.getParam("bridge_adapter", bridge_adapter);
    nh.getParam("headless", headless);

    if (bridge_adapter.empty()) {
      throw std::invalid_argument{"bridge adapter must be specified."};
    }

    // pass system calls
    system("VBoxManage modifyvm \"Windows_XP_32_DT100\" --nic1 none");
    std::string bridged_cmd =
        "VBoxManage modifyvm \"Windows_XP_32_DT100\" --nic1 bridged "
        "--bridgeadapter1 " +
        bridge_adapter;
    system(bridged_cmd.c_str());
    if (headless){
      system("VBoxManage startvm \"Windows_XP_32_DT100\" --type headless");
    } else {
      system("VBoxManage startvm \"Windows_XP_32_DT100\"");
    }
    signal(SIGINT, shutdown);

    ros::spin();
  } catch (std::exception &e) {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}
