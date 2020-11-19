#include <mari_relay/DT100RelayClient.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "mari_relay_node");
  try {
    boost::asio::io_service io_service;
    mari_sync::DT100RelayClient client(io_service);
    io_service.run();

    while (ros::ok()) {
      ros::spinOnce();
      ros::Duration(0.0001).sleep();
    }

  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}
