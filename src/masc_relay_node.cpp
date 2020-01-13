#include "masc_relay/DT100RelayServer.h"

#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "masc_relay_node");
  try {
    boost::asio::io_service io_service;
    masc_sync::DT100RelayServer server(io_service);
    io_service.run();

    while (ros::ok()){
      ros::spinOnce();
      ros::Duration(0.0001).sleep();
    }

  } catch (std::exception& e) { std::cerr << e.what() << std::endl; }

  return 0;
}
