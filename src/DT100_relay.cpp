#include <DT100_driver/DT100RelayClient.h>
#include <ros/ros.h>
#include <boost/asio.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "DT100_relay");
  try {
    boost::asio::io_service io_service;
    DT100RelayClient client(io_service);
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
