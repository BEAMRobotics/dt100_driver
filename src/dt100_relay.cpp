#include <dt100_driver/dt100_relay_client.h>

#include <ros/ros.h>
#include <boost/asio.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "dt100_relay");
  try {
    boost::asio::io_service io_service;
    DT100RelayClient client(io_service);

    while (ros::ok()) {
      io_service.run();
    }
  } catch (std::exception &e) {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}
