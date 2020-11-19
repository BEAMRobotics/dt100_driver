#pragma once

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/asio.hpp>

namespace mari_sync {

using boost::asio::ip::address;
using boost::asio::ip::udp;

class DT100RelayClient {
 public:
  explicit DT100RelayClient(boost::asio::io_service& io_service);

 private:
  int port_ = 4040;
  std::string ip_ = "192.168.0.4";
  udp::socket socket_;
  udp::endpoint remote_endpoint_;
  boost::array<unsigned char, 2175> recv_buffer_;
  std::string frameID_ = "DT100";
  ros::NodeHandle nh_ = ros::NodeHandle{};
  ros::Publisher publisher_ =
      nh_.advertise<sensor_msgs::PointCloud2>("mari_relay", 1);

  void Receive();

  void HandleReceive(const boost::system::error_code& error,
                     std::size_t bytes_transferred);

  void ParseDT100();
};

}  // namespace mari_sync
