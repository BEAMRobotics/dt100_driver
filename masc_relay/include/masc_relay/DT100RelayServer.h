#pragma once

#include <ros/publisher.h>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/asio.hpp>

namespace masc_sync{

using boost::asio::ip::udp;

class DT100RelayServer {
public:
  explicit DT100RelayServer(boost::asio::io_service& io_service);

private:
  void StartReceive();

  void HandleReceive(const boost::system::error_code& error,
    std::size_t bytes_transferred);

  void HandleSend(boost::shared_ptr<std::string>,
    const boost::system::error_code&,
    size_t);

  void ParseDT100();

  udp::socket socket_;
  udp::endpoint remote_endpoint_;

  int destination_port_ = 4040;
  std::string destination_ip_ = "192.168.0.4";

  boost::array<char, 2175> recv_buffer_;
  std::string frameID_ = "DT100";

  ros::NodeHandle nh_ = ros::NodeHandle{};
  ros::Publisher publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("DT100_relay", 1);

};

}
