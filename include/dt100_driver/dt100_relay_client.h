#pragma once

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/asio.hpp>

using boost::asio::ip::address;
using boost::asio::ip::udp;

class DT100RelayClient {
 public:
  /**
   * @brief Explicit Constructor
   *
   * @param[in] io_service - provides the core I/O functionality for users of
   * the asynchronous I/O objects provided by boost::asio
   */
  explicit DT100RelayClient(boost::asio::io_service& io_service);

 private:
  /**
   * @brief Asynchronous reciever for socket. Communication is
   * established over UDP, with the socket bound to the endpoint defined by ip
   * address and port parameters
   */
  void Receive();

  /**
   * @brief Error handler for binding to socket
   * @param[in] error - error thrown when binding to socket fails
   * @param[in] bytes_transferred - number of bytes recieved during transfer
   */
  void HandleReceive(const boost::system::error_code& error,
                     std::size_t bytes_transferred);

  /**
   * @brief Converts recieved buffer in 83P format to point cloud
   * messages representative of sonar scans
   */
  void ParseDT100();

  // remote endpoint parameters
  int port_{4040};
  std::string ip_address_{"192.168.0.4"};

  // I/O objects
  udp::socket socket_;
  udp::endpoint remote_endpoint_;
  boost::array<unsigned char, 2175> recv_buffer_;

  // node properties
  std::string frame_ID_{"DT100"};
  ros::NodeHandle nh_ = ros::NodeHandle{"DT100"};
  ros::Publisher publisher_ =
      nh_.advertise<sensor_msgs::PointCloud2>("sonar_scans", 1);
};
