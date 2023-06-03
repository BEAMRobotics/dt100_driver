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
   * @brief Asynchronous receiver for socket. Communication is
   * established over UDP, with the socket bound to the endpoint defined by ip
   * address and port parameters
   */
  void Receive();

  /**
   * @brief Error handler for binding to socket
   * @param[in] error - error thrown when binding to socket fails
   * @param[in] bytes_transferred - number of bytes received during transfer
   */
  void HandleReceive(const boost::system::error_code& error,
                     const std::size_t& bytes_transferred);

  /**
   * @brief Converts received buffer (in 83P Profile Point format) to point
   * cloud messages representative of sonar scans. Currently, this method
   * assumes:
   * *
   * 1) Output point cloud is referenced to the sonar frame, with the first beam
   * centered at 210 degrees in the 2D cartessian coordinate system. All
   * subsequent beam angles are incremented counter clockwise. Should the user
   * change the tilt angle in DT100.exe (which is typically done to better
   * suite the beam forming algorithms employed by DT100.exe), the output point
   * cloud must be transformed as part of a separate extrinsic calibration
   *
   * 2) Output point cloud is stamped according to the host machine's clock and
   * adjusted for latencies recorded in the received buffer
   *
   * 3) Intensity bytes are not included in the received buffer. This would
   * require refactoring of the for loop used to read range information should
   * default beam forming settings are altered in DT100.exe
   *
   * 4) The sound velocity of acoustic beams in water is 1500.0 m/s. This value
   * is typical and has been adopted in lue of a calibrated value
   *
   * 5) A minimum range of 0.1m and maximum range of 100m have been set
   * according to DT100.exe specifications
   */
  void ParseDT100();

  // ip address
  const std::string ip_address_{"192.168.0.4"};

  // remote endpoint
  const int port_{4040};

  // I/O objects
  udp::socket socket_;
  udp::endpoint remote_endpoint_;
  boost::array<unsigned char, 2175> recv_buffer_;

  // DT100 properties
  const float start_angle_ = 210;  // units [degrees]

  // node properties
  const std::string frame_ID_{"DT100"};
  ros::NodeHandle nh_ = ros::NodeHandle{"DT100"};
  ros::Publisher publisher_ =
      nh_.advertise<sensor_msgs::PointCloud2>("sonar_scans", 10);
};
