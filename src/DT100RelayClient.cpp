#include <dt100_driver/DT100RelayClient.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/time.h>
#include <string>

DT100RelayClient::DT100RelayClient(boost::asio::io_service &io_service)
    : socket_(io_service) {
  nh_.getParam("ip_address", ip_address_);
  ROS_INFO("DT1OO packets will be sent to %s:%i", ip_address_.c_str(), port_);

  socket_.open(udp::v4());
  socket_.bind(udp::endpoint(address::from_string(ip_address_), port_));
  Receive();
}

void DT100RelayClient::Receive() {
  socket_.async_receive_from(
      boost::asio::buffer(recv_buffer_), remote_endpoint_,
      boost::bind(&DT100RelayClient::HandleReceive, this,
                  boost::asio::placeholders::error,
                  boost::asio::placeholders::bytes_transferred));
}

void DT100RelayClient::HandleReceive(const boost::system::error_code &error,
                                     std::size_t bytes_transferred) {
  if (error) {
    ROS_INFO("Receive failed: %s", error.message().c_str());
    return;
  }
  ROS_INFO_STREAM("Received: " << bytes_transferred << " bytes");
  ParseDT100();
  Receive();
}

void DT100RelayClient::ParseDT100() {
  // Parse DT100 bitstreams by:
  // 1) getting sonar parameters (latency in 100 microseconds)
  double num_beams =
      static_cast<double>(recv_buffer_[70] << 8 | recv_buffer_[71]);
  double range_res =
      static_cast<double>(recv_buffer_[85] << 8 | recv_buffer_[86]);
  double ping_latency =
      static_cast<double>(recv_buffer_[118] << 8 | recv_buffer_[119]);
  double data_latency =
      static_cast<double>(recv_buffer_[120] << 8 | recv_buffer_[121]);

  // 2) converting sonar range and bearing measurements into xyz point cloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = num_beams;
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.points.resize(cloud.width * cloud.height);

  int j = 0;
  double theta = 210;
  double del_theta = 120 / num_beams;

  for (int i = 256; i < (256 + 2 * num_beams - 1); i += 2) {
    unsigned char r = (recv_buffer_[i] << 8 | recv_buffer_[i + 1]);
    double range = static_cast<double>(r) * range_res / 1000;
    cloud.points[j].x = range * cos(theta * M_PI / 180);
    cloud.points[j].y = range * sin(theta * M_PI / 180);
    cloud.points[j].z = 0;
    theta += del_theta;
    j++;
  }

  // 3) stamping sonar pings in ROS time
  ros::Duration latency_1(data_latency * 1e-4);
  ros::Duration latency_2(ping_latency * 1e-4);
  ros::Time stamp = ros::Time::now() - latency_1 - latency_2;

  // 4) Publish PointCloud Message
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.stamp = stamp;
  msg.header.frame_id = frameID_;
  publisher_.publish(msg);
  ros::spinOnce();
}