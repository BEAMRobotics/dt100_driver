#include <dt100_driver/dt100_relay_client.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/time.h>
#include <string>

DT100RelayClient::DT100RelayClient(boost::asio::io_service &io_service)
    : socket_(io_service) {
  ROS_INFO("DT1OO packets will be sent to %s:%i on the host machine",
           ip_address_.c_str(), port_);

  // open and bind to socket
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
                                     const std::size_t &bytes_transferred) {
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
  // 1) getting sonar parameters, where:
  //
  //    num_beams: units [m]
  //    sector_size: units [degrees]
  //    range_resolution: units [mm]
  //    data_latency: units [100 microseconds]

  int num_beams = static_cast<int>(recv_buffer_[70] << 8 | recv_buffer_[71]);
  float sector_size =
      static_cast<float>(recv_buffer_[74] << 8 | recv_buffer_[75]);
  float acoustic_range =
      static_cast<float>(recv_buffer_[79] << 8 | recv_buffer_[80]);
  float range_resolution =
      static_cast<float>(recv_buffer_[85] << 8 | recv_buffer_[86]);
  float data_latency =
      static_cast<float>(recv_buffer_[120] << 8 | recv_buffer_[121]);

  // 2) converting sonar range and bearing measurements into XYZ point cloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = num_beams;
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.points.resize(cloud.width * cloud.height);

  int j = 0;
  float theta = start_angle_;
  const float del_theta = sector_size / static_cast<float>(num_beams);

  // iterate through beams
  for (int i = 256; i < (256 + 2 * num_beams - 1); i += 2) {
    // get range
    float range =
        static_cast<float>(recv_buffer_[i] << 8 | recv_buffer_[i + 1]);
    float corrected_range = range * range_resolution / 1000;  // units [m]

    // populate point cloud
    cloud.points[j].x = corrected_range * cos(theta * M_PI / 180);
    cloud.points[j].y = corrected_range * sin(theta * M_PI / 180);
    cloud.points[j].z = 0;

    // increment
    theta += del_theta;
    j++;
  }

  // 3) stamping sonar pings in ROS time
  ros::Duration latency(data_latency * 1e-4);
  ros::Time stamp = ros::Time::now() - latency;

  // 4) Publish PointCloud Message
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_ID_;
  publisher_.publish(msg);
  ros::spinOnce();
}