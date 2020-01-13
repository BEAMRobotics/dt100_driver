#include "masc_relay/DT100RelayServer.h"

#include <beam_utils/angles.hpp>
#include <beam_utils/time.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <chrono>
#include <ctime>
#include <ros/time.h>
#include <sys/time.h>
#include <string>

namespace masc_sync {

DT100RelayServer::DT100RelayServer(boost::asio::io_service& io_service)
    : socket_(io_service, udp::endpoint(udp::v4(), 4040)) {

  nh_.param<std::string>("destination_ip", destination_ip_, "192.168.0.4");
  ROS_INFO("DT100 packets will be sent to %s:%i",destination_ip_.c_str(), destination_port_);
}

void DT100RelayServer::StartReceive() {

  socket_.async_receive_from(
      boost::asio::buffer(recv_buffer_),
      remote_endpoint_,
      boost::bind(&DT100RelayServer::HandleReceive,
                  this,
                  boost::asio::placeholders::error,
                  boost::asio::placeholders::bytes_transferred));
}

void DT100RelayServer::HandleReceive(const boost::system::error_code& error,
                                    std::size_t /*bytes_transferred*/) {

  if (!error || error == boost::asio::error::message_size) {

    boost::shared_ptr<std::string> message(
        new std::string(recv_buffer_.data(), 256)); // CHECK

    socket_.async_send_to(
        boost::asio::buffer(*message),
        remote_endpoint_,
        boost::bind(&DT100RelayServer::HandleSend,
                    this,
                    message,
                    boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred));

    StartReceive();
  }
}

void DT100RelayServer::HandleSend(boost::shared_ptr<std::string> /*message*/,
                                    const boost::system::error_code& /*error*/,
                                    std::size_t /*bytes_transferred*/) {}


template<size_t N>
std::string convertToString(boost::array<char, N>& a, int idx, int size) {
    int i;
    std::string s = "";
    for (i = idx; i < size; i++) {
        s = s + a[i];
    }
    return s;
}

void DT100RelayServer::ParseDT100() {

    // Parse DT100 bitstreams by:
    // 1) extracting relavent sonar information
    int num_beams = (recv_buffer_[70]<<8)|recv_buffer_[71];
    double range_res = (recv_buffer_[85]<<8)|recv_buffer_[86];
    double range = 0; // initialize
    double theta = 210; // referenced from positive x
    double del_theta = 120/num_beams; // constant 120 deg swath

    // 2) converting sonar range and bearing data into point cloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = num_beams;
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize (cloud.width * cloud.height);
    int j = 0;

    for (int i = 256; i < (256 + 4*num_beams-1); i+=4){
        range = ((recv_buffer_[i]<<8)|recv_buffer_[i+1])*range_res/1000;
        cloud.points[j].x = range*cos(beam::deg2rad(theta));
        cloud.points[j].y = range*sin(beam::deg2rad(theta));
        cloud.points[j].z = 0;
        theta+=del_theta;
        j++;
    }

    // 3) extracing time of sonar ping into ROS time
    std::string str1=convertToString(recv_buffer_,8,11); // "DD-MMM-YYYY"
    std::string str2=convertToString(recv_buffer_,20,8); // "HH:MM:SS"
    std::string str3=convertToString(recv_buffer_,112,4); // ".mmm"

    // ping and data latency are recorded in microseconds
    long int ping_latency = ((recv_buffer_[118]<<8) | recv_buffer_[119])/100;  // ### Verify accuracy
    long int data_latency = ((recv_buffer_[120]<<8) | recv_buffer_[121])/100;

    str1.append(str2);

    std::tm tm = {};
    std::stringstream ss(str1);
    ss >> std::get_time(&tm, "%d-%b-%Y %H:%M:%S");
    auto tp = std::chrono::system_clock::from_time_t(std::mktime(&tm));

    // ### include beam::TimePoint somehow

    std::string::size_type sz;
    tp += std::chrono::milliseconds(std::stoi(str3.substr(1,3),&sz));
    tp -= std::chrono::microseconds(ping_latency);
    tp -= std::chrono::microseconds(data_latency);

    // #### REPLACE WITH FUNCTION
    uint32_t seconds, nanoseconds;
    seconds = std::round(tp.time_since_epoch().count() / 1000000000);
    nanoseconds = tp.time_since_epoch().count() - seconds * 1000000000;
    ros::Time stamp(seconds, nanoseconds);
    // ###
    //ros::Time stamp = beam::ChronoToRosTime((beam::TimePoint)tp);

    // // 4) Publish PointCloud Message
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = frameID_;
    publisher_.publish(msg);
    ros::spinOnce();

}

}
