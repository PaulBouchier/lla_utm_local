#include "lla_utm_converter.hpp"

//#include "tf/transform_datatypes.h"//for tf, getYaw
//#include "std_msgs/Float64.h"
//#include "std_msgs/Float64MultiArray.h"

#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <algorithm>
#include<cctype>
#include <string>
#include <math.h>

using std::placeholders::_1;
using namespace std;


ULConverter::ULConverter() : Node("lla_utm_converter")
{
  this->declare_parameter("utm_zone", "14n");
  // TODO: deal with not declaring parameter better
  string zone_param = this->get_parameter("utm_zone").as_string();
  // TODO: test illegal values throw
  zone_ = stoi(zone_param.substr(0, 2));
  std::string hemi_s = zone_param.substr(2,1);
  transform(hemi_s.begin(), hemi_s.end(), hemi_s.begin(), ::toupper);
  if (zone_ < 1 || zone_ > 60 || (hemi_s != "N" && hemi_s != "S"))
  {
    // TODO: support 1 or 2 chars for zone number
    RCLCPP_FATAL(this->get_logger(),
      "Invalid UTM zone %s - must be 2-chars, 01-60, then N or S", zone_param.c_str());
    throw std::runtime_error("Invalid zone");
  }

  gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps", 10);
  xyz_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("utm", 10);
  gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>
    ("gps/fix", 10, std::bind(&ULConverter::GPSCallback, this, _1 ));
  xyz_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>
    ("utm_pose", 10, std::bind(&ULConverter::PoseCallback, this, _1 ));

  RCLCPP_INFO(this->get_logger(), "lla_utm_local is running");
}

void ULConverter::GPSCallback(const sensor_msgs::msg::NavSatFix& msg)
{
  basic bs;
  RCLCPP_INFO(this->get_logger(), "Got data from GPS (lat, lon, alt): %f, %f, %f"
    , msg.latitude, msg.longitude, msg.altitude);

  double easting;
  double northing;
  int lon0  = (zone_ * 6) - 183;

  tm_->Forward(lon0, msg.latitude, msg.longitude, easting, northing);

  easting += kE0_;
  northing += (hemi_ == "N") ? kNN_ : kNS_;

  utm_.push_back(easting);
  utm_.push_back(northing);
  utm_.push_back(msg.altitude);

  geometry_msgs::msg::PoseStamped local_pose;

  local_pose.header = msg.header;
  local_pose.pose.position.x = utm_[0] + bs.bx;
  local_pose.pose.position.y = utm_[1] + bs.by;
  local_pose.pose.position.z = utm_[2] + bs.bz;

  RCLCPP_INFO(this->get_logger(), "Converted to (n, e, z): %f %f %f"
    , local_pose.pose.position.x, local_pose.pose.position.y, local_pose.pose.position.z);

  xyz_pub_->publish(local_pose);

  utm_.clear();
}

void ULConverter::PoseCallback(const geometry_msgs::msg::PoseStamped& msg)
{ 
  basic bs;
  sensor_msgs::msg::NavSatFix gps;
  gps.header = msg.header;

  //position in UTM
  double easting = -bs.bx + msg.pose.position.x ;
  double northing = -bs.by + msg.pose.position.y ;
  double pz = -bs.bz + msg.pose.position.z;//height not change, just output unit in meters
  RCLCPP_DEBUG(this->get_logger(), "Got UTM data (x, y, z): %f %f %f"
    , easting, northing, pz);
  
  double latitude;
  double longitude;

  int lon0 = (zone_ * 6) - 183;

  easting  -= kE0_;
  northing -= (hemi_ == "N") ? kNN_ : kNS_; 

  tm_->Reverse(lon0, easting, northing, latitude, longitude); 

  lla_.push_back(latitude);
  lla_.push_back(longitude);
  lla_.push_back(pz);
  
  gps.latitude  = lla_[0];
  gps.longitude = lla_[1];
  gps.altitude  = lla_[2];
  
  RCLCPP_DEBUG(this->get_logger(), "Converted to (lat lon, alt): %f %f %f"
    , lla_[0], lla_[1], lla_[2]);
  
  gps_pub_->publish(gps);

  lla_.clear();//don't forget this
}

std::vector<double> ULConverter::get_lla()
{
  return lla_;
}
