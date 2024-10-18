#ifndef LLA_UTM_CONVERTER_H
#define LLA_UTM_CONVERTER_H

//resource from wiki  Universal Transverse Mercator coordinate system. WGS 84
#include <GeographicLib/TransverseMercator.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include <vector>
#include <string>


class ULConverter : public rclcpp::Node
{ 
  public:
    ULConverter();
    void PoseCallback(const geometry_msgs::msg::PoseStamped & msgs);
    void GPSCallback(const sensor_msgs::msg::NavSatFix & msgs);
    std::vector<double> get_lla();
  private:
    // origin point
    struct basic
    {
      double bx = 0.0;//-594929.9431329881;//east
      double by = 0.0;//-4139043.529676078;//north
      double bz = 0.0;//unit in m
    };

    const double kNN_      = 0;
    const double kNS_      = 10000000.0;
    const double kE0_      = 500000.0;

    // values to initialize GeographicLib with, passed in initializer list
    const double at      = 6378137.0; //unit in m
    const double fla    = 1.0/298.257223563; 
    const double k0      = 0.9996;

    //tm needs initial values for constructor:  at(earth radius),fla(inverse flattening), k0
    std::shared_ptr<GeographicLib::TransverseMercator> tm_ = std::make_shared<GeographicLib::TransverseMercator>(at, fla, k0);  

    std::vector<double> lla_;
    std::vector<double> utm_;
    //double utm_[3];

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr xyz_pub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr xyz_sub_;

    int zone_;
    std::string hemi_;
};

#endif


