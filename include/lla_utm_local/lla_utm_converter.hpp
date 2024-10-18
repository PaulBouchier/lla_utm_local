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
    struct local_origin
    {
      double lo_x = 0.0;//692363.740222;//east
      double lo_y = 0.0;//13670672.203989;//north
      double lo_z = 0.0;//unit in m
    } lo_;

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
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr utm_pub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr utm_sub_;

    int zone_;
    std::string hemi_;
};

#endif


