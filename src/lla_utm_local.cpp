#include "lla_utm_converter.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<ULConverter>());

  rclcpp::shutdown();
  return 0;
}
