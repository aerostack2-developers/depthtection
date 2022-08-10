#include "rclcpp/rclcpp.hpp"
#include "depthtection.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Depthtection>());
  rclcpp::shutdown();
  return 0;
}
