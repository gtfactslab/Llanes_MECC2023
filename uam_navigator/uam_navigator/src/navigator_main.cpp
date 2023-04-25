// ROS2 includes
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
// other libraries
#include "navigator.hpp"


int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<uam_navigator::Navigator>();
	rclcpp::spin(node->get_node_base_interface());
	rclcpp::shutdown();
	return 0;
}