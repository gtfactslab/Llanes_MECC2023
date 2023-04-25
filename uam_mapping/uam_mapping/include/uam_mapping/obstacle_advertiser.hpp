#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometric_shapes/bodies.h>
#include <uam_mapping_msgs/msg/obstacle.hpp>
#include <uam_mapping_msgs/msg/obstacle_array.hpp>

namespace uam_mapping
{
class ObstacleAdvertiser: public rclcpp::Node
{
public:
	ObstacleAdvertiser();
	virtual ~ObstacleAdvertiser() = default;
	using ObstacleIdPositionMap = std::unordered_map<std::string, std::pair<double, double>>;
private:

	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<uam_mapping_msgs::msg::ObstacleArray>::SharedPtr obstacle_array_pub_;
	ObstacleIdPositionMap obstacles_map_;

	std::vector<std::string> obstacle_ids_;
	std::vector<double> obstacles_x_;
	std::vector<double> obstacles_y_;

	void publish_obstacles();
};
}