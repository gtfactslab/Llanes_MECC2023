#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <uam_mapping_msgs/msg/obstacle.hpp>
#include <uam_mapping_msgs/msg/obstacle_array.hpp>

namespace uam_visualization
{
class Visualization: public rclcpp::Node
{
public:
	Visualization();
	virtual ~Visualization() = default;

private:
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_marker_pub_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr explored_path_pub_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr frontier_path_pub_;

	rclcpp::Subscription<uam_mapping_msgs::msg::ObstacleArray>::SharedPtr obstacle_array_sub_;

	void publish_obstacle_markers(const uam_mapping_msgs::msg::ObstacleArray::SharedPtr msg);
	void publish_waypoint();
};
}