#include <uam_mapping/obstacle_advertiser.hpp>

using namespace uam_mapping;
using namespace std::chrono;

ObstacleAdvertiser::ObstacleAdvertiser() : Node("static_obstacle_advertiser")
{

	// ----------------------- Publishers --------------------------
	obstacle_array_pub_ =
			this->create_publisher<uam_mapping_msgs::msg::ObstacleArray>("uam_mapping/obstacles", 10);

	// ----------------------- Subscribers --------------------------
	declare_parameter("obstacle_ids", rclcpp::ParameterValue(std::vector<std::string>()));
	declare_parameter("obstacles_x", rclcpp::ParameterValue(std::vector<double>()));
	declare_parameter("obstacles_y", rclcpp::ParameterValue(std::vector<double>()));

	get_parameter("obstacle_ids", obstacle_ids_);
	get_parameter("obstacles_x", obstacles_x_);
	get_parameter("obstacles_y", obstacles_y_);

	if (obstacle_ids_.size() == obstacles_x_.size() && obstacles_x_.size() == obstacles_y_.size()){
		for (size_t i = 0; i < obstacle_ids_.size(); i++) {
			obstacles_map_.insert({obstacle_ids_[i], std::make_pair(obstacles_x_[i], obstacles_y_[i])});
		}
	} else {
		RCLCPP_ERROR(get_logger(), "Invalid size of obstacle parameter vectors");
	}

	auto timer_callback = [this]() -> void
	{
		publish_obstacles();
	};

	timer_ = this->create_wall_timer(100ms, timer_callback);
}

void ObstacleAdvertiser::publish_obstacles()
{
	uam_mapping_msgs::msg::ObstacleArray obstacles;

	obstacles.header.frame_id = "map";
	obstacles.header.stamp = this->get_clock()->now();

	for (auto const & obstacle : obstacles_map_) {
		uam_mapping_msgs::msg::Obstacle obstacle_msg;
		obstacle_msg.obstacle.type = shape_msgs::msg::SolidPrimitive::BOX;
		obstacle_msg.obstacle_id = std::stoi(obstacle.first);
		obstacle_msg.obstacle.dimensions = {0.4191, 0.4191, 0.9779};
		obstacle_msg.pose.position.x = obstacle.second.first;
		obstacle_msg.pose.position.y = obstacle.second.second;
		obstacle_msg.pose.position.z = obstacle_msg.obstacle.dimensions[2] / 2.0;
		obstacles.obstacles.push_back(obstacle_msg);
	}

	obstacle_array_pub_->publish(obstacles);
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<uam_mapping::ObstacleAdvertiser>());
	rclcpp::shutdown();
	return 0;
}