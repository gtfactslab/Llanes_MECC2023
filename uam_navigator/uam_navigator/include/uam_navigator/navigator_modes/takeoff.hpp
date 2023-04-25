#pragma once

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "navigator_modes/navigator_mode.hpp"

namespace uam_navigator
{

class Takeoff: public NavigatorMode
{
public:
	Takeoff();
	~Takeoff();

protected:
	bool configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
	               std::shared_ptr<uam_navigator::Navigator> navigator,
	               std::string nav_mode) override;
	bool activate(uam_navigator_msgs::action::NavigatorCommand::Goal::ConstSharedPtr goal) override;
	bool deactivate() override;
	bool cleanup() override;
	void publish_navigator_setpoint() override;
	bool mission_complete();
	void on_loop_callback();

	// ROS2
	rclcpp::TimerBase::SharedPtr timer_;

	double takeoff_altitude_;
	double takeoff_position_tolerance_;
	double takeoff_velocity_tolerance_;
	geometry_msgs::msg::PoseStamped takeoff_position_;
	nav_msgs::msg::Odometry vehicle_odom_;
	bool mission_complete_{false};
	double update_frequency_;

};

}
