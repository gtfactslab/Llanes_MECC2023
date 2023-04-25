#include <chrono>

#include "navigator_modes/takeoff.hpp"
#include "navigator.hpp"

namespace uam_navigator
{

Takeoff::Takeoff()
{
}
Takeoff::~Takeoff()
{
}

bool Takeoff::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                               std::shared_ptr<uam_navigator::Navigator> navigator,
                               std::string nav_mode)
{
	node_ = parent;
	auto node = node_.lock();
	nav_mode_name_ = nav_mode;
	logger_ = node->get_logger();
	clock_ = node->get_clock();
	navigator_ = navigator;

	RCLCPP_INFO(logger_, "Configuring");

	node->declare_parameter("takeoff.altitude", 0.6);
	node->get_parameter("takeoff.altitude", takeoff_altitude_);
	node->declare_parameter("takeoff.position_tolerance", 0.05);
	node->get_parameter("takeoff.position_tolerance", takeoff_position_tolerance_);
	node->declare_parameter("takeoff.velocity_tolerance", 0.3);
	node->get_parameter("takeoff.velocity_tolerance", takeoff_velocity_tolerance_);
	node->declare_parameter("takeoff.update_frequency", 30.0);
	node->get_parameter("takeoff.update_frequency", update_frequency_);
	return true;
}

bool Takeoff::activate(uam_navigator_msgs::action::NavigatorCommand::Goal::ConstSharedPtr goal)
{
	auto node = node_.lock();
	RCLCPP_INFO(logger_, "Activating takeoff flight mode");
	(void)goal;
	vehicle_odom_ = navigator_->get_current_odom();
	takeoff_position_.header.frame_id = vehicle_odom_.header.frame_id;
	takeoff_position_.header.stamp = vehicle_odom_.header.stamp;
	takeoff_position_.pose.position.x = vehicle_odom_.pose.pose.position.x;
	takeoff_position_.pose.position.y = vehicle_odom_.pose.pose.position.y;
	takeoff_position_.pose.position.z = -takeoff_altitude_;

	timer_ = node->create_wall_timer(
			std::chrono::duration<double>(1.0/update_frequency_),
			std::bind(&Takeoff::on_loop_callback, this));
	return true;
}

void Takeoff::on_loop_callback()
{
	vehicle_odom_ = navigator_->get_current_odom();
	if (mission_complete() && !mission_complete_) {
		mission_complete_ = true;
		navigator_->loiter();
	}
}

bool Takeoff::deactivate()
{
	auto node = node_.lock();
	RCLCPP_INFO(logger_, "Deactivating takeoff flight mode");
	timer_->cancel();
	timer_.reset();
	takeoff_position_ = geometry_msgs::msg::PoseStamped();
	return true;
}

bool Takeoff::cleanup()
{
	node_.reset();
	navigator_.reset();
	return true;
}

void Takeoff::publish_navigator_setpoint()
{
	auto node = node_.lock();

	nav_msgs::msg::Odometry odom;
	odom.header.frame_id = takeoff_position_.header.frame_id;
	odom.header.stamp = node->get_clock()->now();
	odom.child_frame_id = "baselink_frd";
	odom.pose.pose = takeoff_position_.pose;
	navigator_->publish_odometry_setpoint(odom);
}

bool Takeoff::mission_complete()
{
	bool mission_complete = false;
	if (std::hypot(
			vehicle_odom_.pose.pose.position.x - takeoff_position_.pose.position.x,
			vehicle_odom_.pose.pose.position.y - takeoff_position_.pose.position.y,
			vehicle_odom_.pose.pose.position.z - takeoff_position_.pose.position.z) <= takeoff_position_tolerance_
		&&
		std::hypot(
				vehicle_odom_.twist.twist.linear.x,
				vehicle_odom_.twist.twist.linear.y,
				vehicle_odom_.twist.twist.linear.z) <= takeoff_velocity_tolerance_) {
		mission_complete = true;
	}
	return mission_complete;
}

} // namespace uam_navigator