#include "navigator_modes/loiter.hpp"
#include "navigator.hpp"

namespace uam_navigator
{


Loiter::Loiter()
{
}

Loiter::~Loiter()
{
}

bool Loiter::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                       std::shared_ptr<uam_navigator::Navigator> navigator,
                       std::string nav_mode)
{
	node_ = parent;
	auto node = node_.lock();
	nav_mode_name_ = nav_mode;
	logger_ = node->get_logger();
	clock_ = node->get_clock();
	navigator_ = navigator;
	RCLCPP_INFO(node->get_logger(), "Configuring");
	return true;
}

bool Loiter::activate(uam_navigator_msgs::action::NavigatorCommand::Goal::ConstSharedPtr goal)
{
	auto node = node_.lock();
	(void)goal;
	RCLCPP_INFO(node->get_logger(), "Activating loiter flight mode");
	vehicle_odom_ = navigator_->get_current_odom();
	loiter_position_.header.frame_id = vehicle_odom_.header.frame_id;
	loiter_position_.header.stamp = vehicle_odom_.header.stamp;
	loiter_position_.pose.position.x = vehicle_odom_.pose.pose.position.x;
	loiter_position_.pose.position.y = vehicle_odom_.pose.pose.position.y;
	loiter_position_.pose.position.z = vehicle_odom_.pose.pose.position.z;
	return true;
}

bool Loiter::deactivate()
{
	auto node = node_.lock();
	RCLCPP_INFO(node->get_logger(), "Deactivating loiter flight mode");
	loiter_position_ = geometry_msgs::msg::PoseStamped();
	vehicle_odom_ = nav_msgs::msg::Odometry();
	return true;
}

bool Loiter::cleanup()
{
	node_.reset();
	navigator_.reset();
	return true;
}

void Loiter::publish_navigator_setpoint()
{
	auto node = node_.lock();
	nav_msgs::msg::Odometry odom;
	odom.header.frame_id = loiter_position_.header.frame_id;
	odom.header.stamp = clock_->now();
	odom.child_frame_id = "baselink_frd";
	odom.pose.pose = loiter_position_.pose;
	navigator_->publish_odometry_setpoint(odom);
}

}