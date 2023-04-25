#pragma once

#include <string>
#include <mutex>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "uam_util/lifecycle_node.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "uam_navigator_msgs/action/navigator_command.hpp"

namespace uam_navigator
{

class Navigator;

class NavigatorMode
{
public:
	using Ptr = std::shared_ptr<NavigatorMode>;

	~NavigatorMode() = default;

	virtual bool configure(
			const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
			std::shared_ptr<uam_navigator::Navigator> navigator,
			std::string nav_mode) = 0;
	virtual bool activate(uam_navigator_msgs::action::NavigatorCommand::Goal::ConstSharedPtr goal) = 0;
	virtual bool deactivate() = 0;
	virtual bool cleanup() = 0;
	virtual void publish_navigator_setpoint() = 0;

protected:
	rclcpp::Clock::SharedPtr clock_;
	rclcpp::Logger logger_{rclcpp::get_logger("navigator")};
	std::shared_ptr<Navigator> navigator_;
	rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
	std::string nav_mode_name_;
}; // class NavigatorMode

//
//class NavigatorMode : public NavigatorModeBase
//{
//public:
//	NavigatorMode() = default;
//	~NavigatorMode() = default;
//
//protected:
//	bool on_configure(
//			const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
//			const NavMode & nav_mode) final
//	{
//		node_ = parent;
//
//		bool ok = true;
//		if(!configure()) {
//			ok = false;
//		}
//		return ok;
//	}
//
//	bool on_activate(const nav_msgs::msg::Odometry & start, const nav_msgs::msg::Odometry & goal) final
//	{
//		bool ok = true;
//		if (!activate(start, goal)) {
//			ok = false;
//		}
//		return ok;
//	}
//
//	bool on_deactivate() final
//	{
//		bool ok = true;
//		if(!deactivate()) {
//			ok = false;
//		}
//		return ok;
//	}
//
//	bool on_cleanup() final
//	{
//		bool ok = true;
//		if(!cleanup()) {
//			ok = false;
//		}
//		return ok;
//	}
//
//	virtual bool configure() = 0;
//	virtual bool activate(const nav_msgs::msg::Odometry & start, const nav_msgs::msg::Odometry & goal) = 0;
//	virtual bool deactivate() = 0;
//	virtual bool cleanup() = 0;
//
//	rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
//}; // class NavigatorMode

} // namespace uam_navigator