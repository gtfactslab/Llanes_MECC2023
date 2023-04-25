#pragma once

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"

namespace uam_planner {

class PlannerBase
{
public:
	using Ptr = std::shared_ptr<PlannerBase>;

	virtual ~PlannerBase() = default;

	virtual void configure(
			const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
			std::string name) = 0;

	virtual void cleanup() = 0;
	virtual void activate() = 0;
	virtual void deactivate() = 0;

	virtual nav_msgs::msg::Path create_path(
			const geometry_msgs::msg::PoseStamped & start,
			const geometry_msgs::msg::PoseStamped & goal) = 0;
};

}