// Copyright (c) 2019 Samsung Research America (Source)
// Modified version of planner_server.hpp from Navigation2 project (https://navigation.ros.org/)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#pragma once

#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "uam_planner_msgs/action/compute_path_to_pose.hpp"
#include "uam_util/simple_action_server.hpp"
#include "planner_base.hpp"
#include "planner_exceptions.hpp"
#include "uam_util/lifecycle_node.hpp"
namespace uam_planner
{

class PlannerServer : public rclcpp_lifecycle::LifecycleNode {
public:
	explicit PlannerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
	~PlannerServer();

	using PlannerMap = std::unordered_map<std::string, PlannerBase::Ptr>;

	nav_msgs::msg::Path get_path(
			const geometry_msgs::msg::PoseStamped & start,
			const geometry_msgs::msg::PoseStamped & goal,
			const std::string & planner_id);

protected:
	using ActionToPose = uam_planner_msgs::action::ComputePathToPose;
	using ActionToPoseGoal = ActionToPose::Goal;
	using ActionServerToPose = uam_util::SimpleActionServer<ActionToPose>;

	template<typename T>
	bool is_server_inactive(std::unique_ptr<uam_util::SimpleActionServer<T>> & action_server);
	template<typename T>
	bool is_cancel_requested(std::unique_ptr<uam_util::SimpleActionServer<T>> & action_server);
	uam_util::CallbackReturn  on_configure(const rclcpp_lifecycle::State & state) override;
	uam_util::CallbackReturn  on_activate(const rclcpp_lifecycle::State & state) override;
	uam_util::CallbackReturn  on_deactivate(const rclcpp_lifecycle::State & state) override;
	uam_util::CallbackReturn  on_cleanup(const rclcpp_lifecycle::State & state) override;
	uam_util::CallbackReturn  on_shutdown(const rclcpp_lifecycle::State & state) override;
	void compute_path();
	void publish_path(const nav_msgs::msg::Path & path);
	void exception_warning(
			const geometry_msgs::msg::PoseStamped & start,
			const geometry_msgs::msg::PoseStamped & goal,
			const std::string & planner_id,
			const std::exception & ex);

	std::unique_ptr<ActionServerToPose> action_server_pose_;

	// Planner
	PlannerMap planners_;
	pluginlib::ClassLoader<uam_planner::PlannerBase> planner_loader_;
	std::vector<std::string> planner_ids_;
	std::vector<std::string> planner_types_;
	std::vector<std::string> default_ids_;
	std::vector<std::string> default_types_;
	std::string planner_ids_concat_;

	// Clock
	rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

	std::string global_frame_;
	std::string robot_base_frame_;

	// Path publisher for visualization
	rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

private:

};
}