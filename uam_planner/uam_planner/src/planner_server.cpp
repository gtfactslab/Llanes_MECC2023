// Copyright (c) 2019 Samsung Research America (Source)
// Modified version of planner_server.cpp from Navigation2 project (https://navigation.ros.org/)
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

#include "uam_util/node_utils.hpp"
#include "uam_planner/planner_server.hpp"

namespace uam_planner
{

PlannerServer::PlannerServer(const rclcpp::NodeOptions &options)
: rclcpp_lifecycle::LifecycleNode("planner_server", "", options),
  planner_loader_("uam_planner", "uam_planner::PlannerBase"),
  default_ids_{"rrtx_static"},
  default_types_{"rrtx_static_planner::RrtxStatic"}
{
	declare_parameter("planner_plugins", default_ids_);
	declare_parameter("global_frame", rclcpp::ParameterValue(std::string("map")));
	declare_parameter("robot_base_frame", rclcpp::ParameterValue(std::string("base_link")));
//	declare_parameter("expected_planner_frequency", 1.0);

	get_parameter("planner_plugins", planner_ids_);
	if (planner_ids_ == default_ids_) {
		for (size_t i = 0; i < default_ids_.size(); ++i) {
			declare_parameter(default_ids_[i] + ".plugin", default_types_[i]);
		}
	}
}

PlannerServer::~PlannerServer()
{
	planners_.clear();
}

uam_util::CallbackReturn
PlannerServer::on_configure(const rclcpp_lifecycle::State &state)
{
	RCLCPP_INFO(get_logger(), "Configuring");

	get_parameter("global_frame", global_frame_);
	get_parameter("robot_base_frame", robot_base_frame_);
	planner_types_.resize(planner_ids_.size());

	auto node = shared_from_this();

	for (size_t i=0; i != planner_ids_.size(); i++) {
		try{
			planner_types_[i] = uam_util::get_plugin_type_param(node, planner_ids_[i]);
			PlannerBase::Ptr planner = planner_loader_.createSharedInstance(planner_types_[i]);
			RCLCPP_INFO(
					get_logger(), "Created planner plugin %s of type %s",
					planner_ids_[i].c_str(), planner_types_[i].c_str());
			planner->configure(node, planner_ids_[i]);
			planners_.insert({planner_ids_[i], planner});
		} catch (const pluginlib::PluginlibException & ex){
			RCLCPP_FATAL(
					get_logger(), "Failed to create planner. Exception %s",
					ex.what());
			return uam_util::CallbackReturn::FAILURE;
		}
	}

	for (size_t i = 0; i != planner_ids_.size(); i++) {
		planner_ids_concat_ += planner_ids_[i] + std::string(" ");
	}

	RCLCPP_INFO(
			get_logger(),
			"Planner Server has %s planners available.", planner_ids_concat_.c_str());

//	double expected_planner_frequency;
//	get_parameter("expected_planner_frequency", expected_planner_frequency);
//	if (expected_planner_frequency > 0) {
//		max_planner_duration_ = 1.0 / expected_planner_frequency;
//	} else {
//		RCLCPP_WARN(
//				get_logger(),
//				"The expected planner frequency parameter is %.4f Hz. The value should to be greater"
//				" than 0.0 to turn on duration overrrun warning messages", expected_planner_frequency);
//		max_planner_duration_ = 0.0;
//	}

	path_publisher_ = create_publisher<nav_msgs::msg::Path>("planner_server/path", 1);

	action_server_pose_ = std::make_unique<ActionServerToPose>(
			shared_from_this(),
			"compute_path_to_pose",
			std::bind(&PlannerServer::compute_path, this),
			nullptr,
			std::chrono::milliseconds(500),
			true);

	return uam_util::CallbackReturn::SUCCESS;
}

uam_util::CallbackReturn
PlannerServer::on_activate(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(get_logger(), "Activating");

	path_publisher_->on_activate();
	action_server_pose_->activate();

	PlannerMap::iterator it;
	for (it = planners_.begin(); it != planners_.end(); ++it) {
		it->second->activate();
	}

	auto node = shared_from_this();


	return uam_util::CallbackReturn::SUCCESS;
}

uam_util::CallbackReturn
PlannerServer::on_deactivate(const rclcpp_lifecycle::State &state)
{
	RCLCPP_INFO(get_logger(), "Deactivating");

	action_server_pose_->deactivate();
	path_publisher_->on_deactivate();

	PlannerMap::iterator it;
	for (it = planners_.begin(); it != planners_.end(); ++it) {
		it->second->deactivate();
	}

	return uam_util::CallbackReturn::SUCCESS;
}

uam_util::CallbackReturn
PlannerServer::on_cleanup(const rclcpp_lifecycle::State &state)
{
	RCLCPP_INFO(get_logger(), "Cleaning up");

	action_server_pose_.reset();
	path_publisher_.reset();

	PlannerMap::iterator it;
	for (it = planners_.begin(); it != planners_.end(); ++it) {
		it->second->cleanup();
	}

	planners_.clear();
	return uam_util::CallbackReturn::SUCCESS;
}

uam_util::CallbackReturn
PlannerServer::on_shutdown(const rclcpp_lifecycle::State &state)
{
	RCLCPP_INFO(get_logger(), "Shutting down");

	return uam_util::CallbackReturn::SUCCESS;
}

template<typename T>
bool PlannerServer::is_server_inactive(std::unique_ptr<uam_util::SimpleActionServer<T>> & action_server)
{
	if (action_server == nullptr || !action_server->is_server_active()) {
		RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
		return true;
	}

	return false;
}

template<typename T>
bool PlannerServer::is_cancel_requested(
		std::unique_ptr<uam_util::SimpleActionServer<T>> & action_server)
{
	if (action_server->is_cancel_requested()) {
		RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling planning action.");
		action_server->terminate_all();
		return true;
	}

	return false;
}


void PlannerServer::publish_path(const nav_msgs::msg::Path & path)
{
	auto msg = std::make_unique<nav_msgs::msg::Path>(path);
	if (path_publisher_->is_activated() && path_publisher_->get_subscription_count() > 0) {
		path_publisher_->publish(std::move(msg));
	}
}

void PlannerServer::compute_path()
{
	auto start_time = steady_clock_.now();

	auto goal = action_server_pose_->get_current_goal();
	auto result = std::make_shared<ActionToPose::Result>();

	geometry_msgs::msg::PoseStamped start_pose;

	try{
		if (is_server_inactive(action_server_pose_) || is_cancel_requested(action_server_pose_)) {
			return;
		}

		result->path = get_path(goal->start, goal->goal, goal->planner_id);

		publish_path(result->path);
		auto cycle_duration = steady_clock_.now() - start_time;
		result->planning_time = cycle_duration;
		action_server_pose_->succeeded_current(result);
	} catch (uam_planner::InvalidPlanner & ex) {
		exception_warning(goal->start, goal->goal, goal->planner_id, ex);
		result->error_code = ActionToPoseGoal::INVALID_PLANNER;
		action_server_pose_->terminate_current(result);
	} catch (uam_planner::StartOccupied & ex) {
		exception_warning(goal->start, goal->goal, goal->planner_id, ex);
		result->error_code = ActionToPoseGoal::START_OCCUPIED;
		action_server_pose_->terminate_current(result);
	} catch (uam_planner::GoalOccupied & ex) {
		exception_warning(goal->start, goal->goal, goal->planner_id, ex);
		result->error_code = ActionToPoseGoal::GOAL_OCCUPIED;
		action_server_pose_->terminate_current(result);
	} catch (uam_planner::NoValidPathCouldBeFound & ex) {
		exception_warning(goal->start, goal->goal, goal->planner_id, ex);
		result->error_code = ActionToPoseGoal::NO_VALID_PATH;
		action_server_pose_->terminate_current(result);
	} catch (uam_planner::PlannerTimedOut & ex) {
		exception_warning(goal->start, goal->goal, goal->planner_id, ex);
		result->error_code = ActionToPoseGoal::TIMEOUT;
		action_server_pose_->terminate_current(result);
	} catch (uam_planner::StartOutsideMapBounds & ex) {
		exception_warning(goal->start, goal->goal, goal->planner_id, ex);
		result->error_code = ActionToPoseGoal::START_OUTSIDE_MAP;
		action_server_pose_->terminate_current(result);
	} catch (uam_planner::GoalOutsideMapBounds & ex) {
		exception_warning(goal->start, goal->goal, goal->planner_id, ex);
		result->error_code = ActionToPoseGoal::GOAL_OUTSIDE_MAP;
		action_server_pose_->terminate_current(result);
	} catch (std::exception & ex) {
		exception_warning(goal->start, goal->goal, goal->planner_id, ex);
		result->error_code = ActionToPoseGoal::UNKNOWN;
		action_server_pose_->terminate_current(result);
	}
}

nav_msgs::msg::Path PlannerServer::get_path(
		const geometry_msgs::msg::PoseStamped & start,
		const geometry_msgs::msg::PoseStamped & goal,
		const std::string & planner_id)
{
	RCLCPP_DEBUG(
			get_logger(), "Attempting to a find path from (%.2f, %.2f) to "
			              "(%.2f, %.2f).", start.pose.position.x, start.pose.position.y,
			goal.pose.position.x, goal.pose.position.y);

	if (planners_.find(planner_id) != planners_.end()) {
		return planners_[planner_id]->create_path(start, goal);
	} else {
		if (planners_.size() == 1 && planner_id.empty()) {
			RCLCPP_WARN_ONCE(
					get_logger(), "No planners specified in action call. "
					              "Server will use only plugin %s in server."
					              " This warning will appear once.", planner_ids_concat_.c_str());
			return planners_[planners_.begin()->first]->create_path(start, goal);
		} else {
			RCLCPP_ERROR(
					get_logger(), "planner %s is not a valid planner. "
					              "Planner names are: %s", planner_id.c_str(),
					planner_ids_concat_.c_str());
			throw uam_planner::InvalidPlanner("Planner id " + planner_id + " is invalid");
		}
	}

	return nav_msgs::msg::Path();
}


void PlannerServer::exception_warning(
		const geometry_msgs::msg::PoseStamped & start,
		const geometry_msgs::msg::PoseStamped & goal,
		const std::string & planner_id,
		const std::exception & ex)
{
	RCLCPP_WARN(
			get_logger(), "%s plugin failed to plan from (%.2f, %.2f) to (%0.2f, %.2f): \"%s\"",
			planner_id.c_str(),
			start.pose.position.x, start.pose.position.y,
			goal.pose.position.x, goal.pose.position.y,
			ex.what());
}

} // namespace uam_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(uam_planner::PlannerServer)