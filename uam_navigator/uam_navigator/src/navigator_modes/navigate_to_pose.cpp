#include <chrono>

#include "navigator_modes/navigate_to_pose.hpp"
#include "navigator.hpp"

namespace uam_navigator
{

NavigateToPose::NavigateToPose()
{
}
NavigateToPose::~NavigateToPose()
{
}

bool NavigateToPose::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
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

	node->declare_parameter("navigate_to_pose.planner", std::string("rrtx_static"));
	node->get_parameter("navigate_to_pose.planner", planner_name_);
	node->declare_parameter("navigate_to_pose.waypoint_position_tolerance", 0.1);
	node->get_parameter("navigate_to_pose.waypoint_position_tolerance", waypoint_position_tolerance_);
	node->declare_parameter("navigate_to_pose.goal_position_tolerance", 0.1);
	node->get_parameter("navigate_to_pose.goal_position_tolerance", goal_position_tolerance_);
	node->declare_parameter("navigate_to_pose.goal_velocity_tolerance", 0.1);
	node->get_parameter("navigate_to_pose.goal_velocity_tolerance", goal_velocity_tolerance_);
	node->declare_parameter("navigate_to_pose.update_frequency", 100.0);
	node->get_parameter("navigate_to_pose.update_frequency", update_frequency_);
	planner_client_ptr_ = rclcpp_action::create_client<PlannerActionT>(node, "compute_path_to_pose");
	return true;
}

bool NavigateToPose::activate(uam_navigator_msgs::action::NavigatorCommand::Goal::ConstSharedPtr goal)
{

	auto node = node_.lock();
	//TODO: check configured state before activating

	RCLCPP_INFO(logger_, "Attempting to activate navigate to pose flight mode");
	if (!planner_client_ptr_->wait_for_action_server()) {
		RCLCPP_ERROR(logger_, "Planner server is not available after waiting");
		return false;
	}
	nav_msgs::msg::Odometry tmp_odom = navigator_->get_current_odom();
	// Remove when I migrate to just map static frame
	if (tmp_odom.header.frame_id == "map") {
		vehicle_odom_.header.frame_id = "map";
		vehicle_odom_.pose.pose = tmp_odom.pose.pose;
	} else if (tmp_odom.header.frame_id == "map_ned") {
		vehicle_odom_.header.frame_id = "map";
		vehicle_odom_.pose.pose.position.x = tmp_odom.pose.pose.position.y;
		vehicle_odom_.pose.pose.position.y  = tmp_odom.pose.pose.position.x;
		vehicle_odom_.pose.pose.position.z  = -tmp_odom.pose.pose.position.z;
		vehicle_odom_.twist.twist.linear.x = tmp_odom.twist.twist.linear.y;
		vehicle_odom_.twist.twist.linear.y = tmp_odom.twist.twist.linear.x;
		vehicle_odom_.twist.twist.linear.z = -tmp_odom.twist.twist.linear.z;
	}
	start_pose_.header.stamp = vehicle_odom_.header.stamp;
	start_pose_.header.frame_id = vehicle_odom_.header.frame_id;
	start_pose_.pose = vehicle_odom_.pose.pose;
	goal_pose_.header.stamp = goal->goal.header.stamp;
	goal_pose_.header.frame_id = goal->goal.header.frame_id;
	goal_pose_.pose = goal->goal.pose;
	auto planner_goal_msg = PlannerActionT::Goal();

	planner_goal_msg.planner_id = planner_name_;
	planner_goal_msg.start = start_pose_;
	planner_goal_msg.goal = goal_pose_;

	RCLCPP_INFO(logger_, "Requesting path from planner (%s)", planner_name_.c_str());
	path_received_ = false;
	planner_response_received_ = false;
	mission_complete_ = false;
	auto send_goal_options = rclcpp_action::Client<PlannerActionT>::SendGoalOptions();
	send_goal_options.goal_response_callback =
			std::bind(&NavigateToPose::goal_response_callback, this, std::placeholders::_1);
	send_goal_options.result_callback =
			std::bind(&NavigateToPose::result_callback, this, std::placeholders::_1);
	planner_client_ptr_->async_send_goal(planner_goal_msg, send_goal_options);

	while (rclcpp::ok() && !planner_response_received_) {}

	if (path_received_) {
		timer_ = node->create_wall_timer(
				std::chrono::duration<double>(1.0/update_frequency_),
				std::bind(&NavigateToPose::on_loop_callback, this));
	}

	return path_received_;
}

void NavigateToPose::on_loop_callback()
{
	nav_msgs::msg::Odometry tmp_odom = navigator_->get_current_odom();
	// Remove when I migrate to just map static frame
	if (tmp_odom.header.frame_id == "map") {
		vehicle_odom_.header.frame_id = "map";
		vehicle_odom_.pose.pose = tmp_odom.pose.pose;
	} else if (tmp_odom.header.frame_id == "map_ned") {
		vehicle_odom_.header.frame_id = "map";
		vehicle_odom_.pose.pose.position.x = tmp_odom.pose.pose.position.y;
		vehicle_odom_.pose.pose.position.y  = tmp_odom.pose.pose.position.x;
		vehicle_odom_.pose.pose.position.z  = -tmp_odom.pose.pose.position.z;
		vehicle_odom_.twist.twist.linear.x = tmp_odom.twist.twist.linear.y;
		vehicle_odom_.twist.twist.linear.y = tmp_odom.twist.twist.linear.x;
		vehicle_odom_.twist.twist.linear.z = -tmp_odom.twist.twist.linear.z;
	}
	update_waypoint();

	if (mission_complete() && !mission_complete_) {
		mission_complete_ = true;
		navigator_->loiter();
	}
}

void NavigateToPose::goal_response_callback(std::shared_future<PlannerGoalHandleActionT::SharedPtr> future)
{
	auto node = node_.lock();
	auto goal_handle = future.get();
	if (!goal_handle) {
		RCLCPP_ERROR(logger_, "Goal was rejected by server");
		planner_response_received_ = true;
	} else {
		RCLCPP_INFO(logger_, "Goal accepted by server, waiting for result");
	}
}

void NavigateToPose::result_callback(const PlannerGoalHandleActionT::WrappedResult & result)
{
	auto node = node_.lock();

	planner_response_received_ = true;
	switch (result.code) {
		case rclcpp_action::ResultCode::SUCCEEDED:
			path_received_ = true;
			current_path_waypoint_ = 0;
			path_ = result.result->path;
			return;
		case rclcpp_action::ResultCode::ABORTED: RCLCPP_ERROR(logger_, "Goal was aborted");
			return;
		case rclcpp_action::ResultCode::CANCELED: RCLCPP_ERROR(logger_, "Goal was canceled");
			return;
		default: RCLCPP_ERROR(logger_, "Unknown result code");
			return;
	}
}
bool NavigateToPose::deactivate()
{
	auto node = node_.lock();
	RCLCPP_INFO(logger_, "Deactivating navigate to pose flight mode");
	timer_->cancel();
	timer_.reset();
	start_pose_ = geometry_msgs::msg::PoseStamped();
	goal_pose_ = geometry_msgs::msg::PoseStamped();
	path_ = nav_msgs::msg::Path();
	path_received_ = false;
	path_requested_ = false;
	mission_complete_ = false;

	return true;
}

bool NavigateToPose::cleanup()
{
	node_.reset();
	navigator_.reset();
	return true;
}

void NavigateToPose::publish_navigator_setpoint()
{
	auto node = node_.lock();
	nav_msgs::msg::Odometry odom;
	odom.header.frame_id = "map_ned";
	odom.header.stamp = clock_->now();
	odom.child_frame_id = "baselink_frd";
	odom.pose.pose.position.x = path_.poses[current_path_waypoint_].pose.position.y;
	odom.pose.pose.position.y = path_.poses[current_path_waypoint_].pose.position.x;
	odom.pose.pose.position.z = -path_.poses[current_path_waypoint_].pose.position.z;

	navigator_->publish_odometry_setpoint(odom);
}

void NavigateToPose::update_waypoint()
{
	if (std::hypot(
			vehicle_odom_.pose.pose.position.x - path_.poses[current_path_waypoint_].pose.position.x,
			vehicle_odom_.pose.pose.position.y - path_.poses[current_path_waypoint_].pose.position.y,
			vehicle_odom_.pose.pose.position.z - path_.poses[current_path_waypoint_].pose.position.z) <= waypoint_position_tolerance_
		&& (size_t)(current_path_waypoint_ + 1) < path_.poses.size()) {
		current_path_waypoint_ += 1;
	}
}

bool NavigateToPose::mission_complete()
{
	bool mission_complete = false;
	if (std::hypot(
			vehicle_odom_.pose.pose.position.x - goal_pose_.pose.position.x,
			vehicle_odom_.pose.pose.position.y - goal_pose_.pose.position.y,
			vehicle_odom_.pose.pose.position.z - goal_pose_.pose.position.z) <= goal_position_tolerance_
		&&
		std::hypot(
				vehicle_odom_.twist.twist.linear.x,
				vehicle_odom_.twist.twist.linear.y,
				vehicle_odom_.twist.twist.linear.z) <= goal_velocity_tolerance_) {
		mission_complete = true;
	}
	auto node = node_.lock();

	return mission_complete;
}

} // namespace uam_navigator