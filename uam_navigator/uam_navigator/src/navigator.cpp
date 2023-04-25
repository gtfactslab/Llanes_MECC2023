#include "uam_util/node_utils.hpp"
#include "navigator.hpp"

namespace uam_navigator
{
Navigator::Navigator(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("navigator", "", options), default_navigator_plugin_ids_{"navigate_to_pose_rrtx_static"}
{
	declare_parameter("navigator_plugins", default_navigator_plugin_ids_);
	get_parameter("navigator_plugins", navigator_plugin_ids_);
}

Navigator::~Navigator()
{
}

uam_util::CallbackReturn
Navigator::on_configure(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(get_logger(), "Configuring");

	auto node = shared_from_this();

	navigators_.insert({"Takeoff", std::make_shared<Takeoff>()});
//	navigators_.insert({NavMode::NAV_MODE_LAND, std::make_unique<Land>()});
	navigators_.insert({"Loiter", std::make_shared<Loiter>()});
	navigators_.insert({"navigate_to_pose_rrtx_static", std::make_shared<NavigateToPose>()});
	position_setpoint_publisher_ = node->create_publisher<nav_msgs::msg::Odometry>("/uam_navigator/position_setpoint", 10);
	navigator_status_publisher_ = node->create_publisher<uam_navigator_msgs::msg::NavigatorStatus>("/uam_navigator/navigator_status", 10);

	for (const auto & navigator : navigators_) {
		navigator.second->configure(node, nav_shared_from_this(), navigator.first);
	}

	navigator_transitions_.insert({"Takeoff", std::vector<std::string>{"Loiter", "Land"}});
	auto tmp = navigator_plugin_ids_;
	tmp.emplace_back("Land");
	navigator_transitions_.insert({"Loiter", tmp});
	for (const auto & nav_mode : navigator_plugin_ids_) {
		navigator_transitions_.insert({nav_mode, std::vector<std::string>{"Loiter", "Land"}});
	}
	navigator_transitions_.insert({"Idle", std::vector<std::string>{"Takeoff"}});

	timer_ = node->create_wall_timer(std::chrono::milliseconds(10),
								  std::bind(&Navigator::on_loop, this));

	vehicle_odometry_sub_ =
			node->create_subscription<nav_msgs::msg::Odometry>(
					"/uam_vehicle_interface/odometry",
					10,
					[this](const nav_msgs::msg::Odometry::UniquePtr msg)
					{
						vehicle_odom_ = *msg;
					});

	current_nav_mode_ = "Idle";
	requested_nav_mode_ = "Idle";

	nav_command_action_server_ = std::make_unique<ActionServerNavigatorCommand>(
			shared_from_this(),
			"send_navigator_command",
			std::bind(&Navigator::command_callback, this),
			nullptr,
			std::chrono::milliseconds(500),
			true);

	nav_command_action_client_ = rclcpp_action::create_client<ActionNavigatorCommand>(node, "send_navigator_command");

	return uam_util::CallbackReturn::SUCCESS;
}

uam_util::CallbackReturn
Navigator::on_activate(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(get_logger(), "Activating");

	position_setpoint_publisher_->on_activate();
	navigator_status_publisher_->on_activate();
	nav_command_action_server_->activate();

	auto node = shared_from_this();
	return uam_util::CallbackReturn::SUCCESS;
}

uam_util::CallbackReturn
Navigator::on_deactivate(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(get_logger(), "Deactivating");

	position_setpoint_publisher_->on_deactivate();
	navigator_status_publisher_->on_deactivate();
	nav_command_action_server_->deactivate();

	NavigatorMap ::iterator it;
	for (it = navigators_.begin(); it != navigators_.end(); ++it) {
		it->second->deactivate();
	}

	return uam_util::CallbackReturn::SUCCESS;
}

uam_util::CallbackReturn
Navigator::on_cleanup(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(get_logger(), "Cleaning up");

	vehicle_odometry_sub_.reset();
	position_setpoint_publisher_.reset();
	navigator_status_publisher_.reset();
	nav_command_action_server_.reset();
	vehicle_odom_ = nav_msgs::msg::Odometry();
	timer_->reset();

	NavigatorMap ::iterator it;
	for (it = navigators_.begin(); it != navigators_.end(); ++it) {
		it->second->cleanup();
	}

	navigators_.clear();
	return uam_util::CallbackReturn::SUCCESS;
}

uam_util::CallbackReturn
Navigator::on_shutdown(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(get_logger(), "Shutting down");

	vehicle_odometry_sub_.reset();
	position_setpoint_publisher_.reset();
	navigator_status_publisher_.reset();
	timer_->reset();

	return uam_util::CallbackReturn::SUCCESS;
}

void Navigator::on_loop()
{
	auto node = shared_from_this();

	uam_navigator_msgs::msg::NavigatorStatus nav_status_msg;
	nav_status_msg.stamp = node->get_clock()->now();
	nav_status_msg.nav_mode = current_nav_mode_;
	navigator_status_publisher_->publish(nav_status_msg);

	if (current_nav_mode_ == "Idle") {
		return;
	}


//	// Transition from command
//	if (current_nav_mode_ != requested_nav_mode_) {
//		if (std::find(
//				navigator_transitions_[current_nav_mode_].begin(),
//				navigator_transitions_[current_nav_mode_].end(),
//				requested_nav_mode_) != navigator_transitions_[current_nav_mode_].end()) {
//			bool activate_request_success = false;
//			switch (requested_nav_mode_) {
//				case NAV_MODE_TAKEOFF:
//				case NAV_MODE_LOITER:
//				{
//					activate_request_success = navigators_[requested_nav_mode_]->activate(vehicle_odom_, nav_msgs::msg::Odometry());
//					break;
//				}
//				case NAV_MODE_NAVIGATE_TO_POSE:
//				{
//					nav_msgs::msg::Odometry goal;
//					goal.header.stamp = node->get_clock()->now();
//					goal.header.frame_id = "map_ned";
//					goal.pose.pose.position.x =  3.9624;
//					goal.pose.pose.position.y =  3.3528;
//					goal.pose.pose.position.z = -0.6;
//					activate_request_success = navigators_[requested_nav_mode_]->activate(goal);
//					break;
//				}
//				case NAV_MODE_LAND:
//				{
//					RCLCPP_DEBUG(node->get_logger(), "Land mode not implemented yet.");
//					requested_nav_mode_ = current_nav_mode_;
//					break;
//				}
//			}
//			if (activate_request_success) {
//				std::cout << "Activate request success... " << std::endl;
//				if (current_nav_mode_ != NAV_MODE_IDLE) {
//					navigators_[current_nav_mode_]->deactivate();
//				}
//				current_nav_mode_ = requested_nav_mode_;
//			}
//		} else {
//			RCLCPP_DEBUG(node->get_logger(), "Requested navigator flight mode invalid transition.");
//			requested_nav_mode_ = current_nav_mode_;
//		}
//	}

	navigators_[current_nav_mode_]->publish_navigator_setpoint();

}

void Navigator::loiter()
{
	ActionNavigatorCommandGoal nav_command_request;
	nav_command_request.command = "Loiter";
	nav_command_action_client_->async_send_goal(nav_command_request);
}

void Navigator::takeoff()
{
	ActionNavigatorCommandGoal nav_command_request;
	nav_command_request.command = "Takeoff";
	nav_command_action_client_->async_send_goal(nav_command_request);
}

void Navigator::land()
{
	ActionNavigatorCommandGoal nav_command_request;
	nav_command_request.command = "Land";
	nav_command_action_client_->async_send_goal(nav_command_request);
}

void Navigator::publish_odometry_setpoint(nav_msgs::msg::Odometry odom_msg)
{
	position_setpoint_publisher_->publish(odom_msg);
}


void Navigator::command_callback()
{

	auto goal = nav_command_action_server_->get_current_goal();
	auto result = std::make_shared<ActionNavigatorCommand::Result>();

	RCLCPP_DEBUG(get_logger(), "Received command %s", goal->command.c_str());

	try {
		if (is_server_inactive(nav_command_action_server_) || is_cancel_requested(nav_command_action_server_)) {
			return;
		}
		if (navigators_.find(goal->command) == navigators_.end()) {
			throw uam_navigator::InvalidNavigator("Navigator plugin " + goal->command + " is invalid");
		}
		if (std::find(
				navigator_transitions_[current_nav_mode_].begin(),
				navigator_transitions_[current_nav_mode_].end(),
				goal->command) != navigator_transitions_[current_nav_mode_].end()) {
			if (navigators_[goal->command]->activate(goal)) {
				auto previous_nav_mode = current_nav_mode_;
				current_nav_mode_ = goal->command;
				mission_complete_ = false;
				if (previous_nav_mode != "Idle") {
					navigators_[previous_nav_mode]->deactivate();
				}
				nav_command_action_server_->succeeded_current(result);
			} else {
				throw uam_navigator::NavigatorModeActivationFailed("Failed to activate " + goal->command);
			}
		} else if (std::find(
				navigator_plugin_ids_.begin(),
				navigator_plugin_ids_.end(),
				current_nav_mode_) != navigator_plugin_ids_.end() &&
				std::find(
						navigator_plugin_ids_.begin(),
						navigator_plugin_ids_.end(),
						goal->command) != navigator_plugin_ids_.end()) {
			auto tmp_goal = *goal;
			loiter();
			nav_command_action_client_->async_send_goal(tmp_goal);
		} else {
			throw uam_navigator::InvalidTransition("Navigator plugin " + goal->command + " invalid transition");
		}
	} catch (uam_navigator::InvalidNavigator & ex) {
		result->error_code = ActionNavigatorCommandGoal::NAV_RESPONSE_INVALID_NAVIGATOR;
		nav_command_action_server_->terminate_current(result);
	} catch (uam_navigator::InvalidTransition & ex) {
		result->error_code = ActionNavigatorCommandGoal::NAV_RESPONSE_INVALID_TRANSITION;
		nav_command_action_server_->terminate_current(result);
	} catch (uam_navigator::NavigatorModeActivationFailed & ex) {
		result->error_code = ActionNavigatorCommandGoal::NAV_RESPONSE_NAVIGATOR_MODE_ACTIVATION_FAILED;
		nav_command_action_server_->terminate_current(result);
	}
}

template<typename T>
bool Navigator::is_server_inactive(std::unique_ptr<uam_util::SimpleActionServer<T>> & action_server)
{
	if (action_server == nullptr || !action_server->is_server_active()) {
		RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
		return true;
	}

	return false;
}

template<typename T>
bool Navigator::is_cancel_requested(
		std::unique_ptr<uam_util::SimpleActionServer<T>> & action_server)
{
	if (action_server->is_cancel_requested()) {
		RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling navigator command action.");
		action_server->terminate_all();
		return true;
	}

	return false;
}

} // namespace uam_navigator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(uam_navigator::Navigator)