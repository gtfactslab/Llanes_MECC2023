// ROS2 includes
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/clock.hpp"

// other libraries
#include <Eigen/Dense>
#include <chrono>
#include "navigator_modes/navigator_mode.hpp"
#include "navigator_modes/loiter.hpp"
#include "navigator_modes/takeoff.hpp"
#include "navigator_modes/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "uam_navigator_msgs/msg/nav_cmd.hpp"
#include "uam_navigator_msgs/action/navigator_command.hpp"
#include "uam_navigator_msgs/msg/navigator_status.hpp"
#include "uam_util/simple_action_server.hpp"
#include "navigator_exceptions.hpp"

namespace uam_navigator
{

//using nav_state = std::variant()

class Navigator: public rclcpp_lifecycle::LifecycleNode
{
public:
	using ActionNavigatorCommand = uam_navigator_msgs::action::NavigatorCommand;
	using ActionNavigatorCommandGoal = ActionNavigatorCommand::Goal;
	using ActionServerNavigatorCommand = uam_util::SimpleActionServer<ActionNavigatorCommand>;

	explicit Navigator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
	~Navigator();

	using NavigatorMap = std::unordered_map<std::string, NavigatorMode::Ptr>;
	using TransitionMap = std::unordered_map<std::string, std::vector<std::string>>;

	uam_util::CallbackReturn  on_configure(const rclcpp_lifecycle::State & state) override;
	uam_util::CallbackReturn  on_activate(const rclcpp_lifecycle::State & state) override;
	uam_util::CallbackReturn  on_deactivate(const rclcpp_lifecycle::State & state) override;
	uam_util::CallbackReturn  on_cleanup(const rclcpp_lifecycle::State & state) override;
	uam_util::CallbackReturn  on_shutdown(const rclcpp_lifecycle::State & state) override;

	nav_msgs::msg::Odometry get_current_odom() {return vehicle_odom_;}
	void publish_odometry_setpoint(nav_msgs::msg::Odometry odom_msg);

	void land();
	void takeoff();
	void loiter();

	std::shared_ptr<uam_navigator::Navigator> nav_shared_from_this()
	{
		return std::static_pointer_cast<uam_navigator::Navigator>(rclcpp_lifecycle::LifecycleNode::shared_from_this());
	}
protected:
	// ----------------------- Publishers --------------------------
	rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr position_setpoint_publisher_;
	rclcpp_lifecycle::LifecyclePublisher<uam_navigator_msgs::msg::NavigatorStatus>::SharedPtr navigator_status_publisher_;


	// ----------------------- Subscribers --------------------------
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vehicle_odometry_sub_;

	// ROS2 Variables
	rclcpp::TimerBase::SharedPtr timer_;
	nav_msgs::msg::Odometry vehicle_odom_;
	std::unique_ptr<ActionServerNavigatorCommand> nav_command_action_server_;
	rclcpp_action::Client<ActionNavigatorCommand>::SharedPtr nav_command_action_client_;

	// Class Variables
	TransitionMap navigator_transitions_;
	NavigatorMap navigators_;
	std::string current_nav_mode_;
	std::string requested_nav_mode_;
	std::vector<std::string> default_navigator_plugin_ids_;
	std::vector<std::string> navigator_plugin_ids_;
	std::vector<std::string> navigator_plugin_types_;
	bool mission_complete_{false};

	// Class methods
	void on_loop();
	void command_callback();
	template<typename T>
	bool is_server_inactive(std::unique_ptr<uam_util::SimpleActionServer<T>> & action_server);
	template<typename T>
	bool is_cancel_requested(std::unique_ptr<uam_util::SimpleActionServer<T>> & action_server);
}; // Class RRTX
}

