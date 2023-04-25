#include <rclcpp_action/rclcpp_action.hpp>
#pragma once


// Implementation based on ROS2 Nav2 Action Server
namespace uam_navigator
{

enum class NavStatus { RUNNING, SUCCESS, FAILURE, CANCELED};
enum class NavCompletionStatus {SUCCEEDED, FAILED, CANCELED};

template<class ActionT>
class NavActionServer
{
public:
	using GoalHandleActionT = rclcpp_action::ServerGoalHandle<ActionT>;
	using OnGoalReceivedCallback = std::function<rclcpp_action::GoalResponse (typename ActionT::Goal::ConstSharedPtr)>;
	using OnCancelCallback = std::function<rclcpp_action::CancelResponse (const std::shared_ptr<GoalHandleActionT>)> ;
	using OnLoopCallback = std::function<NavStatus ()>;
	using OnCompletionCallback = std::function<void (typename ActionT::Result::SharedPtr, NavCompletionStatus)>;

	template<typename NodeT>
	explicit NavActionServer(
		const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
		const std::string & action_name,
		std::chrono::milliseconds nav_loop_duration,
		OnGoalReceivedCallback on_goal_received_callback,
		OnLoopCallback on_loop_callback,
		OnCancelCallback on_cancel_callback,
		OnCompletionCallback on_completion_callback)
	: action_name_(action_name),
	  node_(parent),
	  nav_loop_duration_(nav_loop_duration),
	  on_goal_received_callback_(on_goal_received_callback),
	  on_loop_callback_(on_loop_callback),
	  on_cancel_callback_(on_cancel_callback),
	  on_completion_callback_(on_completion_callback)
    {
	  	auto node = node_.lock();
	  	logger_ = node->get_logger();
	  	clock_ = node->get_clock();
	}

	~NavActionServer() = default;

	bool on_configure()
	{
		using namespace std::placeholders;

		auto node = node_.lock();
		if (!node) {
			throw std::runtime_error{"Failed to lock node"};
		}

		action_server_ = rclcpp_action::create_server<ActionT>(
				node,
				action_name_,
				std::bind(&NavActionServer::handle_goal, this, _1, _2),
				std::bind(&NavActionServer::handle_cancel, this, _1),
				std::bind(&NavActionServer::handle_accepted, this, _1));
	}

	bool on_activate()
	{
		activate();
		return true;
	}
	bool on_deactivate()
	{
		deactivate();
		return true;
	}
	bool on_cleanup()
	{
		action_server_.reset();
		current_handle_.reset();
		return true;
	}

	void publish_feedback(typename std::shared_ptr<typename ActionT::Feedback> feedback)
	{
		if (!is_active(current_handle_)) {
			error_msg("Attempting to publish feedback while current goal handle is not active");
			return;
		}
		current_handle_->publish_feedback(feedback);
	}
protected:
	rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_;
	rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface_;
	rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface_;
	rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface_;

	typename rclcpp_action::Server<ActionT>::SharedPtr action_server_;
	std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> current_handle_;
	std::string action_name_;
	rclcpp::Node::SharedPtr client_node_;
	rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
	rclcpp::Clock::SharedPtr clock_;
	rclcpp::Logger logger_;
	std::chrono::milliseconds nav_loop_duration_;

	OnGoalReceivedCallback on_goal_received_callback_;
	OnCancelCallback on_cancel_callback_;
	OnLoopCallback on_loop_callback_;
	OnCompletionCallback on_completion_callback_;

	constexpr bool is_active(const std::shared_ptr<GoalHandleActionT> handle) const
	{
		return handle != nullptr && handle->is_active();
	}

	void info_msg(const std::string & msg) const
	{
		RCLCPP_INFO(
				logger_,
				"[%s] [ActionServer] %s", action_name_.c_str(), msg.c_str());
	}

	void debug_msg(const std::string & msg) const
	{
		RCLCPP_DEBUG(
				logger_,
				"[%s] [ActionServer] %s", action_name_.c_str(), msg.c_str());
	}

	void error_msg(const std::string & msg) const
	{
		RCLCPP_ERROR(
				logger_,
				"[%s] [ActionServer] %s", action_name_.c_str(), msg.c_str());
	}

	void warn_msg(const std::string & msg) const
	{
		RCLCPP_WARN(
				logger_,
				"[%s] [ActionServer] %s", action_name_.c_str(), msg.c_str());
	}

private:
	mutable std::recursive_mutex update_mutex_;
	std::future<void> execution_future_;
	bool stop_execution_{false};
	bool server_active_{false};

	void activate()
	{
		std::lock_guard<std::recursive_mutex> lock(update_mutex_);
		server_active_ = true;
		stop_execution_ = false;
	}

	void deactivate()
	{
		debug_msg("Deactivating Navigator Action Server...");

		{
			std::lock_guard<std::recursive_mutex> lock(update_mutex_);
			server_active_ = false;
			stop_execution_ = true;
		}

		if (!execution_future_.valid()) {
			return;
		}
	}
	rclcpp_action::GoalResponse handle_goal(
			const rclcpp_action::GoalUUID &,
			std::shared_ptr<const typename ActionT::Goal>)
	{
		std::lock_guard<std::recursive_mutex> lock(update_mutex_);

		if (!server_active_) {
			return rclcpp_action::GoalResponse::REJECT;
		}

		debug_msg("Received navigator mode command");
		return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}

	rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleActionT> handle)
	{
		std::lock_guard<std::recursive_mutex> lock(update_mutex_);

		if (!handle->is_action()) {
			warn_msg("Received request for goal cancellation while handle is inactive.");
			return rclcpp_action::CancelResponse::REJECT;
		}

		debug_msg("Received request for goal cancellation");
		return rclcpp_action::CancelResponse::ACCEPT;
	}

	void handle_accepted(const std::shared_ptr<GoalHandleActionT> handle)
	{
		std::lock_guard<std::recursive_mutex> lock(update_mutex_);
		if (!is_active(current_handle_)) {
			debug_msg("Navigation mode accepted"
			 "Spawning new thread asynchronously");
			execution_future_ = std::async(std::launch::async,
		        [this]() {
			        auto result = std::make_shared<typename ActionT::Result>();

			        NavStatus nav_status = work();
			        switch (nav_status) {
			        	case NavStatus::SUCCESS:
					        on_completion_callback_(result, NavCompletionStatus::SUCCEEDED);
			        	    break;
			        	case NavStatus::FAILURE:
			        		on_completion_callback_(result, NavCompletionStatus::FAILED);
					        break;
			        	case NavStatus::CANCELED:
			        		on_completion_callback_(result, NavCompletionStatus::CANCELED);
			        		break;
		                // TODO: Add a better condition for this
				        case NavStatus::RUNNING:
				        	error_msg("Action server ending while navigator running");
				        	break;
			        }
			});
		}
	}

	NavStatus work()
	{
		rclcpp::WallRate nav_loop_rate(nav_loop_duration_);
		NavStatus nav_status = NavStatus::RUNNING;
		while (rclcpp::ok() && !stop_execution_ && is_active(current_handle_) && nav_status == NavStatus::RUNNING) {
			debug_msg("Executing the navigation loop...");
			try {
				nav_status = on_loop_callback_();
				nav_loop_rate.sleep();
				if (nav_status == NavStatus::CANCELED) {
					stop_execution_ = true;
				}
			} catch (std::exception & ex) {
				RCLCPP_ERROR(
						logger_,
						"Action server failed while executing action callback: \"%s\"", ex.what());
				terminate();
				return NavStatus::FAILURE;
			}

			if (stop_execution_) {
				warn_msg("Stopping execution of navigator action server thread per request.");
				terminate();
				return NavStatus::CANCELED;
			}

		}

		debug_msg("Worker thread completed.");

		return nav_status;
	}

	void terminate(
			typename std::shared_ptr<typename ActionT::Result> result = std::make_shared<typename ActionT::Result>())
	{
		std::lock_guard<std::recursive_mutex> lock(update_mutex_);

		if (is_active(current_handle_)) {
			if (current_handle_->is_cancelling()) {
				warn_msg("Cancelling current navigator mode.");
				current_handle_->cancelled(result);
			} else {
				warn_msg("Aborting handle.");
				current_handle_->abort(result);
			}
			current_handle_.reset();
		}
	}
};


}