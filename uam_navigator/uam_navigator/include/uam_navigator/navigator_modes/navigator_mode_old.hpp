#pragma once

#include <string>
#include <mutex>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav_action_server.hpp"

namespace uam_navigator
{
// Code design based on the ROS2 Nav2 project navigator
class NavigatorModeMuxer
{
public:
	NavigatorModeMuxer() : current_navigator_(std::string("")) {};

	bool is_navigating()
	{
		std::scoped_lock lock(mutex_);
		return !current_navigator_.empty();
	}

	void start_navigating(const std::string & navigator_name)
	{
		std::scoped_lock lock(mutex_);
		if (!current_navigator_.empty()) {
			RCLCPP_ERROR(
					rclcpp::get_logger("NavigatorMuxer"),
					"Navigation start request while another navigation task is running.");
		}
		current_navigator_ = navigator_name;
	}

	void stop_navigating(const std::string & navigator_name)
	{
		std::scoped_lock lock(mutex_);
		if (current_navigator_ != navigator_name) {
			RCLCPP_ERROR(
					rclcpp::get_logger("NavigatorMuxer"),
					"Navigation stop request sent while another navigation task is running.");
		} else {
			current_navigator_ = std::string("");
		}
	}
protected:
	std::string current_navigator_;
	std::mutex mutex_;
};

class NavigatorModeBase
{
public:
	NavigatorModeBase() = default;
	~NavigatorModeBase() = default;

	virtual bool on_configure(
			rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
			NavigatorModeMuxer * nav_mode_muxer) = 0;

	virtual bool on_activate() = 0;
	virtual bool on_deactivate() = 0;
	virtual bool on_cleanup() = 0;
};

template<class ActionT>
class NavigatorMode : public NavigatorModeBase
{
public:
	using Ptr = std::shared_ptr<NavigatorMode<ActionT>>;

	NavigatorMode() : NavigatorModeBase()
	{
		nav_mode_muxer_ = nullptr;
	}

	bool on_configure(
			rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
			NavigatorModeMuxer * nav_mode_muxer) final
	{
		auto node = parent.lock();
		logger_ = node->get_logger();
		clock_ = node->get_clock();
		nav_mode_muxer_ = nav_mode_muxer;

		nav_action_server_ = std::make_unique<NavActionServer<ActionT>>(
				node,
				std::bind(&NavigatorMode::on_goal_received, this, std::placeholders::_1),
				std::bind(&NavigatorMode::on_loop, this),
				std::bind(&NavigatorMode::on_completion, this, std::placeholders::_1));
		bool ok = true;
		if (!nav_action_server_->on_configure()){
			ok = false;
		}

		return configure(node) && ok;
	}

	bool on_activate() final
	{
		bool ok = true;

		if (!nav_action_server_->on_activate()) {
			ok = false;
		}

		return activate() && ok;
	}

	bool on_deactivate() final
	{
		bool ok = true;
		if (!nav_action_server_->on_deactivate()) {
			ok = false;
		}

		return deactivate() && ok;
	}

	bool on_cleanup() final
	{
		bool ok = true;
		if (!nav_action_server_->on_cleanup()) {
			ok = false;
		}
		nav_action_server_.reset();
		return cleanup() && ok;
	}
	virtual std::string get_name() = 0;
protected:
	bool on_goal_received(typename ActionT::Goal::ConstSharedPtr goal)
	{
		if (nav_mode_muxer_->is_navigating()) {
			RCLCPP_ERROR(
					logger_,
					"Rejecting navigation request from %s while another navigator is running",get_name());
			return false;
		}

		bool goal_accepted = goal_received(goal);

		if (goal_accepted) {
			nav_mode_muxer_->start_navigating(get_name());
		}

		return goal_accepted;
	}


	virtual bool goal_received(typename ActionT::Goal::ConstSharedPtr goal) = 0;
	virtual void on_loop() = 0;
	void on_completion(typename ActionT::Goal::SharedPtr result, NavCompletionStatus nav_completion_status)
	{
		nav_mode_muxer_->stop_navigating(get_name());
		goal_completed(result);
	}
	virtual void goal_completed(
			typename ActionT::Result::SharedPtr result) = 0;
	virtual bool configure(
			rclcpp_lifecycle::LifecycleNode::WeakPtr)
	{
		return true;
	}
	virtual bool cleanup() {return true;}
	virtual bool activate() {return true;}
	virtual bool deactivate() {return true;}
	std::unique_ptr<NavActionServer<ActionT>> nav_action_server_;
	rclcpp::Logger logger_{rclcpp::get_logger("Navigator")};
	rclcpp::Clock::SharedPtr clock_;
	NavigatorModeMuxer * nav_mode_muxer_;
};
}