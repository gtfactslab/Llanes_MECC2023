#pragma once
#include "navigator_modes/navigator_mode.hpp"

namespace uam_navigator
{

class Loiter: public NavigatorMode
{
public:
	Loiter();
	~Loiter();

protected:
	bool configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
	               std::shared_ptr<uam_navigator::Navigator> navigator,
	               std::string nav_mode) override;
	bool activate(uam_navigator_msgs::action::NavigatorCommand::Goal::ConstSharedPtr goal) override;
	bool deactivate() override;
	bool cleanup() override;
	void publish_navigator_setpoint() override;

	// Class Variables
	nav_msgs::msg::Odometry vehicle_odom_;
	geometry_msgs::msg::PoseStamped loiter_position_;
};

}