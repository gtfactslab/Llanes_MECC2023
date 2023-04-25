#ifndef VEHICLE_INTERFACE_GUI_HPP
#define VEHICLE_INTERFACE_GUI_HPP

#include <QtWidgets>
#include <QBasicTimer>

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rviz_common/panel.hpp"

#include "uam_vehicle_interface_msgs/msg/vehicle_interface_command.hpp"
#include "uam_vehicle_interface_msgs/msg/vehicle_interface_status.hpp"
#include "uam_navigator_msgs/action/navigator_command.hpp"
#include "uam_navigator_msgs/msg/navigator_status.hpp"
#include "nav_msgs/msg/odometry.hpp"

class QPushButton;

namespace uam_vehicle_interface_gui
{

class VehicleInterfacePanel : public rviz_common::Panel
{
	Q_OBJECT
public:
	explicit VehicleInterfacePanel(QWidget * parent = 0);
	virtual ~VehicleInterfacePanel();

	void onInitialize() override;
	void save(rviz_common::Config config) const override;
	void load(const rviz_common::Config & config) override;

public Q_SLOTS:
	void sendTakeoffLandCommand();
	void sendArmDisarmCommand();
	void sendVehicleKillSwitchCommand();
private:
	using ActionNavigatorCommand = uam_navigator_msgs::action::NavigatorCommand;
	using ActionNavigatorCommandGoal = ActionNavigatorCommand::Goal;

	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vehicle_odometry_sub_;
	rclcpp::Subscription<uam_navigator_msgs::msg::NavigatorStatus>::SharedPtr navigator_status_sub_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr navigator_goal_command_sub_;
	rclcpp::Subscription<uam_vehicle_interface_msgs::msg::VehicleInterfaceStatus>::SharedPtr vehicle_interface_status_sub_;
	rclcpp::Publisher<uam_vehicle_interface_msgs::msg::VehicleInterfaceCommand>::SharedPtr vehicle_interface_command_pub_;

	std::string base_frame_;

	rclcpp_action::Client<ActionNavigatorCommand>::SharedPtr nav_command_action_client_;

	QWidget * widget_;

	// Vehicle Buttons
	// Arm / Disarm
	QPushButton * arm_disarm_button_{nullptr};
	QState * armed_{nullptr};
	QState * disarmed_{nullptr};
	QStateMachine arm_disarm_state_machine_;

	// Navigator Mode
	QState * takeoff_{nullptr};
	QState * loiter_{nullptr};
	QState * navigate_{nullptr};
	QState * land_{nullptr};
	QState * idle_{nullptr};
	QStateMachine navigator_mode_state_machine_;
	QPushButton * takeoff_land_button_{nullptr};

	// Kill Switch
	QState * kill_switch_active_;
	QState * kill_switch_inactive_;
	QStateMachine kill_switch_state_machine_;
	QPushButton * kill_switch_button_{nullptr};

	// Vehicle Status
	QLabel * label_navigator_mode_;
	QLabel * label_vehicle_status_;

	// Vehicle Odometry
	QLabel * label_frame_id_odometry_;
	QLabel * label_position_odometry_x_;
	QLabel * label_position_odometry_y_;
	QLabel * label_position_odometry_z_;
	QLabel * label_velocity_odometry_x_;
	QLabel * label_velocity_odometry_y_;
	QLabel * label_velocity_odometry_z_;

	void updateOdometry(nav_msgs::msg::Odometry::UniquePtr msg);
	void updateNavigatorStatus(uam_navigator_msgs::msg::NavigatorStatus::UniquePtr msg);
	void commandNavigatorGoalPose(geometry_msgs::msg::PoseStamped::UniquePtr msg);
	void updateVehicleInterfaceStatus(uam_vehicle_interface_msgs::msg::VehicleInterfaceStatus::UniquePtr msg);
	void updateVehicleStatus();

};

struct StringEvent : public QEvent
{
	StringEvent(const QString &val)
			: QEvent(QEvent::Type(QEvent::User+1)),
			  value(val) {}

	QString value;
};

class StringTransition : public QAbstractTransition
{
public:
	StringTransition(const QString &value)
			: m_value(value) {}

protected:
	bool eventTest(QEvent *e) override
	{
		if (e->type() != QEvent::Type(QEvent::User+1)) // StringEvent
			return false;
		StringEvent *se = static_cast<StringEvent*>(e);
		return (m_value == se->value);
	}

	virtual void onTransition(QEvent *) {}

private:
	QString m_value;
};

}


#endif //VEHICLE_INTERFACE_GUI_HPP
