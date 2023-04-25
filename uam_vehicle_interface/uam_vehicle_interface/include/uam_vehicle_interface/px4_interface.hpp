#pragma once
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"

#include "Eigen/Dense"
#include "px4_msgs/msg/battery_status.hpp"
#include "px4_msgs/msg/rc_channels.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_rates_setpoint.hpp"
#include "px4_msgs/msg/vehicle_attitude_setpoint.hpp"
#include "px4_msgs/msg/vehicle_local_position_setpoint.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_control_mode.hpp"
#include "uam_control_msgs/msg/attitude_setpoint.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "uam_vehicle_interface_msgs/msg/vehicle_interface_command.hpp"
#include "uam_vehicle_interface_msgs/msg/vehicle_interface_status.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_eigen/tf2_eigen.h"
#include "px4_ros_com/frame_transforms.h"

namespace uam_vehicle_interface
{

static constexpr uint8_t OFFBOARD_ENABLE_CHANNEL = 7;

class Px4Interface : public rclcpp::Node
{
public:
	Px4Interface();
private:
	// ----------------------- Publishers --------------------------
	rclcpp::Publisher<px4_msgs::msg::VehicleLocalPositionSetpoint>::SharedPtr vehicle_local_position_setpoint_pub_;
	rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr vehicle_attitude_setpoint_pub_;
	rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr vehicle_rates_setpoint_pub_;
	rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr vehicle_odometry_pub_;
	rclcpp::Publisher<uam_vehicle_interface_msgs::msg::VehicleInterfaceStatus>::SharedPtr vehicle_interface_status_pub_;

	std::unique_ptr<tf2_ros::TransformBroadcaster> tf_dynamic_broadcaster_;
	std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

	// ----------------------- Subscribers --------------------------
	rclcpp::Subscription<px4_msgs::msg::RcChannels>::SharedPtr channels_sub_;
	rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_status_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
	rclcpp::Subscription<uam_control_msgs::msg::AttitudeSetpoint>::SharedPtr  attitude_setpoint_sub_;
	rclcpp::Subscription<uam_vehicle_interface_msgs::msg::VehicleInterfaceCommand>::SharedPtr vehicle_interface_command_sub_;

	// Class Variables
	bool kill_switch_enabled_{true};
	uint8_t offboard_counter_{0};
	double vehicle_mass_;
	double motor_thrust_max_;
	double motor_thrust_armed_;
	double motor_constant_;
	double motor_input_scaling_;
	double motor_velocity_armed_{100.0f};
	std::string environment_;

	// ROS2 Variables
	px4_msgs::msg::BatteryStatus battery_status_;
	px4_msgs::msg::VehicleStatus vehicle_status_;
	px4_msgs::msg::RcChannels channels_;
	px4_msgs::msg::VehicleOdometry vehicle_odometry_;
	rclcpp::TimerBase::SharedPtr timer_;

	// Private class methods
	void publish_attitude_setpoint(const uam_control_msgs::msg::AttitudeSetpoint& msg);
	void vehicle_odometry_callback(px4_msgs::msg::VehicleOdometry::UniquePtr msg);
	void publish_offboard_control_mode();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0,float param2 = 0.0);
	double compute_relative_thrust(const double &collective_thrust) const;
	void enable_offboard_control();
	void setup_static_transforms();
	void setup_parameters();
	void arm();
	void disarm();
};
}