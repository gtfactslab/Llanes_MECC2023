#pragma once

#include "rclcpp/rclcpp.hpp"

namespace uam_util
{

rclcpp::QoS px4_qos_sub = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
rclcpp::QoS px4_qos_pub = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().transient_local();

}