#pragma once

#include <Eigen/Dense>

namespace uam_px4_interface
{

#define MAV_BASE_STATE_VECTOR_SIZE 6
#define MAV_BASE_CONTROL_VECTOR_SIZE 3


typedef Eigen::Vector<double, MAV_BASE_STATE_VECTOR_SIZE> state_vector_t;
typedef Eigen::Vector<double, MAV_BASE_CONTROL_VECTOR_SIZE> control_vector_t;
typedef Eigen::Matrix<double, MAV_BASE_STATE_VECTOR_SIZE, MAV_BASE_STATE_VECTOR_SIZE> state_matrix_t;
typedef Eigen::Matrix<double, MAV_BASE_CONTROL_VECTOR_SIZE, MAV_BASE_CONTROL_VECTOR_SIZE> control_matrix_t;

static constexpr uint8_t OFFBOARD_ENABLE_CHANNEL = 7;
static constexpr uint8_t POSITION_SETPOINT_CHANNEL = 5;
static constexpr double GRAVITY = 9.80665;

typedef struct mavRateControl {
	double roll_rate; // rad/s
	double pitch_rate; // rad/s
	double yaw_rate; // rad/s
	double thrust_normalized; // m/s^2
#if defined(__cplusplus)
	mavRateControl()
			: roll_rate(0)
			, pitch_rate(0)
			, yaw_rate(0)
			, thrust_normalized(0) {};
#endif
} mavRateControl;

typedef struct mavAttitudeControl {
	double roll; // rad
	double pitch; // rad
	double yaw; // rad
	double thrust_normalized; // m/s^2
	std::array<double,4> q_d;
#if defined(__cplusplus)
	mavAttitudeControl()
			: roll(0)
			, pitch(0)
			, yaw(0)
			, q_d{0,0,0,0}
			, thrust_normalized(0) {};
#endif
} mavAttitudeControl;

typedef struct mavLocalPositionSetpoint {
	double x; // m
	double y; // m
	double z; // m
	double vx; // m/s
	double vy; // m/s
	double vz; // m/s
	double ax; // m/s^2
	double ay; // m/s^2
	double az; // m/s^2
	double thrust_normalized; // m/s^2
#if defined(__cplusplus)
	mavLocalPositionSetpoint()
			: x(0)
			, y(0)
			, z(0)
			, vx(0)
			, vy(0)
			, vz(0)
			, ax(0)
			, ay(0)
			, az(0)
			, thrust_normalized(0) {};
#endif
} mavLocalPositionSetpoint;

static inline Eigen::Vector3d quaternion_to_rpy_wrap(Eigen::Quaterniond)
{
	Eigen::Vector3d rpy;
	double roll = atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy));
	double pitch = asin(2 * (qw * qy - qz * qx));
	double yaw = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));

	rpy << roll,
	       pitch,
	       yaw;

	return rpy;
}

static inline std::array<double,4> rpy_to_quaternion(double roll, double pitch, double yaw)
{
	std::array<double, 4> q{0,0,0,0};
	double cr = cos(roll * 0.5);
	double sr = sin(roll * 0.5);
	double cp = cos(pitch * 0.5);
	double sp = sin(pitch * 0.5);
	double cy = cos(yaw * 0.5);
	double sy = sin(yaw * 0.5);

	q[0] = cr * cp * cy + sr * sp * sy;
	q[1] = sr * cp * cy - cr * sp * sy;
	q[2] = cr * sp * cy + sr * cp * sy;
	q[3] = cr * cp * sy - sr * sp * cy;
	return q;
}

static inline Eigen::Matrix3d dcm(double phi,double theta,double psi) {
	double cphi = cos(phi);
	double sphi = sin(phi);
	double ctheta = cos(theta);
	double stheta = sin(theta);
	double cpsi = cos(psi);
	double spsi = sin(psi);
	Eigen::Matrix3d dcm;
	dcm << ctheta*cpsi, -cphi*spsi+sphi*stheta*cpsi,  sphi*spsi+cphi*stheta*cpsi,
	       ctheta*spsi,  cphi*cpsi+sphi*stheta*spsi, -sphi*cpsi+cphi*stheta*spsi,
	           -stheta,                 sphi*ctheta,                 cphi*ctheta;
	return dcm;
}

}