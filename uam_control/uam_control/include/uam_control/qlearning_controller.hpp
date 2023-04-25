#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/rc_channels.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <uam_navigator_msgs/msg/path.hpp>
#include <Eigen/Dense>
#include <eigen3/unsupported/Eigen/KroneckerProduct>
#include <unsupported/Eigen/EulerAngles>
#include <uam_control_msgs/msg/attitude_setpoint.hpp>
#include <uam_control_msgs/msg/q_learning_status.hpp>

namespace uam_control
{

#define QLEARNING_CALLBACK_RATE_MS 10
#define QLEARNING_STATE_VECTOR_SIZE 6
#define QLEARNING_CONTROL_VECTOR_SIZE 3
#define AUGMENTED_STATE_VECTOR_SIZE (QLEARNING_STATE_VECTOR_SIZE + QLEARNING_CONTROL_VECTOR_SIZE)
#define CRITIC_WEIGHT_VECTOR_SIZE (AUGMENTED_STATE_VECTOR_SIZE)*(AUGMENTED_STATE_VECTOR_SIZE + 1) / 2
#define QLEARNING_MINIMUM_LEARNING_ALTITUDE 0.6

class mavState: public Eigen::Matrix<double, QLEARNING_STATE_VECTOR_SIZE, 1> {
public:
	// Constructors
	mavState()
			: Eigen::Matrix<double, QLEARNING_STATE_VECTOR_SIZE, 1>()
	{};

	typedef Eigen::Matrix<double, QLEARNING_STATE_VECTOR_SIZE, 1> Base;

	mavState(const Eigen::Matrix<double, QLEARNING_STATE_VECTOR_SIZE, 1> &other)
			: Eigen::Matrix<double, QLEARNING_STATE_VECTOR_SIZE, 1>(other)
	{
	};

	//Operators
	mavState &operator=(const Eigen::Matrix<double, QLEARNING_STATE_VECTOR_SIZE, 1> &other)
	{
		this->Base::operator=(other);
		return *this;
	}

	template<typename OtherDerived>
	Eigen::Product<mavState, OtherDerived> operator*(const MatrixBase<OtherDerived> &other) const
	{
		this->Base::operator*(other);
		return *this;
	}

	EIGEN_DEVICE_FUNC
	EIGEN_STRONG_INLINE Scalar&
	vx() { return (*this)[3]; }


	EIGEN_DEVICE_FUNC
	EIGEN_STRONG_INLINE Scalar&
	vy() { return (*this)[4]; }


	EIGEN_DEVICE_FUNC
	EIGEN_STRONG_INLINE Scalar&
	vz() { return (*this)[5]; }

//	EIGEN_DEVICE_FUNC
//	EIGEN_STRONG_INLINE Scalar&
//	ax() { return (*this)[6]; }
//
//
//	EIGEN_DEVICE_FUNC
//	EIGEN_STRONG_INLINE Scalar&
//	ay() { return (*this)[7]; }
//
//
//	EIGEN_DEVICE_FUNC
//	EIGEN_STRONG_INLINE Scalar&
//	az() { return (*this)[8]; }
};

typedef Eigen::Vector<double, QLEARNING_CONTROL_VECTOR_SIZE> control_vector_t;
typedef Eigen::Matrix<double, QLEARNING_STATE_VECTOR_SIZE, QLEARNING_STATE_VECTOR_SIZE> state_matrix_t;
typedef Eigen::Matrix<double, QLEARNING_CONTROL_VECTOR_SIZE, QLEARNING_CONTROL_VECTOR_SIZE> control_matrix_t;

typedef Eigen::Vector<double, AUGMENTED_STATE_VECTOR_SIZE> augmented_state_vector_t;
typedef Eigen::Matrix<double, AUGMENTED_STATE_VECTOR_SIZE, AUGMENTED_STATE_VECTOR_SIZE> augmented_state_matrix_t;
typedef Eigen::Vector<double, CRITIC_WEIGHT_VECTOR_SIZE> critic_weight_vector_t;
typedef Eigen::Vector<double, CRITIC_WEIGHT_VECTOR_SIZE> critic_augmented_state_kronecker_t;
typedef Eigen::Matrix<double, CRITIC_WEIGHT_VECTOR_SIZE, CRITIC_WEIGHT_VECTOR_SIZE> critic_radial_basis_matrix_t;
typedef Eigen::Matrix<double, QLEARNING_STATE_VECTOR_SIZE, QLEARNING_CONTROL_VECTOR_SIZE> actor_weight_matrix_t;
typedef Eigen::Matrix<double, QLEARNING_STATE_VECTOR_SIZE, QLEARNING_STATE_VECTOR_SIZE> actor_radial_basis_matrix_t;
typedef Eigen::Matrix<double, CRITIC_WEIGHT_VECTOR_SIZE, AUGMENTED_STATE_VECTOR_SIZE * AUGMENTED_STATE_VECTOR_SIZE> svec_matrix_t;
typedef mavState state_vector_t;

class QLearningController : public rclcpp::Node
{
public:
	QLearningController();
private:
	// ----------------------- Publishers --------------------------
	rclcpp::Publisher<uam_control_msgs::msg::AttitudeSetpoint>::SharedPtr vehicle_attitude_setpoint_pub_;
	rclcpp::Publisher<uam_control_msgs::msg::QLearningStatus>::SharedPtr qlearning_status_pub_;


	// ----------------------- Subscribers --------------------------
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr navigator_setpoint_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vehicle_odometry_sub_;

	// Message Callback Variables
	nav_msgs::msg::Odometry navigator_setpoint_;
	px4_msgs::msg::VehicleStatus vehicle_status_;
	px4_msgs::msg::RcChannels vehicle_channels_;
	nav_msgs::msg::Odometry vehicle_odometry_;

	// Class Variables
	px4_msgs::msg::RcChannels rc_channels_;
	uint8_t mav_id_;
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Time start_time_;
	state_vector_t state_vector_; // x,y,z,vx,vy,vz
	control_vector_t control_vector_;
	augmented_state_vector_t augmented_state_vector_;
	state_matrix_t state_penalty_matrix_;
	control_matrix_t control_penalty_matrix_;
	state_matrix_t terminal_riccati_matrix_;
	critic_weight_vector_t critic_weight_vector_;
	critic_augmented_state_kronecker_t critic_augmented_state_kronecker_;
	critic_augmented_state_kronecker_t critic_augmented_state_kronecker_prior_;
	critic_radial_basis_matrix_t critic_radial_basis_matrix_;
	actor_weight_matrix_t actor_weight_matrix_;
	actor_radial_basis_matrix_t actor_radial_basis_matrix_;
	double critic_error_1_;
	double critic_error_2_;
	double running_cost_;
	double running_cost_prior_;
	control_vector_t actor_error_;
	double minimum_altitude_for_learning_;
	double critic_convergence_rate_;
	double actor_convergence_rate_;
	double learning_update_frequency_;
	double time_resolution_;
	double quality_value_;
	svec_matrix_t vec_to_svec_transform_matrix_;
	bool is_setpoint_new_{true};

	// Class methods
	void learn();
	control_vector_t compute_control(const state_vector_t& state_vector);
	state_vector_t compute_state_vector() const;
	bool can_learn() const;

	critic_radial_basis_matrix_t compute_critic_radial_basis_matrix(double t);
	actor_radial_basis_matrix_t compute_actor_radial_basis_matrix(double t);
	void compute_svec_matrix();
	void publish_control_mellinger(const control_vector_t& control_vector);
	void publish_qlearning_status();
//	void publish_control_fbl();
//	void arm() const;
//	void disarm() const;
//	void publish_control();
//	void publish_offboard_control_mode() const;
//	void publish_vehicle_command(uint16_t command, float param1 = 0.0,
//	                             float param2 = 0.0) const;
};// Class QLearningController
}