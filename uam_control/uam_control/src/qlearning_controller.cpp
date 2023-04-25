#include <uam_control/qlearning_controller.hpp>
#include "uam_util/qos_profiles.hpp"

using namespace uam_control;
using namespace rclcpp;
using namespace Eigen;

QLearningController::QLearningController() : rclcpp::Node("uam_control")
{
	compute_svec_matrix();
	state_vector_.setZero();
	control_vector_.setZero();
	critic_augmented_state_kronecker_prior_.setZero();

	terminal_riccati_matrix_.diagonal() << 100.0, 100.0, 100.0, 10.0, 10.0, 10.0;

	state_penalty_matrix_.setZero();
	state_penalty_matrix_.diagonal() << 10.0, 10.0, 10.0, 30.0, 30.0, 30.0;
	control_penalty_matrix_.setZero();
	control_penalty_matrix_.diagonal() << 20.0, 20.0, 20.0;

	try {
		declare_parameter("rrtx_static.critic_convergence_rate",rclcpp::ParameterValue(0.1));
		declare_parameter("rrtx_static.actor_convergence_rate",rclcpp::ParameterValue(0.1));
		declare_parameter("rrtx_static.minimum_altitude_for_learning",rclcpp::ParameterValue(0.6));
		declare_parameter("rrtx_static.learning_update_frequency",rclcpp::ParameterValue(100.0));

		get_parameter("rrtx_static.critic_convergence_rate",critic_convergence_rate_);
		get_parameter("rrtx_static.actor_convergence_rate",actor_convergence_rate_);
		get_parameter("rrtx_static.minimum_altitude_for_learning",minimum_altitude_for_learning_);
		get_parameter("rrtx_static.learning_update_frequency",learning_update_frequency_);

	} catch (const rclcpp::ParameterTypeException & ex) {
		RCLCPP_ERROR(get_logger(), "Parameter type exception thrown");
	}
	// ----------------------- Subscribers --------------------------
	navigator_setpoint_sub_ =
			this->create_subscription<nav_msgs::msg::Odometry>(
					"/uam_navigator/position_setpoint", 10,
					[this](const nav_msgs::msg::Odometry::UniquePtr msg) {
						if (navigator_setpoint_.pose.pose != msg->pose.pose) {
							is_setpoint_new_ = true;
							start_time_ = this->get_clock()->now();
							// reset weights
							critic_weight_vector_ << 10.83,0.00,0.00,11.72,0.00,0.00,2.93,0.00,0.00,10.83,0.00,0.00,11.72,0.00,0.00,2.93,0.00,10.83,0.00,0.00,11.72,0.00,0.00,2.93,11.99,0.00,0.00,4.24,0.00,0.00,11.99,0.00,0.00,4.24,0.00,11.99,0.00,0.00,4.24,1.00,0.00,0.00,1.00,0.00,1.00;

							actor_weight_matrix_ <<  -2.07,0.00,0.00,0.00,-2.07,0.00,0.00,0.00,-2.07,-3.00,0.00,0.00,0.00,-3.00,0.00,0.00,0.00,-3.00;

							running_cost_ = 0;
							running_cost_prior_ = 0;
						}
						navigator_setpoint_ = *msg;

						auto state_vector = compute_state_vector();
						auto control = compute_control(state_vector);
						publish_control_mellinger(control);
					});
	// TODO: remove dependence on px4_msgs
	vehicle_status_sub_ =
			this->create_subscription<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status", uam_util::px4_qos_sub,
					[this](const px4_msgs::msg::VehicleStatus::UniquePtr msg) {
						vehicle_status_ = *msg;
					});
	vehicle_odometry_sub_ =
			this->create_subscription<nav_msgs::msg::Odometry>("/uam_vehicle_interface/odometry", 10,
					[this](const nav_msgs::msg::Odometry::UniquePtr msg) {
						rclcpp::Time vehicle_odom_time(vehicle_odometry_.header.stamp.sec, vehicle_odometry_.header.stamp.nanosec);
						rclcpp::Time msg_time(msg->header.stamp.sec, msg->header.stamp.nanosec);
						time_resolution_ = msg_time.seconds() - vehicle_odom_time.seconds();
//						RCLCPP_INFO(get_logger(), "time resolution: %.4f", time_resolution_);
					    vehicle_odometry_ = *msg;
						if (can_learn() && time_resolution_ < 0.1) {
							//------- Q-Learning -------------
							learn();
						} else {
							start_time_ = this->get_clock()->now();
						}
					});
	// ----------------------- Publishers ---------------------------

	vehicle_attitude_setpoint_pub_ =
			this->create_publisher<uam_control_msgs::msg::AttitudeSetpoint>("/uam_control/attitude_setpoint", 10);
	qlearning_status_pub_ =
			this->create_publisher<uam_control_msgs::msg::QLearningStatus>("/uam_control/qlearning_status", 10);

//	auto timer_callback = [this]() -> void
//	{
//		if (can_learn()) {
//			//------- Q-Learning -------------
//			learn();
//		} else {
//			start_time_ = this->get_clock()->now();
//		}
//	};

//	time_resolution_ = 1.0 / learning_update_frequency_;
//	timer_ = this->create_wall_timer(std::chrono::duration<double>(time_resolution_), timer_callback);
}

bool QLearningController::can_learn() const
{
	bool out;
	out =   abs(vehicle_odometry_.pose.pose.position.z) > minimum_altitude_for_learning_ &&
			vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED &&
			vehicle_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
	return out;
}

state_vector_t QLearningController::compute_state_vector() const
{
	state_vector_t state_vector;
	state_vector << vehicle_odometry_.pose.pose.position.x - navigator_setpoint_.pose.pose.position.x,
					vehicle_odometry_.pose.pose.position.y - navigator_setpoint_.pose.pose.position.y,
					vehicle_odometry_.pose.pose.position.z - navigator_setpoint_.pose.pose.position.z,
					vehicle_odometry_.twist.twist.linear.x - navigator_setpoint_.twist.twist.linear.x,
					vehicle_odometry_.twist.twist.linear.y - navigator_setpoint_.twist.twist.linear.y,
					vehicle_odometry_.twist.twist.linear.z - navigator_setpoint_.twist.twist.linear.z;
	return state_vector;
}
void QLearningController::learn()
{
	double current_time = (this->get_clock()->now().seconds() - start_time_.seconds());

	// Setup
	state_vector_ = compute_state_vector();
	control_vector_ = compute_control(state_vector_);
	augmented_state_vector_ << state_vector_, control_vector_;
	critic_augmented_state_kronecker_ = vec_to_svec_transform_matrix_ * kroneckerProduct(augmented_state_vector_, augmented_state_vector_);
	critic_radial_basis_matrix_ = compute_critic_radial_basis_matrix(current_time);
	actor_radial_basis_matrix_ = compute_actor_radial_basis_matrix(current_time);


	if (is_setpoint_new_) {
		critic_augmented_state_kronecker_prior_ = critic_augmented_state_kronecker_;
		is_setpoint_new_ = false;
		RCLCPP_INFO(get_logger(), "Point is new.........");
	}

	// Critic
	critic_augmented_state_kronecker_t sigma;
	critic_augmented_state_kronecker_t sigma_f;

	sigma << critic_radial_basis_matrix_ * (critic_augmented_state_kronecker_ - critic_augmented_state_kronecker_prior_);
	sigma_f << -critic_radial_basis_matrix_ * critic_augmented_state_kronecker_;

	critic_error_1_ = critic_weight_vector_.transpose() * (critic_radial_basis_matrix_ * critic_augmented_state_kronecker_
			- compute_critic_radial_basis_matrix(current_time - time_resolution_) * critic_augmented_state_kronecker_prior_)
			+ running_cost_ - running_cost_prior_;
	critic_error_2_ = 0.5 * (state_vector_.transpose() * terminal_riccati_matrix_).dot(state_vector_)
			- critic_weight_vector_.transpose().dot(sigma_f);
//
	critic_weight_vector_t critic_weight_dot = -critic_convergence_rate_
			* ((sigma /pow(1.0 + sigma.squaredNorm(), 2))*critic_error_1_)
			- 0.0*critic_convergence_rate_ * (sigma_f /pow(1.0 + sigma_f.squaredNorm(), 2))*critic_error_2_;
//	critic_weight_vector_t critic_weight_dot = -critic_convergence_rate_
//			* ((sigma /pow(1.0 + sigma.transpose()*sigma, 2))*critic_error_1_)
//			- exp(8.0*current_time - 5.0) * (critic_convergence_rate_ / 2.0) *(sigma_f /pow(1.0 + sigma_f.transpose()*sigma_f, 2))*critic_error_2_;
//	critic_weight_vector_t critic_weight_dot = -critic_convergence_rate_
//			* ((sigma /pow(1.0 + sigma.transpose()*sigma, 2))*critic_error_1_);
	// Actor
	augmented_state_matrix_t Q_bar;
	Q_bar << (vec_to_svec_transform_matrix_.transpose() *
			(2 * critic_radial_basis_matrix_.transpose() * critic_weight_vector_))
			.reshaped(AUGMENTED_STATE_VECTOR_SIZE,AUGMENTED_STATE_VECTOR_SIZE);
	control_matrix_t Quu = Q_bar.bottomRightCorner<QLEARNING_CONTROL_VECTOR_SIZE, QLEARNING_CONTROL_VECTOR_SIZE>();
	actor_error_ = actor_weight_matrix_.transpose() * actor_radial_basis_matrix_ * state_vector_
			+ Quu.inverse() * Q_bar.bottomLeftCorner<QLEARNING_CONTROL_VECTOR_SIZE, QLEARNING_STATE_VECTOR_SIZE>() * state_vector_;
	actor_weight_matrix_t actor_weight_dot = -actor_convergence_rate_ * state_vector_ / (1.0 + state_vector_.dot(state_vector_)) * actor_error_.transpose();

	// Q value
	quality_value_ = critic_weight_vector_.transpose() * critic_radial_basis_matrix_ * critic_augmented_state_kronecker_;

	publish_qlearning_status();

	// Update weights
	critic_weight_vector_ += critic_weight_dot * time_resolution_;
	actor_weight_matrix_ += actor_weight_dot * time_resolution_;
	running_cost_prior_ = running_cost_;
	running_cost_ += 0.5 * ((state_vector_.transpose() * state_penalty_matrix_).dot(state_vector_)
			+ (control_vector_.transpose() * control_penalty_matrix_).dot(control_vector_)) * time_resolution_;
	critic_augmented_state_kronecker_prior_ = critic_augmented_state_kronecker_;
}
control_vector_t QLearningController::compute_control(const mavState& state_vector)
{
	double current_time = (this->get_clock()->now().seconds() - start_time_.seconds());
	return actor_weight_matrix_.transpose() * compute_actor_radial_basis_matrix(current_time) * state_vector;
}

critic_radial_basis_matrix_t QLearningController::compute_critic_radial_basis_matrix(double t)
{
	(void) t; // fix unused time argument warnings
	return critic_radial_basis_matrix_t::Identity();
}
actor_radial_basis_matrix_t QLearningController::compute_actor_radial_basis_matrix(double t)
{
	(void) t; // fix unused time argument warnings
	return actor_radial_basis_matrix_t::Identity();
}
void QLearningController::compute_svec_matrix()
{
	svec_matrix_t vec_to_svec_transform_matrix;
	vec_to_svec_transform_matrix.setZero();
	size_t n = AUGMENTED_STATE_VECTOR_SIZE;
	for (size_t j = 0; j < n; ++j)
	{
		for(size_t i = j; i < n; ++i)
		{
			critic_augmented_state_kronecker_t uij;
			uij.setZero();
			uij(j*n + i - (j+1)*j/2) = 1.0;
			augmented_state_matrix_t Tij;
			Tij.setZero();
			if (i == j)
			{
				Tij(i, j) = 1.0;
			} else {
				Tij(i, j) = 1.0/sqrt(2.0);
				Tij(j, i) = 1.0/sqrt(2.0);
			}

			vec_to_svec_transform_matrix += uij * Tij.reshaped().transpose();
		}
	}
	vec_to_svec_transform_matrix_ = vec_to_svec_transform_matrix;
}
void QLearningController::publish_control_mellinger(const control_vector_t& control_vector)
{
	Vector3d thrust_des;
	uam_control_msgs::msg::AttitudeSetpoint attitude_setpoint;
	Vector3d x_c_des;
	double yaw_des = EIGEN_PI / 2.0;
	Vector3d z_axis;
	Vector3d x_B_desired, y_B_desired, z_B_desired;
	double thrust_proj;
	Vector3d rpy;

	Quaterniond q(vehicle_odometry_.pose.pose.orientation.w,
	              vehicle_odometry_.pose.pose.orientation.x,
	              vehicle_odometry_.pose.pose.orientation.y,
	              vehicle_odometry_.pose.pose.orientation.z);

	Matrix3d R = q.toRotationMatrix();
	//Matrix3d R = q.toRotationMatrix();
	z_axis = R.col(2);

	//TODO common config params
	thrust_des = control_vector + Vector3d(0.0,0.0,-9.80665);
	thrust_proj = thrust_des.dot(-z_axis);

	x_c_des << cos(yaw_des), sin(yaw_des), 0;
	z_B_desired = -thrust_des.normalized();
	y_B_desired = (z_B_desired.cross(x_c_des)).normalized();
	x_B_desired = y_B_desired.cross(z_B_desired);

	Matrix3d R_d;
	R_d.col(0) = x_B_desired;
	R_d.col(1) = y_B_desired;
	R_d.col(2) = z_B_desired;
	Vector3d ypr_d = R_d.eulerAngles(2,1,0);
	Quaterniond q_d(R_d);
//	auto e_R_matrix = (R_d.transpose() * R - R.transpose() * R_d);
//	auto e_R = Eigen::Vector3d(0.5 * e_R_matrix(2,1),
//							   0.5 * e_R_matrix(0,2),
//							   0.5 * e_R_matrix(1,0));
//	auto rpy_des = rpy - e_R;
	attitude_setpoint.header.stamp = this->get_clock()->now();
	attitude_setpoint.header.frame_id = "map_ned";
	attitude_setpoint.roll_body = ypr_d(2);
	attitude_setpoint.pitch_body = ypr_d(1);
	attitude_setpoint.yaw_body = ypr_d(0);
	attitude_setpoint.thrust_body_normalized = thrust_proj;
	attitude_setpoint.q_d[0] = q_d.w();
	attitude_setpoint.q_d[1] = q_d.x();
	attitude_setpoint.q_d[2] = q_d.y();
	attitude_setpoint.q_d[3] = q_d.z();
	vehicle_attitude_setpoint_pub_->publish(attitude_setpoint);
}

void QLearningController::publish_qlearning_status()
{
	uam_control_msgs::msg::QLearningStatus qlearning_status;

	qlearning_status.stamp = this->get_clock()->now();
	qlearning_status.start_time = start_time_;
	qlearning_status.quality_value = quality_value_;

	for(size_t i = 0; i < qlearning_status.actor_weight_matrix.size(); i++)
		qlearning_status.actor_weight_matrix[i] = *(actor_weight_matrix_.data() + i);

	for(size_t i = 0; i < qlearning_status.critic_weight_vector.size(); i++)
		qlearning_status.critic_weight_vector[i] = critic_weight_vector_[i];

	for(size_t i = 0; i < qlearning_status.control_vector.size(); i++)
		qlearning_status.control_vector[i] = control_vector_[i];

	for(size_t i = 0; i < qlearning_status.state_vector.size(); i++)
		qlearning_status.state_vector[i] = state_vector_[i];

	for(size_t i = 0; i < qlearning_status.critic_augmented_state_kronecker.size(); i++)
		qlearning_status.critic_augmented_state_kronecker[i] = critic_augmented_state_kronecker_[i];

	for(size_t i = 0; i < qlearning_status.critic_augmented_state_kronecker_prior.size(); i++)
		qlearning_status.critic_augmented_state_kronecker_prior[i] = critic_augmented_state_kronecker_prior_[i];

	qlearning_status.critic_error_1 = critic_error_1_;
	qlearning_status.critic_error_2 = critic_error_2_;

	for(size_t i = 0; i < qlearning_status.actor_error.size(); i++)
		qlearning_status.actor_error[i] = actor_error_[i];

	qlearning_status_pub_->publish(qlearning_status);
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<uam_control::QLearningController>());
	rclcpp::shutdown();
	return 0;
}