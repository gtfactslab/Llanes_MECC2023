#include "rrtx_static_planner/rrtx_static.hpp"
#include "ompl/base/spaces/SE3StateSpace.h"


namespace rrtx_static_planner
{

RrtxStatic::RrtxStatic()
{

}

RrtxStatic::~RrtxStatic() noexcept
{
	RCLCPP_INFO(
			logger_, "Destroying plugin %s of type RrtxStatic",
			name_.c_str());
}

void RrtxStatic::configure(
		const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
		std::string name)
{
	name_ = name;
	global_frame_ = "map";
	node_ = parent;
	auto node = parent.lock();
	clock_ = node->get_clock();
	logger_ = node->get_logger();
	RCLCPP_INFO(
			logger_, "Configuring plugin %s of type RrtxStatic",
			name_.c_str());
	try {
		node->declare_parameter(name + ".obstacle_scaling", rclcpp::ParameterValue(1.3));
		node->get_parameter(name + ".obstacle_scaling", obstacle_scaling_);

		node->declare_parameter(name + ".bounds_low",
		                        rclcpp::ParameterValue(std::vector<double>RRTX_ARENA_BOUNDS_LOW_DEFAULT));
		node->get_parameter(name + ".bounds_low", planner_map_bounds_low_);
		node->declare_parameter(name + ".bounds_high",
		                        rclcpp::ParameterValue(std::vector<double>RRTX_ARENA_BOUNDS_HIGH_DEFAULT));
		node->get_parameter(name + ".bounds_high", planner_map_bounds_high_);
		node->declare_parameter(name + ".solve_time", rclcpp::ParameterValue((double)RRTX_PLANNER_MAX_SOLVE_TIME_S));
		node->get_parameter(name + ".solve_time", planner_solve_time_);
	} catch (const rclcpp::ParameterTypeException & ex) {
		RCLCPP_ERROR(node->get_logger(), "Plugin RrtxStatic parameter type exception:  %s", ex.what());
	}


	obstacle_sub_ = node->create_subscription<uam_mapping_msgs::msg::ObstacleArray>(
			"uam_mapping/obstacles", 10,
			[this](const uam_mapping_msgs::msg::ObstacleArray::UniquePtr msg) {
				for (const auto& obstacle : obstacles_) {
					if (std::find(std::begin(msg->obstacle_ids), std::end(msg->obstacle_ids), obstacle.first) == std::end(msg->obstacle_ids))
						obstacles_.erase(obstacle.first);
				}
				for (const auto& msg_obstacle : msg->obstacles) {
					bodies::BodyPtr body;
					shapes::ShapeConstPtr shape;
					switch (msg_obstacle.obstacle.type) {
						case shape_msgs::msg::SolidPrimitive::BOX:
							body = std::make_shared<bodies::Box>();

							const Eigen::Isometry3d pose =
									Eigen::Translation3d(
											msg_obstacle.pose.position.x,
											msg_obstacle.pose.position.y,
											msg_obstacle.pose.position.z)
									* Eigen::Quaterniond(
											msg_obstacle.pose.orientation.w,
											msg_obstacle.pose.orientation.x,
											msg_obstacle.pose.orientation.y,
											msg_obstacle.pose.orientation.z);
							shape = std::make_shared<const shapes::Box>(
									msg_obstacle.obstacle.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X],
									msg_obstacle.obstacle.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y],
									msg_obstacle.obstacle.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z]);
							// TODO: Find a better solution to scaling
							body->setScaleDirty(obstacle_scaling_);
							body->setPoseDirty(pose);
							body->setDimensionsDirty(shape.get());
							body->updateInternalData();
							break;
					}

					if (auto search = obstacles_.find(msg_obstacle.obstacle_id); search != obstacles_.end()) {
							search->second = body;
					} else {
						obstacles_.insert({msg_obstacle.obstacle_id, body});
					}
				}
			});

	ompl::msg::noOutputHandler();
	state_space_ptr_ = std::make_shared<ompl::base::SE3StateSpace>();
	ompl::base::RealVectorBounds bounds(3);
	bounds.setLow(0, planner_map_bounds_low_[0]);
	bounds.setHigh(0, planner_map_bounds_high_[0]);
	bounds.setLow(1, planner_map_bounds_low_[1]);
	bounds.setHigh(1, planner_map_bounds_high_[1]);
	bounds.setLow(2, planner_map_bounds_low_[2]);
	bounds.setHigh(2, planner_map_bounds_high_[2]);
	state_space_ptr_->as<ompl::base::SE3StateSpace>()->setBounds(bounds);
	state_space_ptr_->setValidSegmentCountFactor(100);
	space_info_ptr_ = std::make_shared<ompl::base::SpaceInformation>(state_space_ptr_);
	space_info_ptr_->setStateValidityChecker([this](const ompl::base::State *state)
	                                         {
		                                         return isStateValid(state);
	                                         });
	space_info_ptr_->setMotionValidator(std::make_shared<ompl::base::DiscreteMotionValidator>(space_info_ptr_));
	space_info_ptr_->setStateValidityCheckingResolution(0.01);
	prob_def_ptr_ = std::make_shared<ompl::base::ProblemDefinition>(space_info_ptr_);
	planner_ptr_ = std::make_shared<ompl::geometric::RRTXstatic>(space_info_ptr_);
	planner_ptr_->setProblemDefinition(prob_def_ptr_);
}

void RrtxStatic::activate()
{
	RCLCPP_INFO(
			logger_, "Activating plugin %s of type RrtxStatic",
			name_.c_str());
}

void RrtxStatic::deactivate()
{
	RCLCPP_INFO(
			logger_, "Deactivating plugin %s of type RrtxStatic",
			name_.c_str());
}

void RrtxStatic::cleanup()
{
	RCLCPP_INFO(
			logger_, "Cleaning up plugin %s of type RrtxStatic",
			name_.c_str());
	state_space_ptr_.reset();
	space_info_ptr_.reset();
	prob_def_ptr_.reset();
	planner_ptr_.reset();

}

nav_msgs::msg::Path RrtxStatic::create_path(
		const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal)
{
	prob_def_ptr_->clearStartStates();
	prob_def_ptr_->clearGoal();
	prob_def_ptr_->clearSolutionPaths();
	planner_ptr_->clear();
	ompl::base::ScopedState<ompl::base::SE3StateSpace> start_state(state_space_ptr_);
	if (start.header.frame_id == "map") {
		start_state->setXYZ(
				start.pose.position.x,
				start.pose.position.y,
				start.pose.position.z);
	} else if (start.header.frame_id == "map_ned") {
		start_state->setXYZ(
				start.pose.position.y,
				start.pose.position.x,
				-start.pose.position.z);
	}
	start_state->rotation().setIdentity();

	ompl::base::ScopedState<ompl::base::SE3StateSpace> goal_state(state_space_ptr_);
	if (goal.header.frame_id == "map") {
		goal_state->setXYZ(
				goal.pose.position.x,
				goal.pose.position.y,
				goal.pose.position.z);
	} else if (goal.header.frame_id == "map_ned") {
		goal_state->setXYZ(
				goal.pose.position.y,
				goal.pose.position.x,
				-goal.pose.position.z);
	}
	goal_state->rotation().setIdentity();
	prob_def_ptr_->setStartAndGoalStates(start_state, goal_state);
	planner_ptr_->setup();

//	space_info_ptr_->printSettings(std::cout);
//	prob_def_ptr_->print(std::cout);

	auto planner_status = planner_ptr_->ompl::base::Planner::solve(planner_solve_time_);

//	prob_def_ptr_->print(std::cout);
	// TODO: add more exception checking
	if (planner_status != ompl::base::PlannerStatus::EXACT_SOLUTION &&
		planner_status != ompl::base::PlannerStatus::APPROXIMATE_SOLUTION) {
		throw uam_planner::NoValidPathCouldBeFound("Failed to create a valid path");
	}
	nav_msgs::msg::Path path;

	path.poses.clear();
	path.header.stamp = clock_->now();
	path.header.frame_id = global_frame_;
	auto path_ptr = prob_def_ptr_->getSolutionPath()->as<ompl::geometric::PathGeometric>();
	for(size_t i = 0; i < path_ptr->getStateCount(); i++) {
		auto state = path_ptr->getState(i)->as<ompl::base::SE3StateSpace::StateType>();
		geometry_msgs::msg::PoseStamped pose;
		pose.header.frame_id = global_frame_;
		pose.pose.position.x = state->getX();
		pose.pose.position.y = state->getY();
		pose.pose.position.z = state->getZ();
		path.poses.push_back(pose);
	}

	return path;
}

bool RrtxStatic::isStateValid(const ompl::base::State *state) const
{
	const auto *se3state = state->as<ompl::base::SE3StateSpace::StateType>();

	Eigen::Vector3d node(se3state->getX(), se3state->getY(), se3state->getZ());
	Eigen::Vector3d corner1;
	bool collision = false;
	for (const auto& obstacle : obstacles_) {
		collision |= obstacle.second->containsPoint(node);
	}

	return (space_info_ptr_->satisfiesBounds(se3state) && (!collision));
}

} // namespace rrtx_static_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rrtx_static_planner::RrtxStatic, uam_planner::PlannerBase)