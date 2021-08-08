#ifndef MOVEIT_UTILS_H
#define MOVEIT_UTILS_H

#include "ObjectiveFunctionExecutors.h"

bool move_to_pose (moveit::planning_interface::MoveGroupInterface &move_group, geometry_msgs::Pose pose);

bool move_in_joint_space (moveit::planning_interface::MoveGroupInterface &move_group, std::vector<double> joint_group_positions);

bool move_to_initial_pose (moveit::planning_interface::MoveGroupInterface &move_group, const robot_state::JointModelGroup* joint_model_group, moveit::core::RobotStatePtr currentState);

bool move_to_overview_pose (moveit::planning_interface::MoveGroupInterface &move_group, const robot_state::JointModelGroup* joint_model_group, moveit::core::RobotStatePtr currentState);

#endif