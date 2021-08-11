#include "moveit_utils.h"

bool move_to_pose (moveit::planning_interface::MoveGroupInterface &move_group, geometry_msgs::Pose pose) {

    move_group.setPoseTarget(pose);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    std::cout << (success ? "Planning to pose in catesian space worked" : "Planning to pose in cartesian space failed") << std::endl;

    if (success) {
        move_group.move();
        return true;
    }

    return false;
}

bool move_in_joint_space (moveit::planning_interface::MoveGroupInterface &move_group, std::vector<double> joint_group_positions) {
	
	move_group.setJointValueTarget(joint_group_positions);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    std::cout << (success ? "Planning to pose in joint space worked" : "Planning to pose in joint space failed") << std::endl;

    if (success) {
        move_group.move();
        return true;
    }

    return false;
}

bool move_to_initial_pose (moveit::planning_interface::MoveGroupInterface &move_group, const robot_state::JointModelGroup* joint_model_group, moveit::core::RobotStatePtr currentState) {
    std::vector<double> joint_group_positions;
    currentState = move_group.getCurrentState();
    currentState->copyJointGroupPositions(joint_model_group, joint_group_positions);
    joint_group_positions[0] = 0.0;
    joint_group_positions[1] = -2.09;
    joint_group_positions[2] = 1.74;
    joint_group_positions[3] = 0.34;
    joint_group_positions[4] = 1.57;
    joint_group_positions[5] = -1.57;
    return move_in_joint_space(move_group, joint_group_positions);
}

bool move_to_overview_pose (moveit::planning_interface::MoveGroupInterface &move_group, const robot_state::JointModelGroup* joint_model_group, moveit::core::RobotStatePtr currentState) {
    std::vector<double> joint_group_positions;
    currentState = move_group.getCurrentState();
    currentState->copyJointGroupPositions(joint_model_group, joint_group_positions);
    joint_group_positions[0] = -0.22;
    joint_group_positions[1] = -2.35;
    joint_group_positions[2] = 1.22;
    joint_group_positions[3] = 1.3;
    joint_group_positions[4] = 1.57;
    joint_group_positions[5] = -1.57;
    return move_in_joint_space(move_group, joint_group_positions);
}