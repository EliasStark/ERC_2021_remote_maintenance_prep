#include "ObjectiveFunctionExecutors.h"
#include "geometry_msgs/Pose.h"

//##################################################################################################
//#################### Constructor and Destructor ##################################################
//##################################################################################################

ObjectiveFunction9Executor::ObjectiveFunction9Executor() {}

ObjectiveFunction9Executor::~ObjectiveFunction9Executor() {
    std::cout << "Objective 9 successfully finished" << std::endl;
}

//#############################################################################################
//#################### Public member methods ##################################################
//#############################################################################################

bool ObjectiveFunction9Executor::execute(moveit::planning_interface::MoveGroupInterface &move_group, const robot_state::JointModelGroup* joint_model_group, moveit::core::RobotStatePtr currentState) {

    // drive to the position
    if (move_to_initial_pose(move_group, joint_model_group, currentState)) {
        return true;
    } else {
        return false;
    }
}