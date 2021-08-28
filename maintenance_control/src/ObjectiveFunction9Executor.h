#ifndef OBJECTIVEFUNCTION9EXECUTOR_H
#define OBJECTIVEFUNCTION9EXECUTOR_H

#include "ObjectiveFunctionExecutor.h"
#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>

class ObjectiveFunction9Executor : public ObjectiveFunctionExecutor {
    public:
        ObjectiveFunction9Executor();
        ~ObjectiveFunction9Executor();
        bool execute (moveit::planning_interface::MoveGroupInterface &move_group, const robot_state::JointModelGroup* joint_model_group, moveit::core::RobotStatePtr currentState);
};

#endif
