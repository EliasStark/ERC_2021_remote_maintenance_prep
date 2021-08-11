#ifndef OBJECTIVEFUNCTIONEXECUTOR_H
#define OBJECTIVEFUNCTIONEXECUTOR_H

#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>

class ObjectiveFunctionExecutor {
    public:
        virtual ~ObjectiveFunctionExecutor(){}
        virtual bool execute (moveit::planning_interface::MoveGroupInterface &move_group, const robot_state::JointModelGroup* joint_model_group, moveit::core::RobotStatePtr currentState) = 0;
};

#endif
