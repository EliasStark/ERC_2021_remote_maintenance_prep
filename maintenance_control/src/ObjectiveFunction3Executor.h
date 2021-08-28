#ifndef OBJECTIVEFUNCTION3EXECUTOR_H
#define OBJECTIVEFUNCTION3EXECUTOR_H

#include "ObjectiveFunctionExecutor.h"
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"

class ObjectiveFunction3Executor : public ObjectiveFunctionExecutor {
    public:
        ObjectiveFunction3Executor(ros::NodeHandle nHandle);
        ~ObjectiveFunction3Executor();
        bool execute (moveit::planning_interface::MoveGroupInterface &move_group, const robot_state::JointModelGroup* joint_model_group, moveit::core::RobotStatePtr currentState);

    private:
        ros::NodeHandle nHandle;
        ros::Subscriber detectedPoseSubscriber;
        geometry_msgs::Pose detectedPose;
};

#endif