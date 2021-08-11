#ifndef OBJECTIVEFUNCTION2EXECUTOR_H
#define OBJECTIVEFUNCTION2EXECUTOR_H

#include <cstdarg>

#include "ObjectiveFunctionExecutor.h"
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"

class ObjectiveFunction2Executor : public ObjectiveFunctionExecutor {
    public:
        ObjectiveFunction2Executor(ros::NodeHandle nHandle, unsigned int num, ...);
        ~ObjectiveFunction2Executor();
        bool execute (moveit::planning_interface::MoveGroupInterface &move_group, const robot_state::JointModelGroup* joint_model_group, moveit::core::RobotStatePtr currentState);

    private:
        ros::NodeHandle nHandle;
        bool allButtonsPressed;
        unsigned int numberMarkersToDetect;
        unsigned int targetIDs[9];
        bool btnsPressed[9];
        ros::Subscriber btnPressedSubcribers[9];
        ros::Subscriber detectedPoseSubscribers[9];
        geometry_msgs::Pose detectedPoses[9];
        std::mutex mtx;
};

#endif