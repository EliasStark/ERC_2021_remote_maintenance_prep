#ifndef OBJECTIVEFUNCTION1EXECUTOR_H
#define OBJECTIVEFUNCTION1EXECUTOR_H

#include "ObjectiveFunctionExecutor.h"
#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class ObjectiveFunction1Executor : public ObjectiveFunctionExecutor {
    public:
        ObjectiveFunction1Executor(ros::NodeHandle nHandle);
        ~ObjectiveFunction1Executor();
        void execute (moveit::planning_interface::MoveGroupInterface &move_group, const robot_state::JointModelGroup* joint_model_group, moveit::core::RobotStatePtr currentState);
        const char* tellFeedbackTopic ();

    private:
        ros::NodeHandle nHandle;
        ros::Publisher feedbackPublisher;
        bool marker_1_detected;
        bool marker_9_detected;
        bool both_detected;

        void marker_1_searcher_callback(const geometry_msgs::Pose::ConstPtr& msg);
        void marker_9_searcher_callback(const geometry_msgs::Pose::ConstPtr& msg);
};

#endif
