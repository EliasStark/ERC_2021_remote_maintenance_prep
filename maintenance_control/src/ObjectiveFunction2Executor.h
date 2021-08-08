#ifndef OBJECTIVEFUNCTION2EXECUTOR_H
#define OBJECTIVEFUNCTION2EXECUTOR_H

#include "ObjectiveFunctionExecutor.h"
#include "ros/ros.h"

class ObjectiveFunction2Executor : public ObjectiveFunctionExecutor {
    public:
        ObjectiveFunction2Executor(ros::NodeHandle nHandle, unsigned int target1, unsigned int target2, unsigned int target3, unsigned int target4);
        ~ObjectiveFunction2Executor();
        void execute (moveit::planning_interface::MoveGroupInterface &move_group, const robot_state::JointModelGroup* joint_model_group, moveit::core::RobotStatePtr currentState);
        const char* tellFeedbackTopic ();

    private:
        ros::NodeHandle nHandle;
        ros::Subscriber subBtn1;
        ros::Subscriber subBtn2;
        ros::Subscriber subBtn3;
        ros::Subscriber subBtn4;
        unsigned int target1;
        unsigned int target2;
        unsigned int target3;
        unsigned int target4;
        bool btn1pressed;
        bool btn2pressed;
        bool btn3pressed;
        bool btn4pressed;
        void button1Callback(const std_msgs::Bool::ConstPtr& msg);
        void button2Callback(const std_msgs::Bool::ConstPtr& msg);
        void button3Callback(const std_msgs::Bool::ConstPtr& msg);
        void button4Callback(const std_msgs::Bool::ConstPtr& msg);
};

#endif
