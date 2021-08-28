#include "ObjectiveFunctionExecutors.h"
#include "geometry_msgs/Pose.h"

//########################################################################################################
//#################### Callback methods for subscribers ##################################################
//########################################################################################################

void ObjectiveFunction1Executor::marker_1_searcher_callback(const geometry_msgs::Pose::ConstPtr& msg) {
    this->marker_1_detected = true;
    if (this->marker_9_detected) {
        this->both_detected = true;
    }
}

void ObjectiveFunction1Executor::marker_9_searcher_callback(const geometry_msgs::Pose::ConstPtr& msg) {
    this->marker_9_detected = true;
    if (this->marker_1_detected) {
        this->both_detected = true;
    }
}

//##################################################################################################
//#################### Constructor and Destructor ##################################################
//##################################################################################################

ObjectiveFunction1Executor::ObjectiveFunction1Executor(ros::NodeHandle nHandle) {
    this->nHandle = nHandle;
    this->marker_1_detected = false;
    this->marker_9_detected = false;
    this->both_detected = false;
}

ObjectiveFunction1Executor::~ObjectiveFunction1Executor() {
    std::cout << "Objective 1 successfully finished" << std::endl;
}

//#############################################################################################
//#################### Public member methods ##################################################
//#############################################################################################

bool ObjectiveFunction1Executor::execute(moveit::planning_interface::MoveGroupInterface &move_group, const robot_state::JointModelGroup* joint_model_group, moveit::core::RobotStatePtr currentState) {

    // drive to a position where the whole middle panel can be seen
    if (move_to_overview_pose(move_group, joint_model_group, currentState)) {
        
        ros::Subscriber marker_1_searcher = this->nHandle.subscribe("aruco_simple/pose", 10, &ObjectiveFunction1Executor::marker_1_searcher_callback, this);
        ros::Subscriber marker_9_searcher = this->nHandle.subscribe("aruco_simple/pose2", 10, &ObjectiveFunction1Executor::marker_9_searcher_callback, this);

        ros::Duration duration(3.0);
        duration.sleep();

        if (this->both_detected) {
            return true;
        } else {
            return false;
        }
    }
}