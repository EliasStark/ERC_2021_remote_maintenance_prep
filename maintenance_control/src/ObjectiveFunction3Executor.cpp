#include "ObjectiveFunctionExecutors.h"

//##################################################################################################
//#################### Constructor and Destructor ##################################################
//##################################################################################################

ObjectiveFunction3Executor::ObjectiveFunction3Executor(ros::NodeHandle nHandle) {

    this->nHandle = nHandle;

    // define callback function als lambda expression
    // receives detected poses of aruco markes and stores them in member variable
    auto callbackFunctionDetectedPoseSubcriber = [this](const geometry_msgs::PoseStamped::ConstPtr& msg) {
        this->detectedPose = msg->pose;
    };
        
    // these subscribers listen to the poses of the searched aruco marker
    this->detectedPoseSubscriber = this->nHandle.subscribe<geometry_msgs::PoseStamped>("aruco_simple/pose", 10, callbackFunctionDetectedPoseSubcriber);
}

ObjectiveFunction3Executor::~ObjectiveFunction3Executor() {
    std::cout << "Objective 3 successfully finished" << std::endl;
}

//#############################################################################################
//#################### Public member methods ##################################################
//#############################################################################################

bool ObjectiveFunction3Executor::execute(moveit::planning_interface::MoveGroupInterface &move_group, const robot_state::JointModelGroup* joint_model_group, moveit::core::RobotStatePtr currentState) {

    bool movement_success = false;
    
    // open gripper
    ros::Publisher gripperPublisher = this->nHandle.advertise<std_msgs::String>("gripper_command", 10);
    std_msgs::String msg;
    msg.data = "open";
    gripperPublisher.publish(msg);

    // move to an overview pose (joint space)
    std::vector<double> joint_group_positions;
    currentState = move_group.getCurrentState();
    currentState->copyJointGroupPositions(joint_model_group, joint_group_positions);
    joint_group_positions[0] = 0.75;
    joint_group_positions[1] = -1.7;
    joint_group_positions[2] = 1.74;
    joint_group_positions[3] = 1.0;
    joint_group_positions[4] = 1.57;
    joint_group_positions[5] = -1.57;

    movement_success = move_in_joint_space(move_group, joint_group_positions);

    // wait some time in order to enable the detection of the aruco markers
    ros::Duration duration(2.0);
    duration.sleep();

    // important poses
    geometry_msgs::Pose detectedPose;               // detected pose relative to camera
    geometry_msgs::PoseStamped poseInBaseFrame;     // same pose relative to base frame
    geometry_msgs::PoseStamped approachPose;        // pose near to target relative to base frame
    geometry_msgs::PoseStamped finalPose;           // computed pose so that the gripper presses the button

    // approach
    detectedPose = this->detectedPose;
    poseInBaseFrame = getDetectedPoseInBaseFrame(this->nHandle, move_group, detectedPose);
    approachPose = pickUpIMU_computeApproachPose(poseInBaseFrame);
    movement_success = move_to_pose(move_group, approachPose.pose);
/*
    // finalize
    detectedPose = this->detectedPose;
    poseInBaseFrame = getDetectedPoseInBaseFrame(this->nHandle, move_group, detectedPose);
    finalPose = pickUpIMU_computeTargetForGripper(poseInBaseFrame);
    movement_success = move_to_pose(move_group, finalPose.pose);*/

    // grabb
    if (movement_success) {
        msg.data = "close";
        gripperPublisher.publish(msg);
        return true;
    }

    return false;
}
