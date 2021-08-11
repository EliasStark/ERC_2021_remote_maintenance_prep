#include "ObjectiveFunctionExecutors.h"

//##################################################################################################
//#################### Constructor and Destructor ##################################################
//##################################################################################################

ObjectiveFunction2Executor::ObjectiveFunction2Executor(ros::NodeHandle nHandle, unsigned int num, ...){

    this->nHandle = nHandle;
    this->allButtonsPressed = false;
    this->numberMarkersToDetect = num;
    this->mtx.unlock();

    va_list arguments;
    va_start(arguments, num);

    for (int i = 0; i < num; i++) {
        this->targetIDs[i] = va_arg(arguments, unsigned int);
        this->btnsPressed[i] = false;
        
        // define callback function als lambda expression
        // receives feedback of buttons; if button is pressed, store in a member function and check if all others are pressed as well; if so, store in an other member function
        auto callbackFunctionButtonPressedSubcriber = [this, i](const std_msgs::Bool::ConstPtr& msg) {
            if (msg->data) {
                this->btnsPressed[i] = true;
                for (int v = 0; v < this->numberMarkersToDetect; v++) {
                    if (!this->btnsPressed[v]) {
                        return;
                    }
                }
                this->allButtonsPressed = true;
            }
        };

        // define callback function als lambda expression
        // receives detected poses of aruco markes and stores them in member variable
        auto callbackFunctionDetectedPoseSubcriber = [this, i](const geometry_msgs::Pose::ConstPtr& msg) {
            this->mtx.lock();
            this->detectedPoses[i] = *msg;
            this->mtx.unlock();
        };

        // these subscribers listen to the button topics which return true when the button is pressed
        this->btnPressedSubcribers[i] = this->nHandle.subscribe<std_msgs::Bool>(("button" + std::to_string(this->targetIDs[i]+1)), 10, callbackFunctionButtonPressedSubcriber);
        
        // these subscribers listen to the poses of the searched aruco markers
        if (i == 0) {
            this->detectedPoseSubscribers[i] = this->nHandle.subscribe<geometry_msgs::Pose>("aruco_simple/pose", 10, callbackFunctionDetectedPoseSubcriber);
        } else {
            this->detectedPoseSubscribers[i] = this->nHandle.subscribe<geometry_msgs::Pose>(("aruco_simple/pose" + std::to_string(this->targetIDs[i]+1)), 10, callbackFunctionDetectedPoseSubcriber);
        }
    }
    va_end(arguments);
}

ObjectiveFunction2Executor::~ObjectiveFunction2Executor(){
    std::cout << "Objective 2 successfully finished" << std::endl;
}

//#############################################################################################
//#################### Public member methods ##################################################
//#############################################################################################

bool ObjectiveFunction2Executor::execute(moveit::planning_interface::MoveGroupInterface &move_group, const robot_state::JointModelGroup* joint_model_group, moveit::core::RobotStatePtr currentState) {

    ros::Publisher gripperPublisher = this->nHandle.advertise<std_msgs::String>("gripper_command", 10);
    std_msgs::String msg;
    msg.data = "close";
    gripperPublisher.publish(msg);

    bool success;

    geometry_msgs::PoseStamped initialPose = move_group.getCurrentPose();
    std::cout << "initial pose:" << std::endl;
    std::cout << initialPose.pose.position.x << std::endl;
    std::cout << initialPose.pose.position.y << std::endl;
    std::cout << initialPose.pose.position.z << std::endl;
    std::cout << initialPose.pose.orientation.w << std::endl;
    std::cout << initialPose.pose.orientation.x << std::endl;
    std::cout << initialPose.pose.orientation.y << std::endl;
    std::cout << initialPose.pose.orientation.z << std::endl;

    ros::Duration duration(3.0);
    duration.sleep();

    for (int i = 0; i < this->numberMarkersToDetect; i++) {

        this->mtx.lock();
        geometry_msgs::Pose detectedPose = this->detectedPoses[i];
        this->mtx.unlock();

        std::cout << "detected pose:" << std::endl;
        std::cout << detectedPose.position.x << std::endl;
        std::cout << detectedPose.position.y << std::endl;
        std::cout << detectedPose.position.z << std::endl;
        std::cout << detectedPose.orientation.w << std::endl;
        std::cout << detectedPose.orientation.x << std::endl;
        std::cout << detectedPose.orientation.y << std::endl;
        std::cout << detectedPose.orientation.z << std::endl;

        // fetch target pose
        geometry_msgs::PoseStamped computedTargetPoseCloseToTarget = getPoseCloseToTarget(this->nHandle, this->detectedPoses[i]);

        std::cout << "Transformed pose:" << std::endl;
        std::cout << computedTargetPoseCloseToTarget.pose.position.x << std::endl;
        std::cout << computedTargetPoseCloseToTarget.pose.position.y << std::endl;
        std::cout << computedTargetPoseCloseToTarget.pose.position.z << std::endl;
        std::cout << computedTargetPoseCloseToTarget.pose.orientation.w << std::endl;
        std::cout << computedTargetPoseCloseToTarget.pose.orientation.x << std::endl;
        std::cout << computedTargetPoseCloseToTarget.pose.orientation.y << std::endl;
        std::cout << computedTargetPoseCloseToTarget.pose.orientation.z << std::endl;

        success = move_to_pose(move_group, computedTargetPoseCloseToTarget.pose);
    }

    if (success) {
        return true;
    }

    return false;
}

//##############################################################################################
//#################### Private member methods ##################################################
//##############################################################################################
