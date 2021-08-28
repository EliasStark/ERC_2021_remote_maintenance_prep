#include "ObjectiveFunctionExecutors.h"

//##################################################################################################
//#################### Constructor and Destructor ##################################################
//##################################################################################################

ObjectiveFunction2Executor::ObjectiveFunction2Executor(ros::NodeHandle nHandle, int num, char* argv[]) {

    this->nHandle = nHandle;
    this->allButtonsPressed = false;
    this->numberMarkersToDetect = num;
    this->mtx.unlock();

    for (int i = 0; i < num; i++) {
        this->targetIDs[i] = argv[i+2][0]-'0';
        this->btnsPressed[i] = false;
        
        // define callback function als lambda expression
        // receives feedback of buttons; if button is pressed, store in a member variable and check if all others are pressed as well; if so, store in an other member variable
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
        auto callbackFunctionDetectedPosesSubcriber = [this, i](const geometry_msgs::Pose::ConstPtr& msg) {
            this->mtx.lock();
            this->detectedPoses[i] = *msg;
            this->mtx.unlock();
        };

        // these subscribers listen to the button topics which return true when the button is pressed
        this->btnPressedSubcribers[i] = this->nHandle.subscribe<std_msgs::Bool>(("button" + std::to_string(this->targetIDs[i])), 10, callbackFunctionButtonPressedSubcriber);
        
        // these subscribers listen to the poses of the searched aruco markers
        this->detectedPosesSubscriber = this->nHandle.subscribe<geometry_msgs::Pose>("aruco_simple/pose", 10, callbackFunctionDetectedPosesSubcriber);

    }
}

ObjectiveFunction2Executor::~ObjectiveFunction2Executor() {
    std::cout << "Objective 2 successfully finished" << std::endl;
}

//#############################################################################################
//#################### Public member methods ##################################################
//#############################################################################################

bool ObjectiveFunction2Executor::execute(moveit::planning_interface::MoveGroupInterface &move_group, const robot_state::JointModelGroup* joint_model_group, moveit::core::RobotStatePtr currentState) {

    bool movement_success = false;
    
    // close gripper
    ros::Publisher gripperPublisher = this->nHandle.advertise<std_msgs::String>("gripper_command", 10);
    std_msgs::String msg;
    msg.data = "close";
    gripperPublisher.publish(msg);

    // wait some time in order to enable the detection of the aruco markers
    ros::Duration duration(2.0);
    duration.sleep();

    // important poses
    geometry_msgs::Pose detectedPose;               // detected pose relative to camera
    geometry_msgs::PoseStamped poseInBaseFrame;     // same pose relative to base frame
    geometry_msgs::PoseStamped approachPose;        // pose near to target relative to base frame
    geometry_msgs::PoseStamped finalPose;           // computed pose so that the gripper presses the button

    // loop for all buttons to be pressed
    for (int i = 0; i < this->numberMarkersToDetect; i++) {

        this->mtx.lock();
        detectedPose = this->detectedPoses[i];
        this->mtx.unlock();

        poseInBaseFrame = getDetectedPoseInBaseFrame(this->nHandle, move_group, detectedPose);

        approachPose = pressButton_computeApproachPose(poseInBaseFrame);

        movement_success = move_to_pose(move_group, approachPose.pose);

        this->mtx.lock();
        detectedPose = this->detectedPoses[i];
        this->mtx.unlock();

        poseInBaseFrame = getDetectedPoseInBaseFrame(this->nHandle, move_group, detectedPose);

        // loop for approaching button
        for (int n = 0; n < 10; n++) {
            finalPose = pressButton_computeTargetForGripper(poseInBaseFrame, n);

            movement_success = move_to_pose(move_group, finalPose.pose);

            if (this->btnsPressed[i]) {
                break;
            }
        }
    }

    if (this->allButtonsPressed) {
        return true;
    }

    return false;
}

//##############################################################################################
//#################### Private member methods ##################################################
//##############################################################################################
