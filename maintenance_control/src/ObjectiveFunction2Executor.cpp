#include "ObjectiveFunctionExecutors.h"

//########################################################################################################
//#################### Callback methods for subscribers ##################################################
//########################################################################################################

void ObjectiveFunction2Executor::button1Callback(const std_msgs::Bool::ConstPtr& msg) {
    this->btn1pressed = true;
    if (this->btn1pressed && this->btn2pressed && this->btn3pressed && this->btn4pressed) {
        std_msgs::Bool msg;
        msg.data = true;
        this->successPublisher.publish(msg);
    }
}

void ObjectiveFunction2Executor::button2Callback(const std_msgs::Bool::ConstPtr& msg) {
    this->btn2pressed = true;
    if (this->btn1pressed && this->btn2pressed && this->btn3pressed && this->btn4pressed) {
        std_msgs::Bool msg;
        msg.data = true;
        this->successPublisher.publish(msg);    }
}

void ObjectiveFunction2Executor::button3Callback(const std_msgs::Bool::ConstPtr& msg) {
    this->btn3pressed = true;
    if (this->btn1pressed && this->btn2pressed && this->btn3pressed && this->btn4pressed) {
        std_msgs::Bool msg;
        msg.data = true;
        this->successPublisher.publish(msg);    }
}

void ObjectiveFunction2Executor::button4Callback(const std_msgs::Bool::ConstPtr& msg) {
    this->btn4pressed = true;
    if (this->btn1pressed && this->btn2pressed && this->btn3pressed && this->btn4pressed) {
        std_msgs::Bool msg;
        msg.data = true;
        this->successPublisher.publish(msg);    }
}

//##################################################################################################
//#################### Constructor and Destructor ##################################################
//##################################################################################################

ObjectiveFunction2Executor::ObjectiveFunction2Executor(ros::NodeHandle nHandle, unsigned int target1, unsigned int target2, unsigned int target3, unsigned int target4){
    this->nHandle = nHandle;
    this->target1 = target1;
    this->target2 = target2;
    this->target3 = target3;
    this->target4 = target4;
    this->feedbackTopic = "objective2successful";
    this->btn1pressed = false;
    this->btn2pressed = false;
    this->btn3pressed = false;
    this->btn4pressed = false;

    this->successPublisher = this->nHandle.advertise<std_msgs::Bool>(this->feedbackTopic, 10);

    std::cout << "setting up listeners" << std::endl;
    this->subBtn1 = this->nHandle.subscribe("button1", 10, &ObjectiveFunction2Executor::button1Callback, this);
    this->subBtn2 = this->nHandle.subscribe(("button" + std::to_string(target2)), 10, &ObjectiveFunction2Executor::button2Callback, this);
    this->subBtn3 = this->nHandle.subscribe(("button" + std::to_string(target3)), 10, &ObjectiveFunction2Executor::button3Callback, this);
    this->subBtn4 = this->nHandle.subscribe(("button" + std::to_string(target4)), 10, &ObjectiveFunction2Executor::button4Callback, this);
}

ObjectiveFunction2Executor::~ObjectiveFunction2Executor(){
    std::cout << "Destructor called" << std::endl;
}

//#############################################################################################
//#################### Public member methods ##################################################
//#############################################################################################

void ObjectiveFunction2Executor::execute(moveit::planning_interface::MoveGroupInterface &move_group, const robot_state::JointModelGroup* joint_model_group, moveit::core::RobotStatePtr currentState) {
    std::cout << "Executing O2" << std::endl;
}

const char* ObjectiveFunction2Executor::tellFeedbackTopic() {
    return this->feedbackTopic;
}

//##############################################################################################
//#################### Private member methods ##################################################
//##############################################################################################
