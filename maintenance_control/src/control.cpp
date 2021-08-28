#include "ObjectiveFunctionExecutors.h"

ObjectiveFunctionExecutor *executor;

int main (int argc, char* argv[]) {

    ros::init(argc, argv, "control");

    ros::NodeHandle n;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // setup moveit
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    moveit::core::RobotStatePtr currentState;

    // move to inital pose
    //if (!move_to_initial_pose(move_group_manipulator, joint_model_group, currentState)) {
    //    std::cout << "Could not move to inital position!" << std::endl;
    //    exit (EXIT_FAILURE);
    //}

    // check if there are enough arguments and assign current objective to variable
    if (argc < 1) {
        std::cout << "There are not enough arguments given for the control node!" << std::endl;
        exit (EXIT_FAILURE);
    }

    unsigned int start;
    unsigned int stop;
    unsigned int mode = argv[1][0] - '0';
    if (mode == 0) {
        start = 1;
        stop = 9;
    } else if (mode >= 1 && mode <= 9) {
        start = mode;
        stop = mode;
    } else {
        start = 0;
        stop = 0;
    }

    for (int i = start; i <= stop; i++) {
        switch (i) {
            case 1: executor = new ObjectiveFunction1Executor(n); break;
            case 2: executor = new ObjectiveFunction2Executor(n, argc-2, argv); break;
            case 3: executor = new ObjectiveFunction3Executor(n); break;
            //case 4: executor = new ObjectiveFunction4Executor(); break;
            //case 5: executor = new ObjectiveFunction5Executor(); break;
            //case 6: executor = new ObjectiveFunction6Executor(); break;
            //case 7: executor = new ObjectiveFunction7Executor(); break;
            //case 8: executor = new ObjectiveFunction8Executor(); break;
            case 9: executor = new ObjectiveFunction9Executor(); break;
            default: ROS_INFO("No valid objective function given!"); exit (EXIT_FAILURE);
        }

        bool success = executor->execute(move_group, joint_model_group, currentState);

        if (success){
            delete executor;
        } else {
            std::cout << "Objective " << i << " failed!" << std::endl;
            exit (EXIT_FAILURE);
        }
    }
}
