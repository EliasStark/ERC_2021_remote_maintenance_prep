#ifndef MAINTENANCE_UTILS_H
#define MAINTENANCE_UTILS_H

#include "ObjectiveFunctionExecutors.h"

geometry_msgs::PoseStamped correctCameraOffset (geometry_msgs::PoseStamped currentPose);

geometry_msgs::PoseStamped pressButton_computeTargetForGripper (geometry_msgs::PoseStamped detectedPose, unsigned int delta);

geometry_msgs::PoseStamped pressButton_computeApproachPose (geometry_msgs::PoseStamped detectedPose);

geometry_msgs::PoseStamped pickUpIMU_computeTargetForGripper (geometry_msgs::PoseStamped detectedPose);

geometry_msgs::PoseStamped pickUpIMU_computeApproachPose (geometry_msgs::PoseStamped detectedPose);

geometry_msgs::PoseStamped getDetectedPoseInBaseFrame (ros::NodeHandle nHandle, moveit::planning_interface::MoveGroupInterface &move_group, geometry_msgs::Pose detectedPose);

void broadcastCameraFrame(ros::NodeHandle nHandle, geometry_msgs::PoseStamped cameraPose, std::mutex& mtx);

#endif