#ifndef MAINTENANCE_UTILS_H
#define MAINTENANCE_UTILS_H

#include "ObjectiveFunctionExecutors.h"

geometry_msgs::Pose correctCameraOffset (geometry_msgs::Pose pose);

geometry_msgs::PoseStamped getPoseCloseToTarget (ros::NodeHandle n, geometry_msgs::Pose detectedPose);

#endif