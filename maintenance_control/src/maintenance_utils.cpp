#include "maintenance_utils.h"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "geometry_msgs/Pose.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

geometry_msgs::Pose correctCameraOffset (geometry_msgs::Pose pose) {
	geometry_msgs::Pose returnPose;
	returnPose.position.x = pose.position.x - 0.135;
	returnPose.position.y = pose.position.y;
	returnPose.position.z = pose.position.z + 0.09014;
	returnPose.orientation.w = pose.orientation.w;
	returnPose.orientation.x = pose.orientation.x;
	returnPose.orientation.y = pose.orientation.y;
	returnPose.orientation.z = pose.orientation.z;
	return returnPose;
}

geometry_msgs::PoseStamped getPoseCloseToTarget (ros::NodeHandle n, geometry_msgs::Pose detectedPose) {

	//geometry_msgs::Pose targetPoseForEEInCameraFrame = correctCameraOffset(detectedPose);
	geometry_msgs::PoseStamped targetPoseForEEInCameraFrameStamped;
	targetPoseForEEInCameraFrameStamped.header.frame_id = "camera_camera_lens";
	//targetPoseForEEInCameraFrameStamped.header.stamp = ros::Time::now();
	//targetPoseForEEInCameraFrameStamped.pose = targetPoseForEEInCameraFrame;
	targetPoseForEEInCameraFrameStamped.pose = detectedPose;

	tf2_ros::Buffer buffer_;
	tf2_ros::TransformListener listener_(buffer_);

	geometry_msgs::PoseStamped transformedPose;
	while (n.ok()) {
		try {

			transformedPose = buffer_.transform(targetPoseForEEInCameraFrameStamped, "base_link");

			/*double remember = targetPoseForEEInCameraFrameStamped.pose.position.x;
			targetPoseForEEInCameraFrameStamped.pose.position.x = targetPoseForEEInCameraFrameStamped.pose.position.z;
			targetPoseForEEInCameraFrameStamped.pose.position.z = remember;

			transformedPose = buffer_.transform(targetPoseForEEInCameraFrameStamped, "base_link");*/

			std::cout << "WORKING!!!!!!!!!!!!!!" << std::endl;
			break;
		} catch (tf2::LookupException &ex) {
			
		} catch (tf2::ConnectivityException &ex) {
			
		} catch (tf2::ExtrapolationException &ex) {

		}
	}
	
	// correction
	//transformedPose.pose.position.x = transformedPose.pose.position.x - 0.18;
	
	// button offset under marker
	transformedPose.pose.position.z = transformedPose.pose.position.z - 0.055;
	
	// go to straight orientation
	transformedPose.pose.orientation.w = 0.706832;
	transformedPose.pose.orientation.x = 0.707363;
	transformedPose.pose.orientation.y = -0.00320606;
	transformedPose.pose.orientation.z = 0.00386977;

	return transformedPose;
}