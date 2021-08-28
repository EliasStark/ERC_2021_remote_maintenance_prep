#include "maintenance_utils.h"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "geometry_msgs/Pose.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_broadcaster.h>
#include <thread>

geometry_msgs::PoseStamped correctCameraOffset (geometry_msgs::PoseStamped currentPose) {
	
	geometry_msgs::Vector3 originalOffset;
	geometry_msgs::Vector3 transformedOffset;
	geometry_msgs::TransformStamped transform;

	// do translational offset
	originalOffset.x = 0.006;
	originalOffset.y = 0.085;
	originalOffset.z = 0;

	transform.transform.translation.x = 0;
	transform.transform.translation.y = 0;
	transform.transform.translation.z = 0;
	transform.transform.rotation.w = currentPose.pose.orientation.w;
	transform.transform.rotation.x = currentPose.pose.orientation.x;
	transform.transform.rotation.y = currentPose.pose.orientation.y;
	transform.transform.rotation.z = currentPose.pose.orientation.z;

	tf2::doTransform(originalOffset, transformedOffset, transform);

    currentPose.pose.position.x += transformedOffset.x;
    currentPose.pose.position.y += transformedOffset.y;
    currentPose.pose.position.z += transformedOffset.z;

    // do rotational offset
    geometry_msgs::PoseStamped originalRotation1;
    geometry_msgs::PoseStamped transformedRotation1;

    originalRotation1.pose.position.x = 0;
    originalRotation1.pose.position.y = 0;
    originalRotation1.pose.position.z = 0;
    originalRotation1.pose.orientation.w = 0.9961743;
    originalRotation1.pose.orientation.x = 0;
    originalRotation1.pose.orientation.y = 0;
    originalRotation1.pose.orientation.z = -0.0873884;

	tf2::doTransform(originalRotation1, transformedRotation1, transform);

	transform.transform.rotation.w = transformedRotation1.pose.orientation.w;
	transform.transform.rotation.x = transformedRotation1.pose.orientation.x;
	transform.transform.rotation.y = transformedRotation1.pose.orientation.y;
	transform.transform.rotation.z = transformedRotation1.pose.orientation.z;

    geometry_msgs::PoseStamped originalRotation2;
    geometry_msgs::PoseStamped transformedRotation2;

    originalRotation2.pose.position.x = 0;
    originalRotation2.pose.position.y = 0;
    originalRotation2.pose.position.z = 0;
    originalRotation2.pose.orientation.w = 0.7073883;
    originalRotation2.pose.orientation.x = 0;
    originalRotation2.pose.orientation.y = 0.7068252;
    originalRotation2.pose.orientation.z = 0;

	tf2::doTransform(originalRotation2, transformedRotation2, transform);

	currentPose.pose.orientation = transformedRotation2.pose.orientation;

	return currentPose;
}

geometry_msgs::PoseStamped pressButton_computeTargetForGripper (geometry_msgs::PoseStamped detectedPose, unsigned int delta) {

	detectedPose.pose.position.x += -0.191 + delta*0.005;
	detectedPose.pose.position.y += 0;
	detectedPose.pose.position.z += -0.055;
	detectedPose.pose.orientation.w = 0.7073883;
	detectedPose.pose.orientation.x = 0.7068252;
	detectedPose.pose.orientation.y = 0;
	detectedPose.pose.orientation.z = 0;

	return detectedPose;
}

geometry_msgs::PoseStamped pressButton_computeApproachPose (geometry_msgs::PoseStamped pose) {

	pose.pose.position.x += -0.22;
	pose.pose.position.y += 0;
	pose.pose.position.z += -0.055;
	pose.pose.orientation.w = 0.7073883;
	pose.pose.orientation.x = 0.7068252;
	pose.pose.orientation.y = 0;
	pose.pose.orientation.z = 0;

	return pose;
}

geometry_msgs::PoseStamped pickUpIMU_computeApproachPose (geometry_msgs::PoseStamped pose) {

	pose.pose.position.x += 0;
	pose.pose.position.y += 0;
	pose.pose.position.z += 0.2;
	
	geometry_msgs::TransformStamped transform;
	geometry_msgs::PoseStamped originalRotation1;
	geometry_msgs::PoseStamped originalRotation2;
    geometry_msgs::PoseStamped transformedRotation1;
    geometry_msgs::PoseStamped transformedRotation2;

    originalRotation1.pose.position.x = 0;
    originalRotation1.pose.position.y = 0;
    originalRotation1.pose.position.z = 0;
    originalRotation1.pose.orientation.w = 0.7073883;
    originalRotation1.pose.orientation.x = 0;
    originalRotation1.pose.orientation.y = 0.7068252;
    originalRotation1.pose.orientation.z = 0;

    originalRotation2.pose.position.x = 0;
    originalRotation2.pose.position.y = 0;
    originalRotation2.pose.position.z = 0;
    originalRotation2.pose.orientation.w = 0.7073883;
    originalRotation2.pose.orientation.x = -0.7068252;
    originalRotation2.pose.orientation.y = 0;
    originalRotation2.pose.orientation.z = 0;

	transform.transform.translation.x = 0;
	transform.transform.translation.y = 0;
	transform.transform.translation.z = 0;
	transform.transform.rotation.w = pose.pose.orientation.w;
	transform.transform.rotation.x = pose.pose.orientation.x;
	transform.transform.rotation.y = pose.pose.orientation.y;
	transform.transform.rotation.z = pose.pose.orientation.z;

	tf2::doTransform(originalRotation1, transformedRotation1, transform);

	transform.transform.rotation.w = transformedRotation1.pose.orientation.w;
	transform.transform.rotation.x = transformedRotation1.pose.orientation.x;
	transform.transform.rotation.y = transformedRotation1.pose.orientation.y;
	transform.transform.rotation.z = transformedRotation1.pose.orientation.z;

	tf2::doTransform(originalRotation2, transformedRotation2, transform);

	pose.pose.orientation = transformedRotation2.pose.orientation;

	tf::TransformBroadcaster br;
  	tf::Transform transform2;

  	ros::Rate rate(10.0);
	while (1){
		std::cout << "br" << std::endl;
    	transform2.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
    	transform2.setRotation(tf::Quaternion(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w));
    	br.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "base_link", "target"));
    	rate.sleep();
	}

	return pose;
}

geometry_msgs::PoseStamped pickUpIMU_computeTargetForGripper (geometry_msgs::PoseStamped detectedPose) {

	detectedPose.pose.position.x += -0.191;
	detectedPose.pose.position.y += 0;
	detectedPose.pose.position.z += -0.055;
	detectedPose.pose.orientation.w = 0.7073883;
	detectedPose.pose.orientation.x = 0.7068252;
	detectedPose.pose.orientation.y = 0;
	detectedPose.pose.orientation.z = 0;

	return detectedPose;
}

void broadcastCameraFrame(ros::NodeHandle nHandle, geometry_msgs::PoseStamped cameraPose, std::mutex& mtx) {
  	
	tf::TransformBroadcaster br;
  	tf::Transform transform;

  	ros::Rate rate(10.0);
	while (nHandle.ok()){
		std::cout << "executing thread loop" << std::endl;
    	transform.setOrigin(tf::Vector3(cameraPose.pose.position.x, cameraPose.pose.position.y, cameraPose.pose.position.z));
    	transform.setRotation(tf::Quaternion(cameraPose.pose.orientation.x, cameraPose.pose.orientation.y, cameraPose.pose.orientation.z, cameraPose.pose.orientation.w));
    	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "camera"));
    	if (mtx.try_lock()) {
    		return;
    	} else {
    		rate.sleep();
    	}
	}
}

geometry_msgs::PoseStamped getDetectedPoseInBaseFrame (ros::NodeHandle nHandle, moveit::planning_interface::MoveGroupInterface &move_group, geometry_msgs::Pose detectedPose) {
	
	// get current pose of camera
	geometry_msgs::PoseStamped currentCameraPose = correctCameraOffset(move_group.getCurrentPose());

	std::mutex mtx;
	mtx.lock();
	std::thread frameBroadcasterThread(broadcastCameraFrame, nHandle, currentCameraPose, std::ref(mtx)); 

	ros::Duration duration(4.0);
    duration.sleep();

    // transform detected pose to world frame
    tf2_ros::Buffer buffer_;
	tf2_ros::TransformListener listener_(buffer_);

	geometry_msgs::PoseStamped detectedPoseStamped;
	detectedPoseStamped.header.seq = 1;
	detectedPoseStamped.header.frame_id = "camera";
	
	detectedPoseStamped.pose = detectedPose;
	geometry_msgs::PoseStamped transformedPose;
	while (nHandle.ok()) {
		try {
			detectedPoseStamped.header.stamp = ros::Time::now();
			transformedPose = buffer_.transform(detectedPoseStamped, "base_link");
			//std::cout << "WORKING!!!!!!!!!!!!!!" << std::endl;
			mtx.unlock();
			break;
		} catch (tf2::LookupException &ex) {
			//std::cout << "not working! Lookup ex" << std::endl;
		} catch (tf2::ConnectivityException &ex) {
			//std::cout << "not working! Connectivity ex" << std::endl;
		} catch (tf2::ExtrapolationException &ex) {
			//std::cout << "not working! Extrapolation ex" << std::endl;
		}
	}

	frameBroadcasterThread.join();

	return transformedPose;
}