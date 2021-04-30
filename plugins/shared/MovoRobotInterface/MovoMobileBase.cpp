#include "MovoMobileBase.hpp"
#include <tf/transform_listener.h>

namespace movo {
bool MovoMobileBase::init() {
	ac_ = std::make_unique<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>("/move_base_navi", true);
	LOGGING("Waiting for move_base_navi server");
	moveBaseActionServerRunning_ = ac_->waitForServer(ros::Duration(3.0));
	LOGGING("Done");
	return true;
}

geometric::Pose MovoMobileBase::getCurrentPose() const {
	if (!moveBaseActionServerRunning_)
		return geometric::Pose();
	tf::TransformListener listener;
	std::string odomFrame = "/odom";
	std::string baseLinkFrame = "/base_link";
	bool canTransform = false;
	while (!canTransform) {
		canTransform = listener.canTransform(odomFrame, baseLinkFrame,
		                                     ros::Time(0));
	}

	tf::StampedTransform transform;

	listener.lookupTransform(odomFrame, baseLinkFrame,
	                         ros::Time(0), transform);
	geometric::Pose pose(transform.getOrigin().getX(), 
		transform.getOrigin().getY(), 
		transform.getOrigin().getZ(),
		0.0,
		0.0,
		0.0);	
	return pose;
}

bool MovoMobileBase::moveToPose(const geometric::Pose &pose) {
	if (!moveBaseActionServerRunning_)
		return true;
	move_base_msgs::MoveBaseGoal goal;	

	geometry_msgs::PoseStamped poseStamped;
	poseStamped.header.stamp = ros::Time::now();
	poseStamped.header.frame_id = "/odom";
	poseStamped.pose.position.x = pose.position.x();
	poseStamped.pose.position.y = pose.position.y();
	poseStamped.pose.orientation.w = 1.0;
	goal.target_pose = poseStamped;

	ac_->sendGoal(goal);
	bool finishedBeforeTimeout = ac_->waitForResult(ros::Duration(20.0));
	if (finishedBeforeTimeout)
	{
		return true;
	} else {
		WARNING("Timeout while moving mobile base");
		return false;
	}
}

}