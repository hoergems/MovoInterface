#include "RobotiqInterface.hpp"
#include <chrono>

namespace movo {

RobotiqInterface::RobotiqInterface(ros::NodeHandle *nh):
	pub_(nh->advertise<movo_msgs::GripperCmd>("/movo/left_gripper/cmd", 1000)),
	sub_(nh->subscribe<movo_msgs::GripperStat>("/movo/left_gripper/stat", 1, &RobotiqInterface::gripperCallback_, this)) {
	spThread_ = std::unique_ptr<boost::thread>(new boost::thread(&RobotiqInterface::spinThread_, this));
}

RobotiqInterface::~RobotiqInterface() {
	if (spThread_) {
		spThread_->interrupt();
		spThread_->join();
	}
}

bool RobotiqInterface::openGripper() {
	LOGGING("Opening gripper");
	sendGripperPosition_(0.185);
	return true;
}

bool RobotiqInterface::closeGripper() {
	sendGripperPosition_(0.0);
	return true;
}

void RobotiqInterface::sendGripperPosition_(const FloatType &position) {
	movo_msgs::GripperCmd cmd;
	cmd.speed = 0.02;
	cmd.force = 100.0;
	cmd.position = position;	
	pub_.publish(cmd);
	//ros::spinOnce();	
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	while (isMoving()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}

void RobotiqInterface::gripperCallback_(const movo_msgs::GripperStat::ConstPtr & msg) {	
	mtx_.lock();
	isMoving_ = msg->is_moving;
	mtx_.unlock();
}

bool RobotiqInterface::isMoving() {
	bool moving;
	mtx_.lock();
	moving = isMoving_;
	mtx_.unlock();
	return moving;
}

void RobotiqInterface::spinThread_() {
	try {
		while (true) {
			boost::this_thread::interruption_point();
			ros::spinOnce();
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
	} catch (boost::thread_interrupted&) {
		return;
	}

}


}