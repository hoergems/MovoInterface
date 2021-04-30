#ifndef _ROBOTIQ_INTERFACE_HPP_
#define _ROBOTIQ_INTERFACE_HPP_
#include <oppt/opptCore/core.hpp>
#include <movo_msgs/GripperCmd.h>
#include <movo_msgs/GripperStat.h>
#include <ros/ros.h>
#include <thread>
#include <boost/thread.hpp>


using namespace oppt;

namespace movo {
class RobotiqInterface {
public:
	RobotiqInterface(ros::NodeHandle *nh);

	~RobotiqInterface();

	_NO_COPY_BUT_MOVE(RobotiqInterface)

	bool openGripper();

	bool closeGripper();

	bool isMoving();

private:
	ros::Publisher pub_;

	ros::Subscriber sub_;

	bool isMoving_ = false;

	std::mutex mtx_;

	std::unique_ptr<boost::thread> spThread_ = nullptr;

	void spinThread_();

	void gripperCallback_(const movo_msgs::GripperStat::ConstPtr & msg);

	void sendGripperPosition_(const FloatType &position);
	

};

}

#endif