#ifndef _MOVO_MOBILE_BASE_HPP_
#define _MOVO_MOBILE_BASE_HPP_
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>

#include <oppt/opptCore/core.hpp>

using namespace oppt;
namespace movo {
class MovoMobileBase {
public:
	bool init();

	geometric::Pose getCurrentPose() const;

	bool moveToPose(const geometric::Pose &pose);

private:
	std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> ac_ = nullptr;

	bool moveBaseActionServerRunning_ = false;


};
}

#endif