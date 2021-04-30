#ifndef _MOVO_ROBOT_INTERFACE_HPP_
#define _MOVO_ROBOT_INTERFACE_HPP_
#include <oppt/opptCore/core.hpp>
#include "MovoAPI.hpp"
#include "MotionPlanner.hpp"


namespace oppt {
enum MovoArms {
	LEFT,
	RIGHT
};


class MovoRobotInterface {
public:
	MovoRobotInterface(const RobotEnvironment* robotEnvironment);

	~MovoRobotInterface() = default;

	_NO_COPY_BUT_MOVE(MovoRobotInterface)

	/**
	 * @brief Initializes the interface to the robot by performing a hardware startup
	 */
	void init(const std::string &localIP, const MovoArms &arm);

	/**
	 * @brief Get the current joint angles of the robot
	 */
	VectorFloat getCurrentJointAngles() const;

	/**
	 * @brief Open the gripper
	 */
	void openGripper();

	/**
	 * @brief Close the gripper
	 */
	void closeGripper();

	/**
	 * @brief Move to initial jointAngles
	 */
	void moveToInitialJointAngles(const VectorFloat &initialJointAngles);

	/**
	 * @brief Apply the joint velocities for a duration given by durationMs (the duration in milliseconds)
	 */
	bool applyJointVelocities(const VectorFloat &jointVelocities, const FloatType &durationMS) const;

	/** 
	 * @brief Move the arm to the target joint angles for a duration of 'duration' milliseconds
	 */
	bool sendTargetJointAngles(const VectorFloat &jointAngles, const FloatType &durationMS);

private:
	std::unique_ptr<movo::MovoAPI> movoAPI_ = nullptr;

	const RobotEnvironment *robotEnvironment_ = nullptr;

	std::unique_ptr<MovoMotionPlanner> movoMotionPlanner_ = nullptr;

private:

	bool sendTargetJointVelocities_(const VectorFloat &jointVelocities) const;

};
}

#endif