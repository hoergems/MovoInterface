#include <thread>
#include <chrono>
#include "MovoRobotInterface.hpp"
#include "HardwareStartup.hpp"
#include "Utils.hpp"
#include <oppt/robotEnvironment/include/RobotEnvironment.hpp>

//using namespace std::chrono_literals;

namespace oppt {
MovoRobotInterface::MovoRobotInterface(const RobotEnvironment* robotEnvironment):
	robotEnvironment_(robotEnvironment),
	movoMotionPlanner_(new MovoMotionPlanner(robotEnvironment)) {

}

void MovoRobotInterface::init(const std::string &localIP, const MovoArms &arm) {	
	HardwareStartup hardwareStartup;
	int armInt;
	if (arm == MovoArms::LEFT)
		armInt = 1;
	else
		armInt = 0;
	if (hardwareStartup.startupSequence(localIP, armInt) == false)
		ERROR("Couldn't execute startup sequence");
	movoAPI_ = std::move(hardwareStartup.releaseMovoAPI());
}

void MovoRobotInterface::moveToInitialJointAngles(const VectorFloat &initialJointAngles) {
	// First open the gripper
	openGripper();

	VectorFloat currentJointAngles = getCurrentJointAngles();
	FloatType dist = math::euclideanDistance<FloatType>(currentJointAngles, initialJointAngles);
	cout << "dist: " << dist << endl;
	if (dist < 0.02) {
		LOGGING("Arm is already at the initial configuration");		
		return;
	}

	TrajectorySharedPtr trajectory = movoMotionPlanner_->solve(currentJointAngles, initialJointAngles, 10.0);	
	if (!trajectory)
		ERROR("Couldn't find a trajectory to the initial state. Move the arm manually towards the initial configuration and try again.");

	VectorFloat targetJointAngles = currentJointAngles;
	FloatType stepDuration = 2000.0;
	LOGGING("EXECUTING TRAJECTORY. BE CAREFUL!!! Press Enter to continue");
	getchar();
	for (size_t i = 1; i != trajectory->stateTrajectory.size(); ++i) {
		targetJointAngles = trajectory->stateTrajectory[i]->as<VectorState>()->asVector();
		LOGGING("Sending target joint angles " +
		        std::to_string(i) +
		        "/" +
		        std::to_string(trajectory->stateTrajectory.size() - 1));
		printVector(targetJointAngles, "targetJointAngles");
		sendTargetJointAngles(targetJointAngles, stepDuration);
	}

	targetJointAngles = initialJointAngles;
	sendTargetJointAngles(targetJointAngles, stepDuration);

	LOGGING("Done. Press enter to continue");
	getchar();

}

void MovoRobotInterface::openGripper() {
	AngularPosition currentPosition;
	currentPosition.InitStruct();
	int getAngularPositionRes = 0;
	while (getAngularPositionRes != NO_ERROR_KINOVA) {
		getAngularPositionRes = movoAPI_->GetAngularPosition(currentPosition);
	}

	TrajectoryPoint trajectoryPoint;
	trajectoryPoint.InitStruct();
	trajectoryPoint.Position.HandMode = POSITION_MODE;
	trajectoryPoint.Position.Type = ANGULAR_POSITION;
	trajectoryPoint.Position.Actuators = currentPosition.Actuators;
	trajectoryPoint.Position.Fingers.Finger1 = 0.0;
	trajectoryPoint.Position.Fingers.Finger2 = 0.0;
	trajectoryPoint.Position.Fingers.Finger3 = 0.0;
	int moveRes = 0;
	while (moveRes != NO_ERROR_KINOVA) {
		moveRes = movoAPI_->SendBasicTrajectory(trajectoryPoint);
	}
}

void MovoRobotInterface::closeGripper() {
	AngularPosition currentPosition;
	currentPosition.InitStruct();

	int getAngularPositionRes = 0;
	while (getAngularPositionRes != NO_ERROR_KINOVA) {
		getAngularPositionRes = movoAPI_->GetAngularPosition(currentPosition);
	}

	TrajectoryPoint trajectoryPoint;
	trajectoryPoint.InitStruct();
	trajectoryPoint.Position.HandMode = POSITION_MODE;
	trajectoryPoint.Position.Type = ANGULAR_POSITION;
	trajectoryPoint.Position.Actuators = currentPosition.Actuators;
	trajectoryPoint.Position.Fingers.Finger1 = 9000.0;
	trajectoryPoint.Position.Fingers.Finger2 = 9000.0;
	trajectoryPoint.Position.Fingers.Finger3 = 9000.0;

	int moveRes = 0;
	while (moveRes != NO_ERROR_KINOVA) {
		moveRes = movoAPI_->SendBasicTrajectory(trajectoryPoint);
	}

}

VectorFloat MovoRobotInterface::getCurrentJointAngles() const {
	AngularPosition positions;
	positions.InitStruct();
	VectorFloat angles;
	int res = 0;
	while (true) {
		res = movoAPI_->GetAngularPosition(positions);
		angles = toRadianPosition(toStandardVec(positions.Actuators));
		// Check for errors
		if (angles[0] != 0.0 && angles[3] != 0.0 && angles[4] != 0.0)
			break;
	}

	return angles;
}

bool MovoRobotInterface::sendTargetJointAngles(const VectorFloat &jointAngles, const FloatType &durationMS) {
	if (jointAngles.size() != NUM_JOINTS)
		ERROR("Vector of joint angles has wrong size. Should be " + std::to_string(NUM_JOINTS));

	movoAPI_->EraseAllTrajectories();
	VectorFloat currentJointAngles = getCurrentJointAngles();
	VectorFloat jointVelocities = oppt::scaleVector(oppt::subtractVectors(jointAngles, currentJointAngles), 1000.0 / durationMS);
	return applyJointVelocities(jointVelocities, durationMS);
}

bool MovoRobotInterface::applyJointVelocities(const VectorFloat &jointVelocities, const FloatType &durationMS) const {
	if (jointVelocities.size() != NUM_JOINTS)
		ERROR("Vector of joint angles has wring size. Should be " + std::to_string(NUM_JOINTS));

	movoAPI_->EraseAllTrajectories();
	FloatType elapsedSinceStart = 0.0;
	auto timeStart = std::chrono::system_clock::now();
	while (true) {
		sendTargetJointVelocities_(jointVelocities);
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		elapsedSinceStart = (FloatType)(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - timeStart).count());
		if (elapsedSinceStart >= durationMS)
			break;
	}

	movoAPI_->EraseAllTrajectories();
	return true;
}

bool MovoRobotInterface::sendTargetJointVelocities_(const VectorFloat &jointVelocities) const {
	VectorFloat angularVelocities = toDegreesVelocity(jointVelocities);
	TrajectoryPoint trajectoryPoint;
	trajectoryPoint.InitStruct();

	trajectoryPoint.Position.HandMode = HAND_NOMOVEMENT;
	trajectoryPoint.Position.Type = ANGULAR_VELOCITY;

	trajectoryPoint.Position.Actuators.Actuator1 = angularVelocities[0];
	trajectoryPoint.Position.Actuators.Actuator2 = angularVelocities[1];
	trajectoryPoint.Position.Actuators.Actuator3 = angularVelocities[2];
	trajectoryPoint.Position.Actuators.Actuator4 = angularVelocities[3];
	trajectoryPoint.Position.Actuators.Actuator5 = angularVelocities[4];
	trajectoryPoint.Position.Actuators.Actuator6 = angularVelocities[5];
	trajectoryPoint.Position.Actuators.Actuator7 = angularVelocities[6];

	int trajectoryRes = 0;
	trajectoryRes = movoAPI_->SendBasicTrajectory(trajectoryPoint);
	if (trajectoryRes != NO_ERROR_KINOVA)
		ERROR("Failed to send velocity target");

	//movoAPI_->EraseAllTrajectories();
	return true;
}

}