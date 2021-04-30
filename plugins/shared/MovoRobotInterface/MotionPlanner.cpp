#include "MotionPlanner.hpp"
#include <oppt/gazeboInterface/GazeboInterface.hpp>
#include <oppt/robotEnvironment/include/RobotEnvironment.hpp>

namespace oppt {
MovoMotionPlanner::MovoMotionPlanner(const RobotEnvironment* robotEnvironment):
	robotEnvironment_(robotEnvironment) {
	setupRRTConnect_();
}

TrajectorySharedPtr MovoMotionPlanner::solve(const VectorFloat &currentJointAngles,
        const VectorFloat &targetJointAngles,
        const FloatType &timeout) {

	RobotStateSharedPtr startState(new VectorState(currentJointAngles));
	if (rrtConnect_->isValid(startState) == false)
		ERROR("The current joint angles of the arm are not valid. Move the arm manually to a different configuration");

	RobotStateSharedPtr goalState(new VectorState(targetJointAngles));
	if (rrtConnect_->isValid(goalState) == false)
		ERROR("The initial joint angles are not valid");

	// Try a linear trajectory first
	TrajectorySharedPtr linearTrajectory = tryLinearTrajectory(startState, goalState);
	if (linearTrajectory)
		return linearTrajectory;

	std::vector<VectorFloat> goalJointAngles({targetJointAngles});
	rrtConnect_->setGoalStates(goalJointAngles);

	LOGGING("Getting trajectory to the initial state ...");
	return rrtConnect_->solve(startState, timeout);
}

TrajectorySharedPtr MovoMotionPlanner::tryLinearTrajectory(const RobotStateSharedPtr &startState,
        const RobotStateSharedPtr &goalState) const {
	TrajectorySharedPtr trajectory(new Trajectory);
	trajectory->stateTrajectory.push_back(startState);
	for (size_t i = 0; i != 10; ++i) {
		RobotStateSharedPtr interpolatedState = (rrtConnect_->getInterpolationFunction())(startState, goalState, (1.0 / 10.0) * ((FloatType)(i + 1)));
		if ((rrtConnect_->getIsValidFunction())(interpolatedState)) {
			trajectory->stateTrajectory.push_back(interpolatedState);
		} else {
			return nullptr;
		}
	}

	LOGGING("LINEAR TRAJECTORY IS VALID");
	return trajectory;
}

void MovoMotionPlanner::setupRRTConnect_() {
	rrtConnect_ = std::make_unique<oppt::RRTConnect>(robotEnvironment_, 0.5);
	oppt::DistanceFunction distanceFunction = [](const RobotStateSharedPtr & s1, const RobotStateSharedPtr & s2) {
		VectorFloat s1Vec = s1->as<VectorState>()->asVector();
		VectorFloat s2Vec = s2->as<VectorState>()->asVector();

		VectorFloat state1Vec(s1Vec.begin(), s1Vec.begin() + 7);
		VectorFloat state2Vec(s2Vec.begin(), s2Vec.begin() + 7);
		return math::euclideanDistance<FloatType>(state1Vec, state2Vec);
	};

	rrtConnect_->setDistanceFunction(distanceFunction);

	VectorString jointNames({"movo::right_shoulder_pan_joint",
	                         "movo::right_shoulder_lift_joint",
	                         "movo::right_arm_half_joint",
	                         "movo::right_elbow_joint",
	                         "movo::right_wrist_spherical_1_joint",
	                         "movo::right_wrist_spherical_2_joint",
	                         "movo::right_wrist_3_joint"
	                        });

	auto allJoints = robotEnvironment_->getGazeboInterface()->getJoints();
	std::vector<gazebo::physics::Joint *> joints;
	for (size_t i = 0; i != jointNames.size(); ++i) {
		for (auto &joint : allJoints) {
			if (joint->GetScopedName() == jointNames[i]) {
				joints.push_back(joint);
				break;
			}
		}
	}

	oppt::IsValidFunction isValidFunction = [this, joints](const RobotStateSharedPtr & state) {
		VectorFloat stateVec = state->as<VectorState>()->asVector();
		for (size_t i = 0; i != 7; ++i) {
			joints[i]->SetPosition(0, stateVec[i]);
		}

		CollisionReportSharedPtr collisionReport = robotEnvironment_->getRobot()->makeDiscreteCollisionReportDirty();
		if (collisionReport->collides) {
			for (auto &collisionPair : collisionReport->collisionPairs) {
				//cout << "(" << collisionPair.first << ", " << collisionPair.second << ")" << endl;
				if (collisionPair.first.find("robotBoundingBoxes") != std::string::npos or
				        collisionPair.second.find("robotBoundingBoxes") != std::string::npos or
				        collisionPair.first.find("table") or
				        collisionPair.second.find("table")) {
					return false;
				}
			}
		}

		return true;
	};

	rrtConnect_->setIsValidFunction(isValidFunction);

	oppt::InterpolationFunction interpolationFunction = [](const RobotStateSharedPtr & start,
	        const RobotStateSharedPtr & goal,
	const FloatType & distance) {
		VectorFloat s1 = start->as<VectorState>()->asVector();
		VectorFloat s2 = goal->as<VectorState>()->asVector();

		VectorFloat startVector(s1.begin(), s1.begin() + 7);
		VectorFloat endVector(s2.begin(), s2.begin() + 7);
		VectorFloat resultingVector =
		    oppt::addVectors(startVector, oppt::scaleVector(oppt::subtractVectors(endVector, startVector), distance));

		RobotStateSharedPtr interpolatedState(new VectorState(resultingVector));
		return interpolatedState;
	};

	rrtConnect_->setInterpolationFunction(interpolationFunction);

	VectorFloat upper(7, 0.0);
	VectorFloat lower(7, 0.0);
	for (size_t i = 0; i != joints.size(); ++i) {
#ifdef GZ_GT_7
		upper[i] = joints[i]->UpperLimit(0);
		lower[i] = joints[i]->LowerLimit(0);
#else
		upper[i] = joints[i]->GetUpperLimit(0).Radian();
		lower[i] = joints[i]->GetLowerLimit(0).Radian();
#endif
	}
	auto randomEngine = robotEnvironment_->getRobot()->getRandomEngine();
	uniformDistribution_ = std::make_unique<UniformDistribution<FloatType>>(upper, lower, randomEngine);

	oppt::StateSamplingFunction stateSamplingFunction = [this]() {
		VectorFloat stateVec = toStdVec<FloatType>(uniformDistribution_->sample(1));
		RobotStateSharedPtr sampledState(new VectorState(stateVec));
		return sampledState;
	};

	rrtConnect_->setStateSamplingFunction(stateSamplingFunction);
}


}