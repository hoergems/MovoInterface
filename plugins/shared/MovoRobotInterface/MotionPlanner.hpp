#ifndef _OPPT_MOVO_MOTION_PLANNER_HPP_
#define _OPPT_MOVO_MOTION_PLANNER_HPP_
#include "RRTConnect/RRTConnect.hpp"
#include <oppt/opptCore/Distribution.hpp>

namespace oppt {
class MovoMotionPlanner {
public:
	MovoMotionPlanner(const RobotEnvironment* robotEnvironment);

	~MovoMotionPlanner() = default;

	_NO_COPY_BUT_MOVE(MovoMotionPlanner)

	/**
	 * @brief Computes a collision-free trajectory from the currentJointAngles to the targetJointAngles
	 */
	TrajectorySharedPtr solve(const VectorFloat &currentJointAngles,
	                          const VectorFloat &targetJointAngles,
	                          const FloatType &timeout);

private:
	const RobotEnvironment *robotEnvironment_ = nullptr;

	std::unique_ptr<oppt::RRTConnect> rrtConnect_ = nullptr;

	std::unique_ptr<UniformDistribution<FloatType>> uniformDistribution_ = nullptr;

private:
	void setupRRTConnect_();

	TrajectorySharedPtr tryLinearTrajectory(const RobotStateSharedPtr &startState,
	                                        const RobotStateSharedPtr &goalState) const;

};
}

#endif