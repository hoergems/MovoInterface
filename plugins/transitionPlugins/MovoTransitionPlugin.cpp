/**
 * Copyright 2017
 *
 * This file is part of On-line POMDP Planning Toolkit (OPPT).
 * OPPT is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License published by the Free Software Foundation,
 * either version 2 of the License, or (at your option) any later version.
 *
 * OPPT is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with OPPT.
 * If not, see http://www.gnu.org/licenses/.
 */
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <oppt/plugin/Plugin.hpp>
#include <oppt/robotHeaders/InverseKinematics/InverseKinematics.hpp>
#include <oppt/gazeboInterface/GazeboInterface.hpp>
#include "MovoTransitionPluginOptions.hpp"
#include "MovoUserData.hpp"
#include "MovoRobotInterface/MovoRobotInterface.hpp"
#include "MovoRobotInterface/RobotiqInterface.hpp"
#include <chrono>

namespace oppt
{

class MovoTransitionPlugin: public TransitionPlugin
{
public :
    MovoTransitionPlugin():
        TransitionPlugin() {
    }

    virtual ~MovoTransitionPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        parseOptions_<MovoTransitionPluginOptions>(optionsFile);

        auto options = static_cast<const MovoTransitionPluginOptions *>(options_.get());

        // Get a pointer to the end-effector link
        std::string endEffectorLinkName = options->endEffectorLink;
        endEffectorLink_ = getLinkPointer_(endEffectorLinkName);
        if (!endEffectorLink_)
            ERROR("End effector link '" + endEffectorLinkName + "' couldn't be found");

        // Setup the ik solver
        setupIKSolver_();

        // If this is the execution plugin, initialize the Movo API
        if (robotEnvironment_->isExecutionEnvironment())
            initializeMovoInterface_();
        return true;
    }

    virtual PropagationResultSharedPtr propagateState(const PropagationRequest* propagationRequest) const override {
        // First update Gazebo with the world state contained in the current state
        robotEnvironment_->getGazeboInterface()->setWorldState(propagationRequest->currentState->getGazeboWorldState().get());
        VectorFloat currentStateVector = propagationRequest->currentState->as<VectorState>()->asVector();
        VectorFloat endEffectorVelocity(6, 0.0);

        endEffectorVelocity[0] = 0.01;

        if (robotEnvironment_->isExecutionEnvironment())
        {
            cout << "Robot Execution : Press Enter to continue" << endl;
            getchar();

        }

        VectorFloat nextJointAngles = applyEndEffectorVelocity_(currentStateVector, endEffectorVelocity);

        PropagationResultSharedPtr propagationResult(new PropagationResult);
        propagationResult->nextState = RobotStateSharedPtr(new VectorState(nextJointAngles));
        propagationResult->nextState->setUserData(makeUserData_());
        propagationResult->nextState->setGazeboWorldState(robotEnvironment_->getGazeboInterface()->getWorldState(true));

        auto userDataNew = makeUserData();
        propagationResult->nextState->setUserData(userDataNew);
        propagationResult->collisionReport = static_cast<MovoUserData *>(propagationResult->nextState->getUserData().get())->collisionReport;
        return propagationResult;
    }

private:

    /** @brief A pointer to the end effector link */
    gazebo::physics::Link *endEffectorLink_ = nullptr;

    /**
     * @brief The default motion distance of the end-effector (in meters) for each directional action
     * This distance can be modified in the configuration file (via transitionPluginOptions.endEffectorMotionDistance)
     */
    FloatType endEffectorMotionDistance_ = 0.05;

    /** @brief The interface to the physical robot */
    std::unique_ptr<MovoRobotInterface> movoRobotInterface_ = nullptr;

    std::unique_ptr<movo::RobotiqInterface> robotiQInterface_ = nullptr;

    std::unique_ptr<ros::NodeHandle> nh_ = nullptr;


private:
    /** @brief Initializes the MovoInterface*/
    void initializeMovoInterface_() {
        LOGGING("Initializing");
        getchar();
        std::string localIP =
            static_cast<const MovoTransitionPluginOptions *>(options_.get())->localIP;
        movoRobotInterface_ = std::unique_ptr<MovoRobotInterface>(new MovoRobotInterface(robotEnvironment_));
        movoRobotInterface_->init(localIP, MovoArms::LEFT);

        LOGGING("Initialized");
        getchar();

        int argc = 0;
        char** argv;
        ros::init(argc, argv, "MovoRobotInterface");
        nh_ = std::make_unique<ros::NodeHandle>("~");
        robotiQInterface_ = std::unique_ptr<movo::RobotiqInterface>(new movo::RobotiqInterface(nh_.get()));

        // Move the arm to the initial joint angles
        VectorFloat initialState = static_cast<const MovoTransitionPluginOptions *>(options_.get())->initialState;
        VectorFloat initialJointAngles(initialState.begin(), initialState.begin() + 7);

        LOGGING("Opening gripper");
        getchar();
        robotiQInterface_->openGripper();
        std::this_thread::sleep_for(std::chrono::seconds(1));
        movoRobotInterface_->moveToInitialJointAngles(initialJointAngles);

        LOGGING("DONE");
        getchar();
    }



    RobotStateUserDataSharedPtr makeUserData() const {
        RobotStateUserDataSharedPtr userData(new MovoUserData());
        auto ud = static_cast<MovoUserData*>(userData.get());

        ud->collisionReport = robotEnvironment_->getRobot()->makeDiscreteCollisionReportDirty();

        return userData;
    }

    VectorFloat applyEndEffectorVelocity_(const VectorFloat &currentStateVector, const VectorFloat &endEffectorVelocity) const {
        auto tracIkSolver = static_cast<oppt::TracIKSolver *>(robotEnvironment_->getRobot()->getIKSolver());
        VectorFloat currentJointAngles = getCurrentJointAngles_(currentStateVector);

        // Get the vector of joint velocities corresponding to the end effector velocity
        VectorFloat jointVelocities =
            tracIkSolver->jointVelocitiesFromTwist(currentJointAngles, endEffectorVelocity);

        // The next joint angles are then simply the current joint angles, plus the joint velocities
        VectorFloat newJointAngles = applyJointVelocities_(currentJointAngles, jointVelocities, 1000.0);

        // Update the Gazebo model with the next joint angles
        robotEnvironment_->getGazeboInterface()->setStateVector(newJointAngles);

        return newJointAngles;
    }

    /**
     * @brief Get the current joint angles of the robot. If this is the planning plugin,
     * the current joint angles are simply obtained from the current state vector. If this is the
     * execution plugin, we read the current joint angles from the robot
     */
    VectorFloat getCurrentJointAngles_(const VectorFloat &currentStateVector) const {
        if (robotEnvironment_->isExecutionEnvironment())
            return movoRobotInterface_->getCurrentJointAngles();
        VectorFloat currentJointAngles(currentStateVector.begin(), currentStateVector.begin() + 7);
        return currentJointAngles;
    }

    /**
     * @brief Apply a vector of joint velocities to the robot for a duration of 'durationMS' (in milliseconds) and return the resulting vector of joint angles
     */
    VectorFloat applyJointVelocities_(const VectorFloat &currentJointAngles, const VectorFloat &jointVelocities, const FloatType &durationMS) const {
        if (robotEnvironment_->isExecutionEnvironment()) {
            movoRobotInterface_->applyJointVelocities(jointVelocities, durationMS);
            return movoRobotInterface_->getCurrentJointAngles();
        }

        return addVectors(currentJointAngles, jointVelocities);
    }

    /**
     * @brief Initialized the inverse kinematics solver
     */
    void setupIKSolver_() {
        auto options = static_cast<const MovoTransitionPluginOptions *>(options_.get());
        std::string urdfFile = options->urdfFile;
        std::string baseLink = options->baseLink;
        std::string endEffectorLink = options->endEffectorLink;

        if (oppt::resources::FileExists(urdfFile) == false)
            ERROR("URDF file '" + urdfFile + "' doesn't exist");

        std::string urdfPath = oppt::resources::FindFile(urdfFile);
        auto randomEngine = robotEnvironment_->getRobot()->getRandomEngine();
        IKSolverUniquePtr ikSolver =
            std::make_unique<TracIKSolver>(randomEngine.get(), urdfPath, baseLink, endEffectorLink);
        auto tracIkSolver = static_cast<oppt::TracIKSolver *>(ikSolver.get());
        if (tracIkSolver->init() == false)
            ERROR("IKSolver could not be initialized");
        robotEnvironment_->getRobot()->setIKSolver(std::move(ikSolver));
    }

    OpptUserDataSharedPtr makeUserData_() const {
        OpptUserDataSharedPtr userData(new MovoUserData);
        userData->as<MovoUserData>()->endEffectorPose = geometric::Pose(LinkWorldPose(endEffectorLink_));
        return userData;

    }

    /**
     * @brief Helper function to get a pointer to a link with the given link name
     */
    gazebo::physics::Link *getLinkPointer_(const std::string & linkName) const {
        gazebo::physics::Link *linkPtr = nullptr;
        auto links = robotEnvironment_->getGazeboInterface()->getLinks();
        for (auto &link : links) {
            if (link->GetScopedName().find(linkName) != std::string::npos) {
                linkPtr = link;
                break;
            }
        }

        return linkPtr;
    }

    GZPose LinkWorldPose(const gazebo::physics::Link* link) const {
        // Returns link world pose according to gazebo api enabled
#ifdef GZ_GT_7
        return link->WorldPose();
#else
        return link->GetWorldPose();
#endif

    }


};

OPPT_REGISTER_TRANSITION_PLUGIN(MovoTransitionPlugin)

}
