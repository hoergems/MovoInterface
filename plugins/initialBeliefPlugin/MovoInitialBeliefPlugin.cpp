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
#ifndef _DEFAULT_INITIAL_STATE_SAMPLER_PLUGIN_HPP_
#define _DEFAULT_INITIAL_STATE_SAMPLER_PLUGIN_HPP_
#include <oppt/plugin/Plugin.hpp>
#include <oppt/gazeboInterface/GazeboInterface.hpp>
#include "MovoInitialBeliefOptions.hpp"
#include "MovoUserData.hpp"

namespace oppt
{
class MovoInitialBeliefPlugin: public InitialBeliefPlugin
{
public:
    MovoInitialBeliefPlugin():
        InitialBeliefPlugin() {
    }

    virtual ~MovoInitialBeliefPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        parseOptions_<MovoInitialBeliefOptions>(optionsFile);
        initialStateVector_ = static_cast<const MovoInitialBeliefOptions *>(options_.get())->initialState;

        // Get a pointer to the end-effector link
        std::string endEffectorLinkName =
            static_cast<const MovoInitialBeliefOptions *>(options_.get())->endEffectorLink;
        endEffectorLink_ = getLinkPointer_(endEffectorLinkName);        

        if (!endEffectorLink_)
            ERROR("End effector link '" + endEffectorLinkName + "' couldn't be found");

        return true;
    }

    virtual RobotStateSharedPtr sampleAnInitState() override {
        // First, force a full reset of the gazebo world
        auto world = robotEnvironment_->getGazeboInterface()->getWorld();
        world->Reset();
        world->ResetPhysicsStates();
        world->ResetTime();

        VectorFloat initialStateVector = initialStateVector_;

        // This will set the joint angles to the ones defined in initialStateVector_
        robotEnvironment_->getGazeboInterface()->setStateVector(initialStateVector);

        // Now compute the cup pose in world coordinates
        GZPose endEffectorPose = LinkWorldPose(endEffectorLink_);        

        // Now that we've set the initial joint angles, construct a gazebo world state
        robotEnvironment_->getGazeboInterface()->makeInitialWorldState(initialStateVector, false);

        // Then get the initial world state
        GazeboWorldStatePtr initialWorldState = robotEnvironment_->getGazeboInterface()->getInitialWorldState();       

        // Construct the initial state
        RobotStateSharedPtr initialState(new VectorState(initialStateVector));
        initialState->setGazeboWorldState(initialWorldState);        
        initialState->setUserData(makeUserData_());
        return initialState;
    }

private:    
    VectorFloat initialStateVector_;

    // Pointer to the end effector link
    gazebo::physics::Link *endEffectorLink_ = nullptr;    

private:
    OpptUserDataSharedPtr makeUserData_() const {
        OpptUserDataSharedPtr userData(new MovoUserData);
        userData->as<MovoUserData>()->endEffectorPose = geometric::Pose(LinkWorldPose(endEffectorLink_));
        userData->as<MovoUserData>()->collisionReport = std::make_shared<CollisionReport>();
        return userData;
    }

    gazebo::physics::Link *getLinkPointer_(const std::string &linkName) const {
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

OPPT_REGISTER_INITIAL_BELIEF_PLUGIN(MovoInitialBeliefPlugin)

}

#endif
