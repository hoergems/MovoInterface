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
#include <oppt/plugin/Plugin.hpp>

namespace oppt
{
class MovoRewardPlugin: public RewardPlugin
{
public :
    MovoRewardPlugin():
        RewardPlugin() {

    }

    virtual ~MovoRewardPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        return true;
    }

    virtual double getReward(const PropagationResultSharedPtr& propagationResult) const override {
        return 0.0;
    }

    virtual std::pair<double, double> getMinMaxReward() const override  {
        return std::make_pair(0.0,
                              0.0);
    }
};

OPPT_REGISTER_REWARD_PLUGIN(MovoRewardPlugin)

}
