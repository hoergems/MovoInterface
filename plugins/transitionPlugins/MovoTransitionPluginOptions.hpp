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
#ifndef _MOVO_TRANSITION_PLUGIN_OPTIONS_HPP_
#define _MOVO_TRANSITION_PLUGIN_OPTIONS_HPP_
#include "oppt/plugin/PluginOptions.hpp"

namespace oppt
{
class MovoTransitionPluginOptions: public PluginOptions
{
public:
  MovoTransitionPluginOptions() = default;

  virtual ~MovoTransitionPluginOptions() = default;

  std::string urdfFile = "";

  std::string baseLink = "";

  std::string endEffectorLink = "";

  std::string cupLink = "";

  FloatType endEffectorMotionDistance = 0.0;

  VectorFloat initialState;

  std::string localIP = "";

  static std::unique_ptr<options::OptionParser> makeParser() {
    std::unique_ptr<options::OptionParser> parser =
      PluginOptions::makeParser();
    addAudioClassificationTransitionPluginOptions(parser.get());
    return std::move(parser);
  }

  static void addAudioClassificationTransitionPluginOptions(options::OptionParser* parser) {
    parser->addOption<std::string>("movoOptions",
                                   "urdfFile",
                                   &MovoTransitionPluginOptions::urdfFile);
    parser->addOption<std::string>("movoOptions",
                                   "baseLink",
                                   &MovoTransitionPluginOptions::baseLink);
    parser->addOption<std::string>("movoOptions",
                                   "endEffectorLink",
                                   &MovoTransitionPluginOptions::endEffectorLink);
    parser->addOption<std::string>("movoOptions",
                                   "cupLink",
                                   &MovoTransitionPluginOptions::cupLink);
    parser->addOption<std::string>("movoOptions",
                                   "localIP",
                                   &MovoTransitionPluginOptions::localIP);
    parser->addOption<FloatType>("transitionPluginOptions",
                                 "endEffectorMotionDistance",
                                 &MovoTransitionPluginOptions::endEffectorMotionDistance);
    parser->addOption<VectorFloat>("initialBeliefOptions",
                                   "initialState",
                                   &MovoTransitionPluginOptions::initialState);
  }

};
}

#endif
