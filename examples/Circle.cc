/*
 * Circle.cc
 * AVO2 Library
 *
 * Copyright 2010 University of North Carolina at Chapel Hill
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Jamie Snape, Stephen J. Guy, and Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/AVO/>
 */

/**
 * \file   Circle.cc
 * \brief  Example with 250 agents navigating through a circular environment.
 */

#include <cmath>

#if AVO_OUTPUT_TIME_AND_POSITIONS
#include <iostream>
#endif

#include <vector>

#include "AVO.h"

const float AVO_TWO_PI = 6.283185307179586f;

bool haveReachedGoals(const AVO::Simulator &simulator,
                      const std::vector<AVO::Vector2> &goals) {
  for (std::size_t i = 0; i < simulator.getNumAgents(); ++i) {
    if (AVO::absSq(simulator.getAgentPosition(i) - goals[i]) > 0.25f) {
      return false;
    }
  }

  return true;
}

int main() {
  AVO::Simulator simulator;

  simulator.setTimeStep(0.25f);
  simulator.setAgentDefaults(15.0f, 10, 10.0f, 1.5f, 4.0f, 2.0f, 2.0f);

  std::vector<AVO::Vector2> goals;

  for (std::size_t i = 0; i < 250; ++i) {
    const AVO::Vector2 position =
        200.0f * AVO::Vector2(std::cos(0.004f * i * AVO_TWO_PI),
                              std::sin(0.004f * i * AVO_TWO_PI));

    simulator.addAgent(position);
    goals.push_back(-position);
  }

  do {
#if AVO_OUTPUT_TIME_AND_POSITIONS
    std::cout << simulator.getGlobalTime();

    for (std::size_t i = 0; i < simulator.getNumAgents(); ++i) {
      std::cout << " " << simulator.getAgentPosition(i);
    }

    std::cout << std::endl;
#endif  // AVO_OUTPUT_TIME_AND_POSITIONS

    for (std::size_t i = 0; i < simulator.getNumAgents(); ++i) {
      AVO::Vector2 toGoal = goals[i] - simulator.getAgentPosition(i);

      if (AVO::absSq(toGoal) > 1.0f) {
        toGoal = normalize(toGoal);
      }

      simulator.setAgentPrefVelocity(i, toGoal);
    }

    simulator.doStep();
  } while (!haveReachedGoals(simulator, goals));

  return 0;
}
