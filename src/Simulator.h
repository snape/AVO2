/*
 * Simulator.h
 * AVO2 Library
 *
 * SPDX-FileCopyrightText: 2010 University of North Carolina at Chapel Hill
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
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
 * <https://gamma.cs.unc.edu/AVO/>
 */

#ifndef AVO_SIMULATOR_H_
#define AVO_SIMULATOR_H_

/**
 * \file   Simulator.h
 * \brief  Declares the Simulator class.
 */

#include <cstddef>
#include <vector>

#include "Export.h"
#include "Vector2.h"

namespace AVO {
class Agent;
class KdTree;
class Line;

/**
 * \class  Simulator
 * \brief  The simulation.
 */
class AVO_EXPORT Simulator {
 public:
  /**
   * \brief  Constructor.
   */
  Simulator();

  /**
   * \brief  Destructor.
   */
  ~Simulator();

  /**
   * \brief      Adds a new agent with default properties to the simulation.
   * \param[in]  position  The starting position of this agent.
   * \return     The number of the agent.
   */
  std::size_t addAgent(const Vector2 &position);

  /**
   * \brief      Adds a new agent to the simulation.
   * \param[in]  position       The starting position of this agent.
   * \param[in]  neighborDist   The maximum neighbor distance of this agent.
   * \param[in]  maxNeighbors   The maximum neighbor count of this agent.
   * \param[in]  timeHorizon    The time horizon of this agent.
   * \param[in]  radius         The radius of this agent.
   * \param[in]  maxSpeed       The maximum speed of this agent.
   * \param[in]  maxAccel       The maximum acceleration of this agent.
   * \param[in]  accelInterval  The acceleration interval of this agent.
   * \return     The number of the agent.
   */
  std::size_t addAgent(const Vector2 &position, float neighborDist,
                       std::size_t maxNeighbors, float timeHorizon,
                       float radius, float maxSpeed, float maxAccel,
                       float accelInterval);

  /**
   * \brief      Adds a new agent to the simulation.
   * \param[in]  position       The starting position of this agent.
   * \param[in]  neighborDist   The maximum neighbor distance of this agent.
   * \param[in]  maxNeighbors   The maximum neighbor count of this agent.
   * \param[in]  timeHorizon    The time horizon of this agent.
   * \param[in]  radius         The radius of this agent.
   * \param[in]  maxSpeed       The maximum speed of this agent.
   * \param[in]  maxAccel       The maximum acceleration of this agent.
   * \param[in]  accelInterval  The acceleration interval of this agent.
   * \param[in]  velocity       The initial velocity of this agent.
   * \return     The number of the agent.
   */
  std::size_t addAgent(const Vector2 &position, float neighborDist,
                       std::size_t maxNeighbors, float timeHorizon,
                       float radius, float maxSpeed, float maxAccel,
                       float accelInterval, const Vector2 &velocity);

  /**
   * \brief  Performs a simulation step and updates the position and velocity of
   *         each agent.
   */
  void doStep();

  /**
   * \brief      Returns the acceleration interval of a specified agent.
   * \param[in]  agentNo  The number of the agent whose acceleration interval
   *                      is to be retrieved.
   * \return     The present acceleration interval of the agent.
   */
  float getAgentAccelInterval(std::size_t agentNo) const;

  /**
   *  \brief      Returns the specified agent neighbor of the specified agent.
   *  \param[in]  agentNo     The number of the agent whose agent neighbor is to
   *                          be retrieved.
   *  \param[in]  neighborNo  The number of the agent neighbor to be retrieved.
   *  \return     The number of the neighboring agent.
   */
  std::size_t getAgentNeighbor(std::size_t agentNo,
                               std::size_t neighborNo) const;

  /**
   * \brief      Returns the maximum acceleration of a specified agent.
   * \param[in]  agentNo  The number of the agent whose maximum acceleration is
   *                      to be retrieved.
   * \return     The present maximum acceleration of the agent.
   */
  float getAgentMaxAccel(std::size_t agentNo) const;

  /**
   * \brief      Returns the maximum neighbor count of a specified agent.
   * \param[in]  agentNo  The number of the agent whose maximum neighbor count
   *                      is to be retrieved.
   * \return     The present maximum neighbor count of the agent.
   */
  std::size_t getAgentMaxNeighbors(std::size_t agentNo) const;

  /**
   * \brief      Returns the maximum speed of a specified agent.
   * \param[in]  agentNo  The number of the agent whose maximum speed is to be
   *                      retrieved.
   * \return     The present maximum speed of the agent.
   */
  float getAgentMaxSpeed(std::size_t agentNo) const;

  /**
   * \brief      Returns the maximum neighbor distance of a specified agent.
   * \param[in]  agentNo  The number of the agent whose maximum neighbor
   *                      distance is to be retrieved.
   * \return     The present maximum neighbor distance of the agent.
   */
  float getAgentNeighborDist(std::size_t agentNo) const;

  /**
   *  \brief      Returns the count of agent neighbors taken into account to
   *              compute the current velocity for the specified agent.
   *  \param[in]  agentNo  The number of the agent whose count of agent
   *                       neighbors is to be retrieved.
   *  \return     The count of agent neighbors taken into account to compute the
   *              current velocity for the specified agent.
   */
  std::size_t getAgentNumNeighbors(std::size_t agentNo) const;

  /**
   *  \brief      Returns the count of ORCA constraints used to compute the
   *              current velocity for the specified agent.
   *  \param[in]  agentNo  The number of the agent whose count of ORCA
   *                       constraints is to be retrieved.
   *  \return     The count of ORCA constraints used to compute the current
   *              velocity for the specified agent.
   */
  std::size_t getAgentNumOrcaLines(std::size_t agentNo) const;

  /**
   *  \brief      Returns the specified ORCA constraint of the specified agent.
   *
   *  \details    The halfplane to the left of the line is the region of
   *              permissible velocities with respect to the specified ORCA
   *              constraint.
   *  \param[in]  agentNo  The number of the agent whose ORCA constraint is to
   *                       be retrieved.
   *  \param[in]  lineNo   The number of the ORCA constraint to be retrieved.
   *  \return     A line representing the specified ORCA constraint.
   */
  const Line &getAgentOrcaLine(std::size_t agentNo, std::size_t lineNo) const;

  /**
   * \brief      Returns the position of a specified agent.
   * \param[in]  agentNo  The number of the agent whose position is to be
   *                      retrieved.
   * \return     The present position of the (center of) the agent.
   */
  const Vector2 &getAgentPosition(std::size_t agentNo) const;

  /**
   * \brief      Returns the preferred velocity of a specified agent.
   *
   * \details    The preferred velocity of an agent is the velocity it would
   *             choose to take if it were not influenced by other agents.
   * \param[in]  agentNo  The number of the agent whose preferred velocity is to
   *                      be retrieved.
   * \return     The present preferred velocity of the agent.
   */
  const Vector2 &getAgentPrefVelocity(std::size_t agentNo) const;

  /**
   * \brief      Returns the radius of a specified agent.
   * \param[in]  agentNo  The number of the agent whose radius is to be
   *                      retrieved.
   * \return     The present radius of the agent.
   */
  float getAgentRadius(std::size_t agentNo) const;

  /**
   * \brief      Returns the time horizon of a specified agent.
   * \param[in]  agentNo  The number of the agent whose time horizon is to be
   *                      retrieved.
   * \return     The present time horizon of the agent.
   */
  float getAgentTimeHorizon(std::size_t agentNo) const;

  /**
   * \brief      Returns the velocity of a specified agent.
   * \param[in]  agentNo  The number of the agent whose velocity is to be
   *                      retrieved.
   * \return     The present velocity of the agent.
   */
  const Vector2 &getAgentVelocity(std::size_t agentNo) const;

  /**
   * \brief   Returns the global time of the simulation.
   * \return  The present global time of the simulation (zero initially).
   */
  float getGlobalTime() const { return globalTime_; }

  /**
   * \brief   Returns the count of agents in the simulation.
   * \return  The count of agents in the simulation.
   */
  std::size_t getNumAgents() const { return agents_.size(); }

  /**
   * \brief   Returns the time step of the simulation.
   * \return  The present time step of the simulation.
   */
  float getTimeStep() const { return timeStep_; }

  /**
   * \brief      Sets the acceleration interval of a specified agent.
   * \param[in]  agentNo        The number of the agent whose acceleration
   *                            interval is to be modified.
   * \param[in]  accelInterval  The replacement acceleration interval.
   */
  void setAgentAccelInterval(std::size_t agentNo, float accelInterval);

  /**
   * \brief      Sets the default properties for any new agent that is added.
   * \param[in]  neighborDist   The default maximum neighbor distance of a new
   *                            agent.
   * \param[in]  maxNeighbors   The default maximum neighbor count of a new
   *                            agent.
   * \param[in]  timeHorizon    The time horizon of a new agent.
   * \param[in]  radius         The default radius of a new agent.
   * \param[in]  maxSpeed       The default maximum speed of a new agent.
   * \param[in]  maxAccel       The default maximum acceleration of a new agent.
   * \param[in]  accelInterval  The default acceleration interval of a new
   *                            agent.
   */
  void setAgentDefaults(float neighborDist, std::size_t maxNeighbors,
                        float timeHorizon, float radius, float maxSpeed,
                        float maxAccel, float accelInterval);

  /**
   * \brief      Sets the default properties for any new agent that is added.
   * \param[in]  neighborDist   The default maximum neighbor distance of a new
   *                            agent.
   * \param[in]  maxNeighbors   The default maximum neighbor count of a new
   *                            agent.
   * \param[in]  timeHorizon    The time horizon of a new agent.
   * \param[in]  radius         The default radius of a new agent.
   * \param[in]  maxSpeed       The default maximum speed of a new agent.
   * \param[in]  maxAccel       The default maximum acceleration of a new agent.
   * \param[in]  accelInterval  The default acceleration interval of a new
   *                            agent.
   * \param[in]  velocity       The default initial velocity of a new agent.
   */
  void setAgentDefaults(float neighborDist, std::size_t maxNeighbors,
                        float timeHorizon, float radius, float maxSpeed,
                        float maxAccel, float accelInterval,
                        const Vector2 &velocity);

  /**
   * \brief      Sets the maximum acceleration of a specified agent.
   * \param[in]  agentNo   The number of the agent whose maximum acceleration is
   *                       to be modified.
   * \param[in]  maxAccel  The replacement maximum acceleration.
   */
  void setAgentMaxAccel(std::size_t agentNo, float maxAccel);

  /**
   * \brief      Sets the maximum neighbor count of a specified agent.
   * \param[in]  agentNo       The number of the agent whose maximum neighbor
   *                           count is to be modified.
   * \param[in]  maxNeighbors  The replacement maximum neighbor count.
   */
  void setAgentMaxNeighbors(std::size_t agentNo, std::size_t maxNeighbors);

  /**
   * \brief      Sets the maximum speed of a specified agent.
   * \param[in]  agentNo   The number of the agent whose maximum speed is to be
   *                       modified.
   * \param[in]  maxSpeed  The replacement maximum speed.
   */
  void setAgentMaxSpeed(std::size_t agentNo, float maxSpeed);

  /**
   * \brief      Sets the maximum neighbor distance of a specified agent.
   * \param[in]  agentNo       The number of the agent whose maximum neighbor
   *                           distance is to be modified.
   * \param[in]  neighborDist  The replacement maximum neighbor distance.
   */
  void setAgentNeighborDist(std::size_t agentNo, float neighborDist);

  /**
   * \brief      Sets the position of a specified agent.
   * \param[in]  agentNo   The number of the agent whose position is to be
   *                       modified.
   * \param[in]  position  The replacement position.
   */
  void setAgentPosition(std::size_t agentNo, const Vector2 &position);

  /**
   * \brief      Sets the preferred velocity of a specified agent.
   *
   * \details    The preferred velocity of an agent is the velocity it would
   *             choose to take if it were not influenced by other agents.
   * \param[in]  agentNo       The number of the agent whose preferred velocity
   *                           is to be modified.
   * \param[in]  prefVelocity  The replacement preferred velocity.
   */
  void setAgentPrefVelocity(std::size_t agentNo, const Vector2 &prefVelocity);

  /**
   * \brief      Sets the radius of a specified agent.
   * \param[in]  agentNo  The number of the agent whose radius is to be
   *                      modified.
   * \param[in]  radius   The replacement radius.
   */
  void setAgentRadius(std::size_t agentNo, float radius);

  /**
   * \brief      Sets the time horizon of a specified agent.
   * \param[in]  agentNo      The number of the agent whose time horizon is to
   *                          be modified.
   * \param[in]  timeHorizon  The replacement time horizon.
   */
  void setAgentTimeHorizon(std::size_t agentNo, float timeHorizon);

  /**
   * \brief      Sets the velocity of a specified agent.
   * \param[in]  agentNo   The number of the agent whose velocity is to be
   *                       modified.
   * \param[in]  velocity  The replacement velocity.
   */
  void setAgentVelocity(std::size_t agentNo, const Vector2 &velocity);

  /**
   * \brief      Sets the global time of the simulation.
   * \param[in]  globalTime  The replacement global time of the simulation.
   */
  void setGlobalTime(float globalTime) { globalTime_ = globalTime; }

  /**
   * \brief      Sets the time step of the simulation.
   * \param[in]  timeStep  The replacement time step of the simulation.
   */
  void setTimeStep(float timeStep) { timeStep_ = timeStep; }

 private:
  // Not implemented.
  Simulator(const Simulator &other);

  // Not implemented.
  Simulator &operator=(const Simulator &other);

  Agent *defaultAgent_;
  KdTree *kdTree_;
  float globalTime_;
  float timeStep_;
  std::vector<Agent *> agents_;

  friend class KdTree;
};
}  // namespace AVO

#endif  // AVO_SIMULATOR_H_
