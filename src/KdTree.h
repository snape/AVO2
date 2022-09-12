/*
 * KdTree.h
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

#ifndef AVO_KD_TREE_H_
#define AVO_KD_TREE_H_

/**
 * @file  KdTree.h
 * @brief Declares the KdTree class.
 */

#include <cstddef>
#include <vector>

namespace AVO {
class Agent;
class Simulator;

/*
 * @brief k-D tree for agents in the simulation.
 */
class KdTree {
 private:
  class AgentTreeNode;

  /**
   * @brief     Constructor.
   * @param[in] simulator The simulation.
   */
  explicit KdTree(Simulator *simulator);

  /**
   * @brief Destructor.
   */
  ~KdTree();

  /**
   * @brief Builds an agent k-D tree.
   */
  void buildAgentTree();

  /**
   * @brief     Recursive function to build a k-D tree.
   * @param[in] begin The beginning k-D tree node.
   * @param[in] end   The ending k-D tree node.
   * @param[in] node  The current k-D tree node.
   */
  void buildAgentTreeRecursive(std::size_t begin, std::size_t end,
                               std::size_t node);

  /**
   * @brief     Computes the neighbors of the specified agent.
   * @param[in] agent   A pointer to the agent for which neighbors are to be
   *                    computed.
   * @param[in] rangeSq The squared range around the agent.
   */
  void computeAgentNeighbors(
      Agent *agent, float &rangeSq) const;  // NOLINT(runtime/references)

  /**
   * @brief         Recursive function to compute the neighbors of the specified
   *                agent.
   * @param[in]     agent   A pointer to the agent for which neighbors are to be
   *                        computed.
   * @param[in,out] rangeSq The squared range around the agent.
   * @param[in]     node    The current k-D tree node.
   */
  void queryAgentTreeRecursive(Agent *agent,
                               float &rangeSq,  // NOLINT(runtime/references)
                               std::size_t node) const;

  // Not implemented.
  KdTree(const KdTree &other);

  // Not implemented.
  KdTree &operator=(const KdTree &other);

  Simulator *simulator_;
  std::vector<Agent *> agents_;
  std::vector<AgentTreeNode> agentTree_;

  friend class Agent;
  friend class Simulator;
};
}  // namespace AVO

#endif  // AVO_KD_TREE_H_
