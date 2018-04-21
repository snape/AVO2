/*
 * KdTree.cc
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

#include "KdTree.h"

#include <algorithm>

#include "Agent.h"
#include "Simulator.h"
#include "Vector2.h"

namespace AVO {
namespace {
const std::size_t AVO_MAX_LEAF_SIZE = 10;
}  // namespace

void KdTree::buildAgentTree() {
  if (agents_.size() < simulator_->agents_.size()) {
    for (std::size_t agentNo = agents_.size();
         agentNo < simulator_->agents_.size(); ++agentNo) {
      agents_.push_back(simulator_->agents_[agentNo]);
    }

    agentTree_.resize(2 * agents_.size() - 1);
  }

  if (!agents_.empty()) {
    buildAgentTreeRecursive(0, agents_.size(), 0);
  }
}

void KdTree::buildAgentTreeRecursive(std::size_t begin, std::size_t end,
                                     std::size_t node) {
  agentTree_[node].begin = begin;
  agentTree_[node].end = end;
  agentTree_[node].minX = agentTree_[node].maxX = agents_[begin]->position_.x_;
  agentTree_[node].minY = agentTree_[node].maxY = agents_[begin]->position_.y_;

  for (std::size_t i = begin + 1; i < end; ++i) {
    agentTree_[node].maxX =
        std::max(agentTree_[node].maxX, agents_[i]->position_.x_);
    agentTree_[node].minX =
        std::min(agentTree_[node].minX, agents_[i]->position_.x_);
    agentTree_[node].maxY =
        std::max(agentTree_[node].maxY, agents_[i]->position_.y_);
    agentTree_[node].minY =
        std::min(agentTree_[node].minY, agents_[i]->position_.y_);
  }

  if (end - begin > AVO_MAX_LEAF_SIZE) {
    // No leaf node.
    const bool isVertical = (agentTree_[node].maxX - agentTree_[node].minX >
                             agentTree_[node].maxY - agentTree_[node].minY);
    const float splitValue =
        (isVertical ? 0.5f * (agentTree_[node].maxX + agentTree_[node].minX)
                    : 0.5f * (agentTree_[node].maxY + agentTree_[node].minY));

    std::size_t left = begin;
    std::size_t right = end;

    while (left < right) {
      while (left < right &&
             (isVertical ? agents_[left]->position_.x_
                         : agents_[left]->position_.y_) < splitValue) {
        ++left;
      }

      while (right > left &&
             (isVertical ? agents_[right - 1]->position_.x_
                         : agents_[right - 1]->position_.y_) >= splitValue) {
        --right;
      }

      if (left < right) {
        std::swap(agents_[left], agents_[right - 1]);
        ++left;
        --right;
      }
    }

    std::size_t leftSize = left - begin;

    if (leftSize == 0) {
      ++leftSize;
      ++left;
      ++right;
    }

    agentTree_[node].left = node + 1;
    agentTree_[node].right = node + 1 + (2 * leftSize - 1);

    buildAgentTreeRecursive(begin, left, agentTree_[node].left);
    buildAgentTreeRecursive(left, end, agentTree_[node].right);
  }
}

void KdTree::queryAgentTreeRecursive(Agent *agent, float &rangeSq,
                                     std::size_t node) const {
  if (agentTree_[node].end - agentTree_[node].begin <= AVO_MAX_LEAF_SIZE) {
    for (std::size_t i = agentTree_[node].begin; i < agentTree_[node].end;
         ++i) {
      agent->insertAgentNeighbor(agents_[i], rangeSq);
    }
  } else {
    const float distLeftMinX = std::max(
        0.0f, agentTree_[agentTree_[node].left].minX - agent->position_.x_);
    const float distLeftMaxX = std::max(
        0.0f, agent->position_.x_ - agentTree_[agentTree_[node].left].maxX);
    const float distLeftMinY = std::max(
        0.0f, agentTree_[agentTree_[node].left].minY - agent->position_.y_);
    const float distLeftMaxY = std::max(
        0.0f, agent->position_.y_ - agentTree_[agentTree_[node].left].maxY);

    const float distSqLeft =
        distLeftMinX * distLeftMinX + distLeftMaxX * distLeftMaxX +
        distLeftMinY * distLeftMinY + distLeftMaxY * distLeftMaxY;

    const float distRightMinX = std::max(
        0.0f, agentTree_[agentTree_[node].right].minX - agent->position_.x_);
    const float distRightMaxX = std::max(
        0.0f, agent->position_.x_ - agentTree_[agentTree_[node].right].maxX);
    const float distRightMinY = std::max(
        0.0f, agentTree_[agentTree_[node].right].minY - agent->position_.y_);
    const float distRightMaxY = std::max(
        0.0f, agent->position_.y_ - agentTree_[agentTree_[node].right].maxY);

    const float distSqRight =
        distRightMinX * distRightMinX + distRightMaxX * distRightMaxX +
        distRightMinY * distRightMinY + distRightMaxY * distRightMaxY;

    if (distSqLeft < distSqRight) {
      if (distSqLeft < rangeSq) {
        queryAgentTreeRecursive(agent, rangeSq, agentTree_[node].left);

        if (distSqRight < rangeSq) {
          queryAgentTreeRecursive(agent, rangeSq, agentTree_[node].right);
        }
      }
    } else {
      if (distSqRight < rangeSq) {
        queryAgentTreeRecursive(agent, rangeSq, agentTree_[node].right);

        if (distSqLeft < rangeSq) {
          queryAgentTreeRecursive(agent, rangeSq, agentTree_[node].left);
        }
      }
    }
  }
}
}  // namespace AVO
