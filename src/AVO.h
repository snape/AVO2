/*
 * AVO.h
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

#ifndef AVO_AVO_H_
#define AVO_AVO_H_

/**
 * \file   AVO.h
 * \brief  Includes all public headers.
 */

/**
 * \namespace  AVO
 * \brief      Contains all classes and functions.
 */

/**
 * \mainpage   AVO2 Library Documentation
 * \authors    Jur van den Berg, Jamie Snape, Stephen J. Guy, Dinesh Manocha
 * \copyright  2010 University of North Carolina at Chapel Hill.
 * \details    We present an approach for collision avoidance for mobile robots
 *             that takes into account acceleration constraints. We discuss both
 *             the case of navigating a single robot among moving obstacles, and
 *             the case of multiple robots reciprocally avoiding collisions with
 *             each other while navigating a common workspace. Inspired by the
 *             concept of velocity obstacles, we introduce the
 *             acceleration-velocity obstacle to let a robot avoid collisions
 *             with moving obstacles while obeying acceleration constraints. AVO
 *             characterizes the set of new velocities the robot can safely
 *             reach and adopt using proportional control of the acceleration.
 *             We extend this concept to reciprocal collision avoidance for
 *             multi-robot settings, by letting each robot take half of the
 *             responsibility of avoiding pairwise collisions. Our formulation
 *             guarantees collision-free navigation even as the robots act
 *             independently and simultaneously, without coordination. Our
 *             approach is designed for holonomic robots, but can also be
 *             applied to kinematically constrained non-holonomic robots such as
 *             cars. We have implemented our approach, and we show simulation
 *             results in challenging environments with large numbers of robots
 *             and obstacles.
 */

// IWYU pragma: begin_exports
#include "Export.h"
#include "Line.h"
#include "Simulator.h"
#include "Vector2.h"
// IWYU pragma: end_exports

#endif  // AVO_AVO_H_
