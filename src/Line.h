/*
 * Line.h
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

#ifndef AVO_LINE_H_
#define AVO_LINE_H_

/**
 * \file   Line.h
 * \brief  Declares the Line class.
 */

#include "Export.h"
#include "Vector2.h"

namespace AVO {
/**
 *  \brief  A directed line.
 */
class AVO_EXPORT Line {
 public:
  /**
   *  \brief  Constructor.
   */
  Line() {}

  /**
   *  \brief  The direction of the directed line.
   */
  Vector2 direction;

  /**
   *  \brief  A point on the directed line.
   */
  Vector2 point;
};
}  // namespace AVO

#endif  // AVO_LINE_H_
