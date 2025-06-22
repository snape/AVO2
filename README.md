<!--
README.md
AVO2 Library

SPDX-FileCopyrightText: 2010 University of North Carolina at Chapel Hill
SPDX-License-Identifier: CC-BY-SA-4.0

Creative Commons Attribution-ShareAlike 4.0 International Public License

You are free to:

* Share — copy and redistribute the material in any medium or format

* ShareAlike — If you remix, transform, or build upon the material, you must
  distribute your contributions under the same license as the original

* Adapt — remix, transform, and build upon the material for any purpose, even
  commercially.

The licensor cannot revoke these freedoms as long as you follow the license
terms.

Under the following terms:

* Attribution — You must give appropriate credit, provide a link to the
  license, and indicate if changes were made. You may do so in any reasonable
  manner, but not in any way that suggests the licensor endorses you or your
  use.

* No additional restrictions — You may not apply legal terms or technological
  measures that legally restrict others from doing anything the license
  permits.

Notices:

* You do not have to comply with the license for elements of the material in
  the public domain or where your use is permitted by an applicable exception
  or limitation.

* No warranties are given. The license may not give you all of the permissions
  necessary for your intended use. For example, other rights such as publicity,
  privacy, or moral rights may limit how you use the material.

Please send all bug reports to <geom@cs.unc.edu>.

The authors may be contacted via:

Jur van den Berg, Jamie Snape, Stephen J. Guy, and Dinesh Manocha
Dept. of Computer Science
201 S. Columbia St.
Frederick P. Brooks, Jr. Computer Science Bldg.
Chapel Hill, N.C. 27599-3175
United States of America

<https://gamma.cs.unc.edu/AVO/>
-->

Reciprocal Collision Avoidance with Acceleration-Velocity Obstacles
===================================================================

<https://gamma.cs.unc.edu/AVO/>

[![DOI](https://zenodo.org/badge/130424630.svg)](https://zenodo.org/badge/latestdoi/130424630)

We present an approach for collision avoidance for mobile robots that takes into
account acceleration constraints. We discuss both the case of navigating a
single robot among moving obstacles, and the case of multiple robots
reciprocally avoiding collisions with each other while navigating a common
workspace. Inspired by the concept of velocity obstacles, we introduce the
acceleration-velocity obstacle (AVO) to let a robot avoid collisions with moving
obstacles while obeying acceleration constraints. AVO characterizes the set of
new velocities the robot can safely reach and adopt using proportional control
of the acceleration. We extend this concept to reciprocal collision avoidance
for multi-robot settings, by letting each robot take half of the responsibility
of avoiding pairwise collisions. Our formulation guarantees collision-free
navigation even as the robots act independently and simultaneously, without
coordination. Our approach is designed for holonomic robots, but can also be
applied to kinematically constrained non-holonomic robots such as cars. We have
implemented our approach, and we show simulation results in challenging
environments with large numbers of robots and obstacles.

![Build Status](https://github.com/snape/AVO2/workflows/ci/badge.svg?branch=main)

<!-- REUSE-IgnoreStart -->
SPDX-FileCopyrightText: 2010 University of North Carolina at Chapel Hill  
SPDX-License-Identifier: Apache-2.0

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

&nbsp;&nbsp;<https://www.apache.org/licenses/LICENSE-2.0>

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

Please send all bug reports to [geom@cs.unc.edu](mailto:geom@cs.unc.edu).

The authors may be contacted via:

Jur van den Berg, Jamie Snape, Stephen J. Guy, and Dinesh Manocha  
Dept. of Computer Science  
201 S. Columbia St.  
Frederick P. Brooks, Jr. Computer Science Bldg.  
Chapel Hill, N.C. 27599-3175  
United States of America
<!-- REUSE-IgnoreEnd -->
