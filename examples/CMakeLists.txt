# -*- mode: cmake -*-
# vi: set ft=cmake :

#
# examples/CMakeLists.txt
# AVO2 Library
#
# Copyright 2010 University of North Carolina at Chapel Hill
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Please send all bug reports to <geom@cs.unc.edu>.
#
# The authors may be contacted via:
#
# Jur van den Berg, Jamie Snape, Stephen J. Guy, and Dinesh Manocha
# Dept. of Computer Science
# 201 S. Columbia St.
# Frederick P. Brooks, Jr. Computer Science Bldg.
# Chapel Hill, N.C. 27599-3175
# United States of America
#
# <http://gamma.cs.unc.edu/AVO/>
#

if(BUILD_TESTING)
    option(OUTPUT_TIME_AND_POSITIONS "Output time and positions" ON)

    if(OUTPUT_TIME_AND_POSITIONS)
        set(AVO_EXAMPLES_COMPILE_DEFINITIONS AVO_OUTPUT_TIME_AND_POSITIONS=1)
    else()
        set(AVO_EXAMPLES_COMPILE_DEFINITIONS AVO_OUTPUT_TIME_AND_POSITIONS=0)
    endif()

    if(ENABLE_OPENMP AND OpenMP_FOUND)
        set(AVO_EXAMPLES_COMPILE_OPTIONS ${OpenMP_CXX_FLAGS})
        set(AVO_EXAMPLES_LINK_FLAGS "${OpenMP_CXX_FLAGS}")
    else()
        set(AVO_EXAMPLES_COMPILE_OPTIONS)
        set(AVO_EXAMPLES_LINK_FLAGS)
    endif()

    add_executable(Circle Circle.cc)
    target_compile_definitions(Circle PRIVATE
        ${AVO_EXAMPLES_COMPILE_DEFINITIONS})
    target_link_libraries(Circle PRIVATE ${AVO_LIBRARY})
    target_compile_options(Circle PRIVATE ${AVO_EXAMPLES_COMPILE_OPTIONS})
    set_target_properties(Circle PROPERTIES
        LINK_FLAGS "${AVO_EXAMPLES_LINK_FLAGS}")
    add_test(NAME Circle COMMAND Circle)
endif()
