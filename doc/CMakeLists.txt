# -*- mode: cmake -*-
# vi: set ft=cmake :

#
# doc/CMakeLists.txt
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

if(BUILD_DOCUMENTATION)
    find_package(Doxygen)

    if(Doxygen_FOUND)
        configure_file(Doxyfile.in Doxyfile @ONLY)

        add_custom_command(
            COMMAND ${DOXYGEN_EXECUTABLE} "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile"
            OUTPUT doxygen_output
            WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")
        set_property(SOURCE doxygen_output PROPERTY SYMBOLIC ON)

        add_custom_target(documentation ALL DEPENDS doxygen_output)

        install(DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/html"
            DESTINATION ${CMAKE_INSTALL_DOCDIR})
    endif()
endif()
