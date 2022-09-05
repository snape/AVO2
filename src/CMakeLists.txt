# -*- mode: cmake; -*-
# vi: set ft=cmake:

#
# src/CMakeLists.txt
# AVO2 Library
#
# SPDX-FileCopyrightText: 2010 University of North Carolina at Chapel Hill
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
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
# <https://gamma.cs.unc.edu/AVO/>
#

set(AVO_HEADERS
  AVO.h
  Line.h
  Simulator.h
  Vector2.h)

set(AVO_SOURCES
  Agent.cc
  Agent.h
  Export.cc
  KdTree.cc
  KdTree.h
  Line.cc
  Simulator.cc
  Vector2.cc)

add_library(${AVO_LIBRARY} ${AVO_HEADERS} ${AVO_SOURCES})

include(GenerateExportHeader)

generate_export_header(${AVO_LIBRARY} EXPORT_FILE_NAME Export.h)

set_target_properties(${AVO_LIBRARY} PROPERTIES
  CXX_VISIBILITY_PRESET hidden
  PUBLIC_HEADER "${AVO_HEADERS}"
  SOVERSION ${PROJECT_VERSION_MAJOR}
  VERSION ${PROJECT_VERSION}
  VISIBILITY_INLINES_HIDDEN ON)

if(ENABLE_INTERPROCEDURAL_OPTIMIZATION
    AND RVO_INTERPROCEDURAL_OPTIMIZATION_SUPPORTED)
  set_target_properties(${AVO_LIBRARY} PROPERTIES
    INTERPROCEDURAL_OPTIMIZATION ON)
endif()

target_include_directories(${AVO_LIBRARY} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

if(WIN32)
  target_compile_definitions(${AVO_LIBRARY} PUBLIC NOMINMAX)
endif()

if(ENABLE_OPENMP AND OpenMP_FOUND)
  set_target_properties(${AVO_LIBRARY} PROPERTIES
    LINK_FLAGS "${OpenMP_CXX_FLAGS}")
  target_compile_options(${AVO_LIBRARY} PRIVATE ${OpenMP_CXX_FLAGS})
endif()

export(TARGETS ${AVO_LIBRARY} NAMESPACE ${PROJECT_NAME}::
  FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake")

if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.12)
  set(AVO_NAMELINK_COMPONENT_ARGUMENT NAMELINK_COMPONENT development)
else()
  set(AVO_NAMELINK_COMPONENT_ARGUMENT)
endif()

install(TARGETS ${AVO_LIBRARY}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE
    COMPONENT development
    DESTINATION ${AVO_LIBRARY_DIR}
  LIBRARY
    COMPONENT runtime
    DESTINATION ${AVO_LIBRARY_DIR}
    ${AVO_NAMELINK_COMPONENT_ARGUMENT}
  PUBLIC_HEADER
    COMPONENT development
    DESTINATION ${AVO_INCLUDE_DIR}
  RUNTIME
    COMPONENT runtime
    DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${AVO_INCLUDE_DIR})

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/Export.h"
  COMPONENT development
  DESTINATION ${AVO_INCLUDE_DIR})

install(EXPORT ${PROJECT_NAME}Targets
  COMPONENT development
  DESTINATION ${AVO_DIR}
  NAMESPACE ${PROJECT_NAME}::)
