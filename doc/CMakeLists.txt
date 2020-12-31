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

option(BUILD_DOCUMENTATION "Build documentation" OFF)

if(BUILD_DOCUMENTATION)
  find_package(Doxygen 1.8.13 MODULE)

  if(Doxygen_FOUND)
    set(DOXYGEN_PROJECT_NAME "${AVO_NAME}")
    set(DOXYGEN_PROJECT_NUMBER ${PROJECT_VERSION})
    set(DOXYGEN_PROJECT_BRIEF "${PROJECT_DESCRIPTION}")

    set(DOXYGEN_STRIP_FROM_PATH
      "${PROJECT_BINARY_DIR}/src" "${PROJECT_SOURCE_DIR}/src")
    set(DOXYGEN_BUILTIN_STL_SUPPORT YES)

    set(DOXYGEN_EXTRACT_ALL YES)

    set(DOXYGEN_FILE_PATTERNS "*.h")
    set(DOXYGEN_EXAMPLE_PATH "${PROJECT_SOURCE_DIR}/examples")
    set(DOXYGEN_EXAMPLE_PATTERNS "*.cc")

    set(DOXYGEN_HTML_HEADER "${CMAKE_CURRENT_SOURCE_DIR}/header.html")
    set(DOXYGEN_HTML_FOOTER "${CMAKE_CURRENT_SOURCE_DIR}/footer.html")
    set(DOXYGEN_HTML_EXTRA_STYLESHEET
      "${CMAKE_CURRENT_SOURCE_DIR}/stylesheet.css")
    set(DOXYGEN_SEARCHENGINE NO)

    set(DOXYGEN_DOT_IMAGE_FORMAT svg)
    set(DOXYGEN_INTERACTIVE_SVG YES)

    if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.12)
      set(AVO_ALL_ARGUMENT ALL)
      set(AVO_EXCLUDE_FROM_ALL_ARGUMENT)
    else()
      set(AVO_ALL_ARGUMENT)
      set(AVO_EXCLUDE_FROM_ALL_ARGUMENT EXCLUDE_FROM_ALL)
    endif()

    if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.16)
      set(AVO_USE_STAMP_FILE_ARGUMENT USE_STAMP_FILE)
    else()
      set(AVO_USE_STAMP_FILE_ARGUMENT)
    endif()

    doxygen_add_docs(documentation
      "${PROJECT_SOURCE_DIR}/src/AVO.h"
      "${PROJECT_BINARY_DIR}/src/Export.h"
      "${PROJECT_SOURCE_DIR}/src/Line.h"
      "${PROJECT_SOURCE_DIR}/src/Simulator.h"
      "${PROJECT_SOURCE_DIR}/src/Vector2.h"
      ${AVO_ALL_ARGUMENT}
      ${AVO_USE_STAMP_FILE_ARGUMENT})

    install(DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/html"
      COMPONENT documentation
      DESTINATION ${CMAKE_INSTALL_DOCDIR}
      ${AVO_EXCLUDE_FROM_ALL_ARGUMENT})
  endif()
endif()
