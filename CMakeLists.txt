# -*- mode: cmake -*-
# vi: set ft=cmake :

#
# CMakeLists.txt
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

cmake_minimum_required(VERSION 3.0)
project(AVO VERSION 1.0.0 LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING
        "Choose the type of build; options are Debug Release RelWithDebInfo MinSizeRel"
        FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY
        STRINGS Debug Release RelWithDebInfo MinSizeRel)
endif()

include(CTest)

option(BUILD_DOCUMENTATION "Build documentation" OFF)

if(WIN32)
  set(BUILD_SHARED_LIBS OFF)
else()
  option(BUILD_SHARED_LIBS "Build all libraries as shared" ON)
endif()

option(ENABLE_OPENMP "Enable OpenMP if available" OFF)

include(GNUInstallDirs)

set(AVO_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})
set(AVO_INCLUDE_DIR ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})
set(AVO_LIBRARY_DIR ${CMAKE_INSTALL_LIBDIR})
set(AVO_LIBRARY AVO)

if(ENABLE_OPENMP)
    find_package(OpenMP MODULE)
endif()

set(CMAKE_CXX_STANDARD 98)
set(CMAKE_CXX_STANDARD_REQUIRED OFF)

add_subdirectory(src)
add_subdirectory(examples)
add_subdirectory(doc)

install(FILES LICENSE DESTINATION ${CMAKE_INSTALL_DOCDIR})

include(CMakePackageConfigHelpers)

configure_package_config_file(AVOConfig.cmake.in AVOConfig.cmake
    INSTALL_DESTINATION ${AVO_DIR}
    PATH_VARS AVO_INCLUDE_DIR AVO_LIBRARY_DIR
    NO_CHECK_REQUIRED_COMPONENTS_MACRO)

write_basic_package_version_file(AVOConfigVersion.cmake
    COMPATIBILITY SameMajorVersion)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/AVOConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/AVOConfigVersion.cmake"
    DESTINATION ${AVO_DIR})

find_package(PkgConfig MODULE)

if(PkgConfig_FOUND)
    configure_file(AVO.pc.in AVO.pc @ONLY)
    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/AVO.pc"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)
endif()

include(InstallRequiredSystemLibraries)

set(CPACK_PACKAGE_NAME ${PROJECT_NAME})
set(CPACK_PACKAGE_CONTACT "Jamie Snape")
set(CPACK_PACKAGE_VENDOR "University of North Carolina at Chapel Hill")
set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY
        "Reciprocal Collision Avoidance with Acceleration-Velocity Obstacles")

set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_STRIP_FILES ON)

set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
set(CPACK_DEBIAN_PACKAGE_HOMEPAGE https://gamma.cs.unc.edu/RVO2/)
set(CPACK_DEBIAN_PACKAGE_SECTION contrib/devel)
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)

set(CPACK_FREEBSD_PACKAGE_LICENSE APACHE20)

set(CPACK_RPM_FILE_NAME RPM-DEFAULT)
set(CPACK_RPM_PACKAGE_LICENSE "ASL 2.0")
set(CPACK_RPM_PACKAGE_URL ${CPACK_DEBIAN_PACKAGE_HOMEPAGE})

include(CPack)
