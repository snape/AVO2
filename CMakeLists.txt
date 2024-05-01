# -*- mode: cmake; -*-
# vi: set ft=cmake:

#
# CMakeLists.txt
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

cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
project(AVO
  VERSION 1.0.1
  DESCRIPTION
    "Reciprocal Collision Avoidance with Acceleration-Velocity Obstacles"
  HOMEPAGE_URL https://gamma.cs.unc.edu/AVO/
  LANGUAGES CXX)

list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake/modules")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  # cmake-lint: disable=C0301
  set(CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build; options are Debug Release RelWithDebInfo MinSizeRel"
    FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY
    STRINGS Debug Release RelWithDebInfo MinSizeRel)
endif()

include(CTest)

if(WIN32)
  set(BUILD_SHARED_LIBS OFF)
else()
  option(BUILD_SHARED_LIBS "Build all libraries as shared" ON)
endif()

include(GNUInstallDirs)

set(AVO_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})
set(AVO_INCLUDE_DIR ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})
set(AVO_LIBRARY_DIR ${CMAKE_INSTALL_LIBDIR})
set(AVO_LIBRARY AVO)

set(AVO_NAME "AVO2 Library")

set(CMAKE_CXX_STANDARD 98)
set(CMAKE_CXX_STANDARD_REQUIRED OFF)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(ENABLE_OPENMP "Enable OpenMP if available" OFF)

if(ENABLE_OPENMP)
  find_package(OpenMP MODULE)
endif()

option(ENABLE_INTERPROCEDURAL_OPTIMIZATION
  "Enable interprocedural optimization if supported" OFF)

if(ENABLE_INTERPROCEDURAL_OPTIMIZATION)
  include(CheckIPOSupported)

  check_ipo_supported(RESULT AVO_INTERPROCEDURAL_OPTIMIZATION_SUPPORTED
    LANGUAGES CXX)

  if(AVO_INTERPROCEDURAL_OPTIMIZATION_SUPPORTED)
    message(STATUS "Interprocedural optimization is supported")
  else()
    message(STATUS "Interprocedural optimization is NOT supported")
  endif()
else()
  set(AVO_INTERPROCEDURAL_OPTIMIZATION_SUPPORTED)
endif()

include(CheckCXXCompilerFlag)
include(CheckCXXLinkerFlag)

option(ENABLE_HARDENING
  "Enable hardening compiler and linker flags if supported" OFF)

if(ENABLE_HARDENING)
  if(MSVC)
    check_cxx_compiler_flag(/GS AVO_COMPILER_SUPPORTS_GS)
    check_cxx_compiler_flag(/guard:cf AVO_COMPILER_SUPPORTS_GUARD_CF)

    if(AVO_COMPILER_SUPPORTS_GS)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /GS")
    endif()

    check_cxx_linker_flag(/DYNAMICBASE AVO_LINKER_SUPPORTS_DYNAMICBASE)
    check_cxx_linker_flag(/GUARD:CF AVO_LINKER_SUPPORTS_GUARD_CF)
    check_cxx_linker_flag(/NXCOMPAT AVO_LINKER_SUPPORTS_NXCOMPAT)

    if(AVO_COMPILER_SUPPORTS_GUARD_CF
        AND AVO_LINKER_SUPPORTS_DYNAMICBASE
        AND AVO_LINKER_SUPPORTS_GUARD_CF)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /guard:cf")
      set(CMAKE_EXE_LINKER_FLAGS
        "${CMAKE_EXE_LINKER_FLAGS} /DYNAMICBASE /GUARD:CF")
      set(CMAKE_SHARED_LINKER_FLAGS
        "${CMAKE_SHARED_LINKER_FLAGS} /DYNAMICBASE /GUARD:CF")
    endif()

    if(AVO_LINKER_SUPPORTS_NXCOMPAT)
      set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /NXCOMPAT")
      set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /NXCOMPAT")
    endif()
  else()
    check_cxx_compiler_flag(-D_FORTIFY_SOURCE=2
      AVO_COMPILER_SUPPORTS_D_FORTIFY_SOURCE_2)
    check_cxx_compiler_flag(-fcf-protection
      AVO_COMPILER_SUPPORTS_FCF_PROTECTION)
    check_cxx_compiler_flag(-fno-common AVO_COMPILER_SUPPORTS_FNO_COMMON)
    check_cxx_compiler_flag(-fsanitize=safe-stack
      AVO_COMPILER_SUPPORTS_FSANITIZE_SAFE_STACK)
    check_cxx_compiler_flag(-fstack-clash-protection
      AVO_COMPILER_SUPPORTS_FSTACK_CLASH_PROTECTION)
    check_cxx_compiler_flag(-fstack-protector-strong
      AVO_COMPILER_SUPPORTS_FSTACK_PROTECTOR_STRONG)

    if(AVO_COMPILER_SUPPORTS_FSTACK_PROTECTOR_STRONG)
      set(AVO_COMPILER_SUPPORTS_FSTACK_PROTECTOR)
    else()
      check_cxx_compiler_flag(-fstack-protector
        AVO_COMPILER_SUPPORTS_FSTACK_PROTECTOR)
    endif()

    if(AVO_COMPILER_SUPPORTS_D_FORTIFY_SOURCE_2)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_FORTIFY_SOURCE=2")
    endif()
    if(AVO_COMPILER_SUPPORTS_FCF_PROTECTION)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fcf-protection")
    endif()
    if(AVO_COMPILER_SUPPORTS_FNO_COMMON)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-common")
    endif()
    if(AVO_COMPILER_SUPPORTS_FSTACK_CLASH_PROTECTION)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fstack-clash-protection")
    endif()
    if(AVO_COMPILER_SUPPORTS_FSTACK_PROTECTOR_STRONG)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fstack-protector-strong")
    elseif(AVO_COMPILER_SUPPORTS_FSTACK_PROTECTOR)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fstack-protector")
    endif()

    check_cxx_linker_flag(-Wl,-Bsymbolic-functions
      AVO_LINKER_SUPPORTS_BSYMBOLIC_FUNCTIONS)
    check_cxx_linker_flag(-fsanitize=safe-stack
      AVO_LINKER_SUPPORTS_FSANITIZE_SAFE_STACK)
    check_cxx_linker_flag(-Wl,-z,defs AVO_LINKER_SUPPORTS_Z_DEFS)
    check_cxx_linker_flag(-Wl,-z,noexecheap AVO_LINKER_SUPPORTS_Z_NOEXECHEAP)
    check_cxx_linker_flag(-Wl,-z,noexecstack AVO_LINKER_SUPPORTS_Z_NOEXECSTACK)
    check_cxx_linker_flag(-Wl,-z,now AVO_LINKER_SUPPORTS_Z_NOW)
    check_cxx_linker_flag(-Wl,-z,relro AVO_LINKER_SUPPORTS_Z_RELRO)

    if(AVO_LINKER_SUPPORTS_BSYMBOLIC_FUNCTIONS)
      set(CMAKE_EXE_LINKER_FLAGS
        "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Bsymbolic-functions")
      set(CMAKE_SHARED_LINKER_FLAGS
        "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-Bsymbolic-functions")
    endif()
    if(AVO_COMPILER_SUPPORTS_FSANITIZE_SAFE_STACK
        AND AVO_LINKER_SUPPORTS_FSANITIZE_SAFE_STACK)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=safe-stack")
      set(CMAKE_EXE_LINKER_FLAGS
        "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=safe-stack")
      set(CMAKE_SHARED_LINKER_FLAGS
        "${CMAKE_SHARED_LINKER_FLAGS} -fsanitize=safe-stack")
    endif()
    if(AVO_LINKER_SUPPORTS_Z_DEFS)
      set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,defs")
      set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-z,defs")
    endif()
    if(AVO_LINKER_SUPPORTS_Z_NOEXECHEAP)
      set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,noexecheap")
      set(CMAKE_SHARED_LINKER_FLAGS
        "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-z,noexecheap")
    endif()
    if(AVO_LINKER_SUPPORTS_Z_NOEXECSTACK)
      set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,noexecstack")
      set(CMAKE_SHARED_LINKER_FLAGS
        "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-z,noexecstack")
    endif()
    if(AVO_LINKER_SUPPORTS_Z_NOW)
      set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,now")
      set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-z,now")
    endif()
    if(AVO_LINKER_SUPPORTS_Z_RELRO)
      set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,relro")
      set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-z,relro")
    endif()
  endif()
endif()

if(MSVC)
  check_cxx_compiler_flag(/W4 AVO_COMPILER_SUPPORTS_W4)

  if(AVO_COMPILER_SUPPORTS_W4)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
  endif()
else()
  check_cxx_compiler_flag(-Wall AVO_COMPILER_SUPPORTS_WALL)
  check_cxx_compiler_flag(-Wformat-security
    AVO_COMPILER_SUPPORTS_WFORMAT_SECURITY)
  check_cxx_compiler_flag(-Werror=format-security
    AVO_COMPILER_SUPPORTS_WERROR_FORMAT_SECURITY)
  check_cxx_compiler_flag(-Wno-unused
    AVO_COMPILER_SUPPORTS_WNO_UNUSED)

  if(AVO_COMPILER_SUPPORTS_WALL)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
  endif()
  if(AVO_COMPILER_SUPPORTS_WFORMAT_SECURITY)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wformat-security")
  endif()
  if(AVO_COMPILER_SUPPORTS_WERROR_FORMAT_SECURITY)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=format-security")
  endif()
  if(AVO_COMPILER_SUPPORTS_WNO_UNUSED)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused")
  endif()
endif()

if(NOT MSVC)
  check_cxx_linker_flag(-Wl,--as-needed AVO_LINKER_SUPPORTS__AS_NEEDED)

  if(AVO_LINKER_SUPPORTS__AS_NEEDED)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--as-needed")
    set(CMAKE_SHARED_LINKER_FLAGS
      "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--as-needed")
  endif()
endif()

option(WARNINGS_AS_ERRORS
  "Turn compiler warnings into errors if supported" OFF)

if(WARNINGS_AS_ERRORS)
  if(MSVC)
    check_cxx_compiler_flag(/WX AVO_COMPILER_SUPPORTS_WX)

    if(AVO_COMPILER_SUPPORTS_WX)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /WX")
    endif()
  else()
    check_cxx_compiler_flag(-Werror AVO_COMPILER_SUPPORTS_WERROR)

    if(AVO_COMPILER_SUPPORTS_WERROR)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror")
    endif()
  endif()
endif()

add_subdirectory(src)
add_subdirectory(examples)
add_subdirectory(doc)

install(FILES LICENSE
  COMPONENT runtime
  DESTINATION ${CMAKE_INSTALL_DOCDIR})

include(CMakePackageConfigHelpers)

configure_package_config_file(
  ${PROJECT_NAME}Config.cmake.in ${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${AVO_DIR}
  PATH_VARS AVO_INCLUDE_DIR AVO_LIBRARY_DIR)

write_basic_package_version_file(${PROJECT_NAME}ConfigVersion.cmake
  COMPATIBILITY SameMajorVersion)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  COMPONENT development
  DESTINATION ${AVO_DIR})

find_package(PkgConfig MODULE)

if(PkgConfig_FOUND)
  configure_file(${PROJECT_NAME}.pc.in ${PROJECT_NAME}.pc @ONLY)
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc"
    COMPONENT development
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)
endif()

include(InstallRequiredSystemLibraries)

set(CPACK_PACKAGE_NAME ${PROJECT_NAME})
set(CPACK_PACKAGE_CONTACT "Jamie Snape")
set(CPACK_PACKAGE_VENDOR "University of North Carolina at Chapel Hill")
set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "${PROJECT_DESCRIPTION}")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_STRIP_FILES ON)

set(CPACK_SOURCE_IGNORE_FILES
  _build/ \\\\.git/ \\\\.gitattributes \\\\.github/ \\\\.gitignore bazel-
  Brewfile)
set(CPACK_SOURCE_STRIP_FILES ON)

set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
set(CPACK_DEBIAN_PACKAGE_DEPENDS)
set(CPACK_DEBIAN_PACKAGE_HOMEPAGE ${PROJECT_HOMEPAGE_URL})
set(CPACK_DEBIAN_PACKAGE_SECTION contrib/devel)
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)

set(CPACK_FREEBSD_PACKAGE_DEPS)
set(CPACK_FREEBSD_PACKAGE_LICENSE APACHE20)
set(CPACK_FREEBSD_PACKAGE_WWW ${PROJECT_HOMEPAGE_URL})

set(CPACK_RPM_FILE_NAME RPM-DEFAULT)
set(CPACK_RPM_PACKAGE_AUTOREQPROV ON)
set(CPACK_RPM_PACKAGE_LICENSE "ASL 2.0")
set(CPACK_RPM_PACKAGE_REQUIRES)
set(CPACK_RPM_PACKAGE_URL ${PROJECT_HOMEPAGE_URL})

include(CPack)
