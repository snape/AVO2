# -*- mode: python -*-
# vi: set ft=python :

#
# src/BUILD
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

load("@bazel_tools//tools/build_defs/pkg:pkg.bzl", "pkg_tar")

licenses(["notice"])

genrule(
    name = "export",
    outs = ["Export.h"],
    cmd = """
cat << 'EOF' > $@
/*
 * Export.h
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

#ifndef AVO_EXPORT_H_
#define AVO_EXPORT_H_

#ifdef AVO_STATIC_DEFINE
#define AVO_EXPORT
#else
#ifdef _WIN32
#ifdef AVO_EXPORTS
#define AVO_EXPORT __declspec(dllexport)
#else
#define AVO_EXPORT __declspec(dllimport)
#endif
#else
#define AVO_EXPORT __attribute__((visibility("default")))
#endif
#endif

#endif  // AVO_EXPORT_H_
EOF
""",
    visibility = ["//visibility:private"],
)

filegroup(
    name = "hdrs",
    srcs = [
        "AVO.h",
        "Export.h",
        "Line.h",
        "Simulator.h",
        "Vector2.h",
    ],
    visibility = ["//visibility:private"],
)

cc_library(
    name = "AVO",
    srcs = [
        "Agent.cc",
        "Agent.h",
        "KdTree.cc",
        "KdTree.h",
        "Line.cc",
        "Simulator.cc",
        "Vector2.cc",
    ],
    hdrs = [":hdrs"],
    copts = [
        "-fvisibility-inlines-hidden",
        "-fvisibility=hidden",
    ],
    includes = ["."],
    visibility = ["//visibility:public"],
)

pkg_tar(
    name = "include",
    srcs = [":hdrs"],
    mode = "0644",
    package_dir = "/usr/include/AVO",
    visibility = ["//:__pkg__"],
)

pkg_tar(
    name = "lib",
    srcs = ["//src:AVO"],
    mode = "0644",
    package_dir = "/usr/lib",
    visibility = ["//:__pkg__"],
)
