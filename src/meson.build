# -*- mode: meson; -*-
# vi: set ft=meson:

# src/meson.build
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

sources = files(
    'Agent.cc',
    'KdTree.cc',
    'Line.cc',
    'Simulator.cc',
    'Vector2.cc',
)

export_h = configure_file(
    input: 'Export.h.in',
    output: 'Export.h',
    configuration: {'PREFIX': 'AVO'},
)

public_headers = files(
    'AVO.h',
    'Line.h',
    'Simulator.h',
    'Vector2.h',
)

inc = include_directories('.')

compile_args = []
dep_compile_args = []
default_library = get_option('default_library')

# Keep inline-visibility tightening on the library target only: this is part of
# the exported ABI/symbol policy for AVO itself, not a project-wide policy that
# needs to affect example executables.
compile_args += cpp_compiler.get_supported_arguments(
    '-fvisibility-inlines-hidden',
)

if host_os == 'windows'
    compile_args += ['-DNOMINMAX']
    dep_compile_args += ['-DNOMINMAX']

    if default_library in ['shared', 'both']
        compile_args += ['-DAVO_EXPORTS']
        if default_library == 'both'
            dep_compile_args += ['-DAVO_STATIC_DEFINE']
        endif
    else
        compile_args += ['-DAVO_STATIC_DEFINE']
        dep_compile_args += ['-DAVO_STATIC_DEFINE']
    endif
else
    if default_library == 'static'
        compile_args += ['-DAVO_STATIC_DEFINE']
        dep_compile_args += ['-DAVO_STATIC_DEFINE']
    endif
endif

lib_avo = library(
    'AVO',
    sources: sources,
    include_directories: inc,
    cpp_args: compile_args,
    gnu_symbol_visibility: 'hidden',
    dependencies: openmp_deps,
    version: meson.project_version(),
    soversion: meson.project_version().split('.')[0],
    install: not meson.is_subproject(),
)

dep_avo = declare_dependency(
    link_with: lib_avo,
    include_directories: inc,
    compile_args: dep_compile_args,
    dependencies: openmp_deps,
)

meson.override_dependency('AVO', dep_avo)

if not meson.is_subproject()
    install_headers(
        public_headers,
        export_h,
        subdir: 'AVO',
    )

    pkgconfig = import('pkgconfig')
    pkgconfig.generate(
        lib_avo,
        name: 'AVO',
        description: 'Reciprocal Collision Avoidance with Acceleration-Velocity Obstacles',
        url: 'https://gamma.cs.unc.edu/AVO/',
        version: meson.project_version(),
        filebase: 'AVO',
        subdirs: 'AVO',
        extra_cflags: dep_compile_args,
    )
endif
