# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

add_library(
  nav STATIC
  GreedyFollower.cpp GreedyFollower.h PathFinder.cpp PathFinder.h
)

target_include_directories(
  nav PRIVATE "${DEPS_DIR}/recastnavigation/Detour/Include"
              "${DEPS_DIR}/recastnavigation/Recast/Include"
)

target_link_libraries(
  nav
  PUBLIC core agent scene
  PRIVATE Detour Recast
)
