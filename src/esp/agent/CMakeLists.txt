# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

add_library(
  agent STATIC
  Agent.cpp Agent.h
)

target_link_libraries(
  agent
  PUBLIC core gfx sensor
)
