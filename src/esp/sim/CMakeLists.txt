# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

add_library(
  sim STATIC
  ReplayBatchRenderer.cpp
  ReplayBatchRenderer.h
  Simulator.cpp
  Simulator.h
  SimulatorConfiguration.cpp
  SimulatorConfiguration.h
)

target_link_libraries(
  sim
  # TODO wait where's gfx and everything else??
  PUBLIC nav
  # TODO actually don't link, i don't to wait for 24 SLOW relinks
  # gfx_batch
)
