# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

add_library(
  sim STATIC
  AbstractReplayRenderer.cpp
  AbstractReplayRenderer.h
  BatchPlayerImplementation.cpp
  BatchPlayerImplementation.h
  BatchReplayRenderer.cpp
  BatchReplayRenderer.h
  ClassicReplayRenderer.cpp
  ClassicReplayRenderer.h
  Simulator.cpp
  Simulator.h
  SimulatorConfiguration.cpp
  SimulatorConfiguration.h
)

target_link_libraries(
  sim
  PUBLIC nav gfx_batch
)
