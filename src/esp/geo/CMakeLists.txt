# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

add_library(
  geo STATIC
  CoordinateFrame.cpp
  CoordinateFrame.h
  Geo.cpp
  Geo.h
  OBB.cpp
  OBB.h
)

target_link_libraries(
  geo
  PUBLIC core gfx
)
