add_library(
  geo STATIC
  CoordinateFrame.cpp
  CoordinateFrame.h
  Geo.cpp
  Geo.h
  OBB.cpp
  OBB.h
  VoxelGrid.cpp
  VoxelGrid.h
  VoxelUtils.cpp
  VoxelUtils.h
  VoxelWrapper.cpp
  VoxelWrapper.h
)

target_link_libraries(
  geo
  PUBLIC core gfx
)
