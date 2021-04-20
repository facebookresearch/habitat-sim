add_library(
  geo STATIC
  CoordinateFrame.cpp
  CoordinateFrame.h
  geo.cpp
  geo.h
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
  PUBLIC core glog io gfx
)
