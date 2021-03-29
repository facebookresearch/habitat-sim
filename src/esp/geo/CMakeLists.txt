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
  VoxelWrapper.cpp
  VoxelWrapper.h
  VoxelUtils.cpp
  VoxelUtils.h
)

target_link_libraries(
  geo
  PUBLIC core glog io gfx
)
