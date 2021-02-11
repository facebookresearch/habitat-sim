add_library(
  geo STATIC
  CoordinateFrame.cpp
  CoordinateFrame.h
  geo.cpp
  geo.h
  OBB.cpp
  OBB.h
  VoxelGrid.h
)

target_link_libraries(
  geo
  PUBLIC core glog io gfx
)
