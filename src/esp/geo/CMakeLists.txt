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
)

target_include_directories(geo PRIVATE "${DEPS_DIR}/tinyxml2")

target_link_libraries(
  geo
  PUBLIC core glog io gfx
  PRIVATE tinyxml2
)
