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

target_link_libraries(
  geo
  PUBLIC core glog io gfx
)

# v-hacd (temporary; get master version of this later (once VHACD_Integration has been pushed))
target_link_libraries(
  geo
  PUBLIC vhacd
)
