find_package(Magnum REQUIRED Primitives)

if(NOT CMAKE_BUILD_TYPE MATCHES "ASAN")
corrade_add_test(gfxDepthUnprojectionTest DepthUnprojectionTest.cpp LIBRARIES
  gfx
  Magnum::MeshTools
  Magnum::OpenGLTester
  Magnum::Trade
  Magnum::Primitives)
endif()
