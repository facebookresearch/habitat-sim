find_package(Magnum REQUIRED Primitives)

corrade_add_test(
  gfxDepthUnprojectionTest
  DepthUnprojectionTest.cpp
  LIBRARIES
  gfx
  Magnum::MeshTools
  Magnum::OpenGLTester
  Magnum::Trade
  Magnum::Primitives
)
