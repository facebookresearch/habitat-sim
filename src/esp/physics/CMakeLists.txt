find_package(MagnumPlugins
  REQUIRED
    StbImageImporter
    StbImageConverter
    TinyGltfImporter
)

add_library(physics STATIC
  PhysicsManager.cpp
  PhysicsManager.h
  RigidBase.h
  RigidObject.cpp
  RigidObject.h
  RigidScene.cpp
  RigidScene.h
  URDFImporter.h
  URDFImporter.cpp
  ArticulatedObject.h
)

if(BUILD_WITH_BULLET)
  add_subdirectory(bullet)

  target_link_libraries(physics
    PUBLIC
      bulletphysics
  )
endif()

target_link_libraries(physics
  PUBLIC
    core
    scene
    assets
    io
    MagnumPlugins::StbImageImporter
    MagnumPlugins::StbImageConverter
    MagnumPlugins::TinyGltfImporter
)

set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)
