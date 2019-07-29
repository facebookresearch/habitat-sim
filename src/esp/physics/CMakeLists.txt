file(GLOB physics_SOURCES "*.cpp")

find_package(Magnum
  REQUIRED
    AnyImageImporter
    AnySceneImporter
    DebugTools
    GL
    MeshTools
    Primitives
    SceneGraph
    Shaders
    Trade
)

find_package(MagnumPlugins
  REQUIRED
# AssimpImporter
    StbImageImporter
    StbImageConverter
    TinyGltfImporter
)

add_library(physics STATIC ${physics_SOURCES})

if(WITH_BULLET)
  target_compile_definitions(physics PUBLIC PHYSICS_WITH_BULLET)
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
    Magnum::AnyImageImporter
    Magnum::AnySceneImporter
    Magnum::GL
    Magnum::Magnum
    Magnum::MeshTools
    Magnum::SceneGraph
    Magnum::Shaders
    Magnum::Trade
# MagnumPlugins::AssimpImporter
    MagnumPlugins::StbImageImporter
    MagnumPlugins::StbImageConverter
    MagnumPlugins::TinyGltfImporter
    tinyply
  PRIVATE
    assimp
    geo
    io
)




set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)

if(OpenMP_CXX_FOUND)
  target_link_libraries(assets PUBLIC OpenMP::OpenMP_CXX)
endif()
