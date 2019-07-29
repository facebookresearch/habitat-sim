file(GLOB_RECURSE assets_SOURCES "*.cpp")

find_package(Magnum
  REQUIRED
    AnyImageImporter
    AnySceneImporter
    GL
    MeshTools
    SceneGraph
    Shaders
    DebugTools
    Trade
)

find_package(MagnumPlugins
  REQUIRED
    AssimpImporter
    StbImageImporter
    StbImageConverter
    TinyGltfImporter
)



add_library(assets
    STATIC
        ${assets_SOURCES}
)

target_link_libraries(assets
  PUBLIC
    core
    physics
    scene
    Magnum::AnyImageImporter
    Magnum::AnySceneImporter
    Magnum::GL
    Magnum::Magnum
    Magnum::MeshTools
    Magnum::SceneGraph
    Magnum::Shaders
    Magnum::Trade
    Magnum::DebugTools
    MagnumPlugins::AssimpImporter
    MagnumPlugins::StbImageImporter
    MagnumPlugins::StbImageConverter
    MagnumPlugins::TinyGltfImporter
    tinyply
  PRIVATE
    Assimp::Assimp
    geo
    io
)

set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)

if(OpenMP_CXX_FOUND)
  target_link_libraries(assets PUBLIC OpenMP::OpenMP_CXX)
endif()
