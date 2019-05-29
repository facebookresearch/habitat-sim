file(GLOB_RECURSE assets_SOURCES "*.cpp")

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
    TextureTools
    Trade
)

find_package(MagnumPlugins
  REQUIRED
# AssimpImporter
    StbImageImporter
    StbImageConverter
    TinyGltfImporter
)

add_library(assets STATIC ${assets_SOURCES})

target_link_libraries(assets
  PUBLIC
    core
    scene
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
    Assimp::Assimp
    geo
    io
)

if(OpenMP_CXX_FOUND)
  target_link_libraries(assets PUBLIC OpenMP::OpenMP_CXX)
endif()
