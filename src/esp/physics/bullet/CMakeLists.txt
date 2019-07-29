file(GLOB_RECURSE bulletphysics_SOURCES "*.cpp")

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

find_package(MagnumIntegration REQUIRED Bullet)

add_library(bulletphysics STATIC ${bulletphysics_SOURCES})

target_link_libraries(bulletphysics
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
    MagnumIntegration::Bullet
    tinyply
  PRIVATE
    assimp
    geo
    io
)


## Enable physics profiling
#add_compile_definitions(BT_ENABLE_PROFILE=0)
#add_definitions(-DBT_ENABLE_PROFILE)

set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)

if(OpenMP_CXX_FOUND)
  target_link_libraries(assets PUBLIC OpenMP::OpenMP_CXX)
endif()
