file(GLOB_RECURSE assets_SOURCES "*.cpp")

if(NOT BUILD_PTEX_SUPPORT)
  list(REMOVE_ITEM assets_SOURCES ${CMAKE_CURRENT_LIST_DIR}/PTexMeshData.cpp)
endif()

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
    StbImageImporter
    StbImageConverter
    TinyGltfImporter
)

if(BUILD_ASSIMP_SUPPORT)
  find_package(MagnumPlugins
    REQUIRED
      AssimpImporter
  )
endif()

add_library(assets STATIC ${assets_SOURCES})

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
    MagnumPlugins::StbImageImporter
    MagnumPlugins::StbImageConverter
    MagnumPlugins::TinyGltfImporter
    tinyply
  PRIVATE
    geo
    io
)

if(BUILD_ASSIMP_SUPPORT)
  target_link_libraries(assets
    PUBLIC
      MagnumPlugins::AssimpImporter
    PRIVATE
      Assimp::Assimp
  )
endif()

if(OpenMP_CXX_FOUND)
  target_link_libraries(assets PUBLIC OpenMP::OpenMP_CXX)
endif()
