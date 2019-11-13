set(assets_SOURCES
  Asset.cpp
  Asset.h
  Attributes.cpp
  Attributes.h
  BaseMesh.cpp
  BaseMesh.h
  CollisionMeshData.h
  GenericInstanceMeshData.cpp
  GenericInstanceMeshData.h
  GltfMeshData.cpp
  GltfMeshData.h
  MeshData.h
  MeshMetaData.h
  Mp3dInstanceMeshData.cpp
  Mp3dInstanceMeshData.h
  ResourceManager.cpp
  ResourceManager.h
)

if(BUILD_PTEX_SUPPORT)
  list(APPEND assets_SOURCES
    PTexMeshData.cpp
    PTexMeshData.h
  )
endif()

find_package(Magnum
  REQUIRED
    AnyImageImporter
    AnySceneImporter
    GL
    MeshTools
    SceneGraph
    Shaders
    Trade
    Primitives
)

find_package(MagnumPlugins
  REQUIRED
    BasisImporter
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
    Magnum::Primitives
    MagnumPlugins::BasisImporter
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
