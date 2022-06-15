set(
  assets_SOURCES
  Asset.cpp
  Asset.h
  BaseMesh.cpp
  BaseMesh.h
  CollisionMeshData.h
  GenericSemanticMeshData.cpp
  GenericSemanticMeshData.h
  GenericMeshData.cpp
  GenericMeshData.h
  MeshData.h
  MeshMetaData.h
  RenderAssetInstanceCreationInfo.cpp
  RenderAssetInstanceCreationInfo.h
  ResourceManager.cpp
  ResourceManager.h
)

if(BUILD_PTEX_SUPPORT)
  list(APPEND assets_SOURCES PTexMeshData.cpp PTexMeshData.h)
endif()

find_package(
  Magnum
  REQUIRED
  AnyImageImporter
  AnySceneImporter
  GL
  MeshTools
  SceneGraph
  SceneTools
  Shaders
  Trade
  Primitives
)

find_package(
  MagnumPlugins
  REQUIRED
  BasisImporter
  GltfImporter
  PrimitiveImporter
  StanfordImporter
  StbImageImporter
  StbImageConverter
)

if(BUILD_ASSIMP_SUPPORT)
  find_package(MagnumPlugins REQUIRED AssimpImporter)
endif()

add_library(
  assets STATIC
  ${assets_SOURCES}
)

target_link_libraries(
  assets
  PUBLIC core
         metadata
         physics
         scene
         nav
         Magnum::AnyImageImporter
         Magnum::AnySceneImporter
         Magnum::GL
         Magnum::Magnum
         Magnum::MeshTools
         Magnum::SceneGraph
         Magnum::SceneTools
         Magnum::Shaders
         Magnum::Trade
         Magnum::Primitives
         MagnumPlugins::BasisImporter
         MagnumPlugins::GltfImporter
         MagnumPlugins::PrimitiveImporter
         MagnumPlugins::StanfordImporter
         MagnumPlugins::StbImageImporter
         MagnumPlugins::StbImageConverter
  PRIVATE geo io
)

if(BUILD_WITH_VHACD)
  target_link_libraries(assets PUBLIC vhacd)
endif()

if(BUILD_ASSIMP_SUPPORT)
  target_link_libraries(
    assets PUBLIC MagnumPlugins::AssimpImporter PRIVATE Assimp::Assimp
  )
endif()

if(OpenMP_CXX_FOUND)
  target_link_libraries(assets PUBLIC OpenMP::OpenMP_CXX)
endif()
