# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

find_package(MagnumBindings REQUIRED Python)

find_package(
  MagnumPlugins
  REQUIRED
  BasisImporter
  GltfImporter
  KtxImporter
  PrimitiveImporter
  StanfordImporter
  StbImageImporter
  StbImageConverter
  UfbxImporter
)
if(BUILD_ASSIMP_SUPPORT)
  find_package(MagnumPlugins REQUIRED AssimpImporter)
endif()

pybind11_add_module(
  habitat_sim_bindings
  Bindings.h
  Bindings.cpp
  AttributesBindings.cpp
  AttributesManagersBindings.cpp
  ConfigBindings.cpp
  CoreBindings.cpp
  GeoBindings.cpp
  GfxBindings.cpp
  MetadataMediatorBindings.cpp
  GfxReplayBindings.cpp
  OpaqueTypes.h
  PhysicsBindings.cpp
  PhysicsObjectBindings.cpp
  PhysicsWrapperManagerBindings.cpp
  SceneBindings.cpp
  SensorBindings.cpp
  ShortestPathBindings.cpp
  SimBindings.cpp
)
target_link_libraries(
  habitat_sim_bindings
  PRIVATE MagnumBindings::Python
          MagnumPlugins::BasisImporter
          MagnumPlugins::GltfImporter
          MagnumPlugins::KtxImporter
          MagnumPlugins::StanfordImporter
          MagnumPlugins::StbImageImporter
          MagnumPlugins::StbImageConverter
          MagnumPlugins::UfbxImporter
  PUBLIC habitat_sim
)
if(BUILD_ASSIMP_SUPPORT)
  target_link_libraries(habitat_sim_bindings PUBLIC MagnumPlugins::AssimpImporter)
endif()

if(BUILD_WITH_AUDIO)
  add_custom_command(
    TARGET habitat_sim_bindings POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${RLRAudioPropagation_LIBRARY}
            $<TARGET_FILE_DIR:habitat_sim_bindings>
  )
endif()

# target_include_directories(environment
#   PUBLIC
#     ${CMAKE_CURRENT_LIST_DIR}
# )

# file(WRITE ${PYTHON_ROOT}/libpath.py "LIB_PATH='${CMAKE_CURRENT_BINARY_DIR}'\n")
