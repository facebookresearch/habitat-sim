# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

find_package(MagnumPlugins REQUIRED GltfImporter StbImageImporter StbImageConverter)

set(ESP_DEFAULT_PHYSICS_CONFIG_REL_PATH ./data/default.physics_config.json)
set(ESP_DEFAULT_PHYSICS_CONFIG
    ${PROJECT_SOURCE_DIR}/.${ESP_DEFAULT_PHYSICS_CONFIG_REL_PATH}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/configure.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/configure.h
)

add_library(
  physics STATIC
  ArticulatedObject.h
  CollisionGroupHelper.cpp
  CollisionGroupHelper.h
  objectManagers/ArticulatedObjectManager.cpp
  objectManagers/ArticulatedObjectManager.h
  objectManagers/PhysicsObjectBaseManager.h
  objectManagers/RigidBaseManager.h
  objectManagers/RigidObjectManager.cpp
  objectManagers/RigidObjectManager.h
  objectWrappers/ManagedArticulatedObject.h
  objectWrappers/ManagedPhysicsObjectBase.h
  objectWrappers/ManagedRigidBase.h
  objectWrappers/ManagedRigidObject.h
  PhysicsManager.cpp
  PhysicsManager.h
  PhysicsObjectBase.h
  RigidBase.h
  RigidObject.cpp
  RigidObject.h
  RigidStage.cpp
  RigidStage.h
  URDFImporter.cpp
  URDFImporter.h
)

if(BUILD_WITH_BULLET)
  add_subdirectory(bullet)

  target_link_libraries(physics PUBLIC bulletphysics)
endif()

target_link_libraries(
  physics
  PUBLIC core
         scene
         assets
         MagnumPlugins::GltfImporter
         MagnumPlugins::StbImageImporter
         MagnumPlugins::StbImageConverter
)

set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)
