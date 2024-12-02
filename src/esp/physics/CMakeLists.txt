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

set(
  physics_SOURCES
  ArticulatedLink.h
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

  find_package(MagnumIntegration REQUIRED Bullet)
  find_package(Bullet REQUIRED Dynamics)
  # Add bullet sources

  list(
    APPEND
    physics_SOURCES
    bullet/BulletArticulatedLink.h
    bullet/BulletArticulatedObject.cpp
    bullet/BulletArticulatedObject.h
    bullet/BulletBase.cpp
    bullet/BulletBase.h
    bullet/BulletCollisionHelper.cpp
    bullet/BulletCollisionHelper.h
    bullet/BulletPhysicsManager.cpp
    bullet/BulletPhysicsManager.h
    bullet/BulletRigidObject.cpp
    bullet/BulletRigidObject.h
    bullet/BulletRigidStage.cpp
    bullet/BulletRigidStage.h
    bullet/BulletURDFImporter.cpp
    bullet/BulletURDFImporter.h
    bullet/objectWrappers/ManagedBulletArticulatedObject.h
    bullet/objectWrappers/ManagedBulletRigidObject.h
  )

  ## Enable physics profiling
  #add_compile_definitions(BT_ENABLE_PROFILE=0)
  #add_definitions(-DBT_ENABLE_PROFILE)

endif() #BUILD_WITH_BULLET

add_library(
  physics STATIC
  ${physics_SOURCES}
)

target_link_libraries(
  physics
  PUBLIC core
         scene
         sensor
         assets
         MagnumPlugins::GltfImporter
         MagnumPlugins::StbImageImporter
         MagnumPlugins::StbImageConverter
)

if(BUILD_WITH_BULLET)
  target_link_libraries(physics PUBLIC MagnumIntegration::Bullet Bullet::Dynamics)
endif() #BUILD_WITH_BULLET

set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)
