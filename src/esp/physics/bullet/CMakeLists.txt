# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

find_package(MagnumIntegration REQUIRED Bullet)
find_package(Bullet REQUIRED Dynamics)

add_library(
  bulletphysics STATIC
  BulletArticulatedObject.cpp
  BulletArticulatedObject.h
  BulletBase.cpp
  BulletBase.h
  BulletCollisionHelper.cpp
  BulletCollisionHelper.h
  BulletPhysicsManager.cpp
  BulletPhysicsManager.h
  BulletRigidObject.cpp
  BulletRigidObject.h
  BulletRigidStage.cpp
  BulletRigidStage.h
  BulletURDFImporter.cpp
  BulletURDFImporter.h
  objectWrappers/ManagedBulletArticulatedObject.h
  objectWrappers/ManagedBulletRigidObject.h
)

target_link_libraries(
  bulletphysics
  PUBLIC assets MagnumIntegration::Bullet Bullet::Dynamics
)

## Enable physics profiling
#add_compile_definitions(BT_ENABLE_PROFILE=0)
#add_definitions(-DBT_ENABLE_PROFILE)

set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)
