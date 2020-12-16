find_package(MagnumIntegration REQUIRED Bullet)
find_package(Bullet REQUIRED Dynamics)

add_library(
  bulletphysics STATIC
  BulletArticulatedObject.cpp
  BulletArticulatedObject.h
  BulletBase.cpp
  BulletBase.h
  BulletDebugManager.cpp
  BulletDebugManager.h
  BulletPhysicsManager.cpp
  BulletPhysicsManager.h
  BulletRigidObject.cpp
  BulletRigidObject.h
  BulletRigidStage.cpp
  BulletRigidStage.h
  BulletURDFImporter.cpp
  BulletURDFImporter.h
)

target_link_libraries(
  bulletphysics
  PUBLIC assets MagnumIntegration::Bullet Bullet::Dynamics
)

## Enable physics profiling
#add_compile_definitions(BT_ENABLE_PROFILE=0)
#add_definitions(-DBT_ENABLE_PROFILE)

set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)
