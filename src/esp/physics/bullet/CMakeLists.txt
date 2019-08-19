find_package(MagnumIntegration REQUIRED Bullet)

add_library(bulletphysics STATIC
  BulletPhysicsManager.cpp
  BulletPhysicsManager.h
  BulletRigidObject.cpp
  BulletRigidObject.h
)

target_link_libraries(bulletphysics
  PUBLIC
    assets
    MagnumIntegration::Bullet
)

## Enable physics profiling
#add_compile_definitions(BT_ENABLE_PROFILE=0)
#add_definitions(-DBT_ENABLE_PROFILE)

set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)
