set(progmesh_SOURCES progmesh.cpp)

add_executable(progmesh ${progmesh_SOURCES})

target_link_libraries(progmesh
                      PUBLIC
                      tinyply)
