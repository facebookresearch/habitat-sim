set(datatool_SOURCES datatool.cpp SceneLoader.cpp)

add_executable(datatool ${datatool_SOURCES})
set(DEPS_DIR "${CMAKE_CURRENT_LIST_DIR}/../../deps")
target_include_directories(datatool SYSTEM PRIVATE "${DEPS_DIR}/tinyobjloader")

target_link_libraries(
  datatool
  PRIVATE assets assimp nav
)
