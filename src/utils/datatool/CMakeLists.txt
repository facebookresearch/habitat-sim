set(Datatool_SOURCES Datatool.cpp SceneLoader.cpp Mp3dInstanceMeshData.cpp)

add_executable(Datatool ${Datatool_SOURCES})
set(DEPS_DIR "${CMAKE_CURRENT_LIST_DIR}/../../deps")
target_include_directories(Datatool SYSTEM PRIVATE "${DEPS_DIR}/tinyobjloader")

target_link_libraries(
  Datatool
  PRIVATE assets assimp nav
)
