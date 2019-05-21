set(datatool_SOURCES datatool.cpp SceneLoader.cpp)

add_executable(datatool ${datatool_SOURCES})

target_link_libraries(datatool
  PRIVATE
    assets
    nav
)
