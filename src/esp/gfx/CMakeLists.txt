file(GLOB_RECURSE gfx_SOURCES "*.cpp")

# If not GUI build, remove Viewer source
# TODO(MS) Viewer really ought to be in separate module that is disabled
if(BUILD_GUI_VIEWERS)
  message(STATUS "Building GUI viewers")
else()
  list(REMOVE_ITEM gfx_SOURCES "${CMAKE_CURRENT_LIST_DIR}/Viewer.cpp")
endif()

if ((UNIX AND NOT APPLE) AND (NOT ${BUILD_GUI_VIEWERS}))
  list(APPEND gfx_SOURCES "${DEPS_DIR}/glad/src/glad_egl.c")
endif()

find_package(Magnum
  REQUIRED
    AnyImageImporter
    AnySceneImporter
    GL
    MeshTools
    SceneGraph
    Shaders
    Trade
)

find_package(MagnumPlugins
  REQUIRED
    # AssimpImporter
    StbImageImporter
    StbImageConverter
    TinyGltfImporter
)

find_package(MagnumIntegration REQUIRED Eigen)

add_library(gfx STATIC ${gfx_SOURCES})

target_include_directories(gfx
  PRIVATE
    "${DEPS_DIR}/glad/include"
)

target_link_libraries(gfx
  PUBLIC
    assets
    core
    io
    Magnum::AnyImageImporter
    Magnum::AnySceneImporter
    Magnum::GL
    Magnum::Magnum
    Magnum::MeshTools
    Magnum::SceneGraph
    Magnum::Shaders
    Magnum::Trade
    # MagnumPlugins::AssimpImporter
    MagnumPlugins::StbImageImporter
    MagnumPlugins::StbImageConverter
    MagnumPlugins::TinyGltfImporter
    MagnumIntegration::Eigen
)

# Link windowed application library if needed
if(BUILD_GUI_VIEWERS)
  find_package(Magnum REQUIRED GlfwApplication)
  target_link_libraries(gfx PUBLIC Magnum::GlfwApplication)
elseif(UNIX)
  target_compile_definitions(gfx PRIVATE __ESP_USE_EGL__)
endif()

# Link appropriate windowless library
if(APPLE)
  find_package(Magnum REQUIRED WindowlessCglApplication)
  target_link_libraries(gfx PUBLIC Magnum::WindowlessCglApplication)
elseif(WIN32)
  find_package(Magnum REQUIRED WindowlessWglApplication)
  target_link_libraries(gfx PUBLIC Magnum::WindowlessWglApplication)
elseif(UNIX)
  if(BUILD_GUI_VIEWERS)
    find_package(Magnum REQUIRED WindowlessGlxApplication)
    target_link_libraries(gfx PUBLIC Magnum::WindowlessGlxApplication)
  else()
    find_package(Magnum REQUIRED WindowlessEglApplication)
    target_link_libraries(gfx PUBLIC Magnum::WindowlessEglApplication)
  endif()
endif()
