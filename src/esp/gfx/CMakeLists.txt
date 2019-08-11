file(GLOB_RECURSE gfx_SOURCES "*.cpp")

# If not GUI build, remove Viewer source
# TODO(MS) Viewer really ought to be in separate module that is disabled
if(NOT BUILD_GUI_VIEWERS)
  list(REMOVE_ITEM gfx_SOURCES "${CMAKE_CURRENT_LIST_DIR}/Viewer.cpp")
endif()

# If ptex support is disabled remove relevant source files
if(NOT BUILD_PTEX_SUPPORT)
  list(REMOVE_ITEM gfx_SOURCES "${CMAKE_CURRENT_LIST_DIR}/PTexMeshDrawable.cpp" "${CMAKE_CURRENT_LIST_DIR}/PTexMeshShader.cpp")
endif()

if ((UNIX AND NOT APPLE) AND (NOT BUILD_GUI_VIEWERS))
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
    StbImageImporter
    StbImageConverter
    TinyGltfImporter
)

find_package(MagnumIntegration REQUIRED Eigen)

find_package(Corrade REQUIRED Utility)
# TODO: enable the following flag and fix the compilation warnings
# set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)
corrade_add_resource(ShaderResources ../../shaders/Shaders.conf)
list(APPEND gfx_SOURCES ${ShaderResources})

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
    MagnumPlugins::StbImageImporter
    MagnumPlugins::StbImageConverter
    MagnumPlugins::TinyGltfImporter
    MagnumIntegration::Eigen
    Corrade::Utility
)

# Link windowed application library if needed
if(BUILD_GUI_VIEWERS)
  if(CORRADE_TARGET_EMSCRIPTEN)
    find_package(Magnum REQUIRED Sdl2Application)
    target_link_libraries(gfx PUBLIC Magnum::Sdl2Application)
  else()
    find_package(Magnum REQUIRED GlfwApplication)
    target_link_libraries(gfx PUBLIC Magnum::GlfwApplication)
  endif()
endif()

# Link appropriate windowless library
if(APPLE)
  find_package(Magnum REQUIRED WindowlessCglApplication)
  target_link_libraries(gfx PUBLIC Magnum::WindowlessCglApplication)
elseif(WIN32)
  find_package(Magnum REQUIRED WindowlessWglApplication)
  target_link_libraries(gfx PUBLIC Magnum::WindowlessWglApplication)
elseif(CORRADE_TARGET_EMSCRIPTEN)
  find_package(Magnum REQUIRED WindowlessEglApplication)
  target_link_libraries(gfx PUBLIC Magnum::WindowlessEglApplication)
elseif(UNIX)
  if(BUILD_GUI_VIEWERS)
    find_package(Magnum REQUIRED WindowlessGlxApplication)
    target_link_libraries(gfx PUBLIC Magnum::WindowlessGlxApplication)
  else()
    find_package(Magnum REQUIRED WindowlessEglApplication)
    target_link_libraries(gfx PUBLIC Magnum::WindowlessEglApplication)
  endif()
endif()
