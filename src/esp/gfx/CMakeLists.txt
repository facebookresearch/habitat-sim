set(gfx_SOURCES
  Drawable.cpp
  GenericDrawable.cpp
  GenericShader.cpp
  RenderCamera.cpp
  Renderer.cpp
)

# If not WebGL build, also include offscreen render-reliant code
# TODO: this should be built as well (with emscripten windowless contexts)
if(NOT BUILD_WEBGL)
  list(APPEND gfx_SOURCES Simulator.cpp WindowlessContext.cpp)
endif()

# If ptex support enabled add relevant source files
if(BUILD_PTEX_SUPPORT)
  list(APPEND gfx_SOURCES PTexMeshDrawable.cpp PTexMeshShader.cpp)
endif()

# If GUI build, add Viewer.cpp source
# TODO(MS) Viewer really ought to be in separate module that is disabled
if(BUILD_GUI_VIEWERS)
  message("Building GUI viewer")
  list(APPEND gfx_SOURCES Viewer.cpp)
endif()

if ((UNIX AND NOT APPLE) AND (NOT BUILD_GUI_VIEWERS))
  list(APPEND gfx_SOURCES "${DEPS_DIR}/glad/src/glad_egl.c")
endif()

find_package(Magnum
  REQUIRED
    AnyImageImporter
    AnySceneImporter
    DebugTools
    GL
    MeshTools
    Primitives
    SceneGraph
    Shaders
    TextureTools
    Trade
)

find_package(MagnumPlugins
  REQUIRED
    StbImageImporter
    StbImageConverter
    TinyGltfImporter
)

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
)

if(BUILD_WEBGL)
  if(BUILD_GUI_VIEWERS)
    find_package(Magnum REQUIRED Sdl2Application)
    target_link_libraries(gfx PUBLIC Magnum::Sdl2Application)
  endif()
else()  # non-WebGL build
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
endif()
