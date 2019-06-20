set(gfx_SOURCES
  Drawable.cpp
  Drawable.h
  GenericDrawable.cpp
  GenericDrawable.h
  magnum.h
  PrimitiveIDTexturedDrawable.cpp
  PrimitiveIDTexturedDrawable.h
  PrimitiveIDTexturedShader.cpp
  PrimitiveIDTexturedShader.h
  RenderCamera.cpp
  RenderCamera.h
  Renderer.cpp
  Renderer.h
  Simulator.cpp
  Simulator.h
  WindowlessContext.cpp
  WindowlessContext.h
)

# If GUI build, add Viewer source
# TODO(MS) Viewer really ought to be in separate module that is disabled
if(BUILD_GUI_VIEWERS)
  list(APPEND gfx_SOURCES
    Viewer.cpp
    Viewer.h
  )
endif()

# If ptex support is enabled add relevant source files
if(BUILD_PTEX_SUPPORT)
  list(APPEND gfx_SOURCES
    PTexMeshDrawable.cpp
    PTexMeshDrawable.h
    PTexMeshShader.cpp
    PTexMeshShader.h
  )
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

if(MSVC)
  # Define common math constants if we compile with MSVC.  This is needed because of Sophus.
  target_compile_definitions(gfx PUBLIC _USE_MATH_DEFINES)
endif (MSVC)

target_link_libraries(gfx
  PUBLIC
    assets
    core
    io
    physics
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
    find_package(Magnum REQUIRED EmscriptenApplication)
    target_link_libraries(gfx PUBLIC Magnum::EmscriptenApplication)
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
