find_package(Magnum REQUIRED GL Shaders Trade)

set(gfx_batch_SOURCES Renderer.cpp Renderer.h RendererStandalone.cpp
                      RendererStandalone.h
)

add_library(
  gfx_batch STATIC
  ${gfx_batch_SOURCES}
)
# For esp/core/configure.h
# TODO move configure.h directly to esp/
target_include_directories(gfx_batch PUBLIC ${PROJECT_BINARY_DIR})
if(BUILD_WITH_CUDA)
  target_include_directories(
    gfx_batch PRIVATE ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
                      ${CMAKE_CURRENT_LIST_DIR}/cuda_helpers
  )

  target_link_libraries(gfx_batch PUBLIC ${CUDART_LIBRARY})
endif()

target_link_libraries(
  gfx_batch
  # Important: this library *deliberately* doesn't depend on any other Habitat
  # libraries, and it should be kept as such. Only Magnum libraries are an
  # allowed dependency. See the original PR for discussion:
  # https://github.com/facebookresearch/habitat-sim/pull/1798#discussion_r911398937
  PUBLIC Magnum::GL Magnum::Magnum Magnum::Shaders Magnum::Trade
  # Not linking any plugins here, as the renderer itself doesn't directly rely
  # on any of them. That also makes the plugins implicitly registered in
  # *every* target that links to gfx_batch, which is far from ideal. Only the
  # leaf executable links to them, and only to those that are actually used
  # in that scenario -- i.e., a test only needs a tiny subset of plugins for
  # data it uses, not everything to handle general file formats.
)

# Link appropriate windowless library for the standalone renderer
if(MAGNUM_TARGET_EGL)
  # Includes also Emscripten
  find_package(Magnum REQUIRED WindowlessEglApplication)
elseif(CORRADE_TARGET_APPLE)
  find_package(Magnum REQUIRED WindowlessCglApplication)
elseif(CORRADE_TARGET_UNIX)
  # Mainly for builds with external Magnum that might not have TARGET_EGL
  # enabled
  find_package(Magnum REQUIRED WindowlessGlxApplication)
elseif(CORRADE_TARGET_WINDOWS)
  find_package(Magnum REQUIRED WindowlessWglApplication)
else()
  message(FATAL_ERROR "Unsupported platform")
endif()
target_link_libraries(
  gfx_batch
  PUBLIC Magnum::WindowlessApplication
)
