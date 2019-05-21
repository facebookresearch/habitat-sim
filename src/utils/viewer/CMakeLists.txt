set(viewer_SOURCES viewer.cpp)

set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)
add_executable(viewer ${viewer_SOURCES})

find_package(GLFW REQUIRED)

find_package(Magnum
  REQUIRED
    DebugTools
    GL
    MeshTools
    Primitives
    SceneGraph
    Shaders
    TextureTools
    Trade
    GlfwApplication
)

find_package(MagnumPlugins
  REQUIRED
    StbImageImporter
    TinyGltfImporter
)

target_link_libraries(viewer
  PRIVATE
    assets
    gfx
    nav
    scene
    GLFW::GLFW
    Magnum::Application
    Magnum::GL
    Magnum::Magnum
    Magnum::MeshTools
    Magnum::Primitives
    Magnum::SceneGraph
    Magnum::Shaders
    Magnum::TextureTools
    Magnum::Trade
    MagnumPlugins::StbImageImporter
    MagnumPlugins::TinyGltfImporter
)
