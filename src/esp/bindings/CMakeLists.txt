find_package(MagnumBindings REQUIRED Python)

pybind11_add_module(habitat_sim_bindings bindings.cpp ShortestPathBindings.cpp geoBindings.cpp)
target_link_libraries(habitat_sim_bindings
  PRIVATE
    MagnumBindings::Python
  PUBLIC
    agent
    assets
    scene
    core
    gfx
    nav
    sensor
)

if(WITH_GL_TENSOR)
  target_compile_definitions(habitat_sim_bindings PRIVATE __ESP_WITH_GL_TENSOR__)
endif()

# target_include_directories(environment
#   PUBLIC
#     ${CMAKE_CURRENT_LIST_DIR}
# )

# file(WRITE ${PYTHON_ROOT}/libpath.py "LIB_PATH='${CMAKE_CURRENT_BINARY_DIR}'\n")
