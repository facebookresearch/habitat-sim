find_package(Magnum REQUIRED AnyImageImporter AnyImageConverter Trade)
# Add links to magnum's imageconverter
target_link_libraries(
  magnum-imageconverter
  PRIVATE Magnum::AnyImageImporter Magnum::AnyImageConverter
          MagnumPlugins::StbImageImporter MagnumPlugins::StbImageConverter
          MagnumPlugins::BasisImageConverter
)

# Move the output to this directory
set_target_properties(
  magnum-imageconverter PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)
