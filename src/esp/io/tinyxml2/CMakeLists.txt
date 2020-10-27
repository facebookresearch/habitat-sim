add_library(
  tinyxml2 STATIC
  tinyxml2.cpp tinyxml2.h
)

set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)
