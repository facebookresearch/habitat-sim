add_library(tinyxml2 STATIC
  tinyxml2.h
  tinyxml2.cpp
)

set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)
