add_library(
  io STATIC
  io.cpp
  io.h
  json.cpp
  json.h
  URDFParser.cpp
  URDFParser.h
)

add_subdirectory(tinyxml2)

target_link_libraries(
  io
  PUBLIC core tinyxml2
)
