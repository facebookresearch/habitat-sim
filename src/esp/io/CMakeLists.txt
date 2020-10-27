add_library(
  io STATIC
  io.cpp
  io.h
  json.cpp
  json.h
  URDFParser.cpp
  URDFParser.h
)

target_include_directories(io PRIVATE "${DEPS_DIR}/tinyxml2")

target_link_libraries(
  io
  PUBLIC core
  PRIVATE tinyxml2
)
