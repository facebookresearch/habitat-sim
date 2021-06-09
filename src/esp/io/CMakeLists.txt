add_library(
  io STATIC
  io.cpp
  io.h
  json.cpp
  json.h
  JsonAllTypes.h
  JsonBuiltinTypes.h
  JsonEspTypes.cpp
  JsonEspTypes.h
  JsonMagnumTypes.h
  JsonStlTypes.h
  URDFParser.cpp
  URDFParser.h
)

target_include_directories(io PRIVATE "${DEPS_DIR}/tinyxml2")

target_link_libraries(
  io
  PUBLIC core
  PRIVATE tinyxml2
)
