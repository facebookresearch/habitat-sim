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
)

target_link_libraries(
  io
  PUBLIC core
)
