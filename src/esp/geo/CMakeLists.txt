file(GLOB_RECURSE geo_SOURCES "*.cpp")

add_library(geo STATIC ${geo_SOURCES})

target_link_libraries(geo
  PUBLIC
   core
   io
)
