set(sensor_SOURCES
  PinholeCamera.cpp
  PinholeCamera.h
  Sensor.cpp
  Sensor.h
)


if(BUILD_WITH_CUDA)
  list(APPEND sensor_SOURCES
    RedwoodNoiseModel.cpp
    RedwoodNoiseModel.h
  )
endif()


add_library(sensor STATIC ${sensor_SOURCES})

target_link_libraries(sensor
  PUBLIC
    core
    gfx
    scene
)

if(BUILD_WITH_CUDA)
  add_library(redwood_kernels STATIC
    RedwoodNoiseModel.cu
    RedwoodNoiseModel.cuh
  )
  target_link_libraries(redwood_kernels
    PUBLIC
      ${CUDART_LIBRARY}
  )
  target_include_directories(redwood_kernels
    PRIVATE
      ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
  )

  target_link_libraries(sensor
    PUBLIC
      redwood_kernels
  )
  target_include_directories(sensor
    PRIVATE
      ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
  )
endif()
