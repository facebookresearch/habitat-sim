set(sensor_SOURCES
  PinholeCamera.cpp
  PinholeCamera.h
  Sensor.cpp
  Sensor.h
  RedwoodNoiseModel.cpp
  RedwoodNoiseModel.h
)


if(BUILD_WITH_CUDA)
  list(APPEND CUDA_NVCC_FLAGS "-std=c++11;-O2")

  cuda_compile(cuda_kernels SHARED "RedwoodNoiseModel.cu")

  list(APPEND sensor_SOURCES ${cuda_kernels})
endif()


add_library(sensor STATIC ${sensor_SOURCES})

target_link_libraries(sensor
  PUBLIC
    core
    gfx
    scene
)

if(BUILD_WITH_CUDA)
  find_package(CUDA 8.0 REQUIRED)
  target_link_libraries(sensor
    PUBLIC
      ${CUDA_LIBRARIES}
  )
  target_include_directories(sensor
    PRIVATE
    ${CUDA_INCLUDE_DIRS})
endif()
