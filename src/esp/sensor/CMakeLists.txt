file(GLOB_RECURSE sensor_SOURCES "*.cpp")

find_package(CUDA)

if (CUDA_FOUND)
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

if (CUDA_FOUND)
  target_link_libraries(sensor
    PUBLIC
    ${CUDA_LIBRARIES})
  target_compile_definitions(sensor
    PUBLIC
      SENSORS_WITH_CUDA
  )
  target_include_directories(sensor
    PRIVATE
    ${CUDA_INCLUDE_DIRS})
endif()
