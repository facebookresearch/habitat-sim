set(
  sensor_SOURCES
  CameraSensor.cpp
  CameraSensor.h
  Sensor.cpp
  Sensor.h
  SensorFactory.cpp
  SensorFactory.h
  VisualSensor.cpp
  VisualSensor.h
)

if(BUILD_WITH_CUDA)
  list(APPEND sensor_SOURCES RedwoodNoiseModel.cpp RedwoodNoiseModel.h)
endif()

add_library(
  sensor STATIC
  ${sensor_SOURCES}
)

target_link_libraries(
  sensor
  PUBLIC core gfx scene
)

if(BUILD_WITH_CUDA)
  add_library(noise_model_kernels STATIC RedwoodNoiseModel.cu RedwoodNoiseModel.cuh)
  target_link_libraries(noise_model_kernels PUBLIC ${CUDART_LIBRARY})
  target_include_directories(
    noise_model_kernels PRIVATE ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
  )
  target_compile_features(noise_model_kernels PUBLIC cxx_std_11)

  target_link_libraries(sensor PUBLIC noise_model_kernels)
  target_include_directories(sensor PRIVATE ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})
endif()
