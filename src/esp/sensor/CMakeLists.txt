set(
  sensor_SOURCES
  CameraSensor.cpp
  CameraSensor.h
  CubeMapSensorBase.cpp
  CubeMapSensorBase.h
  Sensor.cpp
  Sensor.h
  SensorFactory.cpp
  SensorFactory.h
  VisualSensor.cpp
  VisualSensor.h
  FisheyeSensor.cpp
  FisheyeSensor.h
  EquirectangularSensor.cpp
  EquirectangularSensor.h
  AudioSensor.cpp
  AudioSensor.h
  AudioSensorStubs.h
)

if(BUILD_WITH_CUDA)
  list(APPEND sensor_SOURCES RedwoodNoiseModel.cpp RedwoodNoiseModel.h)
endif()

if(BUILD_WITH_AUDIO)
  set(ESP_BUILD_WITH_AUDIO ON)
endif()

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/configure.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/configure.h
)

add_library(
  sensor STATIC
  ${sensor_SOURCES}
)

target_link_libraries(
  sensor
  PUBLIC core gfx scene sim
)

# ATTENTION developers !!!!!!!!!!!!!!!!!!!!
# the codebase has a deep architectural problem that needs solving -- lib dependency cycles !!!!
# the following is just a "crutch", a temporary remedy here.
# https://cmake.org/cmake/help/latest/prop_tgt/LINK_INTERFACE_MULTIPLICITY.html
set_target_properties(sensor PROPERTIES LINK_INTERFACE_MULTIPLICITY 3)

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

if(BUILD_WITH_AUDIO)
  include_directories(${DEPS_DIR}/rlr-audio-propagation/RLRAudioPropagationPkg/headers)
  target_link_libraries(sensor PUBLIC ${RLRAudioPropagation_LIBRARY})
endif()
