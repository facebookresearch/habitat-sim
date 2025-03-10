# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

# https://embeddedartistry.com/blog/2017/3/7/clang-weverything

option(BUILD_WARNINGS_AS_ERRORS "Build with warnings as errors" OFF)
if(BUILD_WARNINGS_AS_ERRORS)
  if(MSVC)
    add_compile_options(/WX)
  else()
    # TODO(msb) remove -Wall once we enable CORRADE_USE_PEDANTIC
    add_compile_options(-Wall)
    add_compile_options(-Werror)
  endif()
endif()

###########################
# Set conditional build variables used in configure.h files

if(BUILD_ASSIMP_SUPPORT)
  set(ESP_BUILD_ASSIMP_SUPPORT ON)
endif()

if(BUILD_WITH_AUDIO)
  set(ESP_BUILD_WITH_AUDIO ON)
endif()

if(BUILD_WITH_BACKGROUND_RENDERER)
  set(ESP_BUILD_WITH_BACKGROUND_RENDERER ON)
endif()

if(BUILD_WITH_BULLET)
  set(ESP_BUILD_WITH_BULLET ON)
endif()

if(BUILD_WITH_CUDA)
  set(ESP_BUILD_WITH_CUDA ON)
endif()

##########################
# Set path variables

# Set PBR configuration default path. Used in gfx/configure.h
set(ESP_DEFAULT_PBRSHADER_CONFIG_REL_PATH ./data/default.pbr_config.json)
set(ESP_DEFAULT_PBRSHADER_CONFIG
    ${PROJECT_SOURCE_DIR}/.${ESP_DEFAULT_PBRSHADER_CONFIG_REL_PATH}
)

# Set Physics Manager configuration default path. Used in physics/configure.h
set(ESP_DEFAULT_PHYSICS_CONFIG_REL_PATH ./data/default.physics_config.json)
set(ESP_DEFAULT_PHYSICS_CONFIG
    ${PROJECT_SOURCE_DIR}/.${ESP_DEFAULT_PHYSICS_CONFIG_REL_PATH}
)

#################################
# Configure.h file generation
# Namespace-specific compile-time generated configurations

# core
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/core/configure.h.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/core/configure.h
)
# gfx
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/gfx/configure.h.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/gfx/configure.h
)
# physics
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/physics/configure.h.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/physics/configure.h
)
# sensor
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/sensor/configure.h.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/sensor/configure.h
)

################################
# Corrade resources registration

# gfx shaders configuration
corrade_add_resource(GfxShaderResources ../shaders/gfx/Shaders.conf)

# pbr IBL image resources configuration
corrade_add_resource(PbrIBlImageResources ../../data/pbr/PbrImages.conf)

################################
# Specify packages to find

find_package(
  Magnum
  REQUIRED
  AnyImageImporter
  AnySceneImporter
  GL
  MaterialTools
  MeshTools
  DebugTools
  SceneGraph
  SceneTools
  Shaders
  Trade
  Primitives
  AnyImageConverter
)

if(BUILD_WITH_BULLET)
  find_package(MagnumIntegration REQUIRED Bullet)
  find_package(Bullet REQUIRED Dynamics)
endif()

find_package(MagnumPlugins REQUIRED PrimitiveImporter)

find_package(MagnumIntegration REQUIRED Eigen)

find_package(Corrade REQUIRED Utility)

# Link appropriate windowless library - gfx and gfx_batch
if(MAGNUM_TARGET_EGL)
  find_package(Magnum REQUIRED WindowlessEglApplication)
elseif(CORRADE_TARGET_APPLE)
  find_package(Magnum REQUIRED WindowlessCglApplication)
elseif(CORRADE_TARGET_UNIX)
  # Mainly for builds with external Magnum that might not have TARGET_EGL
  # enabled
  find_package(Magnum REQUIRED WindowlessGlxApplication)
elseif(CORRADE_TARGET_WINDOWS)
  find_package(Magnum REQUIRED WindowlessWglApplication)
else()
  message(FATAL_ERROR "Unsupported platform")
endif()

#####################
## Set sources
#################
## core
set(
  core_SOURCES
  core/Buffer.cpp
  core/Buffer.h
  core/Check.cpp
  core/Check.h
  core/Configuration.cpp
  core/Configuration.h
  core/Esp.cpp
  core/Esp.h
  core/Logging.cpp
  core/Logging.h
  core/managedContainers/AbstractFileBasedManagedObject.h
  core/managedContainers/AbstractManagedObject.h
  core/managedContainers/ManagedContainer.h
  core/managedContainers/ManagedContainerBase.cpp
  core/managedContainers/ManagedContainerBase.h
  core/managedContainers/ManagedFileBasedContainer.h
  core/Random.h
  core/Spimpl.h
  core/Utility.h
)

#################
## geo
set(
  geo_SOURCES
  geo/CoordinateFrame.cpp
  geo/CoordinateFrame.h
  geo/Geo.cpp
  geo/Geo.h
  geo/OBB.cpp
  geo/OBB.h
)

#################
## gfx
set(
  gfx_SOURCES
  gfx/CubeMap.cpp
  gfx/CubeMap.h
  gfx/Drawable.cpp
  gfx/Drawable.h
  gfx/DrawableConfiguration.cpp
  gfx/DrawableConfiguration.h
  gfx/DrawableGroup.cpp
  gfx/DrawableGroup.h
  gfx/GenericDrawable.cpp
  gfx/GenericDrawable.h
  gfx/SkinData.h
  gfx/MeshVisualizerDrawable.cpp
  gfx/MeshVisualizerDrawable.h
  gfx/LightSetup.cpp
  gfx/LightSetup.h
  gfx/magnum.h
  gfx/RenderCamera.cpp
  gfx/RenderCamera.h
  gfx/CubeMapCamera.cpp
  gfx/CubeMapCamera.h
  gfx/CubeMap.cpp
  gfx/CubeMap.h
  gfx/DebugLineRender.cpp
  gfx/DebugLineRender.h
  gfx/Renderer.cpp
  gfx/Renderer.h
  gfx/replay/Keyframe.h
  gfx/replay/Player.cpp
  gfx/replay/Player.h
  gfx/replay/Recorder.cpp
  gfx/replay/Recorder.h
  gfx/replay/ReplayManager.h
  gfx/replay/ReplayManager.cpp
  gfx/WindowlessContext.cpp
  gfx/WindowlessContext.h
  gfx/RenderTarget.cpp
  gfx/RenderTarget.h
  gfx/ShaderManager.cpp
  gfx/ShaderManager.h
  gfx/PbrShader.cpp
  gfx/PbrShader.h
  gfx/PbrDrawable.cpp
  gfx/PbrDrawable.h
  gfx/TextureVisualizerShader.cpp
  gfx/TextureVisualizerShader.h
  gfx/CubeMapShaderBase.cpp
  gfx/CubeMapShaderBase.h
  gfx/DoubleSphereCameraShader.cpp
  gfx/DoubleSphereCameraShader.h
  gfx/EquirectangularShader.cpp
  gfx/EquirectangularShader.h
  gfx/PbrIBLHelper.cpp
  gfx/PbrIBLHelper.h
  gfx/PbrEquiRectangularToCubeMapShader.cpp
  gfx/PbrEquiRectangularToCubeMapShader.h
  gfx/PbrPrecomputedMapShader.cpp
  gfx/PbrPrecomputedMapShader.h
  gfx/PbrTextureUnit.h
  gfx/GaussianFilterShader.h
  gfx/GaussianFilterShader.cpp
)

if(BUILD_WITH_BACKGROUND_RENDERER)
  list(APPEND gfx_SOURCES gfx/BackgroundRenderer.h gfx/BackgroundRenderer.cpp)
endif()

list(APPEND gfx_SOURCES ${GfxShaderResources})

#################
## assets
set(
  assets_SOURCES
  assets/Asset.cpp
  assets/Asset.h
  assets/BaseMesh.cpp
  assets/BaseMesh.h
  assets/CollisionMeshData.h
  assets/GenericSemanticMeshData.cpp
  assets/GenericSemanticMeshData.h
  assets/GenericMeshData.cpp
  assets/GenericMeshData.h
  assets/MeshData.h
  assets/MeshMetaData.h
  assets/RenderAssetInstanceCreationInfo.cpp
  assets/RenderAssetInstanceCreationInfo.h
  assets/ResourceManager.cpp
  assets/ResourceManager.h
  assets/RigManager.cpp
  assets/RigManager.h
  ${PbrIBlImageResources}
)

#################
## metadata
set(
  metadata_SOURCES
  metadata/attributes/AbstractAttributes.h
  metadata/attributes/AbstractAttributes.cpp
  metadata/attributes/AttributesEnumMaps.h
  metadata/attributes/AttributesEnumMaps.cpp
  metadata/attributes/AbstractObjectAttributes.h
  metadata/attributes/AbstractObjectAttributes.cpp
  metadata/attributes/AbstractSensorAttributes.h
  metadata/attributes/AbstractSensorAttributes.cpp
  metadata/attributes/AbstractVisualSensorAttributes.h
  metadata/attributes/AbstractVisualSensorAttributes.cpp
  metadata/attributes/ArticulatedObjectAttributes.h
  metadata/attributes/ArticulatedObjectAttributes.cpp
  metadata/attributes/AudioSensorAttributes.h
  metadata/attributes/AudioSensorAttributes.cpp
  metadata/attributes/CameraSensorAttributes.h
  metadata/attributes/CameraSensorAttributes.cpp
  metadata/attributes/CustomSensorAttributes.h
  metadata/attributes/CustomSensorAttributes.cpp
  metadata/attributes/CubeMapSensorAttributes.h
  metadata/attributes/CubeMapSensorAttributes.cpp
  metadata/attributes/LightLayoutAttributes.h
  metadata/attributes/LightLayoutAttributes.cpp
  metadata/attributes/MarkerSets.h
  metadata/attributes/ObjectAttributes.h
  metadata/attributes/ObjectAttributes.cpp
  metadata/attributes/PhysicsManagerAttributes.h
  metadata/attributes/PhysicsManagerAttributes.cpp
  metadata/attributes/PbrShaderAttributes.h
  metadata/attributes/PbrShaderAttributes.cpp
  metadata/attributes/PrimitiveAssetAttributes.h
  metadata/attributes/PrimitiveAssetAttributes.cpp
  metadata/attributes/SceneInstanceAttributes.h
  metadata/attributes/SceneInstanceAttributes.cpp
  metadata/attributes/SceneDatasetAttributes.h
  metadata/attributes/SceneDatasetAttributes.cpp
  metadata/attributes/SemanticAttributes.h
  metadata/attributes/SemanticAttributes.cpp
  metadata/attributes/StageAttributes.h
  metadata/attributes/StageAttributes.cpp
  metadata/managers/AbstractAttributesManager.h
  metadata/managers/AbstractObjectAttributesManager.h
  metadata/managers/AOAttributesManager.h
  metadata/managers/AOAttributesManager.cpp
  metadata/managers/AssetAttributesManager.h
  metadata/managers/AssetAttributesManager.cpp
  metadata/managers/LightLayoutAttributesManager.h
  metadata/managers/LightLayoutAttributesManager.cpp
  metadata/managers/ObjectAttributesManager.h
  metadata/managers/ObjectAttributesManager.cpp
  metadata/managers/PbrShaderAttributesManager.h
  metadata/managers/PbrShaderAttributesManager.cpp
  metadata/managers/PhysicsAttributesManager.h
  metadata/managers/PhysicsAttributesManager.cpp
  metadata/managers/SceneInstanceAttributesManager.h
  metadata/managers/SceneInstanceAttributesManager.cpp
  metadata/managers/SceneDatasetAttributesManager.h
  metadata/managers/SceneDatasetAttributesManager.cpp
  metadata/managers/SemanticAttributesManager.h
  metadata/managers/SemanticAttributesManager.cpp
  metadata/managers/SensorAttributesManager.h
  metadata/managers/SensorAttributesManager.cpp
  metadata/managers/StageAttributesManager.h
  metadata/managers/StageAttributesManager.cpp
  metadata/MetadataMediator.h
  metadata/MetadataMediator.cpp
  metadata/URDFParser.cpp
  metadata/URDFParser.h
)

#################
## io
set(
  io_SOURCES
  io/Io.cpp
  io/Io.h
  io/Json.cpp
  io/Json.h
  io/JsonAllTypes.h
  io/JsonBuiltinTypes.h
  io/JsonEspTypes.cpp
  io/JsonEspTypes.h
  io/JsonMagnumTypes.cpp
  io/JsonMagnumTypes.h
  io/JsonStlTypes.cpp
  io/JsonStlTypes.h
  io/JsonUtils.h
)

#################
## scene
set(
  scene_SOURCES
  scene/GibsonSemanticScene.cpp
  scene/GibsonSemanticScene.h
  scene/HM3DSemanticScene.cpp
  scene/HM3DSemanticScene.h
  scene/Mp3dSemanticScene.cpp
  scene/Mp3dSemanticScene.h
  scene/ReplicaSemanticScene.cpp
  scene/ReplicaSemanticScene.h
  scene/SceneGraph.cpp
  scene/SceneGraph.h
  scene/SceneManager.cpp
  scene/SceneManager.h
  scene/SceneNode.cpp
  scene/SceneNode.h
  scene/SemanticScene.cpp
  scene/SemanticScene.h
)

#################
## physics
set(
  physics_SOURCES
  physics/ArticulatedLink.h
  physics/ArticulatedObject.h
  physics/CollisionGroupHelper.cpp
  physics/CollisionGroupHelper.h
  physics/objectManagers/ArticulatedObjectManager.cpp
  physics/objectManagers/ArticulatedObjectManager.h
  physics/objectManagers/PhysicsObjectBaseManager.h
  physics/objectManagers/RigidObjectManager.cpp
  physics/objectManagers/RigidObjectManager.h
  physics/objectWrappers/ManagedArticulatedObject.h
  physics/objectWrappers/ManagedPhysicsObjectBase.h
  physics/objectWrappers/ManagedRigidBase.h
  physics/objectWrappers/ManagedRigidObject.h
  physics/PhysicsManager.cpp
  physics/PhysicsManager.h
  physics/PhysicsObjectBase.h
  physics/RigidBase.h
  physics/RigidObject.cpp
  physics/RigidObject.h
  physics/RigidStage.cpp
  physics/RigidStage.h
  physics/URDFImporter.cpp
  physics/URDFImporter.h
)

if(BUILD_WITH_BULLET)

  # Add bullet sources
  list(
    APPEND
    physics_SOURCES
    physics/bullet/BulletArticulatedLink.h
    physics/bullet/BulletArticulatedObject.cpp
    physics/bullet/BulletArticulatedObject.h
    physics/bullet/BulletBase.cpp
    physics/bullet/BulletBase.h
    physics/bullet/BulletCollisionHelper.cpp
    physics/bullet/BulletCollisionHelper.h
    physics/bullet/BulletPhysicsManager.cpp
    physics/bullet/BulletPhysicsManager.h
    physics/bullet/BulletRigidObject.cpp
    physics/bullet/BulletRigidObject.h
    physics/bullet/BulletRigidStage.cpp
    physics/bullet/BulletRigidStage.h
    physics/bullet/BulletURDFImporter.cpp
    physics/bullet/BulletURDFImporter.h
    physics/bullet/objectWrappers/ManagedBulletArticulatedObject.h
    physics/bullet/objectWrappers/ManagedBulletRigidObject.h
  )

  ## Enable physics profiling
  #add_compile_definitions(BT_ENABLE_PROFILE=0)
  #add_definitions(-DBT_ENABLE_PROFILE)

endif() #BUILD_WITH_BULLET

#################
## nav
# cmake-format: off
set(nav_SOURCES
  nav/GreedyFollower.cpp
  nav/GreedyFollower.h
  nav/PathFinder.cpp
  nav/PathFinder.h
)
# cmake-format: on

#################
## sensor
set(
  sensor_SOURCES
  sensor/CameraSensor.cpp
  sensor/CameraSensor.h
  sensor/CubeMapSensorBase.cpp
  sensor/CubeMapSensorBase.h
  sensor/Sensor.cpp
  sensor/Sensor.h
  sensor/SensorFactory.cpp
  sensor/SensorFactory.h
  sensor/VisualSensor.cpp
  sensor/VisualSensor.h
  sensor/FisheyeSensor.cpp
  sensor/FisheyeSensor.h
  sensor/EquirectangularSensor.cpp
  sensor/EquirectangularSensor.h
  sensor/AudioSensor.cpp
  sensor/AudioSensor.h
  sensor/AudioSensorStubs.h
)

if(BUILD_WITH_CUDA)
  list(APPEND sensor_SOURCES sensor/RedwoodNoiseModel.cpp sensor/RedwoodNoiseModel.h)
  # CUDA kernel sources
  set(noise_model_SOURCES sensor/RedwoodNoiseModel.cu sensor/RedwoodNoiseModel.cuh)
endif()

#################
## sim
set(
  sim_SOURCES
  sim/AbstractReplayRenderer.cpp
  sim/AbstractReplayRenderer.h
  sim/BatchPlayerImplementation.cpp
  sim/BatchPlayerImplementation.h
  sim/BatchReplayRenderer.cpp
  sim/BatchReplayRenderer.h
  sim/ClassicReplayRenderer.cpp
  sim/ClassicReplayRenderer.h
  sim/Simulator.cpp
  sim/Simulator.h
  sim/SimulatorConfiguration.cpp
  sim/SimulatorConfiguration.h
  sim/RenderInstanceHelper.cpp
  sim/RenderInstanceHelper.h
)

#################
## build library

################################
# Keeping gfx_batch a separate library
add_subdirectory(gfx_batch)
##############################

set(
  habsim_SOURCES
  ${core_SOURCES}
  ${geo_SOURCES}
  ${gfx_SOURCES}
  ${assets_SOURCES}
  ${metadata_SOURCES}
  ${io_SOURCES}
  ${scene_SOURCES}
  ${physics_SOURCES}
  ${nav_SOURCES}
  ${sensor_SOURCES}
  ${sim_SOURCES}
)

add_library(
  habitat_sim STATIC
  ${habsim_SOURCES}
)

target_include_directories(
  habitat_sim PUBLIC ${PROJECT_BINARY_DIR}
  PRIVATE "${DEPS_DIR}/tinyxml2" "${DEPS_DIR}/recastnavigation/Detour/Include"
          "${DEPS_DIR}/recastnavigation/Recast/Include"
)

if(BUILD_WITH_CUDA)
  add_library(noise_model_kernels STATIC ${noise_model_SOURCES})
  # Use this until this change in Corrade is included in Habitat
  # https://github.com/mosra/corrade/commit/280e6b6874e0902086e321d733ce0e90e0c52920
  set_target_properties(noise_model_kernels PROPERTIES CORRADE_USE_PEDANTIC_FLAGS OFF)
  target_link_libraries(noise_model_kernels PUBLIC ${CUDART_LIBRARY})
  target_include_directories(
    noise_model_kernels PRIVATE ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
  )
  target_compile_features(noise_model_kernels PUBLIC cxx_std_11)

  target_link_libraries(habitat_sim PUBLIC noise_model_kernels ${CUDART_LIBRARY})

  target_include_directories(
    habitat_sim PRIVATE ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
  )
endif()

if(BUILD_WITH_AUDIO)
  include_directories(${DEPS_DIR}/rlr-audio-propagation/RLRAudioPropagationPkg/headers)
  target_link_libraries(habitat_sim PUBLIC ${RLRAudioPropagation_LIBRARY})
endif()

target_link_libraries(
  habitat_sim
  PUBLIC gfx_batch
         Magnum::AnyImageImporter
         Magnum::AnySceneImporter
         Magnum::GL
         Magnum::Magnum
         Magnum::MaterialTools
         Magnum::MeshTools
         Magnum::DebugTools
         Magnum::SceneGraph
         Magnum::SceneTools
         Magnum::Shaders
         Magnum::Trade
         Magnum::Primitives
         MagnumPlugins::PrimitiveImporter
         MagnumIntegration::Eigen
         Corrade::Utility
         Magnum::AnyImageConverter
  PRIVATE tinyxml2 Detour Recast
)

if(BUILD_WITH_BACKGROUND_RENDERER)
  target_link_libraries(habitat_sim PUBLIC atomic_wait)
endif()

if(OpenMP_CXX_FOUND)
  target_link_libraries(habitat_sim PUBLIC OpenMP::OpenMP_CXX)
endif()

target_link_libraries(
  habitat_sim
  PUBLIC Magnum::WindowlessApplication
)
if(BUILD_WITH_BULLET)
  target_link_libraries(habitat_sim PUBLIC MagnumIntegration::Bullet Bullet::Dynamics)
endif() #BUILD_WITH_BULLET

# enable when finished? Slows compilation substantially
set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)
