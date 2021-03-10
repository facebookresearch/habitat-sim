add_library(
  scene STATIC
  GibsonSemanticScene.cpp
  GibsonSemanticScene.h
  Mp3dSemanticScene.cpp
  Mp3dSemanticScene.h
  ObjectControls.cpp
  ObjectControls.h
  ReplicaSemanticScene.cpp
  ReplicaSemanticScene.h
  SceneGraph.cpp
  SceneGraph.h
  SceneManager.cpp
  SceneManager.h
  SceneNode.cpp
  SceneNode.h
  SemanticScene.cpp
  SemanticScene.h
  SuncgObjectCategoryMap.h
  SuncgSemanticScene.cpp
  SuncgSemanticScene.h
)

target_link_libraries(
  scene
  PUBLIC assets core geo gfx io
)

if(BUILD_TEST)
  add_subdirectory(test)
endif()
