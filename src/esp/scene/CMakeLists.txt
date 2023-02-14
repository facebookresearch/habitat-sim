# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

add_library(
  scene STATIC
  GibsonSemanticScene.cpp
  GibsonSemanticScene.h
  HM3DSemanticScene.cpp
  HM3DSemanticScene.h
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
)

target_link_libraries(
  scene
  PUBLIC assets core geo gfx io
)
