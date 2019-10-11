// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "esp/core/esp.h"
#include "esp/gfx/magnum.h"

#include "SceneNode.h"
#include "esp/gfx/RenderCamera.h"

#include "esp/sensor/Sensor.h"

namespace esp {
namespace scene {
class SceneGraph {
 public:
  SceneGraph();
  virtual ~SceneGraph() { LOG(INFO) << "Deconstructing SceneGraph"; };

  SceneNode& getRootNode() { return rootNode_; }
  const SceneNode& getRootNode() const { return rootNode_; }

  Magnum::SceneGraph::DrawableGroup3D& getDrawables() { return drawables_; }
  const Magnum::SceneGraph::DrawableGroup3D& getDrawables() const {
    return drawables_;
  }

  // set the transformation, projection matrix to the default camera
  // TODO:
  // in the future, the parameter should be VisualSensor
  void setDefaultRenderCamera(sensor::Sensor& sensor);

  gfx::RenderCamera& getDefaultRenderCamera() { return defaultRenderCamera_; }

  /* @brief check if the scene node is the root node of the scene graph.
   */
  static bool isRootNode(SceneNode& node);

 protected:
  MagnumScene world_;

  // Each item within is a base node, parent of all in that scene, for easy
  // manipulation (e.g., rotate the entire scene)

  // NOTE:
  // Do not define the SceneNode in front of the MagnumScene!
  // construction and destruction order matters!
  // Initialize the scene first.
  // See: https://doc.magnum.graphics/magnum/scenegraph.html

  // The transformation matrix between rootNode_ and world_
  // is ALWAYS an IDENTITY matrix.
  // DO NOT add any other transformation in between!!
  SceneNode rootNode_{world_};

  // Again, order matters! do not change the sequence!!
  // CANNOT make defaultRenderCameraNode_ specified BEFORE rootNode_.
  SceneNode defaultRenderCameraNode_{rootNode_};

  // a default camera to render the scene
  // user can of course define her own RenderCamera for rendering
  gfx::RenderCamera defaultRenderCamera_;

  // ==== Drawables ====
  // for each scene node in a scene graph,
  // we create a drawable object (e.g., PTexMeshDrawable, InstanceMeshDrawable,
  // etc.) and add it to the drawable group of that scene. This is done on the
  // fly when we build the scene graph

  // drawable groups for each scene graph
  // each item is a group of drawables.
  Magnum::SceneGraph::DrawableGroup3D drawables_;
};
}  // namespace scene
}  // namespace esp
