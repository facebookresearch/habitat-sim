// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SCENE_SCENEGRAPH_H
#define ESP_SCENE_SCENEGRAPH_H

#include <Magnum/SceneGraph/Scene.h>
#include <unordered_map>
#include "esp/core/Esp.h"
#include "esp/gfx/magnum.h"

#include "SceneNode.h"
#include "esp/gfx/DrawableGroup.h"

#include "esp/sensor/VisualSensor.h"

namespace esp {
namespace scene {
class SceneGraph {
 public:
  using DrawableGroups = std::unordered_map<std::string, gfx::DrawableGroup>;

  SceneGraph();
  virtual ~SceneGraph() { ESP_DEBUG() << "Deconstructing SceneGraph"; };

  SceneNode& getRootNode() { return rootNode_; }
  const SceneNode& getRootNode() const { return rootNode_; }

  gfx::DrawableGroup& getDrawables(const std::string& groupName = {}) {
    return drawableGroups_.at(groupName);
  }
  const gfx::DrawableGroup& getDrawables(
      const std::string& groupName = {}) const {
    return drawableGroups_.at(groupName);
  }

  /* @brief check if the scene node is the root node of the scene graph.
   */
  static bool isRootNode(SceneNode& node);

  // Drawable group management
  // TODO: move this to separate class

  /**
   * @brief Get all drawable groups in this SceneGraph
   */
  // TODO?: return nicely iterable collection instead of map?
  DrawableGroups& getDrawableGroups() { return drawableGroups_; }

  /** @overload */
  const DrawableGroups& getDrawableGroups() const { return drawableGroups_; }

  /**
   * @brief Get a @ref DrawableGroup by ID
   *
   * @return Pointer to @ref DrawableGroup, or nullptr if shader does not exist.
   *
   * @ref DrawableGroup pointer is only valid until group is deleted!
   */
  gfx::DrawableGroup* getDrawableGroup(const std::string& id);

  /** @overload */
  const gfx::DrawableGroup* getDrawableGroup(const std::string& id) const;

  /**
   * @brief Creates a @ref DrawableGroup
   *
   * @param id    ID of created @ref DrawableGroup
   * @param args  Arguments passed to @ref DrawableGroup constructor
   * @return Pointer to the created @ref DrawableGroup, or nullptr if a
   *  @ref DrawableGroup with the same ID already exists.
   */
  template <typename... DrawableGroupArgs>
  gfx::DrawableGroup* createDrawableGroup(const std::string& id,
                                          DrawableGroupArgs&&... args) {
    auto inserted = drawableGroups_.emplace(
        std::piecewise_construct, std::forward_as_tuple(id),
        std::forward_as_tuple(std::forward<DrawableGroupArgs>(args)...));
    if (!inserted.second) {
      ESP_ERROR() << "DrawableGroup with ID:" << inserted.first->first
                  << "already exists!";
      return nullptr;
    }
    ESP_DEBUG() << "Created DrawableGroup:" << inserted.first->first;
    return &inserted.first->second;
  }

  /**
   * @brief Deletes a @ref DrawableGroup by ID
   *
   * @return If the @ref DrawableGroup with specified ID existed.
   */
  bool deleteDrawableGroup(const std::string& id);

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

  // ==== Drawables ====
  // for each scene node in a scene graph,
  // we create a drawable object (e.g., InstanceMeshDrawable, etc.) and add it
  // to the drawable group of that scene. This is done on the fly when we build
  // the scene graph

  // drawable groups for this scene graph
  // This is a mapping from (groupID -> group of drawables).
  DrawableGroups drawableGroups_;
};
}  // namespace scene
}  // namespace esp

#endif  // ESP_SCENE_SCENEGRAPH_H
