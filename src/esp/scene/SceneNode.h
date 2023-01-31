// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SCENE_SCENENODE_H_
#define ESP_SCENE_SCENENODE_H_

#include <stack>

#include <Corrade/Containers/Containers.h>
#include <Corrade/Containers/Optional.h>
#include <Magnum/Math/Range.h>
#include <Magnum/SceneGraph/Object.h>
#include <Magnum/SceneGraph/TranslationRotationScalingTransformation3D.h>

#include "esp/core/Esp.h"
#include "esp/gfx/magnum.h"

// This class provides routines to:
// set and get local rigid body transformation of the current node w.r.t. the
// parent node; get global rigid body transformation

namespace esp {

namespace sensor {
class SensorSuite;
class Sensor;
}  // namespace sensor

namespace scene {

class SceneGraph;

// Future types may include e.g., "LIGHT"
enum class SceneNodeType {
  EMPTY = 0,
  SENSOR = 1,
  AGENT = 2,
  CAMERA = 3,
  OBJECT = 4,  // objects added via physics api
};

// Enumeration of SceneNodeTags
enum class SceneNodeTag : Magnum::UnsignedShort {
  /**
   * Set node as Leaf node, so no children nodes are allowed
   */
  Leaf = 1 << 0,
};

/**
 * @brief SceneNodeTags
 */
typedef Corrade::Containers::EnumSet<SceneNodeTag> SceneNodeTags;

class SceneNode : public MagnumObject,
                  public Magnum::SceneGraph::AbstractFeature3D {
 public:
  SceneNode(SceneNode& parent);
  ~SceneNode() override;

  // get the type of the attached object
  SceneNodeType getType() const { return type_; }
  void setType(SceneNodeType type) { type_ = type; }

  // Add a feature. Used to avoid naked `new` and makes intent clearer.
  template <class U, class... Args>
  void addFeature(Args&&... args) {
    // NOLINTNEXTLINE(clang-analyzer-cplusplus.NewDeleteLeaks)
    new U{*this, std::forward<Args>(args)...};
  }

  // Returns sceneNodeTags of SceneNode
  SceneNodeTags getSceneNodeTags() const { return sceneNodeTags_; }

  /**
   * @brief Sets sceneNodeTags_ of SceneNode
   *
   * @param[in] sceneNodeTags SceneNodeTags to set enumset sceneNodeTags_ to
   */
  void setSceneNodeTags(SceneNodeTags sceneNodeTags) {
    sceneNodeTags_ = sceneNodeTags;
  }

  /**
   * @brief Add SceneNodeTag to sceneNodeTags_ of SceneNode
   *
   * @param[in] sceneNodeTag SceneNodeTag to add to enumset sceneNodeTags_
   */
  void addSceneNodeTag(SceneNodeTag sceneNodeTag) {
    sceneNodeTags_ |= sceneNodeTag;
  }

  /**
   * @brief Remove SceneNodeTag from sceneNodeTags_ of SceneNode
   *
   * @param[in] sceneNodeTag SceneNodeTag to remove from enumset sceneNodeTags_
   */
  void removeSceneNodeTag(SceneNodeTag sceneNodeTag) {
    sceneNodeTags_ &= ~SceneNodeTags{sceneNodeTag};
  }

  //! Create a new child SceneNode and return it. NOTE: this SceneNode owns and
  //! is responsible for deallocating created child
  //! NOTE: child node inherits parent id by default
  SceneNode& createChild(SceneNodeTags childNodeTag = {});

  /**
   * @brief Sets parent SceneNode of this SceneNode, updates parent's
   * nodeSensorSuite and ancestors' subtreeSensorSuites NOTE: Overloads
   * MagnumObject::setParent
   * @param[in] newParent pointer to SceneNode of new parent SceneNode of this
   * SceneNode
   * @return reference to self
   */
  SceneNode& setParent(SceneNode* newParent);

  //! Returns node id
  virtual int getId() const { return id_; }

  //! Sets node id
  virtual void setId(int id) { id_ = id; }

  //! Returns node semanticId
  virtual int getSemanticId() const { return semanticId_; }

  //! Sets node semanticId
  virtual void setSemanticId(int semanticId) { semanticId_ = semanticId; }

  Magnum::Vector3 absoluteTranslation() const;

  Magnum::Vector3 absoluteTranslation();

  //! recursively compute the cumulative bounding box of the full scene graph
  //! tree for which this node is the root
  const Magnum::Range3D& computeCumulativeBB();

  //! return the local bounding box for meshes stored at this node
  const Magnum::Range3D& getMeshBB() const { return meshBB_; };

  //! return the global bounding box for the mesh stored at this node
  const Magnum::Range3D& getAbsoluteAABB() const;

  //! return the cumulative bounding box of the full scene graph tree for which
  //! this node is the root
  const Magnum::Range3D& getCumulativeBB() const { return cumulativeBB_; };

  /**
   * @brief Return SensorSuite containing references to Sensors this SceneNode
   * holds
   */
  esp::sensor::SensorSuite& getNodeSensorSuite() { return *nodeSensorSuite_; }

  /**
   * @brief Return map containing references to Sensors this SceneNode holds.
   * Keys are uuid strings, values are references to Sensors with that uuid
   */
  std::map<std::string, std::reference_wrapper<esp::sensor::Sensor>>&
  getNodeSensors();

  /**
   * @brief Return SensorSuite containing references to superset of all Sensors
   * held by this SceneNode and its children
   */
  esp::sensor::SensorSuite& getSubtreeSensorSuite() {
    return *subtreeSensorSuite_;
  }

  /**
   * @brief Return map containing references to superset of all Sensors held by
   * this SceneNode and its children values. Keys are uuid strings, values are
   * references to Sensors with that uuid
   */
  std::map<std::string, std::reference_wrapper<esp::sensor::Sensor>>&
  getSubtreeSensors();

  /** @brief Add sensor in this' nodeSensorSuite to parent's
   * nodeSensorSuite
   * NOTE: This only adds a sensor if this is a leaf node and sensor exists
   */
  void addSensorToParentNodeSensorSuite();

  /** @brief Remove sensor in this' nodeSensorSuite from parent's
   * nodeSensorSuite
   * NOTE: This only removes a sensor if this is a leaf node and sensor exists
   */
  void removeSensorFromParentNodeSensorSuite();

  /** @brief Go bottom to the top, and erase all subtreeSensors from this'
   * ancestors' subtreeSensorSuites
   */
  void removeSubtreeSensorsFromAncestors();

  /** @brief Go bottom to the top, and add all subtreeSensors to this'
   * ancestors' subtreeSensorSuites
   */
  void addSubtreeSensorsToAncestors();

  //! set local bounding box for meshes stored at this node
  void setMeshBB(Magnum::Range3D meshBB) { meshBB_ = meshBB; };

  //! set the global bounding box for mesh stored in this node
  void setAbsoluteAABB(Magnum::Range3D aabb) { aabb_ = aabb; };

  //! return the frustum plane in last frame that culls this node
  int getFrustumPlaneIndex() const { return frustumPlaneIndex; };

  //! set frustum plane in last frame that culls this node
  void setFrustumPlaneIndex(int index) { frustumPlaneIndex = index; };

 protected:
  // DO not make the following constructor public!
  // it can ONLY be called from SceneGraph class to initialize the scene graph
  friend class SceneGraph;
  explicit SceneNode(MagnumScene& parentNode);

  // Do not make the following constructor public!
  // This is only used for constructor delegation
  // creating a scene node "in the air" is not allowed.
  // it must set an existing node as its parent node.
  // this is to prevent any sub-tree that is "floating in the air", without a
  // terminate node (e.g., "MagnumScene" defined in SceneGraph) as its ancestor
  SceneNode();

  void clean(const Magnum::Matrix4& absoluteTransformation) override;

  // the type of the attached object (e.g., sensor, agent etc.)
  SceneNodeType type_ = SceneNodeType::EMPTY;
  int id_ = ID_UNDEFINED;

  // SceneNodeTags of this node, used to flag attributes such as leaf node
  SceneNodeTags sceneNodeTags_ = {};

  //! The semantic category of this node. Used to render attached Drawables with
  //! Semantic sensor when no perVertexObjectIds are present.
  uint32_t semanticId_ = 0;

  //! the local bounding box for meshes stored at this node
  Magnum::Range3D meshBB_;

  //! the cumulative bounding box of the full scene graph tree for which this
  //! node is the root
  Magnum::Range3D cumulativeBB_ = {{0.0, 0.0, 0.0}, {1e5, 1e5, 1e5}};

  //! The cumulativeBB in world coordinates
  //! This is returned instead of aabb_ if that doesn't exist
  //! due to this being a node that is part of a dynamic object
  mutable Corrade::Containers::Optional<Magnum::Range3D> worldCumulativeBB_ =
      Corrade::Containers::NullOpt;

  //! The absolute translation of this node, updated in clean
  Magnum::Matrix4 absoluteTransformation_;

  //! the global bounding box for *static* meshes stored at this node
  //  NOTE: this is different from the local bounding box meshBB_ defined above:
  //  -) it only applies to *static* meshes, NOT dynamic meshes in the scene (so
  //  it is an optional object);
  //  -) it was computed using mesh vertex positions in world space;
  Corrade::Containers::Optional<Magnum::Range3D> aabb_ =
      Corrade::Containers::NullOpt;

  //! the frustum plane in last frame that culls this node
  int frustumPlaneIndex = 0;

  // Pointer to SensorSuite containing references to Sensors this SceneNode
  // holds
  esp::sensor::SensorSuite* nodeSensorSuite_;

  // Pointer to SensorSuite containing references to superset of all Sensors
  // held by this SceneNode and its children
  esp::sensor::SensorSuite* subtreeSensorSuite_;
};

// Traversal Helpers

/**
 * @brief Perform a pre-order traversal and invoke a callback at each node
 *
 * @param node Root node for this traversal
 * @param cb Callback which will be called with each SceneNode
 */
template <typename Callable>
void preOrderTraversalWithCallback(const SceneNode& node, Callable&& cb) {
  std::stack<std::reference_wrapper<const SceneNode>> stack;
  stack.emplace(node);

  do {
    const SceneNode& currNode = stack.top();
    stack.pop();
    std::forward<Callable>(cb)(currNode);

    for (const MagnumObject& child : currNode.children()) {
      stack.emplace(static_cast<const SceneNode&>(child));
    }
  } while (!stack.empty());
}

/** @overload */
template <typename Callable>
void preOrderTraversalWithCallback(SceneNode& node, Callable&& cb) {
  auto constCb = [&cb](const SceneNode& node) {
    std::forward<Callable>(cb)(const_cast<SceneNode&>(node));
  };
  preOrderTraversalWithCallback(const_cast<const SceneNode&>(node), constCb);
}

/**
 * @brief Perform a pre-order traversal and invoke a callback on features of
 * the desired type
 *
 * @tparam Feature Feature type to invoke callback on
 * @param node Root node for this traversal
 * @param cb Callback which will be called with each feature
 */
template <typename Feature, typename Callable>
void preOrderFeatureTraversalWithCallback(const SceneNode& node,
                                          Callable&& cb) {
  auto featureCb = [&cb](const SceneNode& node) {
    for (const auto& abstractFeature : node.features()) {
      const auto* feature = dynamic_cast<const Feature*>(&abstractFeature);
      if (feature)
        std::forward<Callable>(cb)(*feature);
    }
  };
  preOrderTraversalWithCallback(node, featureCb);
}

/** @overload */
template <typename Feature, typename Callable>
void preOrderFeatureTraversalWithCallback(SceneNode& node, Callable&& cb) {
  auto constFeatureCb = [&cb](const Feature& feature) {
    std::forward<Callable>(cb)(const_cast<Feature&>(feature));
  };
  preOrderFeatureTraversalWithCallback<Feature>(
      const_cast<const SceneNode&>(node), constFeatureCb);
}

/**
 * @brief Set the semantic ID of a scene graph subtree.
 *
 * @param node Root node of the subtree.
 * @param semanticId Semantic ID to apply to the subtree.
 */
void setSemanticIdForSubtree(SceneNode* node, int semanticId);

CORRADE_ENUMSET_OPERATORS(SceneNodeTags)
}  // namespace scene
}  // namespace esp

#endif  // ESP_SCENE_SCENENODE_H_
