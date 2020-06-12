// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

/** @file
 * @brief Class @ref esp::physics::ArticulatedLink, Class @ref
 * esp::physics::ArticulatedObject
 */

#include "RigidBase.h"
#include "esp/scene/SceneNode.h"

namespace esp {

namespace gfx {
class DrawableGroup;
}

namespace assets {
class ResourceManager;
}

namespace io {
struct URDFLinkContactInfo;
struct UrdfLink;
}  // namespace io

namespace physics {

class URDFImporter;

////////////////////////////////////
// Link
////////////////////////////////////

/**
 * @brief A single rigid link in a kinematic chain. Abstract class. Feature
 * attaches to a SceneNode.
 */
class ArticulatedLink : public RigidBase {
 public:
  ArticulatedLink(scene::SceneNode* bodyNode, int index)
      : mbIndex_(index), RigidBase(bodyNode){};

  virtual ~ArticulatedLink(){};

  /**
   * @brief Get the scene node being attached to.
   */
  scene::SceneNode& node() { return object(); }
  const scene::SceneNode& node() const { return object(); }

  // Overloads to avoid confusion
  scene::SceneNode& object() {
    return static_cast<scene::SceneNode&>(
        Magnum::SceneGraph::AbstractFeature3D::object());
  }
  const scene::SceneNode& object() const {
    return static_cast<const scene::SceneNode&>(
        Magnum::SceneGraph::AbstractFeature3D::object());
  }

  int getIndex() { return mbIndex_; };

  // RigidBase overrides

  virtual void setTransformation(
      CORRADE_UNUSED const Magnum::Matrix4& transformation) override {
    Corrade::Utility::Debug()
        << "(setTransformation) - ArticulatedLink can't do this.";
  };

  virtual void setTranslation(
      CORRADE_UNUSED const Magnum::Vector3& vector) override {
    Corrade::Utility::Debug()
        << "(setTranslation) - ArticulatedLink can't do this.";
  };

  virtual void setRotation(
      CORRADE_UNUSED const Magnum::Quaternion& quaternion) override {
    Corrade::Utility::Debug()
        << "(setRotation) - ArticulatedLink can't do this.";
  };

  virtual void setRigidState(
      CORRADE_UNUSED const core::RigidState& rigidState) override {
    Corrade::Utility::Debug()
        << "(setRigidState) - ArticulatedLink can't do this.";
  };

  virtual void resetTransformation() override {
    Corrade::Utility::Debug()
        << "(resetTransformation) - ArticulatedLink can't do this.";
  }

  virtual void translate(
      CORRADE_UNUSED const Magnum::Vector3& vector) override {
    Corrade::Utility::Debug() << "(translate) - ArticulatedLink can't do this.";
  }

  virtual void translateLocal(
      CORRADE_UNUSED const Magnum::Vector3& vector) override {
    Corrade::Utility::Debug()
        << "(translateLocal) - ArticulatedLink can't do this.";
  }

  virtual void rotate(
      CORRADE_UNUSED const Magnum::Rad angleInRad,
      CORRADE_UNUSED const Magnum::Vector3& normalizedAxis) override {
    Corrade::Utility::Debug() << "(rotate) - ArticulatedLink can't do this.";
  }

  virtual void rotateLocal(
      CORRADE_UNUSED const Magnum::Rad angleInRad,
      CORRADE_UNUSED const Magnum::Vector3& normalizedAxis) override {
    Corrade::Utility::Debug()
        << "(rotateLocal) - ArticulatedLink can't do this.";
  }

  virtual void rotateX(CORRADE_UNUSED const Magnum::Rad angleInRad) override {
    Corrade::Utility::Debug() << "(rotateX) - ArticulatedLink can't do this.";
  }
  virtual void rotateY(CORRADE_UNUSED const Magnum::Rad angleInRad) override {
    Corrade::Utility::Debug() << "(rotateY) - ArticulatedLink can't do this.";
  }
  virtual void rotateZ(CORRADE_UNUSED const Magnum::Rad angleInRad) override {
    Corrade::Utility::Debug() << "(rotateZ) - ArticulatedLink can't do this.";
  }
  virtual void rotateXLocal(
      CORRADE_UNUSED const Magnum::Rad angleInRad) override {
    Corrade::Utility::Debug()
        << "(rotateXLocal) - ArticulatedLink can't do this.";
  }
  virtual void rotateYLocal(
      CORRADE_UNUSED const Magnum::Rad angleInRad) override {
    Corrade::Utility::Debug()
        << "(rotateYLocal) - ArticulatedLink can't do this.";
  }
  virtual void rotateZLocal(
      CORRADE_UNUSED const Magnum::Rad angleInRad) override {
    Corrade::Utility::Debug()
        << "(rotateZLocal) - ArticulatedLink can't do this.";
  }

 protected:
  int mbIndex_;

  ESP_SMART_POINTERS(ArticulatedLink)
};

////////////////////////////////////
// Articulated Object
////////////////////////////////////

/**
 * @brief An articulated rigid object (i.e. kinematic chain). Abstract class to
 * be derived by physics simulator specific implementations.
 */
class ArticulatedObject : public Magnum::SceneGraph::AbstractFeature3D {
 public:
  ArticulatedObject(scene::SceneNode* rootNode)
      : Magnum::SceneGraph::AbstractFeature3D(*rootNode){};

  virtual ~ArticulatedObject() {
    // clear links and delete their SceneNodes
    std::vector<scene::SceneNode*> linkNodes;
    for (auto& link : links_) {
      linkNodes.push_back(&link.second->node());
    }
    links_.clear();
    for (auto node : linkNodes) {
      delete node;
    }
  };

  /**
   * @brief Get the scene node being attached to.
   */
  scene::SceneNode& node() { return object(); }
  const scene::SceneNode& node() const { return object(); }

  // Overloads to avoid confusion
  scene::SceneNode& object() {
    return static_cast<scene::SceneNode&>(
        Magnum::SceneGraph::AbstractFeature3D::object());
  }
  const scene::SceneNode& object() const {
    return static_cast<const scene::SceneNode&>(
        Magnum::SceneGraph::AbstractFeature3D::object());
  }

  virtual bool initializeFromURDF(
      CORRADE_UNUSED URDFImporter& urdfImporter,
      CORRADE_UNUSED const Magnum::Matrix4& worldTransform,
      CORRADE_UNUSED assets::ResourceManager& resourceManager,
      CORRADE_UNUSED gfx::DrawableGroup* drawables,
      CORRADE_UNUSED scene::SceneNode* physicsNode,
      CORRADE_UNUSED bool fixedBase = false) {
    return false;
  };

  virtual Magnum::Matrix4 getRootState() { return {}; };

  // update the SceneNode state to match the simulation state
  virtual void updateNodes(){};

  ArticulatedLink& getLink(int id) {
    CHECK(links_.count(id));
    return *links_.at(id).get();
  };

  int getNumLinks() { return links_.size(); };

  std::vector<int> getLinkIds() {
    std::vector<int> ids;
    for (auto it = links_.begin(); it != links_.end(); ++it) {
      ids.push_back(it->first);
    }
    return ids;
  };

  virtual void setRootState(CORRADE_UNUSED const Magnum::Matrix4& state){};

  virtual void setForces(CORRADE_UNUSED std::vector<float> forces){};

  virtual std::vector<float> getForces() { return {}; };

  virtual void setVelocities(CORRADE_UNUSED std::vector<float> vels){};

  virtual std::vector<float> getVelocities() { return {}; };

  virtual void setPositions(CORRADE_UNUSED std::vector<float> positions){};

  virtual std::vector<float> getPositions() { return {}; };

  virtual std::vector<float> getPositionLowerLimits() { return {}; };

  virtual std::vector<float> getPositionUpperLimits() { return {}; };

  virtual void addArticulatedLinkForce(CORRADE_UNUSED int linkId,
                                       CORRADE_UNUSED Magnum::Vector3 force){};

  /**
   * @brief reset the articulated rigid body to 0 velocities and positions.
   */
  virtual void reset(){};

  virtual void setSleep(CORRADE_UNUSED bool sleep){};

  virtual bool getSleep() { return false; };

  //! Check if this object can be de-activated (i.e. sleep).
  virtual bool getCanSleep() { return false; };

  virtual MotionType getMotionType() { return motionType_; };

  virtual void setMotionType(CORRADE_UNUSED MotionType mt){};

 protected:
  virtual bool attachGeometry(
      CORRADE_UNUSED scene::SceneNode& node,
      CORRADE_UNUSED std::shared_ptr<io::UrdfLink> link,
      CORRADE_UNUSED const
          std::map<std::string, std::shared_ptr<io::UrdfMaterial> >& materials,
      CORRADE_UNUSED assets::ResourceManager& resourceManager,
      CORRADE_UNUSED gfx::DrawableGroup* drawables) {
    return false;
  };

  std::map<int, ArticulatedLink::uptr> links_;

  MotionType motionType_ = MotionType::KINEMATIC;

  ESP_SMART_POINTERS(ArticulatedObject)
};

}  // namespace physics
}  // namespace esp
