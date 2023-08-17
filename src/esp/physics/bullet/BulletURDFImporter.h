// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

// Code adapted from Bullet3/examples/Importers/ImportURDFDemo ...

#ifndef ESP_PHYSICS_BULLET_BULLETURDFIMPORTER_H_
#define ESP_PHYSICS_BULLET_BULLETURDFIMPORTER_H_

#include <btBulletDynamicsCommon.h>
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "esp/physics/URDFImporter.h"
namespace esp {

namespace physics {

/**
 * @brief Structure to hold joint limit constraint info during construction.
 */
struct JointLimitConstraintInfo {
  JointLimitConstraintInfo(int _dof,
                           float _lowerLimit,
                           float _upperLimit,
                           btMultiBodyConstraint* _con)
      : dof(_dof),
        lowerLimit(_lowerLimit),
        upperLimit(_upperLimit),
        con(_con){};
  int dof;
  float lowerLimit, upperLimit;
  btMultiBodyConstraint* con;
};

/**
 * @brief struct to encapsulate mapping between child and parent links ids.
 */
struct childParentIndex {
  int m_index;
  int m_mbIndex;
  int m_parentIndex;
  int m_parentMBIndex;

  std::string m_link_name;
};

/**
 * @brief Structure to hold construction time multi-body data.
 */
struct URDF2BulletCached {
  URDF2BulletCached() = default;
  // these arrays will be initialized in the 'initURDF2BulletCache'

  std::vector<int> m_urdfLinkParentIndices;
  std::vector<int> m_urdfLinkIndices2BulletLinkIndices;
  std::vector<btTransform> m_urdfLinkLocalInertialFrames;

  int m_currentMultiBodyLinkIndex{-1};

  class btMultiBody* m_bulletMultiBody{nullptr};

  std::unordered_map<int, JointLimitConstraintInfo> m_jointLimitConstraints;

  // this will be initialized in the constructor
  int m_totalNumJoints1{0};
  int getParentUrdfIndex(int linkIndex) const {
    return m_urdfLinkParentIndices[linkIndex];
  }
  int getMbIndexFromUrdfIndex(int urdfIndex) const {
    ESP_VERY_VERBOSE() << "::getMbIndexFromUrdfIndex";
    if (urdfIndex == -2)
      return -2;
    return m_urdfLinkIndices2BulletLinkIndices[urdfIndex];
  }
};

/**
 * @brief Derived importer class for loading URDF into Bullet physics
 */
class BulletURDFImporter : public URDFImporter {
 public:
  explicit BulletURDFImporter(esp::assets::ResourceManager& resourceManager)
      : URDFImporter(resourceManager) {}

  ~BulletURDFImporter() override = default;

  /////////////////////////////////////
  // multi-body construction functions

  //! Initialize the temporary Bullet cache for multibody construction from the
  //! active URDF::Model
  void initURDF2BulletCache();

  //! Traverse the kinematic chain recursively constructing the btMultiBody
  Magnum::Matrix4 convertURDF2BulletInternal(
      int urdfLinkIndex,
      const Magnum::Matrix4& parentTransformInWorldSpace,
      btMultiBodyDynamicsWorld* world1,
      std::map<int, std::unique_ptr<btCompoundShape>>& linkCompoundShapes,
      std::map<int, std::vector<std::unique_ptr<btCollisionShape>>>&
          linkChildShapes,
      bool recursive = false);

  //! The temporary Bullet multibody cache initialized by
  //! convertURDF2BulletInternal and cleared after instancing the object
  std::shared_ptr<URDF2BulletCached> cache = nullptr;

  //! Recursively get all indices from the model with mappings between parents
  //! and children
  void getAllIndices(int urdfLinkIndex,
                     int parentIndex,
                     std::vector<childParentIndex>& allIndices);

 protected:
  //! Construct a set of Bullet collision shapes from the URDF::CollisionShape
  //! metadata
  btCollisionShape* convertURDFToCollisionShape(
      const struct metadata::URDF::CollisionShape* collision,
      std::vector<std::unique_ptr<btCollisionShape>>& linkChildShapes);

  //! Construct all Bullet collision shapes for a link in the active URDF::Model
  btCompoundShape* convertLinkCollisionShapes(
      int linkIndex,
      const btTransform& localInertiaFrame,
      std::vector<std::unique_ptr<btCollisionShape>>& linkChildShapes);

  //! Get configured collision groups and masks for a link's collision shape
  int getCollisionGroupAndMask(int linkIndex,
                               int& colGroup,
                               int& colMask) const;

  void computeTotalNumberOfJoints(int linkIndex);

  //! Compute the new Bullet link indices from the URDF::Model
  void computeParentIndices(URDF2BulletCached& bulletCache,
                            int urdfLinkIndex,
                            int urdfParentIndex);
};

void processContactParameters(
    const metadata::URDF::LinkContactInfo& contactInfo,
    btCollisionObject* col);

}  // namespace physics
}  // namespace esp

#endif  // ESP_PHYSICS_BULLET_BULLETURDFIMPORTER_H_
