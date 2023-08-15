// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

// Code adapted from Bullet3/examples/Importers/ImportURDFDemo ...
#include <iostream>

#include <Corrade/Utility/DebugStl.h>
#include <Corrade/Utility/Path.h>
#include <Magnum/BulletIntegration/Integration.h>
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletCollisionHelper.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletURDFImporter.h"
#include "esp/assets/ResourceManager.h"
#include "esp/physics/CollisionGroupHelper.h"
#include "esp/physics/bullet/BulletBase.h"

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace physics {

// TODO: make this configurable?
static float gUrdfDefaultCollisionMargin = 0.001;

btCollisionShape* BulletURDFImporter::convertURDFToCollisionShape(
    const metadata::URDF::CollisionShape* collision,
    std::vector<std::unique_ptr<btCollisionShape>>& linkChildShapes) {
  ESP_VERY_VERBOSE() << "convertURDFToCollisionShape";

  btCollisionShape* shape = nullptr;

  switch (collision->m_geometry.m_type) {
    case metadata::URDF::GEOM_PLANE: {
      Mn::Vector3 planeNormal = collision->m_geometry.m_planeNormal;
      float planeConstant = 0;  // not available?
      auto plane = std::make_unique<btStaticPlaneShape>(btVector3(planeNormal),
                                                        planeConstant);
      shape = plane.get();
      shape->setMargin(gUrdfDefaultCollisionMargin);
      linkChildShapes.emplace_back(std::move(plane));
      break;
    }
    case metadata::URDF::GEOM_CAPSULE: {
      float radius = collision->m_geometry.m_capsuleRadius;
      float height = collision->m_geometry.m_capsuleHeight;
      auto capsuleShape = std::make_unique<btCapsuleShapeZ>(radius, height);
      shape = capsuleShape.get();
      shape->setMargin(gUrdfDefaultCollisionMargin);
      linkChildShapes.emplace_back(std::move(capsuleShape));
      break;
    }

    case metadata::URDF::GEOM_CYLINDER: {
      float cylRadius = collision->m_geometry.m_capsuleRadius;
      float cylHalfLength = 0.5 * collision->m_geometry.m_capsuleHeight;
      // if (m_data->m_flags & CUF_USE_IMPLICIT_CYLINDER) TODO: why not? Should
      // always be better.

      btVector3 halfExtents(cylRadius, cylRadius, cylHalfLength);
      auto cylZShape = std::make_unique<btCylinderShapeZ>(halfExtents);
      shape = cylZShape.get();
      linkChildShapes.emplace_back(std::move(cylZShape));
      break;
    }
    case metadata::URDF::GEOM_BOX: {
      btVector3 extents = btVector3(collision->m_geometry.m_boxSize);
      auto boxShape = std::make_unique<btBoxShape>(extents * 0.5f);
      // TODO: off for now, but may be necessary for better contacts?
      /*
      if (m_data->m_flags & CUF_INITIALIZE_SAT_FEATURES)
                      {
                              boxShape->initializePolyhedralFeatures();
                      }
      */
      shape = boxShape.get();
      shape->setMargin(gUrdfDefaultCollisionMargin);
      linkChildShapes.emplace_back(std::move(boxShape));
      break;
    }
    case metadata::URDF::GEOM_SPHERE: {
      float radius = collision->m_geometry.m_sphereRadius;
      auto sphereShape = std::make_unique<btSphereShape>(radius);
      shape = sphereShape.get();
      shape->setMargin(gUrdfDefaultCollisionMargin);
      linkChildShapes.emplace_back(std::move(sphereShape));
      break;
    }
    case metadata::URDF::GEOM_MESH: {
      const std::vector<assets::CollisionMeshData>& meshGroup =
          resourceManager_.getCollisionMesh(
              collision->m_geometry.m_meshFileName);
      const assets::MeshMetaData& metaData = resourceManager_.getMeshMetaData(
          collision->m_geometry.m_meshFileName);

      auto compoundShape = std::make_unique<btCompoundShape>();
      std::vector<std::unique_ptr<btConvexHullShape>> convexShapes;
      esp::physics::BulletBase::constructConvexShapesFromMeshes(
          Magnum::Matrix4{}, meshGroup, metaData.root, compoundShape.get(),
          convexShapes);
      // move ownership of convexes
      for (auto& convex : convexShapes) {
        linkChildShapes.emplace_back(std::move(convex));
      }
      compoundShape->setLocalScaling(
          btVector3(collision->m_geometry.m_meshScale));
      compoundShape->setMargin(gUrdfDefaultCollisionMargin);
      compoundShape->recalculateLocalAabb();
      shape = compoundShape.get();
      linkChildShapes.emplace_back(std::move(compoundShape));
    } break;  // mesh case

    default:
      ESP_VERY_VERBOSE() << "E - unknown collision geometry type:"
                         << collision->m_geometry.m_type;
  }

  return shape;
}

btCompoundShape* BulletURDFImporter::convertLinkCollisionShapes(
    int urdfLinkIndex,
    const btTransform& localInertiaFrame,
    std::vector<std::unique_ptr<btCollisionShape>>& linkChildShapes) {
  // TODO: smart pointer
  btCompoundShape* compoundShape = new btCompoundShape();

  compoundShape->setMargin(gUrdfDefaultCollisionMargin);

  auto link = activeModel_->getLink(urdfLinkIndex);

  for (size_t v = 0; v < link->m_collisionArray.size(); ++v) {
    const metadata::URDF::CollisionShape& col = link->m_collisionArray[v];
    btCollisionShape* childShape =
        convertURDFToCollisionShape(&col, linkChildShapes);
    if (childShape) {
      Magnum::Matrix4 childTrans = col.m_linkLocalFrame;
      ESP_VERY_VERBOSE() << "col.m_linkLocalFrame:"
                         << Magnum::Matrix4(col.m_linkLocalFrame);

      compoundShape->addChildShape(
          localInertiaFrame.inverse() * btTransform(childTrans), childShape);
    }
  }

  return compoundShape;
}

int BulletURDFImporter::getCollisionGroupAndMask(int urdfLinkIndex,
                                                 int& colGroup,
                                                 int& colMask) const {
  int result = 0;
  std::shared_ptr<metadata::URDF::Link> link =
      activeModel_->getLink(urdfLinkIndex);
  for (size_t v = 0; v < link->m_collisionArray.size(); ++v) {
    const metadata::URDF::CollisionShape& col = link->m_collisionArray[v];
    if ((col.m_flags & metadata::URDF::HAS_COLLISION_GROUP) != 0) {
      colGroup = col.m_collisionGroup;
      result |= metadata::URDF::HAS_COLLISION_GROUP;
    }
    if ((col.m_flags & metadata::URDF::HAS_COLLISION_MASK) != 0) {
      colMask = col.m_collisionMask;
      result |= metadata::URDF::HAS_COLLISION_MASK;
    }
  }
  return result;
}

///////////////////////////////////
// Construction helpers
///////////////////////////////////

void BulletURDFImporter::computeTotalNumberOfJoints(int linkIndex) {
  std::vector<int> childIndices;
  getLinkChildIndices(linkIndex, childIndices);
  cache->m_totalNumJoints1 += childIndices.size();
  for (size_t i = 0; i < childIndices.size(); ++i) {
    int childIndex = childIndices[i];
    computeTotalNumberOfJoints(childIndex);
  }
}

void BulletURDFImporter::getAllIndices(
    int urdfLinkIndex,
    int parentIndex,
    std::vector<childParentIndex>& allIndices) {
  childParentIndex cp;
  cp.m_index = urdfLinkIndex;
  cp.m_link_name = activeModel_->getLink(urdfLinkIndex)->m_name;
  int mbIndex = cache->getMbIndexFromUrdfIndex(urdfLinkIndex);
  cp.m_mbIndex = mbIndex;
  cp.m_parentIndex = parentIndex;
  int parentMbIndex =
      parentIndex >= 0 ? cache->getMbIndexFromUrdfIndex(parentIndex) : -1;
  cp.m_parentMBIndex = parentMbIndex;

  allIndices.emplace_back(std::move(cp));
  std::vector<int> urdfChildIndices;
  getLinkChildIndices(urdfLinkIndex, urdfChildIndices);
  int numChildren = urdfChildIndices.size();
  for (int i = 0; i < numChildren; ++i) {
    int urdfChildLinkIndex = urdfChildIndices[i];
    getAllIndices(urdfChildLinkIndex, urdfLinkIndex, allIndices);
  }
}

void BulletURDFImporter::computeParentIndices(URDF2BulletCached& bulletCache,
                                              int urdfLinkIndex,
                                              int urdfParentIndex) {
  bulletCache.m_urdfLinkParentIndices[urdfLinkIndex] = urdfParentIndex;
  bulletCache.m_urdfLinkIndices2BulletLinkIndices[urdfLinkIndex] =
      bulletCache.m_currentMultiBodyLinkIndex++;

  std::vector<int> childIndices;
  getLinkChildIndices(urdfLinkIndex, childIndices);
  for (size_t i = 0; i < childIndices.size(); ++i) {
    computeParentIndices(bulletCache, childIndices[i], urdfLinkIndex);
  }
}

void BulletURDFImporter::initURDF2BulletCache() {
  // compute the number of links, and compute parent indices array (and possibly
  // other cached ?)
  cache = std::make_shared<URDF2BulletCached>();
  cache->m_totalNumJoints1 = 0;

  int rootLinkIndex = getRootLinkIndex();
  if (rootLinkIndex >= 0) {
    computeTotalNumberOfJoints(rootLinkIndex);
    int numTotalLinksIncludingBase = 1 + cache->m_totalNumJoints1;

    cache->m_urdfLinkParentIndices.resize(numTotalLinksIncludingBase);
    cache->m_urdfLinkIndices2BulletLinkIndices.resize(
        numTotalLinksIncludingBase);
    cache->m_urdfLinkLocalInertialFrames.resize(numTotalLinksIncludingBase);

    cache->m_currentMultiBodyLinkIndex =
        -1;  // multi body base has 'link' index -1

    bool maintainLinkOrder = (flags & CUF_MAINTAIN_LINK_ORDER) != 0;
    if (maintainLinkOrder) {
      URDF2BulletCached cache2 = *cache;

      computeParentIndices(cache2, rootLinkIndex, -2);

      for (int j = 0; j < numTotalLinksIncludingBase; ++j) {
        cache->m_urdfLinkParentIndices[j] = cache2.m_urdfLinkParentIndices[j];
        cache->m_urdfLinkIndices2BulletLinkIndices[j] = j - 1;
      }
    } else {
      computeParentIndices(*cache, rootLinkIndex, -2);
    }
  }
}

void processContactParameters(
    const metadata::URDF::LinkContactInfo& contactInfo,
    btCollisionObject* col) {
  if ((contactInfo.m_flags & metadata::URDF::CONTACT_HAS_LATERAL_FRICTION) !=
      0) {
    col->setFriction(contactInfo.m_lateralFriction);
  }
  if ((contactInfo.m_flags & metadata::URDF::CONTACT_HAS_RESTITUTION) != 0) {
    col->setRestitution(contactInfo.m_restitution);
  }

  if ((contactInfo.m_flags & metadata::URDF::CONTACT_HAS_ROLLING_FRICTION) !=
      0) {
    col->setRollingFriction(contactInfo.m_rollingFriction);
  }
  if ((contactInfo.m_flags & metadata::URDF::CONTACT_HAS_SPINNING_FRICTION) !=
      0) {
    col->setSpinningFriction(contactInfo.m_spinningFriction);
  }
  if ((contactInfo.m_flags & metadata::URDF::CONTACT_HAS_STIFFNESS_DAMPING) !=
      0) {
    col->setContactStiffnessAndDamping(contactInfo.m_contactStiffness,
                                       contactInfo.m_contactDamping);
  }
  if ((contactInfo.m_flags & metadata::URDF::CONTACT_HAS_FRICTION_ANCHOR) !=
      0) {
    col->setCollisionFlags(col->getCollisionFlags() |
                           btCollisionObject::CF_HAS_FRICTION_ANCHOR);
  }
}

Mn::Matrix4 BulletURDFImporter::convertURDF2BulletInternal(
    int urdfLinkIndex,
    const Mn::Matrix4& parentTransformInWorldSpace,
    btMultiBodyDynamicsWorld* world1,
    std::map<int, std::unique_ptr<btCompoundShape>>& linkCompoundShapes,
    std::map<int, std::vector<std::unique_ptr<btCollisionShape>>>&
        linkChildShapes,
    bool recursive) {
  ESP_VERY_VERBOSE() << "++++++++++++++++++++++++++++++++++++++";
  ESP_VERY_VERBOSE() << "convertURDF2BulletInternal...";
  ESP_VERY_VERBOSE() << "   recursive = " << recursive;

  Mn::Matrix4 linkTransformInWorldSpace;

  ESP_VERY_VERBOSE() << "urdfLinkIndex =" << urdfLinkIndex;

  int mbLinkIndex = cache->getMbIndexFromUrdfIndex(urdfLinkIndex);
  ESP_VERY_VERBOSE() << "mbLinkIndex =" << mbLinkIndex;

  int urdfParentIndex = cache->getParentUrdfIndex(urdfLinkIndex);
  ESP_VERY_VERBOSE() << "urdfParentIndex =" << urdfParentIndex;
  int mbParentIndex = cache->getMbIndexFromUrdfIndex(urdfParentIndex);
  ESP_VERY_VERBOSE() << "mbParentIndex =" << mbParentIndex;

  Mn::Matrix4 parentLocalInertialFrame;
  btScalar parentMass(1);
  Mn::Vector3 parentLocalInertiaDiagonal(1);

  if (urdfParentIndex == -2) {
    ESP_VERY_VERBOSE() << "root link has no parent";
  } else {
    getMassAndInertia2(urdfParentIndex, parentMass, parentLocalInertiaDiagonal,
                       parentLocalInertialFrame);
  }

  ESP_VERY_VERBOSE() << "about to get mass/inertia";

  btScalar mass = 0;
  Mn::Matrix4 localInertialFrame;
  Mn::Vector3 localInertiaDiagonal(0);
  getMassAndInertia2(urdfLinkIndex, mass, localInertiaDiagonal,
                     localInertialFrame);

  ESP_VERY_VERBOSE() << "about to get joint info";

  Mn::Matrix4 parent2joint;
  int jointType = 0;
  Mn::Vector3 jointAxisInJointSpace;
  btScalar jointLowerLimit = NAN;
  btScalar jointUpperLimit = NAN;
  btScalar jointDamping = NAN;
  btScalar jointFriction = NAN;
  btScalar jointMaxForce = NAN;
  btScalar jointMaxVelocity = NAN;

  bool hasParentJoint = getJointInfo2(
      urdfLinkIndex, parent2joint, linkTransformInWorldSpace,
      jointAxisInJointSpace, jointType, jointLowerLimit, jointUpperLimit,
      jointDamping, jointFriction, jointMaxForce, jointMaxVelocity);

  if ((flags & CUF_USE_MJCF) != 0) {
    linkTransformInWorldSpace =
        parentTransformInWorldSpace * linkTransformInWorldSpace;
  } else {
    linkTransformInWorldSpace = parentTransformInWorldSpace * parent2joint;
  }

  ESP_VERY_VERBOSE() << "about to convert link collision shapes";

  btCompoundShape* compoundShape =
      convertLinkCollisionShapes(urdfLinkIndex, btTransform(localInertialFrame),
                                 linkChildShapes[mbLinkIndex]);

  ESP_VERY_VERBOSE() << "about to deal with compoundShape";
  if (compoundShape) {
    if (mass != 0) {
      if ((flags & CUF_USE_URDF_INERTIA) == 0) {
        btVector3 btLocalIntertiaDiagonal;
        compoundShape->calculateLocalInertia(mass, btLocalIntertiaDiagonal);
        localInertiaDiagonal = Mn::Vector3(btLocalIntertiaDiagonal);
        btAssert(localInertiaDiagonal[0] < 1e10);
        btAssert(localInertiaDiagonal[1] < 1e10);
        btAssert(localInertiaDiagonal[2] < 1e10);
      }
      metadata::URDF::LinkContactInfo contactInfo;
      getLinkContactInfo(urdfLinkIndex, contactInfo);
      // temporary inertia scaling until we load inertia from URDF
      if ((contactInfo.m_flags & metadata::URDF::CONTACT_HAS_INERTIA_SCALING) !=
          0) {
        localInertiaDiagonal *= contactInfo.m_inertiaScaling;
      }
    }

    bool canSleep = (flags & CUF_ENABLE_SLEEPING) != 0;

    if (cache->m_bulletMultiBody == nullptr) {
      bool isFixedBase = (mass == 0);
      int totalNumJoints = cache->m_totalNumJoints1;
      cache->m_bulletMultiBody =
          new btMultiBody(totalNumJoints, mass, btVector3(localInertiaDiagonal),
                          isFixedBase, canSleep);
      if ((flags & CUF_GLOBAL_VELOCITIES_MB) != 0) {
        cache->m_bulletMultiBody->useGlobalVelocities(true);
      }
      if ((flags & CUF_USE_MJCF) != 0) {
        cache->m_bulletMultiBody->setBaseWorldTransform(
            btTransform(linkTransformInWorldSpace));
      }

      // registerMultiBody
      cache->m_urdfLinkLocalInertialFrames[urdfLinkIndex] =
          btTransform(localInertialFrame);
    }

    // create a joint if necessary
    // TODO: joint friction and damping set for links are not used in Bullet
    // yet. We will need to create motors to simulate these manually. See
    // btMultiBodyLink header file for details.
    if (hasParentJoint) {
      Mn::Matrix4 offsetInA, offsetInB;
      offsetInA = parentLocalInertialFrame.invertedRigid() * parent2joint;
      offsetInB = localInertialFrame.invertedRigid();
      Mn::Quaternion parentRotToThis = Mn::Quaternion::fromMatrix(
          offsetInB.rotation() * offsetInA.invertedRigid().rotation());

      bool disableParentCollision = true;

      if (cache->m_bulletMultiBody) {
        cache->m_bulletMultiBody->getLink(mbLinkIndex).m_jointDamping =
            jointDamping;
        cache->m_bulletMultiBody->getLink(mbLinkIndex).m_jointFriction =
            jointFriction;
        cache->m_bulletMultiBody->getLink(mbLinkIndex).m_jointLowerLimit =
            jointLowerLimit;
        cache->m_bulletMultiBody->getLink(mbLinkIndex).m_jointUpperLimit =
            jointUpperLimit;
        cache->m_bulletMultiBody->getLink(mbLinkIndex).m_jointMaxForce =
            jointMaxForce;
        cache->m_bulletMultiBody->getLink(mbLinkIndex).m_jointMaxVelocity =
            jointMaxVelocity;
      }

      switch (jointType) {
        case metadata::URDF::SphericalJoint: {
          // TODO: link mapping?
          // creation.addLinkMapping(urdfLinkIndex, mbLinkIndex);
          cache->m_bulletMultiBody->setupSpherical(
              mbLinkIndex, mass, btVector3(localInertiaDiagonal), mbParentIndex,
              btQuaternion(parentRotToThis), btVector3(offsetInA.translation()),
              btVector3(-offsetInB.translation()), disableParentCollision);

          break;
        }
        case metadata::URDF::PlanarJoint: {
          // TODO: link mapping?
          // creation.addLinkMapping(urdfLinkIndex, mbLinkIndex);
          cache->m_bulletMultiBody->setupPlanar(
              mbLinkIndex, mass, btVector3(localInertiaDiagonal), mbParentIndex,
              btQuaternion(parentRotToThis),
              quatRotate(btQuaternion(
                             Mn::Quaternion::fromMatrix(offsetInB.rotation())),
                         btVector3(jointAxisInJointSpace)),
              btVector3(offsetInA.translation()), disableParentCollision);
          break;
        }
        case metadata::URDF::FloatingJoint:

        case metadata::URDF::FixedJoint: {
          if ((jointType == metadata::URDF::FloatingJoint) ||
              (jointType == metadata::URDF::PlanarJoint)) {
            printf(
                "Warning: joint unsupported, creating a fixed joint instead.");
          }
          // TODO: link mapping?
          // creation.addLinkMapping(urdfLinkIndex, mbLinkIndex);

          // todo: adjust the center of mass transform and pivot axis properly
          cache->m_bulletMultiBody->setupFixed(
              mbLinkIndex, mass, btVector3(localInertiaDiagonal), mbParentIndex,
              btQuaternion(parentRotToThis), btVector3(offsetInA.translation()),
              btVector3(-offsetInB.translation()));
          break;
        }
        case metadata::URDF::ContinuousJoint:
        case metadata::URDF::RevoluteJoint: {
          // TODO: link mapping?
          // creation.addLinkMapping(urdfLinkIndex, mbLinkIndex);

          cache->m_bulletMultiBody->setupRevolute(
              mbLinkIndex, mass, btVector3(localInertiaDiagonal), mbParentIndex,
              btQuaternion(parentRotToThis),
              quatRotate(btQuaternion(
                             Mn::Quaternion::fromMatrix(offsetInB.rotation())),
                         btVector3(jointAxisInJointSpace)),
              btVector3(offsetInA.translation()),
              btVector3(-offsetInB.translation()), disableParentCollision);

          if (jointType == metadata::URDF::RevoluteJoint &&
              jointLowerLimit <= jointUpperLimit) {
            btMultiBodyConstraint* con = new btMultiBodyJointLimitConstraint(
                cache->m_bulletMultiBody, mbLinkIndex, jointLowerLimit,
                jointUpperLimit);
            world1->addMultiBodyConstraint(con);
            cache->m_jointLimitConstraints.emplace(
                mbLinkIndex,
                JointLimitConstraintInfo(mbLinkIndex, jointLowerLimit,
                                         jointUpperLimit, con));
          }

          break;
        }
        case metadata::URDF::PrismaticJoint: {
          // TODO: link mapping?
          // creation.addLinkMapping(urdfLinkIndex, mbLinkIndex);

          cache->m_bulletMultiBody->setupPrismatic(
              mbLinkIndex, mass, btVector3(localInertiaDiagonal), mbParentIndex,
              btQuaternion(parentRotToThis),
              quatRotate(btQuaternion(
                             Mn::Quaternion::fromMatrix(offsetInB.rotation())),
                         btVector3(jointAxisInJointSpace)),
              btVector3(offsetInA.translation()),  // parent2joint.getOrigin(),
              btVector3(-offsetInB.translation()), disableParentCollision);

          if (jointLowerLimit <= jointUpperLimit) {
            btMultiBodyConstraint* con = new btMultiBodyJointLimitConstraint(
                cache->m_bulletMultiBody, mbLinkIndex, jointLowerLimit,
                jointUpperLimit);
            world1->addMultiBodyConstraint(con);
            cache->m_jointLimitConstraints.emplace(
                mbLinkIndex,
                JointLimitConstraintInfo(mbLinkIndex, jointLowerLimit,
                                         jointUpperLimit, con));
          }

          break;
        }
        default: {
          ESP_VERY_VERBOSE() << "Invalid joint type." btAssert(0);
        }
      }
    }

    {
      btMultiBodyLinkCollider* col =
          new btMultiBodyLinkCollider(cache->m_bulletMultiBody, mbLinkIndex);

      col->setCollisionShape(compoundShape);

      // TODO: better track the collision shapes
      linkCompoundShapes[mbLinkIndex].reset(compoundShape);

      Mn::Matrix4 tr = linkTransformInWorldSpace;
      // if we don't set the initial pose of the btCollisionObject, the
      // simulator will do this when syncing the btMultiBody link transforms to
      // the btMultiBodyLinkCollider

      ESP_VERY_VERBOSE()
          << "~~~~~~~~~~~~~ col->setWorldTransform(btTransform(tr)):" << tr;

      col->setWorldTransform(btTransform(tr));

      // base and fixed? -> static, otherwise flag as dynamic
      bool isDynamic =
          !(mbLinkIndex < 0 && cache->m_bulletMultiBody->hasFixedBase());
      metadata::URDF::LinkContactInfo contactInfo;
      getLinkContactInfo(urdfLinkIndex, contactInfo);

      processContactParameters(contactInfo, col);

      if (mbLinkIndex >= 0)  //???? double-check +/- 1
      {
        // if the base is static and all joints in the chain between this link
        // and the base are fixed, then this link is static too (doesn't merge
        // islands)
        if (cache->m_bulletMultiBody->getBaseMass() == 0) {
          bool allJointsFixed = true;
          int testLinkIndex = mbLinkIndex;
          do {
            if (cache->m_bulletMultiBody->getLink(testLinkIndex).m_jointType !=
                btMultibodyLink::eFixed) {
              allJointsFixed = false;
              break;
            }
            testLinkIndex =
                cache->m_bulletMultiBody->getLink(testLinkIndex).m_parent;
          } while (testLinkIndex > 0);
          if (allJointsFixed) {
            col->setCollisionFlags(col->getCollisionFlags() |
                                   btCollisionObject::CF_STATIC_OBJECT);
            isDynamic = false;
          }
        }
        cache->m_bulletMultiBody->getLink(mbLinkIndex).m_collider = col;
        if ((flags & CUF_USE_SELF_COLLISION_INCLUDE_PARENT) != 0) {
          cache->m_bulletMultiBody->getLink(mbLinkIndex).m_flags &=
              ~BT_MULTIBODYLINKFLAGS_DISABLE_PARENT_COLLISION;
        }
        if ((flags & CUF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS) != 0) {
          cache->m_bulletMultiBody->getLink(mbLinkIndex).m_flags |=
              BT_MULTIBODYLINKFLAGS_DISABLE_ALL_PARENT_COLLISION;
        }
      } else {
        {
          if (cache->m_bulletMultiBody->getBaseMass() == 0) {
            col->setCollisionFlags(col->getCollisionFlags() |
                                   btCollisionObject::CF_STATIC_OBJECT);
            isDynamic = false;
          }
        }

        cache->m_bulletMultiBody->setBaseCollider(col);
      }

      CORRADE_ASSERT(
          isDynamic != col->isStaticOrKinematicObject(),
          "Static or Kinematic object erroneously marked dynamic. This "
          "should not happen.",
          {});

      // By convention, fixed links should be authored in the URDF as
      // Noncollidable. Then we will create fixed rigid bodies, separate from
      // the multibody, which will be collidable (CollisionGroup::Static) (see
      // BulletArticulatedObject.cpp :: "initializeFromURDF()").
      int collisionFilterGroup = int(CollisionGroup::Robot);

      int colGroup = 0, colMask = 0;
      int collisionFlags =
          getCollisionGroupAndMask(urdfLinkIndex, colGroup, colMask);

      if ((collisionFlags & metadata::URDF::HAS_COLLISION_GROUP) != 0) {
        collisionFilterGroup = colGroup;
      }

      int collisionFilterMask = uint32_t(CollisionGroupHelper::getMaskForGroup(
          CollisionGroup(collisionFilterGroup)));
// We don't like overriding the mask in the URDF; we disable support for this.
// We prefer to only override the group, while still using getMaskForGroup
// (above) for mask computation.
#if 0
      if (collisionFlags & metadata::URDF::HAS_COLLISION_MASK) {
        collisionFilterMask = colMask;
      }
#endif

      world1->addCollisionObject(col, collisionFilterGroup,
                                 collisionFilterMask);

      const auto debugModel = getModel();
      std::string linkDebugName = "URDF, " + debugModel->m_name + ", link " +
                                  debugModel->getLink(urdfLinkIndex)->m_name;
      BulletCollisionHelper::get().mapCollisionObjectTo(col, linkDebugName);
    }
  }

  if (recursive) {
    ESP_VERY_VERBOSE() << "about to recurse";

    std::vector<int> urdfChildIndices;
    getLinkChildIndices(urdfLinkIndex, urdfChildIndices);

    int numChildren = urdfChildIndices.size();

    for (int i = 0; i < numChildren; ++i) {
      int urdfChildLinkIndex = urdfChildIndices[i];

      convertURDF2BulletInternal(urdfChildLinkIndex, linkTransformInWorldSpace,
                                 world1, linkCompoundShapes, linkChildShapes,
                                 recursive);
    }
  }
  return linkTransformInWorldSpace;
}

}  // namespace physics
}  // namespace esp
