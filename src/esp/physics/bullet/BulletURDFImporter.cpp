// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

// Code adapted from Bullet3/examples/Importers/ImportURDFDemo ...
#include <iostream>

#include <Corrade/Utility/DebugStl.h>
#include <Corrade/Utility/Directory.h>
#include <Magnum/BulletIntegration/Integration.h>
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletDebugManager.h"
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
    const io::URDF::CollisionShape* collision) {
  Mn::Debug silence{logMessages ? &std::cout : nullptr};
  Mn::Debug{} << "convertURDFToCollisionShape";

  btCollisionShape* shape = 0;

  switch (collision->m_geometry.m_type) {
    case io::URDF::GEOM_PLANE: {
      Mn::Vector3 planeNormal = collision->m_geometry.m_planeNormal;
      float planeConstant = 0;  // not available?
      btStaticPlaneShape* plane =
          new btStaticPlaneShape(btVector3(planeNormal), planeConstant);
      shape = plane;
      shape->setMargin(gUrdfDefaultCollisionMargin);
      break;
    }
    case io::URDF::GEOM_CAPSULE: {
      float radius = collision->m_geometry.m_capsuleRadius;
      float height = collision->m_geometry.m_capsuleHeight;
      btCapsuleShapeZ* capsuleShape = new btCapsuleShapeZ(radius, height);
      shape = capsuleShape;
      shape->setMargin(gUrdfDefaultCollisionMargin);
      break;
    }

    case io::URDF::GEOM_CYLINDER: {
      float cylRadius = collision->m_geometry.m_capsuleRadius;
      float cylHalfLength = 0.5 * collision->m_geometry.m_capsuleHeight;
      // if (m_data->m_flags & CUF_USE_IMPLICIT_CYLINDER) TODO: why not? Should
      // always be better.

      btVector3 halfExtents(cylRadius, cylRadius, cylHalfLength);
      btCylinderShapeZ* cylZShape = new btCylinderShapeZ(halfExtents);
      shape = cylZShape;

      break;
    }
    case io::URDF::GEOM_BOX: {
      btVector3 extents = btVector3(collision->m_geometry.m_boxSize);
      btBoxShape* boxShape = new btBoxShape(extents * 0.5f);
      // TODO: off for now, but may be necessary for better contacts?
      /*
      if (m_data->m_flags & CUF_INITIALIZE_SAT_FEATURES)
                      {
                              boxShape->initializePolyhedralFeatures();
                      }
      */
      shape = boxShape;
      shape->setMargin(gUrdfDefaultCollisionMargin);
      break;
    }
    case io::URDF::GEOM_SPHERE: {
      float radius = collision->m_geometry.m_sphereRadius;
      btSphereShape* sphereShape = new btSphereShape(radius);
      shape = sphereShape;
      shape->setMargin(gUrdfDefaultCollisionMargin);
      break;
    }
    case io::URDF::GEOM_MESH: {
      // TODO: implement this from resourceManager_ structures. Share with
      // RigidBody interface (pull that out?)
      bool meshSuccess =
          resourceManager_.importAsset(collision->m_geometry.m_meshFileName);

      if (meshSuccess) {
        const std::vector<assets::CollisionMeshData>& meshGroup =
            resourceManager_.getCollisionMesh(
                collision->m_geometry.m_meshFileName);
        const assets::MeshMetaData& metaData = resourceManager_.getMeshMetaData(
            collision->m_geometry.m_meshFileName);

        auto convexShape = new btConvexHullShape();
        esp::physics::BulletBase::constructJoinedConvexShapeFromMeshes(
            Magnum::Matrix4{}, meshGroup, metaData.root, convexShape);
        convexShape->setLocalScaling(
            btVector3(collision->m_geometry.m_meshScale));
        convexShape->recalcLocalAabb();
        shape = convexShape;
        shape->setMargin(gUrdfDefaultCollisionMargin);
      } else {
        Mn::Debug{}
            << "BulletURDFImporter::convertURDFToCollisionShape : E - could "
               "not load collision mesh \""
            << collision->m_geometry.m_meshFileName << "\"";
      }
      break;
    }  // mesh case

    default:
      Mn::Debug{} << "E - unknown collision geometry type: "
                  << collision->m_geometry.m_type;
  }
  // TODO: need this to store collision mesh info? May be OK to create another
  // structure for that explicitly.
  /*
        if (shape && collision->m_geometry.m_type == URDF_GEOM_MESH)
        {
                m_data->m_bulletCollisionShape2UrdfCollision.insert(shape,
     *collision);
        }
   */
  return shape;
}

btCompoundShape* BulletURDFImporter::convertLinkCollisionShapes(
    int linkIndex,
    const btTransform& localInertiaFrame) {
  Mn::Debug silence{logMessages ? &std::cout : nullptr};
  // TODO: smart pointer
  btCompoundShape* compoundShape = new btCompoundShape();
  // TODO: store this somewhere
  // allocatedCollisionShapes_.push_back(compoundShape);

  compoundShape->setMargin(gUrdfDefaultCollisionMargin);

  Mn::Debug{} << " num links = " << activeModel_->m_links.size();

  auto itr = activeModel_->m_links.begin();
  for (int i = 0; (i < linkIndex && itr != activeModel_->m_links.end()); i++) {
    itr++;
  }
  if (itr != activeModel_->m_links.end()) {
    std::shared_ptr<io::URDF::Link> link = itr->second;

    for (size_t v = 0; v < link->m_collisionArray.size(); ++v) {
      const io::URDF::CollisionShape& col = link->m_collisionArray[v];
      btCollisionShape* childShape = convertURDFToCollisionShape(&col);
      if (childShape) {
        /*
        allocatedCollisionShapes_.push_back(childShape);
        if (childShape->getShapeType() == COMPOUND_SHAPE_PROXYTYPE) {
          btCompoundShape* compound = (btCompoundShape*)childShape;
          for (int i = 0; i < compound->getNumChildShapes(); i++) {
            allocatedCollisionShapes_.push_back(compound->getChildShape(i));
          }
        }
        */

        Magnum::Matrix4 childTrans = col.m_linkLocalFrame;
        Mn::Debug{} << "col.m_linkLocalFrame: "
                    << Magnum::Matrix4(col.m_linkLocalFrame);

        compoundShape->addChildShape(
            localInertiaFrame.inverse() * btTransform(childTrans), childShape);
      }
    }
  } else {
    Mn::Debug{} << "E - No link: " << linkIndex;
  }

  return compoundShape;
}

int BulletURDFImporter::getCollisionGroupAndMask(int linkIndex,
                                                 int& colGroup,
                                                 int& colMask) const {
  int result = 0;
  auto itr = activeModel_->m_links.begin();
  for (int i = 0; (i < linkIndex && itr != activeModel_->m_links.end()); i++) {
    itr++;
  }
  if (itr != activeModel_->m_links.end()) {
    std::shared_ptr<io::URDF::Link> link = itr->second;
    for (size_t v = 0; v < link->m_collisionArray.size(); ++v) {
      const io::URDF::CollisionShape& col = link->m_collisionArray[v];
      if (col.m_flags & io::URDF::HAS_COLLISION_GROUP) {
        colGroup = col.m_collisionGroup;
        result |= io::URDF::HAS_COLLISION_GROUP;
      }
      if (col.m_flags & io::URDF::HAS_COLLISION_MASK) {
        colMask = col.m_collisionMask;
        result |= io::URDF::HAS_COLLISION_MASK;
      }
    }
  }
  return result;
}

///////////////////////////////////
// Construction helpers
///////////////////////////////////

void BulletURDFImporter::ComputeTotalNumberOfJoints(URDF2BulletCached& cache,
                                                    int linkIndex) {
  std::vector<int> childIndices;
  getLinkChildIndices(linkIndex, childIndices);
  cache.m_totalNumJoints1 += childIndices.size();
  for (size_t i = 0; i < childIndices.size(); i++) {
    int childIndex = childIndices[i];
    ComputeTotalNumberOfJoints(cache, childIndex);
  }
}

void BulletURDFImporter::ComputeParentIndices(URDF2BulletCached& cache,
                                              int urdfLinkIndex,
                                              int urdfParentIndex) {
  cache.m_urdfLinkParentIndices[urdfLinkIndex] = urdfParentIndex;
  cache.m_urdfLinkIndices2BulletLinkIndices[urdfLinkIndex] =
      cache.m_currentMultiBodyLinkIndex++;

  std::vector<int> childIndices;
  getLinkChildIndices(urdfLinkIndex, childIndices);
  for (size_t i = 0; i < childIndices.size(); i++) {
    ComputeParentIndices(cache, childIndices[i], urdfLinkIndex);
  }
}

void BulletURDFImporter::InitURDF2BulletCache(URDF2BulletCached& cache,
                                              int flags) {
  // compute the number of links, and compute parent indices array (and possibly
  // other cached ?)
  cache.m_totalNumJoints1 = 0;

  int rootLinkIndex = getRootLinkIndex();
  if (rootLinkIndex >= 0) {
    ComputeTotalNumberOfJoints(cache, rootLinkIndex);
    int numTotalLinksIncludingBase = 1 + cache.m_totalNumJoints1;

    cache.m_urdfLinkParentIndices.resize(numTotalLinksIncludingBase);
    cache.m_urdfLinkIndices2BulletLinkIndices.resize(
        numTotalLinksIncludingBase);
    cache.m_urdfLinkLocalInertialFrames.resize(numTotalLinksIncludingBase);

    cache.m_currentMultiBodyLinkIndex =
        -1;  // multi body base has 'link' index -1

    bool maintainLinkOrder = (flags & CUF_MAINTAIN_LINK_ORDER) != 0;
    if (maintainLinkOrder) {
      URDF2BulletCached cache2 = cache;

      ComputeParentIndices(cache2, rootLinkIndex, -2);

      for (int j = 0; j < numTotalLinksIncludingBase; j++) {
        cache.m_urdfLinkParentIndices[j] = cache2.m_urdfLinkParentIndices[j];
        cache.m_urdfLinkIndices2BulletLinkIndices[j] = j - 1;
      }
    } else {
      ComputeParentIndices(cache, rootLinkIndex, -2);
    }
  }
}

void processContactParameters(const io::URDF::LinkContactInfo& contactInfo,
                              btCollisionObject* col) {
  if ((contactInfo.m_flags & io::URDF::CONTACT_HAS_LATERAL_FRICTION) != 0) {
    col->setFriction(contactInfo.m_lateralFriction);
  }
  if ((contactInfo.m_flags & io::URDF::CONTACT_HAS_RESTITUTION) != 0) {
    col->setRestitution(contactInfo.m_restitution);
  }

  if ((contactInfo.m_flags & io::URDF::CONTACT_HAS_ROLLING_FRICTION) != 0) {
    col->setRollingFriction(contactInfo.m_rollingFriction);
  }
  if ((contactInfo.m_flags & io::URDF::CONTACT_HAS_SPINNING_FRICTION) != 0) {
    col->setSpinningFriction(contactInfo.m_spinningFriction);
  }
  if ((contactInfo.m_flags & io::URDF::CONTACT_HAS_STIFFNESS_DAMPING) != 0) {
    col->setContactStiffnessAndDamping(contactInfo.m_contactStiffness,
                                       contactInfo.m_contactDamping);
  }
  if ((contactInfo.m_flags & io::URDF::CONTACT_HAS_FRICTION_ANCHOR) != 0) {
    col->setCollisionFlags(col->getCollisionFlags() |
                           btCollisionObject::CF_HAS_FRICTION_ANCHOR);
  }
}

Mn::Matrix4 BulletURDFImporter::ConvertURDF2BulletInternal(
    URDF2BulletCached& cache,
    int urdfLinkIndex,
    const Mn::Matrix4& parentTransformInWorldSpace,
    btMultiBodyDynamicsWorld* world1,
    int flags,
    std::map<int, std::unique_ptr<btCollisionShape>>& linkCollisionShapes) {
  Mn::Debug silence{logMessages ? &std::cout : nullptr};
  Mn::Debug{} << "++++++++++++++++++++++++++++++++++++++";
  Mn::Debug{} << "ConvertURDF2BulletInternal...";

  Mn::Matrix4 linkTransformInWorldSpace;

  Mn::Debug{} << "  urdfLinkIndex = " << urdfLinkIndex;

  int mbLinkIndex = cache.getMbIndexFromUrdfIndex(urdfLinkIndex);
  Mn::Debug{} << "  mbLinkIndex = " << mbLinkIndex;

  int urdfParentIndex = cache.getParentUrdfIndex(urdfLinkIndex);
  Mn::Debug{} << "  urdfParentIndex = " << urdfParentIndex;
  int mbParentIndex = cache.getMbIndexFromUrdfIndex(urdfParentIndex);
  Mn::Debug{} << "  mbParentIndex = " << mbParentIndex;

  Mn::Matrix4 parentLocalInertialFrame;
  btScalar parentMass(1);
  Mn::Vector3 parentLocalInertiaDiagonal(1);

  if (urdfParentIndex == -2) {
    Mn::Debug{} << "root link has no parent";
  } else {
    getMassAndInertia2(urdfParentIndex, parentMass, parentLocalInertiaDiagonal,
                       parentLocalInertialFrame, flags);
  }

  Mn::Debug{} << "  about to get mass/inertia";

  btScalar mass = 0;
  Mn::Matrix4 localInertialFrame;
  Mn::Vector3 localInertiaDiagonal(0);
  getMassAndInertia2(urdfLinkIndex, mass, localInertiaDiagonal,
                     localInertialFrame, flags);

  Mn::Debug{} << "  about to get joint info";

  Mn::Matrix4 parent2joint;
  int jointType;
  Mn::Vector3 jointAxisInJointSpace;
  btScalar jointLowerLimit;
  btScalar jointUpperLimit;
  btScalar jointDamping;
  btScalar jointFriction;
  btScalar jointMaxForce;
  btScalar jointMaxVelocity;

  bool hasParentJoint = getJointInfo2(
      urdfLinkIndex, parent2joint, linkTransformInWorldSpace,
      jointAxisInJointSpace, jointType, jointLowerLimit, jointUpperLimit,
      jointDamping, jointFriction, jointMaxForce, jointMaxVelocity);

  if (flags & CUF_USE_MJCF) {
    linkTransformInWorldSpace =
        parentTransformInWorldSpace * linkTransformInWorldSpace;
  } else {
    linkTransformInWorldSpace = parentTransformInWorldSpace * parent2joint;
  }

  Mn::Debug{} << "  about to convert link collision shapes";

  btCompoundShape* tmpShape = convertLinkCollisionShapes(
      urdfLinkIndex, btTransform(localInertialFrame));
  btCollisionShape* compoundShape = tmpShape;
  if (tmpShape->getNumChildShapes() == 1 &&
      tmpShape->getChildTransform(0) == btTransform::getIdentity()) {
    compoundShape = tmpShape->getChildShape(0);
  }

  Mn::Debug{} << "  about to deal with compoundShape";
  if (compoundShape) {
    if (mass) {
      if (!(flags & CUF_USE_URDF_INERTIA)) {
        btVector3 btLocalIntertiaDiagonal;
        compoundShape->calculateLocalInertia(mass, btLocalIntertiaDiagonal);
        localInertiaDiagonal = Mn::Vector3(btLocalIntertiaDiagonal);
        btAssert(localInertiaDiagonal[0] < 1e10);
        btAssert(localInertiaDiagonal[1] < 1e10);
        btAssert(localInertiaDiagonal[2] < 1e10);
      }
      io::URDF::LinkContactInfo contactInfo;
      getLinkContactInfo(urdfLinkIndex, contactInfo);
      // temporary inertia scaling until we load inertia from URDF
      if (contactInfo.m_flags & io::URDF::CONTACT_HAS_INERTIA_SCALING) {
        localInertiaDiagonal *= contactInfo.m_inertiaScaling;
      }
    }

    Mn::Matrix4 inertialFrameInWorldSpace =
        linkTransformInWorldSpace * localInertialFrame;
    bool canSleep = (flags & CUF_ENABLE_SLEEPING) != 0;

    if (cache.m_bulletMultiBody == 0) {
      bool isFixedBase = (mass == 0);
      int totalNumJoints = cache.m_totalNumJoints1;
      cache.m_bulletMultiBody =
          new btMultiBody(totalNumJoints, mass, btVector3(localInertiaDiagonal),
                          isFixedBase, canSleep);
      if (flags & CUF_GLOBAL_VELOCITIES_MB) {
        cache.m_bulletMultiBody->useGlobalVelocities(true);
      }
      if (flags & CUF_USE_MJCF) {
        cache.m_bulletMultiBody->setBaseWorldTransform(
            btTransform(linkTransformInWorldSpace));
      }

      cache.registerMultiBody(urdfLinkIndex, cache.m_bulletMultiBody,
                              btTransform(inertialFrameInWorldSpace), mass,
                              btVector3(localInertiaDiagonal), compoundShape,
                              btTransform(localInertialFrame));
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

      if (cache.m_bulletMultiBody) {
        cache.m_bulletMultiBody->getLink(mbLinkIndex).m_jointDamping =
            jointDamping;
        cache.m_bulletMultiBody->getLink(mbLinkIndex).m_jointFriction =
            jointFriction;
        cache.m_bulletMultiBody->getLink(mbLinkIndex).m_jointLowerLimit =
            jointLowerLimit;
        cache.m_bulletMultiBody->getLink(mbLinkIndex).m_jointUpperLimit =
            jointUpperLimit;
        cache.m_bulletMultiBody->getLink(mbLinkIndex).m_jointMaxForce =
            jointMaxForce;
        cache.m_bulletMultiBody->getLink(mbLinkIndex).m_jointMaxVelocity =
            jointMaxVelocity;
      }

      switch (jointType) {
        case io::URDF::SphericalJoint: {
          // TODO: link mapping?
          // creation.addLinkMapping(urdfLinkIndex, mbLinkIndex);
          cache.m_bulletMultiBody->setupSpherical(
              mbLinkIndex, mass, btVector3(localInertiaDiagonal), mbParentIndex,
              btQuaternion(parentRotToThis), btVector3(offsetInA.translation()),
              btVector3(-offsetInB.translation()), disableParentCollision);

          break;
        }
        case io::URDF::PlanarJoint: {
          // TODO: link mapping?
          // creation.addLinkMapping(urdfLinkIndex, mbLinkIndex);
          cache.m_bulletMultiBody->setupPlanar(
              mbLinkIndex, mass, btVector3(localInertiaDiagonal), mbParentIndex,
              btQuaternion(parentRotToThis),
              quatRotate(btQuaternion(
                             Mn::Quaternion::fromMatrix(offsetInB.rotation())),
                         btVector3(jointAxisInJointSpace)),
              btVector3(offsetInA.translation()), disableParentCollision);
          break;
        }
        case io::URDF::FloatingJoint:

        case io::URDF::FixedJoint: {
          if ((jointType == io::URDF::FloatingJoint) ||
              (jointType == io::URDF::PlanarJoint)) {
            printf(
                "Warning: joint unsupported, creating a fixed joint instead.");
          }
          // TODO: link mapping?
          // creation.addLinkMapping(urdfLinkIndex, mbLinkIndex);

          // todo: adjust the center of mass transform and pivot axis properly
          cache.m_bulletMultiBody->setupFixed(
              mbLinkIndex, mass, btVector3(localInertiaDiagonal), mbParentIndex,
              btQuaternion(parentRotToThis), btVector3(offsetInA.translation()),
              btVector3(-offsetInB.translation()));
          break;
        }
        case io::URDF::ContinuousJoint:
        case io::URDF::RevoluteJoint: {
          // TODO: link mapping?
          // creation.addLinkMapping(urdfLinkIndex, mbLinkIndex);

          cache.m_bulletMultiBody->setupRevolute(
              mbLinkIndex, mass, btVector3(localInertiaDiagonal), mbParentIndex,
              btQuaternion(parentRotToThis),
              quatRotate(btQuaternion(
                             Mn::Quaternion::fromMatrix(offsetInB.rotation())),
                         btVector3(jointAxisInJointSpace)),
              btVector3(offsetInA.translation()),
              btVector3(-offsetInB.translation()), disableParentCollision);

          if (jointType == io::URDF::RevoluteJoint &&
              jointLowerLimit <= jointUpperLimit) {
            btMultiBodyConstraint* con = new btMultiBodyJointLimitConstraint(
                cache.m_bulletMultiBody, mbLinkIndex, jointLowerLimit,
                jointUpperLimit);
            world1->addMultiBodyConstraint(con);
            cache.m_jointLimitConstraints.emplace(
                mbLinkIndex,
                JointLimitConstraintInfo(mbLinkIndex, jointLowerLimit,
                                         jointUpperLimit, con));
          }

          break;
        }
        case io::URDF::PrismaticJoint: {
          // TODO: link mapping?
          // creation.addLinkMapping(urdfLinkIndex, mbLinkIndex);

          cache.m_bulletMultiBody->setupPrismatic(
              mbLinkIndex, mass, btVector3(localInertiaDiagonal), mbParentIndex,
              btQuaternion(parentRotToThis),
              quatRotate(btQuaternion(
                             Mn::Quaternion::fromMatrix(offsetInB.rotation())),
                         btVector3(jointAxisInJointSpace)),
              btVector3(offsetInA.translation()),  // parent2joint.getOrigin(),
              btVector3(-offsetInB.translation()), disableParentCollision);

          if (jointLowerLimit <= jointUpperLimit) {
            btMultiBodyConstraint* con = new btMultiBodyJointLimitConstraint(
                cache.m_bulletMultiBody, mbLinkIndex, jointLowerLimit,
                jointUpperLimit);
            world1->addMultiBodyConstraint(con);
            cache.m_jointLimitConstraints.emplace(
                mbLinkIndex,
                JointLimitConstraintInfo(mbLinkIndex, jointLowerLimit,
                                         jointUpperLimit, con));
          }

          break;
        }
        default: {
          Mn::Debug{} << "Invalid joint type." btAssert(0);
        }
      }
    }

    // if (compoundShape->getNumChildShapes()>0)
    {
      // btMultiBodyLinkCollider* col =
      // creation.allocateMultiBodyLinkCollider(urdfLinkIndex, mbLinkIndex,
      // cache.m_bulletMultiBody);
      btMultiBodyLinkCollider* col =
          new btMultiBodyLinkCollider(cache.m_bulletMultiBody, mbLinkIndex);

      // compoundShape->setUserIndex(graphicsIndex);

      col->setCollisionShape(compoundShape);

      // TODO: better track the collision shapes
      linkCollisionShapes[mbLinkIndex].reset(compoundShape);

      if (compoundShape->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE) {
        btBvhTriangleMeshShape* trimeshShape =
            static_cast<btBvhTriangleMeshShape*>(compoundShape);
        if (trimeshShape->getTriangleInfoMap()) {
          col->setCollisionFlags(
              col->getCollisionFlags() |
              btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
        }
      }

      Mn::Matrix4 tr = linkTransformInWorldSpace;
      // if we don't set the initial pose of the btCollisionObject, the
      // simulator will do this when syncing the btMultiBody link transforms to
      // the btMultiBodyLinkCollider

      Mn::Debug{} << "~~~~~~~~~~~~~ col->setWorldTransform(btTransform(tr)): "
                  << tr;

      col->setWorldTransform(btTransform(tr));

      // base and fixed? -> static, otherwise flag as dynamic
      bool isDynamic =
          (mbLinkIndex < 0 && cache.m_bulletMultiBody->hasFixedBase()) ? false
                                                                       : true;
      io::URDF::LinkContactInfo contactInfo;
      getLinkContactInfo(urdfLinkIndex, contactInfo);

      processContactParameters(contactInfo, col);

      if (mbLinkIndex >= 0)  //???? double-check +/- 1
      {
        // if the base is static and all joints in the chain between this link
        // and the base are fixed, then this link is static too (doesn't merge
        // islands)
        if (cache.m_bulletMultiBody->getBaseMass() == 0) {
          bool allJointsFixed = true;
          int testLinkIndex = mbLinkIndex;
          do {
            if (cache.m_bulletMultiBody->getLink(testLinkIndex).m_jointType !=
                btMultibodyLink::eFixed) {
              allJointsFixed = false;
              break;
            }
            testLinkIndex =
                cache.m_bulletMultiBody->getLink(testLinkIndex).m_parent;
          } while (testLinkIndex > 0);
          if (allJointsFixed) {
            col->setCollisionFlags(col->getCollisionFlags() |
                                   btCollisionObject::CF_STATIC_OBJECT);
            isDynamic = false;
          }
        }
        cache.m_bulletMultiBody->getLink(mbLinkIndex).m_collider = col;
        if (flags & CUF_USE_SELF_COLLISION_INCLUDE_PARENT) {
          cache.m_bulletMultiBody->getLink(mbLinkIndex).m_flags &=
              ~BT_MULTIBODYLINKFLAGS_DISABLE_PARENT_COLLISION;
        }
        if (flags & CUF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS) {
          cache.m_bulletMultiBody->getLink(mbLinkIndex).m_flags |=
              BT_MULTIBODYLINKFLAGS_DISABLE_ALL_PARENT_COLLISION;
        }
      } else {
        //					if (canSleep)
        {
          if (cache.m_bulletMultiBody->getBaseMass() == 0)
          //&& cache.m_bulletMultiBody->getNumDofs()==0)
          {
            // col->setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT);
            col->setCollisionFlags(col->getCollisionFlags() |
                                   btCollisionObject::CF_STATIC_OBJECT);
            isDynamic = false;
          }
        }

        cache.m_bulletMultiBody->setBaseCollider(col);
      }

      ASSERT(isDynamic != col->isStaticOrKinematicObject());

      // This group-selection logic isn't very useful. We can't distinguish
      // between articulated objects here, e.g. cabinets versus robots. Users
      // will generally have to override the group in their URDFs (see
      // getCollisionGroupAndMask below). Note use of Noncollidable. By
      // convention, fixed links should be authored in the URDF as
      // Noncollidable. Then, we will create fixed rigid bodies, separate from
      // the multibody, which will be collidable (CollisionGroup::Static) (see
      // BulletArticulatedObject.cpp).
      int collisionFilterGroup = isDynamic ? int(CollisionGroup::Robot)
                                           : int(CollisionGroup::Noncollidable);

      int colGroup = 0, colMask = 0;
      int collisionFlags =
          getCollisionGroupAndMask(urdfLinkIndex, colGroup, colMask);

      if (collisionFlags & io::URDF::HAS_COLLISION_GROUP) {
        collisionFilterGroup = colGroup;
      }

      int collisionFilterMask = CollisionGroupHelper::getMaskForGroup(
          CollisionGroup(collisionFilterGroup));
// We don't like overriding the mask in the URDF; we disable support for this.
// We prefer to only override the group, while still using getMaskForGroup
// (above) for mask computation.
#if 0
      if (collisionFlags & io::URDF::HAS_COLLISION_MASK) {
        collisionFilterMask = colMask;
      }
#endif

      // Mn::Debug{}
      //    << "addCollisionObject: " << collisionFilterGroup << " , "
      //    << collisionFilterMask;
      world1->addCollisionObject(col, collisionFilterGroup,
                                 collisionFilterMask);
      // world1->addCollisionObject(col, 2, 1+2);
      // TODO: fix this collision issue

      // TODO: include the articulated object id here
      const auto& debugModel = getModel();
      std::string linkDebugName = "URDF, " + debugModel->m_name + ", link " +
                                  debugModel->getLink(urdfLinkIndex)->m_name;
      BulletDebugManager::get().mapCollisionObjectTo(col, linkDebugName);
    }
  }

  Mn::Debug{} << "  about to recurse";

  std::vector<int> urdfChildIndices;
  getLinkChildIndices(urdfLinkIndex, urdfChildIndices);

  int numChildren = urdfChildIndices.size();

  for (int i = 0; i < numChildren; i++) {
    int urdfChildLinkIndex = urdfChildIndices[i];

    ConvertURDF2BulletInternal(cache, urdfChildLinkIndex,
                               linkTransformInWorldSpace, world1, flags,
                               linkCollisionShapes);
  }
  return linkTransformInWorldSpace;
}

}  // namespace physics
}  // namespace esp
