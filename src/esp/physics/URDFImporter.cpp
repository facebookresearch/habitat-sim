// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

// Code adapted from Bullet3/examples/Importers/ImportURDFDemo ...

#include "URDFImporter.h"
#include <Corrade/Utility/DebugStl.h>
#include <Corrade/Utility/Directory.h>

namespace Mn = Magnum;

namespace esp {
namespace physics {

bool URDFImporter::loadURDF(const std::string& filename) {
  if (!Corrade::Utility::Directory::exists(filename) ||
      Corrade::Utility::Directory::isDirectory(filename)) {
    //Corrade::Utility::Debug()
    //    << "File does not exist: " << filename << ". Aborting URDF parse/load.";
    return false;
  }

  bool success = urdfParser_.parseURDF(filename);

  //Corrade::Utility::Debug() << "Done loading";

  getModel().printKinematicChain();

  return success;
}

int URDFImporter::getRootLinkIndex() const {
  if (urdfParser_.getModel().m_rootLinks.size() == 1) {
    return urdfParser_.getModel().m_rootLinks[0]->m_linkIndex;
  }
  return ID_UNDEFINED;
}

void URDFImporter::getLinkChildIndices(
    int linkIndex,
    std::vector<int>& childLinkIndices) const {
  childLinkIndices.resize(0);
  auto link = urdfParser_.getModel().getLink(linkIndex);

  if (link != nullptr) {
    for (size_t i = 0; i < link->m_childLinks.size(); i++) {
      int childIndex = link->m_childLinks[i]->m_linkIndex;
      childLinkIndices.push_back(childIndex);
    }
  }
}

bool URDFImporter::getJointInfo2(int linkIndex,
                                 Mn::Matrix4& parent2joint,
                                 Mn::Matrix4& linkTransformInWorld,
                                 Mn::Vector3& jointAxisInJointSpace,
                                 int& jointType,
                                 float& jointLowerLimit,
                                 float& jointUpperLimit,
                                 float& jointDamping,
                                 float& jointFriction,
                                 float& jointMaxForce,
                                 float& jointMaxVelocity) const {
  jointLowerLimit = 0.f;
  jointUpperLimit = 0.f;
  jointDamping = 0.f;
  jointFriction = 0.f;
  jointMaxForce = 0.f;
  jointMaxVelocity = 0.f;

  auto link = urdfParser_.getModel().getLink(linkIndex);
  if (link != nullptr) {
    linkTransformInWorld = link->m_linkTransformInWorld;

    if (link->m_parentJoint) {
      auto& pj = link->m_parentJoint;
      parent2joint = pj->m_parentLinkToJointTransform;
      jointType = pj->m_type;
      jointAxisInJointSpace = pj->m_localJointAxis;
      jointLowerLimit = pj->m_lowerLimit;
      jointUpperLimit = pj->m_upperLimit;
      jointDamping = pj->m_jointDamping;
      jointFriction = pj->m_jointFriction;
      jointMaxForce = pj->m_effortLimit;
      jointMaxVelocity = pj->m_velocityLimit;
      //Corrade::Utility::Debug()
      //    << "  parent2joint = " << Magnum::Matrix4{parent2joint};
      return true;
    } else {
      parent2joint = Mn::Matrix4();  // Identity
      return false;
    }
  }

  return false;
};

bool URDFImporter::getJointInfo(int linkIndex,
                                Mn::Matrix4& parent2joint,
                                Mn::Matrix4& linkTransformInWorld,
                                Mn::Vector3& jointAxisInJointSpace,
                                int& jointType,
                                float& jointLowerLimit,
                                float& jointUpperLimit,
                                float& jointDamping,
                                float& jointFriction) const {
  float jointMaxForce;
  float jointMaxVelocity;
  return getJointInfo2(linkIndex, parent2joint, linkTransformInWorld,
                       jointAxisInJointSpace, jointType, jointLowerLimit,
                       jointUpperLimit, jointDamping, jointFriction,
                       jointMaxForce, jointMaxVelocity);
}

void URDFImporter::getMassAndInertia2(int linkIndex,
                                      float& mass,
                                      Mn::Vector3& localInertiaDiagonal,
                                      Mn::Matrix4& inertialFrame,
                                      int flags) const {
  if (flags & CUF_USE_URDF_INERTIA) {
    //Corrade::Utility::Debug() << "using URDF inertia values...";
    getMassAndInertia(linkIndex, mass, localInertiaDiagonal, inertialFrame);
  } else {
    // the link->m_inertia is NOT necessarily aligned with the inertial frame
    // so an additional transform might need to be computed
    auto link = urdfParser_.getModel().getLink(linkIndex);
    if (link != nullptr) {
      float linkMass;
      if (link->m_parentJoint == nullptr &&
          urdfParser_.getModel().m_overrideFixedBase) {
        linkMass = 0.f;
      } else {
        linkMass = link->m_inertia.m_mass;
      }
      mass = linkMass;
      localInertiaDiagonal = Mn::Vector3(0, 0, 0);
      inertialFrame =
          Mn::Matrix4::from(link->m_inertia.m_linkLocalFrame.rotation(),
                            link->m_inertia.m_linkLocalFrame.translation());
    } else {
      mass = 1.f;
      localInertiaDiagonal = Mn::Vector3(1, 1, 1);
      inertialFrame = Mn::Matrix4();  // Identity
    }
  }
}

void URDFImporter::getMassAndInertia(int linkIndex,
                                     float& mass,
                                     Mn::Vector3& localInertiaDiagonal,
                                     Mn::Matrix4& inertialFrame) const {
  // the link->m_inertia is NOT necessarily aligned with the inertial frame
  // so an additional transform might need to be computed
  auto link = urdfParser_.getModel().getLink(linkIndex);
  if (link != nullptr) {
    Mn::Matrix3 linkInertiaBasis;  // Identity
    float linkMass, principalInertiaX, principalInertiaY, principalInertiaZ;
    if (link->m_parentJoint == nullptr &&
        urdfParser_.getModel().m_overrideFixedBase) {
      linkMass = 0.f;
      principalInertiaX = 0.f;
      principalInertiaY = 0.f;
      principalInertiaZ = 0.f;
    } else {
      linkMass = link->m_inertia.m_mass;
      if (link->m_inertia.m_ixy == 0.0 && link->m_inertia.m_ixz == 0.0 &&
          link->m_inertia.m_iyz == 0.0) {
        principalInertiaX = link->m_inertia.m_ixx;
        principalInertiaY = link->m_inertia.m_iyy;
        principalInertiaZ = link->m_inertia.m_izz;
      } else {
        // by column vector
        Mn::Matrix3 inertiaTensor(
            Mn::Vector3(link->m_inertia.m_ixx, link->m_inertia.m_ixy,
                        link->m_inertia.m_ixz),
            Mn::Vector3(link->m_inertia.m_ixy, link->m_inertia.m_iyy,
                        link->m_inertia.m_iyz),
            Mn::Vector3(link->m_inertia.m_ixz, link->m_inertia.m_iyz,
                        link->m_inertia.m_izz));

        // TODO: diagonalization of inertia matrix:
        Mn::Debug{} << "WARNING: getMassAndInertia: intertia not diagonal. "
                       "TODO: diagonalize?";
        /*
        float threshold = 1.0e-6;
        int numIterations = 30;
        inertiaTensor.diagonalize(linkInertiaBasis, threshold,
         numIterations);
         */
        principalInertiaX = inertiaTensor[0][0];
        principalInertiaY = inertiaTensor[1][1];
        principalInertiaZ = inertiaTensor[2][2];
      }
    }
    mass = linkMass;
    if (principalInertiaX < 0 ||
        principalInertiaX > (principalInertiaY + principalInertiaZ) ||
        principalInertiaY < 0 ||
        principalInertiaY > (principalInertiaX + principalInertiaZ) ||
        principalInertiaZ < 0 ||
        principalInertiaZ > (principalInertiaX + principalInertiaY)) {
      //Corrade::Utility::Debug() << "W - Bad inertia tensor properties, setting "
      //                             "inertia to zero for link: "
      //                          << link->m_name;
      principalInertiaX = 0.f;
      principalInertiaY = 0.f;
      principalInertiaZ = 0.f;
      linkInertiaBasis = Mn::Matrix3();  // Identity
    }
    localInertiaDiagonal =
        Mn::Vector3(principalInertiaX, principalInertiaY, principalInertiaZ);

    inertialFrame = Mn::Matrix4::from(
        link->m_inertia.m_linkLocalFrame.rotation() * linkInertiaBasis,
        link->m_inertia.m_linkLocalFrame.translation());
  } else {
    mass = 1.f;
    localInertiaDiagonal = Mn::Vector3(1);
    inertialFrame = Mn::Matrix4();  // Identity
  }
}

bool URDFImporter::getLinkContactInfo(
    int linkIndex,
    io::URDF::LinkContactInfo& contactInfo) const {
  auto link = urdfParser_.getModel().getLink(linkIndex);
  if (link == nullptr) {
    //Corrade::Utility::Debug() << "E - No link with index = " << linkIndex;
    return false;
  }

  contactInfo = link->m_contactInfo;
  return true;
}

const io::URDF::Model& URDFImporter::getModel() const {
  return urdfParser_.getModel();
}

}  // namespace physics
}  // namespace esp
