// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

// Code adapted from Bullet3/examples/Importers/ImportURDFDemo ...

#include "URDFImporter.h"

#include <iostream>

#include <Corrade/Utility/DebugStl.h>
#include <Corrade/Utility/Path.h>
#include "esp/assets/ResourceManager.h"
#include "esp/metadata/managers/AssetAttributesManager.h"

namespace Mn = Magnum;

namespace esp {
namespace physics {

bool URDFImporter::loadURDF(
    const esp::metadata::attributes::ArticulatedObjectAttributes::ptr&
        artObjAttributes,
    bool forceReload) {
  const std::string urdfFilepath = artObjAttributes->getURDFPath();
  auto modelCacheIter = modelCache_.find(urdfFilepath);
  // if map not found or forcing reload
  if ((modelCacheIter == modelCache_.end()) || forceReload) {
    if (!Corrade::Utility::Path::exists(urdfFilepath) ||
        Corrade::Utility::Path::isDirectory(urdfFilepath)) {
      ESP_DEBUG() << "URDF File does not exist:" << urdfFilepath
                  << ". Aborting URDF parse/load.";
      return false;
    }

    // parse the URDF from file
    std::shared_ptr<metadata::URDF::Model> urdfModel;
    bool success = urdfParser_.parseURDF(artObjAttributes, urdfModel);
    if (!success) {
      ESP_DEBUG() << "Failed to parse URDF:" << urdfFilepath << ", aborting.";
      return false;
    }

    if (ESP_LOG_LEVEL_ENABLED(logging::LoggingLevel::VeryVerbose)) {
      ESP_VERY_VERBOSE() << "Done parsing URDF model:";
      urdfModel->printKinematicChain();
    }

    // if reloading, clear the old model if exists
    if (modelCacheIter != modelCache_.end()) {
      modelCache_.erase(modelCacheIter);
    }

    // register the new model and set to iterator
    modelCacheIter = modelCache_.emplace(urdfFilepath, urdfModel).first;
  }
  activeModel_ = modelCacheIter->second;

  // re-scale the cached model
  activeModel_->setGlobalScaling(artObjAttributes->getUniformScale());
  activeModel_->setMassScaling(artObjAttributes->getMassScale());

  return true;
}

int URDFImporter::getRootLinkIndex() const {
  if (activeModel_->m_rootLinks.size() == 1) {
    return activeModel_->m_rootLinks[0]->m_linkIndex;
  }
  return ID_UNDEFINED;
}

void URDFImporter::getLinkChildIndices(
    int linkIndex,
    std::vector<int>& childLinkIndices) const {
  childLinkIndices.resize(0);
  auto link = activeModel_->getLink(linkIndex);

  if (link != nullptr) {
    for (size_t i = 0; i < link->m_childLinks.size(); ++i) {
      int childIndex = link->m_childLinks[i].lock()->m_linkIndex;
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

  auto link = activeModel_->getLink(linkIndex);
  if (link != nullptr) {
    linkTransformInWorld = Mn::Matrix4(Mn::Math::IdentityInit);

    if (auto pj = link->m_parentJoint.lock()) {
      parent2joint = pj->m_parentLinkToJointTransform;
      jointType = pj->m_type;
      jointAxisInJointSpace = pj->m_localJointAxis;
      jointLowerLimit = pj->m_lowerLimit;
      jointUpperLimit = pj->m_upperLimit;
      jointDamping = pj->m_jointDamping;
      jointFriction = pj->m_jointFriction;
      jointMaxForce = pj->m_effortLimit;
      jointMaxVelocity = pj->m_velocityLimit;
      return true;
    }
    parent2joint = Mn::Matrix4();  // Identity
    return false;
  }

  return false;
}

bool URDFImporter::getJointInfo(int linkIndex,
                                Mn::Matrix4& parent2joint,
                                Mn::Matrix4& linkTransformInWorld,
                                Mn::Vector3& jointAxisInJointSpace,
                                int& jointType,
                                float& jointLowerLimit,
                                float& jointUpperLimit,
                                float& jointDamping,
                                float& jointFriction) const {
  float jointMaxForce{0.f};
  float jointMaxVelocity{0.f};
  return getJointInfo2(linkIndex, parent2joint, linkTransformInWorld,
                       jointAxisInJointSpace, jointType, jointLowerLimit,
                       jointUpperLimit, jointDamping, jointFriction,
                       jointMaxForce, jointMaxVelocity);
}

void URDFImporter::getMassAndInertia2(int linkIndex,
                                      float& mass,
                                      Mn::Vector3& localInertiaDiagonal,
                                      Mn::Matrix4& inertialFrame) const {
  if ((flags & CUF_USE_URDF_INERTIA) != 0) {
    getMassAndInertia(linkIndex, mass, localInertiaDiagonal, inertialFrame);
  } else {
    // the link->m_inertia is NOT necessarily aligned with the inertial frame
    // so an additional transform might need to be computed
    auto link = activeModel_->getLink(linkIndex);
    if (link != nullptr) {
      float linkMass{0.f};
      if (!link->m_parentJoint.lock() && activeModel_->m_overrideFixedBase) {
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
  auto link = activeModel_->getLink(linkIndex);
  if (link != nullptr) {
    Mn::Matrix3 linkInertiaBasis;  // Identity
    float linkMass = 0.f;
    float principalInertiaX = 0.f;
    float principalInertiaY = 0.f;
    float principalInertiaZ = 0.f;
    if (!link->m_parentJoint.lock() && activeModel_->m_overrideFixedBase) {
      // no change
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
        ESP_VERY_VERBOSE()
            << "WARNING: getMassAndInertia: intertia not diagonal. "
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
      ESP_VERY_VERBOSE() << "W - Bad inertia tensor properties, setting "
                            "inertia to zero for link:"
                         << link->m_name;
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
    metadata::URDF::LinkContactInfo& contactInfo) const {
  auto link = activeModel_->getLink(linkIndex);
  if (link == nullptr) {
    ESP_DEBUG() << "No link with index =" << linkIndex;
    return false;
  }

  contactInfo = link->m_contactInfo;
  return true;
}

void URDFImporter::importURDFAssets() {
  if (activeModel_ == nullptr) {
    ESP_DEBUG()
        << "URDFImporter::importURDFAssets - No URDF::Model loaded, aborting.";
    return;
  }
  for (size_t linkIx = 0; linkIx < activeModel_->m_links.size(); ++linkIx) {
    auto link = activeModel_->getLink(linkIx);
    // load collision shapes
    for (auto& collision : link->m_collisionArray) {
      if (collision.m_geometry.m_type == metadata::URDF::GEOM_MESH) {
        // pre-load the mesh asset for its collision shape
        assets::AssetInfo meshAsset{assets::AssetType::UNKNOWN,
                                    collision.m_geometry.m_meshFileName};
        CORRADE_ASSERT(resourceManager_.loadRenderAsset(meshAsset),
                       "Failed to load URDF ("
                           << activeModel_->m_name << ") collision asset"
                           << collision.m_geometry.m_meshFileName, );
      }
    }
    // pre-load visual meshes and primitive asset variations and cache the
    // handle
    for (auto& visual : link->m_visualArray) {
      assets::AssetInfo visualMeshInfo{assets::AssetType::UNKNOWN};
      visualMeshInfo.forceFlatShading = false;

      std::shared_ptr<metadata::URDF::Material> material =
          visual.m_geometry.m_localMaterial;
      if (material) {
        visualMeshInfo.overridePhongMaterial = assets::PhongMaterialColor();
        visualMeshInfo.overridePhongMaterial->ambientColor =
            material->m_matColor.m_rgbaColor;
        visualMeshInfo.overridePhongMaterial->diffuseColor =
            material->m_matColor.m_rgbaColor;
        visualMeshInfo.overridePhongMaterial->specularColor =
            Mn::Color4(material->m_matColor.m_specularColor);
      }
      switch (visual.m_geometry.m_type) {
        case metadata::URDF::GEOM_CAPSULE: {
          visualMeshInfo.type = esp::assets::AssetType::PRIMITIVE;
          auto assetMgr = resourceManager_.getAssetAttributesManager();
          auto capTemplate = assetMgr->getDefaultCapsuleTemplate(false);
          // proportions as suggested on magnum docs
          capTemplate->setHalfLength(0.5 * visual.m_geometry.m_capsuleHeight /
                                     visual.m_geometry.m_capsuleRadius);
          assetMgr->registerObject(capTemplate);
          // cache the new capsule asset handle for later instancing
          visual.m_geometry.m_meshFileName = capTemplate->getHandle();
        } break;
        case metadata::URDF::GEOM_CYLINDER:
          visualMeshInfo.type = esp::assets::AssetType::PRIMITIVE;
          visualMeshInfo.filepath =
              "cylinderSolid_rings_1_segments_12_halfLen_1_useTexCoords_false_"
              "useTangents_false_capEnds_true";
          break;
        case metadata::URDF::GEOM_BOX:
          visualMeshInfo.type = esp::assets::AssetType::PRIMITIVE;
          visualMeshInfo.filepath = "cubeSolid";
          break;
        case metadata::URDF::GEOM_SPHERE:
          visualMeshInfo.type = esp::assets::AssetType::PRIMITIVE;
          visualMeshInfo.filepath = "icosphereSolid_subdivs_1";
          break;
        case metadata::URDF::GEOM_MESH:
          visualMeshInfo.filepath = visual.m_geometry.m_meshFileName;
          break;
        default:
          ESP_DEBUG() << "URDFImporter::importURDFAssets - unsupported "
                         "visual geometry type.";
          break;
      }
    }
  }
}

}  // namespace physics
}  // namespace esp
