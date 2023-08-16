// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

// Code adapted from Bullet3/examples/Importers/ImportURDFDemo ...

#ifndef ESP_PHYSICS_URDFIMPORTER_H_
#define ESP_PHYSICS_URDFIMPORTER_H_

#include "esp/metadata/URDFParser.h"

namespace esp {
namespace assets {
class ResourceManager;
}
namespace metadata {
namespace attributes {
class ArticulatedObjectAttributes;
}
}  // namespace metadata

namespace physics {

enum ConvertURDFFlags {
  CUF_USE_SDF = 1,
  // Use inertia values in URDF instead of recomputing them from collision
  // shape.
  CUF_USE_URDF_INERTIA = 2,
  CUF_USE_MJCF = 4,
  CUF_USE_SELF_COLLISION = 8,
  CUF_USE_SELF_COLLISION_EXCLUDE_PARENT = 16,
  CUF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS = 32,
  CUF_RESERVED = 64,
  CUF_USE_IMPLICIT_CYLINDER = 128,
  CUF_GLOBAL_VELOCITIES_MB = 256,
  CUF_MJCF_COLORS_FROM_FILE = 512,
  CUF_ENABLE_CACHED_GRAPHICS_SHAPES = 1024,
  CUF_ENABLE_SLEEPING = 2048,
  CUF_INITIALIZE_SAT_FEATURES = 4096,
  CUF_USE_SELF_COLLISION_INCLUDE_PARENT = 8192,
  CUF_PARSE_SENSORS = 16384,
  CUF_USE_MATERIAL_COLORS_FROM_MTL = 32768,
  CUF_USE_MATERIAL_TRANSPARENCY_FROM_MTL = 65536,
  CUF_MAINTAIN_LINK_ORDER = 131072,
};

/**
 * @brief Virtual base class for loading URDF into various simulator
 * implementations
 */
class URDFImporter {
 public:
  explicit URDFImporter(esp::assets::ResourceManager& resourceManager)
      : resourceManager_(resourceManager){};

  virtual ~URDFImporter() = default;
  /**
   * @brief Sets the activeModel_ for the importer. If new or forceReload, parse
   * a URDF file and cache the resulting model. Note: when applying uniform
   * scaling to a 3D model consider scale^3 mass scaling to approximate uniform
   * density.
   * @param filename The filepath for the URDF and key for the cached model.
   * @param globalScale A global, uniform 3D scale. Does not affect mass. Can be
   * applied to update cached models.
   * @param massScale A uniform scaling of link masses. Can be applied to update
   * cached models.
   * @param forceReload If true, reload the URDF from file, replacing the cached
   * model.
   */
  bool loadURDF(
      const esp::metadata::attributes::ArticulatedObjectAttributes::ptr&
          artObjAttributes,
      bool forceReload = false);

  // NOTE: all of these getter/setters act on the current "activeModel_"
  virtual std::shared_ptr<metadata::URDF::Model> getModel() const {
    return activeModel_;
  };

  void setFixedBase(bool fixedBase) {
    CORRADE_ASSERT(activeModel_, "No model is currently loaded.", );
    activeModel_->m_overrideFixedBase = fixedBase;
  }

  bool getFixedBase() {
    CORRADE_ASSERT(activeModel_, "No model is currently loaded.", false);
    return activeModel_->m_overrideFixedBase;
  }

  virtual std::string getBodyName() const {
    CORRADE_ASSERT(activeModel_, "No model is currently loaded.", "");
    return activeModel_->m_name;
  };

  virtual int getRootLinkIndex() const;

  virtual void getLinkChildIndices(int linkIndex,
                                   std::vector<int>& childLinkIndices) const;

  virtual bool getLinkContactInfo(
      int linkIndex,
      metadata::URDF::LinkContactInfo& contactInfo) const;

  // TODO: refactor this nonsense
  virtual bool getJointInfo(int linkIndex,
                            Magnum::Matrix4& parent2joint,
                            Magnum::Matrix4& linkTransformInWorld,
                            Magnum::Vector3& jointAxisInJointSpace,
                            int& jointType,
                            float& jointLowerLimit,
                            float& jointUpperLimit,
                            float& jointDamping,
                            float& jointFriction) const;
  virtual bool getJointInfo2(int linkIndex,
                             Magnum::Matrix4& parent2joint,
                             Magnum::Matrix4& linkTransformInWorld,
                             Magnum::Vector3& jointAxisInJointSpace,
                             int& jointType,
                             float& jointLowerLimit,
                             float& jointUpperLimit,
                             float& jointDamping,
                             float& jointFriction,
                             float& jointMaxForce,
                             float& jointMaxVelocity) const;

  virtual void getMassAndInertia(int linkIndex,
                                 float& mass,
                                 Magnum::Vector3& localInertiaDiagonal,
                                 Magnum::Matrix4& inertialFrame) const;
  virtual void getMassAndInertia2(int linkIndex,
                                  float& mass,
                                  Magnum::Vector3& localInertiaDiagonal,
                                  Magnum::Matrix4& inertialFrame) const;

  // This is no longer used, instead set the urdf subsystem to veryverbose,
  // i.e. export HABITAT_SIM_LOG="urdf=veryverbose"
  // bool logMessages = false;

  //! collect and return a list of cached model keys (filepaths)
  std::vector<std::string> getCachedModelKeys() {
    std::vector<std::string> keys;
    keys.reserve(modelCache_.size());
    for (auto& it : modelCache_) {
      keys.push_back(it.first);
    }
    return keys;
  };

  /**
   * @brief Load/import any required render and collision assets for the
   * acrive metadata::URDF::Model before instantiating it.
   */
  void importURDFAssets();

  //! importer model conversion flags
  int flags = 0;

 protected:
  // parses the URDF file into general, simulation platform invariant
  // datastructures
  metadata::URDF::Parser urdfParser_;

  esp::assets::ResourceManager& resourceManager_;

  //! cache parsed URDF models by filename
  std::map<std::string, std::shared_ptr<metadata::URDF::Model>> modelCache_;

  //! which model is being actively manipulated. Changed by calling
  //! loadURDF(filename).
  std::shared_ptr<metadata::URDF::Model> activeModel_ = nullptr;
};

}  // namespace physics
}  // namespace esp

#endif  // ESP_PHYSICS_URDFIMPORTER_H_
