// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

// Code adapted from Bullet3/examples/Importers/ImportURDFDemo ...

#pragma once

#include "esp/assets/ResourceManager.h"

#include "esp/io/URDFParser.h"
namespace esp {
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
  CUF_USE_MATERIAL_TRANSPARANCY_FROM_MTL = 65536,
  CUF_MAINTAIN_LINK_ORDER = 131072,
};

/**
 * @brief Virtual base class for loading URDF into various simulator
 * implementations
 */
class URDFImporter {
 public:
  URDFImporter(esp::assets::ResourceManager& resourceManager)
      : resourceManager_(resourceManager){};

  // attempt to load a URDF file
  bool loadURDF(const std::string& filename);

  virtual const io::URDF::Model& getModel() const;

  void setFixedBase(bool fixedBase) {
    urdfParser_.getModel().m_overrideFixedBase = fixedBase;
  }

  bool getFixedBase() { return urdfParser_.getModel().m_overrideFixedBase; }

  virtual std::string getBodyName() const {
    return urdfParser_.getModel().m_name;
  };

  virtual int getRootLinkIndex() const;

  virtual void getLinkChildIndices(int linkIndex,
                                   std::vector<int>& childLinkIndices) const;

  virtual bool getLinkContactInfo(int linkIndex,
                                  io::URDF::LinkContactInfo& contactInfo) const;

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
                                  Magnum::Matrix4& inertialFrame,
                                  int flags) const;

 protected:
  // parses the URDF file into general, simulation platform invariant
  // datastructures
  io::URDF::Parser urdfParser_;

  esp::assets::ResourceManager& resourceManager_;
};

}  // namespace physics
}  // namespace esp
