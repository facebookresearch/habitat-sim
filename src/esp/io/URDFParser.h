// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

// Code adapted from Bullet3/examples/Importers/ImportURDFDemo ...

#pragma once
#include <Magnum/Magnum.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Matrix4.h>
#include <map>
#include <string>
#include <vector>

namespace tinyxml2 {
class XMLElement;
};

////////////////////////////////////
// Utility/storage structs
////////////////////////////////////

namespace esp {
namespace io {

struct UrdfMaterialColor {
  Magnum::Color4 m_rgbaColor;
  Magnum::Color3 m_specularColor;
  UrdfMaterialColor()
      : m_rgbaColor(0.8, 0.8, 0.8, 1), m_specularColor(0.4, 0.4, 0.4) {}
};

struct UrdfMaterial {
  std::string m_name;
  std::string m_textureFilename;
  UrdfMaterialColor m_matColor;

  UrdfMaterial() {}
};

enum UrdfJointTypes {
  URDFRevoluteJoint = 1,
  URDFPrismaticJoint,
  URDFContinuousJoint,
  URDFFloatingJoint,
  URDFPlanarJoint,
  URDFFixedJoint,
  URDFSphericalJoint,

};

enum UrdfGeomTypes {
  URDF_GEOM_SPHERE = 2,
  URDF_GEOM_BOX,
  URDF_GEOM_CYLINDER,
  URDF_GEOM_MESH,
  URDF_GEOM_PLANE,
  URDF_GEOM_CAPSULE,  // non-standard URDF
  URDF_GEOM_UNKNOWN,
};

struct UrdfGeometry {
  UrdfGeomTypes m_type;

  double m_sphereRadius;

  Magnum::Vector3 m_boxSize;

  double m_capsuleRadius;
  double m_capsuleHeight;
  int m_hasFromTo;
  Magnum::Vector3 m_capsuleFrom;
  Magnum::Vector3 m_capsuleTo;

  Magnum::Vector3 m_planeNormal;
  std::string m_meshFileName;
  Magnum::Vector3 m_meshScale;

  std::shared_ptr<UrdfMaterial> m_localMaterial;
  bool m_hasLocalMaterial;

  UrdfGeometry()
      : m_type(URDF_GEOM_UNKNOWN),
        m_sphereRadius(1),
        m_boxSize(1, 1, 1),
        m_capsuleRadius(1),
        m_capsuleHeight(1),
        m_hasFromTo(0),
        m_capsuleFrom(0, 1, 0),
        m_capsuleTo(1, 0, 0),
        m_planeNormal(0, 0, 1),
        m_meshScale(1, 1, 1),
        m_hasLocalMaterial(false) {}
};

struct UrdfShape {
  std::string m_sourceFileLocation;
  Magnum::Matrix4 m_linkLocalFrame;
  UrdfGeometry m_geometry;
  std::string m_name;
};

struct UrdfVisual : UrdfShape {
  std::string m_materialName;
};

// TODO: need this?
enum UrdfCollisionFlags {
  URDF_FORCE_CONCAVE_TRIMESH = 1,
  URDF_HAS_COLLISION_GROUP = 2,
  URDF_HAS_COLLISION_MASK = 4,
};

struct UrdfCollision : UrdfShape {
  int m_flags;
  int m_collisionGroup;
  int m_collisionMask;
  UrdfCollision() : m_flags(0) {}
};

struct UrdfInertia {
  Magnum::Matrix4 m_linkLocalFrame;
  bool m_hasLinkLocalFrame;

  double m_mass;
  double m_ixx, m_ixy, m_ixz, m_iyy, m_iyz, m_izz;

  UrdfInertia() {
    m_hasLinkLocalFrame = false;
    m_mass = 0.f;
    m_ixx = m_ixy = m_ixz = m_iyy = m_iyz = m_izz = 0.f;
  }
};

struct UrdfJoint {
  std::string m_name;
  UrdfJointTypes m_type;
  Magnum::Matrix4 m_parentLinkToJointTransform;
  std::string m_parentLinkName;
  std::string m_childLinkName;
  Magnum::Vector3 m_localJointAxis;

  double m_lowerLimit;
  double m_upperLimit;

  double m_effortLimit;
  double m_velocityLimit;

  double m_jointDamping;
  double m_jointFriction;
  UrdfJoint()
      : m_lowerLimit(0),
        m_upperLimit(-1),
        m_effortLimit(0),
        m_velocityLimit(0),
        m_jointDamping(0),
        m_jointFriction(0) {}
};

// TODO: need this?
enum URDF_LinkContactFlags {
  URDF_CONTACT_HAS_LATERAL_FRICTION = 1,
  URDF_CONTACT_HAS_INERTIA_SCALING = 2,
  URDF_CONTACT_HAS_CONTACT_CFM = 4,
  URDF_CONTACT_HAS_CONTACT_ERP = 8,
  URDF_CONTACT_HAS_STIFFNESS_DAMPING = 16,
  URDF_CONTACT_HAS_ROLLING_FRICTION = 32,
  URDF_CONTACT_HAS_SPINNING_FRICTION = 64,
  URDF_CONTACT_HAS_RESTITUTION = 128,
  URDF_CONTACT_HAS_FRICTION_ANCHOR = 256,
};

struct URDFLinkContactInfo {
  float m_lateralFriction;
  float m_rollingFriction;
  float m_spinningFriction;
  float m_restitution;
  float m_inertiaScaling;
  float m_contactCfm;
  float m_contactErp;
  float m_contactStiffness;
  float m_contactDamping;

  int m_flags;

  URDFLinkContactInfo()
      : m_lateralFriction(0.5),
        m_rollingFriction(0),
        m_spinningFriction(0),
        m_restitution(0),
        m_inertiaScaling(1),
        m_contactCfm(0),
        m_contactErp(0),
        m_contactStiffness(1e4),
        m_contactDamping(1) {
    m_flags = URDF_CONTACT_HAS_LATERAL_FRICTION;
  }
};

struct UrdfLink {
  std::string m_name;
  UrdfInertia m_inertia;
  Magnum::Matrix4 m_linkTransformInWorld;
  std::vector<UrdfVisual> m_visualArray;
  std::vector<UrdfCollision> m_collisionArray;
  std::shared_ptr<UrdfLink> m_parentLink;
  std::shared_ptr<UrdfJoint> m_parentJoint;

  std::vector<std::shared_ptr<UrdfJoint>> m_childJoints;
  std::vector<std::shared_ptr<UrdfLink>> m_childLinks;

  int m_linkIndex;

  URDFLinkContactInfo m_contactInfo;

  UrdfLink() : m_parentLink(0), m_parentJoint(0), m_linkIndex(-2) {}
};

struct UrdfModel {
  std::string m_name;
  std::string m_sourceFile;
  Magnum::Matrix4 m_rootTransformInWorld(Magnum::Math::IdentityInitT);
  std::map<std::string, std::shared_ptr<UrdfMaterial>> m_materials;
  std::map<std::string, std::shared_ptr<UrdfLink>> m_links;
  std::map<std::string, std::shared_ptr<UrdfJoint>> m_joints;

  std::vector<std::shared_ptr<UrdfLink>> m_rootLinks;
  bool m_overrideFixedBase;

  void printKinematicChain() const;

  UrdfModel() : m_overrideFixedBase(false) {}
};

class URDFParser {
  // datastructures
  UrdfModel m_urdfModel;
  float m_urdfScaling = 1.0;

  // URDF file path of last load call
  std::string sourceFilePath_;

  // parser functions
  bool parseTransform(Magnum::Matrix4& tr, tinyxml2::XMLElement* xml);
  bool parseInertia(UrdfInertia& inertia, tinyxml2::XMLElement* config);
  bool parseGeometry(UrdfGeometry& geom, tinyxml2::XMLElement* g);
  bool parseVisual(UrdfModel& model,
                   UrdfVisual& visual,
                   tinyxml2::XMLElement* config);
  bool parseCollision(UrdfCollision& collision, tinyxml2::XMLElement* config);
  bool initTreeAndRoot(UrdfModel& model);
  bool parseMaterial(UrdfMaterial& material, tinyxml2::XMLElement* config);
  bool parseJointLimits(UrdfJoint& joint, tinyxml2::XMLElement* config);
  bool parseJointDynamics(UrdfJoint& joint, tinyxml2::XMLElement* config);
  bool parseJoint(UrdfJoint& joint, tinyxml2::XMLElement* config);
  bool parseLink(UrdfModel& model,
                 UrdfLink& link,
                 tinyxml2::XMLElement* config);
  bool parseSensor(UrdfModel& model,
                   UrdfLink& link,
                   UrdfJoint& joint,
                   tinyxml2::XMLElement* config);

  bool validateMeshFile(std::string& filename);

 public:
  URDFParser(){};

  // parse a loaded URDF string into relevant general data structures
  // return false if the string is not a valid urdf or other error causes abort
  bool parseURDF(const std::string& meshFilename);

  void setGlobalScaling(float scaling) { m_urdfScaling = scaling; }
  float getGlobalScaling() { return m_urdfScaling; }

  const UrdfModel& getModel() const { return m_urdfModel; }

  UrdfModel& getModel() { return m_urdfModel; }
};

}  // namespace io
}  // namespace esp
