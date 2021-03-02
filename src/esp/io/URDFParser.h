// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

// Code adapted from Bullet3/examples/Importers/ImportURDFDemo ...

#pragma once
#include <Magnum/Magnum.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Matrix4.h>
#include <map>
#include <memory>
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

namespace URDF {

struct MaterialColor {
  Magnum::Color4 m_rgbaColor{0.8, 0.8, 0.8, 1};
  Magnum::Color3 m_specularColor{0.4, 0.4, 0.4};
  MaterialColor() = default;
};

struct Material {
  std::string m_name;
  std::string m_textureFilename;
  MaterialColor m_matColor;

  Material() = default;
};

enum JointTypes {
  RevoluteJoint = 1,
  PrismaticJoint,
  ContinuousJoint,
  FloatingJoint,
  PlanarJoint,
  FixedJoint,
  SphericalJoint,

};

enum GeomTypes {
  GEOM_SPHERE = 2,
  GEOM_BOX,
  GEOM_CYLINDER,
  GEOM_MESH,
  GEOM_PLANE,
  GEOM_CAPSULE,  // non-standard URDF
  GEOM_UNKNOWN,
};

struct Geometry {
  GeomTypes m_type{GEOM_UNKNOWN};

  double m_sphereRadius{1};

  Magnum::Vector3 m_boxSize{1, 1, 1};

  double m_capsuleRadius{1};
  double m_capsuleHeight{1};
  int m_hasFromTo{0};
  Magnum::Vector3 m_capsuleFrom{0, 1, 0};
  Magnum::Vector3 m_capsuleTo{1, 0, 0};

  Magnum::Vector3 m_planeNormal{0, 0, 1};
  std::string m_meshFileName;
  Magnum::Vector3 m_meshScale{1, 1, 1};

  std::shared_ptr<Material> m_localMaterial;
  bool m_hasLocalMaterial{false};

  Geometry() = default;
};

struct Shape {
  std::string m_sourceFileLocation;
  Magnum::Matrix4 m_linkLocalFrame;
  Geometry m_geometry;
  std::string m_name;
};

struct VisualShape : Shape {
  std::string m_materialName;
};

// TODO: need this?
enum CollisionFlags {
  FORCE_CONCAVE_TRIMESH = 1,
  HAS_COLLISION_GROUP = 2,
  HAS_COLLISION_MASK = 4,
};

struct CollisionShape : Shape {
  int m_flags{0};
  int m_collisionGroup;
  int m_collisionMask;
  CollisionShape() = default;
};

struct Inertia {
  Magnum::Matrix4 m_linkLocalFrame{false};
  bool m_hasLinkLocalFrame;

  double m_mass{0.0};
  double m_ixx, m_ixy, m_ixz, m_iyy, m_iyz, m_izz = 0.0;

  Inertia() = default;
};

struct Joint {
  std::string m_name;
  JointTypes m_type;
  Magnum::Matrix4 m_parentLinkToJointTransform;
  std::string m_parentLinkName;
  std::string m_childLinkName;
  Magnum::Vector3 m_localJointAxis;

  double m_lowerLimit{0};
  double m_upperLimit{-1};

  double m_effortLimit{0};
  double m_velocityLimit{0};

  double m_jointDamping{0};
  double m_jointFriction{0};
  Joint() = default;
};

// TODO: need this?
enum LinkContactFlags {
  CONTACT_HAS_LATERAL_FRICTION = 1,
  CONTACT_HAS_INERTIA_SCALING = 2,
  CONTACT_HAS_CONTACT_CFM = 4,
  CONTACT_HAS_CONTACT_ERP = 8,
  CONTACT_HAS_STIFFNESS_DAMPING = 16,
  CONTACT_HAS_ROLLING_FRICTION = 32,
  CONTACT_HAS_SPINNING_FRICTION = 64,
  CONTACT_HAS_RESTITUTION = 128,
  CONTACT_HAS_FRICTION_ANCHOR = 256,
};

struct LinkContactInfo {
  float m_lateralFriction{0.5};
  float m_rollingFriction{0};
  float m_spinningFriction{0};
  float m_restitution{0};
  float m_inertiaScaling{1};
  float m_contactCfm{0};
  float m_contactErp{0};
  float m_contactStiffness{1e4};
  float m_contactDamping{1};

  int m_flags;

  LinkContactInfo() { m_flags = CONTACT_HAS_LATERAL_FRICTION; }
};

struct Link {
  std::string m_name;
  Inertia m_inertia;
  Magnum::Matrix4 m_linkTransformInWorld;
  std::vector<VisualShape> m_visualArray;
  std::vector<CollisionShape> m_collisionArray;
  std::weak_ptr<Link> m_parentLink;
  std::weak_ptr<Joint> m_parentJoint;

  std::vector<std::weak_ptr<Joint>> m_childJoints;
  std::vector<std::weak_ptr<Link>> m_childLinks;

  int m_linkIndex{-2};

  LinkContactInfo m_contactInfo;

  Link() = default;
};

class Model {
 public:
  std::string m_name;
  std::string m_sourceFile;
  Magnum::Matrix4 m_rootTransformInWorld(Magnum::Math::IdentityInitT);

  //! map of names to materials
  std::map<std::string, std::shared_ptr<Material>> m_materials;

  //! map of names to links
  std::map<std::string, std::shared_ptr<Link>> m_links;

  //! map of link indices to names
  std::map<int, std::string> m_linkIndicesToNames;

  //! map of names to joints
  std::map<std::string, std::shared_ptr<Joint>> m_joints;

  //! list of root links (usually 1)
  std::vector<std::shared_ptr<Link>> m_rootLinks;
  bool m_overrideFixedBase{false};

  void printKinematicChain() const;

  std::shared_ptr<Link> getLink(const std::string& linkName) const {
    if (m_links.count(linkName)) {
      return m_links.at(linkName);
    }
    return nullptr;
  }
  std::shared_ptr<Link> getLink(int linkIndex) const {
    if (m_linkIndicesToNames.count(linkIndex)) {
      return getLink(m_linkIndicesToNames.at(linkIndex));
    }
    return nullptr;
  }

  //! get the parent joint of a link
  std::shared_ptr<Joint> getJoint(int linkIndex) const {
    if (m_linkIndicesToNames.count(linkIndex)) {
      return getLink(m_linkIndicesToNames.at(linkIndex))->m_parentJoint.lock();
    }
    return nullptr;
  }

  Model() = default;

  //! Set global scaling and re-scale an existing model if already parsed.
  void setGlobalScaling(float scaling) {
    if (scaling == m_globalScaling) {
      // do nothing
      return;
    }

    // Need to re-scale model, so use the ratio of new to current scale
    float scaleCorrection = scaling / m_globalScaling;

    // scale all transforms' translations
    for (const auto& link : m_links) {
      // scale inertial offsets
      link.second->m_inertia.m_linkLocalFrame.translation() *= scaleCorrection;
      // scale visual shape parameters
      for (auto& visual : link.second->m_visualArray) {
        scaleShape(visual, scaleCorrection);
      }
      // scale collision shapes
      for (auto& collision : link.second->m_collisionArray) {
        scaleShape(collision, scaleCorrection);
      }
    }
    for (const auto& joint : m_joints) {
      // scale joint offsets
      joint.second->m_parentLinkToJointTransform.translation() *=
          scaleCorrection;
      // scale prismatic joint limits
      if (joint.second->m_type == PrismaticJoint) {
        joint.second->m_lowerLimit *= double(scaleCorrection);
        joint.second->m_upperLimit *= double(scaleCorrection);
      }
    }

    m_globalScaling = scaling;
  }
  float getGlobalScaling() { return m_globalScaling; }

  //! Set scaling for mass from initial values configured in URDF. Modifies the
  //! cached model if already parsed.
  void setMassScaling(float massScaling) {
    if (massScaling == m_massScaling) {
      // do nothing
      return;
    }
    float massScaleCorrection = massScaling / m_massScaling;

    // Only need to scale the per-link mass values. These will be further
    // processed during import.
    for (const auto& link : m_links) {
      Inertia& linkInertia = link.second->m_inertia;
      linkInertia.m_mass *= double(massScaleCorrection);
    }

    m_massScaling = massScaling;
  }

  float getMassScaling() { return m_massScaling; }

 protected:
  // scaling values which can be applied to the model after parsing
  //! Global euclidean scaling applied to the model's transforms, asset scales,
  //! and prismatic joint limits. Does not affect mass.
  float m_globalScaling = 1.0;

  //! Mass scaling of the model's Link inertias.
  float m_massScaling = 1.0;

  //! Scale the transformation and parameters of a Shape
  void scaleShape(Shape& shape, float scale) {
    shape.m_linkLocalFrame.translation() *= scale;
    switch (shape.m_geometry.m_type) {
      case GEOM_MESH: {
        shape.m_geometry.m_meshScale *= scale;
      } break;
      case GEOM_BOX: {
        shape.m_geometry.m_boxSize *= scale;
      } break;
      case GEOM_SPHERE: {
        shape.m_geometry.m_sphereRadius *= double(scale);
      } break;
      case GEOM_CAPSULE:
      case GEOM_CYLINDER: {
        shape.m_geometry.m_capsuleRadius *= double(scale);
        shape.m_geometry.m_capsuleHeight *= double(scale);
      } break;
      default:
        break;
    }
  };
};

class Parser {
  // datastructures
  std::shared_ptr<Model> m_urdfModel;
  float m_urdfScaling = 1.0;

  // URDF file path of last load call
  std::string sourceFilePath_;

  // parser functions
  bool parseTransform(Magnum::Matrix4& tr, tinyxml2::XMLElement* xml);
  bool parseInertia(Inertia& inertia, tinyxml2::XMLElement* config);
  bool parseGeometry(Geometry& geom, tinyxml2::XMLElement* g);
  bool parseVisual(std::shared_ptr<Model>& model,
                   VisualShape& visual,
                   tinyxml2::XMLElement* config);
  bool parseCollision(CollisionShape& collision, tinyxml2::XMLElement* config);
  bool initTreeAndRoot(std::shared_ptr<Model>& model);
  bool parseMaterial(Material& material, tinyxml2::XMLElement* config);
  bool parseJointLimits(Joint& joint, tinyxml2::XMLElement* config);
  bool parseJointDynamics(Joint& joint, tinyxml2::XMLElement* config);
  bool parseJoint(Joint& joint, tinyxml2::XMLElement* config);
  bool parseLink(std::shared_ptr<Model>&,
                 Link& link,
                 tinyxml2::XMLElement* config);
  bool parseSensor(CORRADE_UNUSED std::shared_ptr<Model>&,
                   CORRADE_UNUSED Link& link,
                   CORRADE_UNUSED Joint& joint,
                   CORRADE_UNUSED tinyxml2::XMLElement* config) {
    // TODO: this
    return false;
  };

  bool validateMeshFile(std::string& filename);

 public:
  Parser() = default;

  // parse a loaded URDF string into relevant general data structures
  // return false if the string is not a valid urdf or other error causes abort
  bool parseURDF(const std::string& meshFilename);

  //! Set the global scale for future parsing and modify the cached model if
  //! applicable.
  void setGlobalScaling(float scaling) {
    m_urdfScaling = scaling;
    m_urdfModel->setGlobalScaling(m_urdfScaling);
  }
  float getGlobalScaling() { return m_urdfScaling; }

  const std::shared_ptr<Model> getModel() const { return m_urdfModel; }

  std::shared_ptr<Model> getModel() { return m_urdfModel; }

  bool logMessages = false;
};

}  // namespace URDF
}  // namespace io
}  // namespace esp
