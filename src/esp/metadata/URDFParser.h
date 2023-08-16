// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

// Code adapted from Bullet3/examples/Importers/ImportURDFDemo ...

#ifndef ESP_IO_URDFPARSER_H_
#define ESP_IO_URDFPARSER_H_

#include <Magnum/Magnum.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Matrix4.h>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include "Corrade/Containers/Optional.h"
#include "attributes/ArticulatedObjectAttributes.h"
#include "esp/core/Configuration.h"

/** @file
 * @brief Storage classes for articulated object metadata and URDF file parsing
 * functionality. Struct @ref metadata::URDF::MaterialColor, struct @ref
 * metadata::URDF::Material, enum @ref metadata::URDF::JointTypes, enum @ref
 * metadata::URDF::GeomTypes, struct @ref metadata::URDF::Geometry, struct @ref
 * metadata::URDF::Shape, struct @ref metadata::URDF::VisualShape, enum @ref
 * metadata::URDF::CollisionFlags, struct @ref metadata::URDF::CollisionShape,
 * struct @ref metadata::URDF::Inertia, struct @ref metadata::URDF::Joint, enum
 * @ref metadata::URDF::LinkContactFlags, struct @ref
 * metadata::URDF::LinkContactInfo, struct @ref metadata::URDF::Link, class @ref
 * metadata::URDF::Model, class @ref metadata::URDF::Parser.
 */

namespace tinyxml2 {
class XMLElement;
}

namespace esp {
namespace metadata {

namespace URDF {

//! Stores basic color properties which can be defined in a URDF file to
//! override default visual asset material colors.
struct MaterialColor {
  //! Diffuse and ambient color
  Magnum::Color4 m_rgbaColor{0.8, 0.8, 0.8, 1};
  //! Specular color
  Magnum::Color3 m_specularColor{0.4, 0.4, 0.4};
  MaterialColor() = default;
};

//! Storage for metadata defining override materials for visual shapes
struct Material {
  //! custom material name
  std::string m_name;
  //! Store material texture filename. Note: Will be cached, but not currently
  //! supported in Habitat rendering pipeline.
  std::string m_textureFilename;
  //! Material color metadata
  MaterialColor m_matColor;

  Material() = default;
};

//! Defines Various types of supported joints.
enum JointTypes {
  RevoluteJoint = 1,
  PrismaticJoint,
  ContinuousJoint,
  FloatingJoint,
  PlanarJoint,
  FixedJoint,
  SphericalJoint,

};

//! Defines various types of supported geometry
enum GeomTypes {
  GEOM_SPHERE = 2,
  GEOM_BOX,
  GEOM_CYLINDER,
  GEOM_MESH,
  GEOM_PLANE,
  GEOM_CAPSULE,  // non-standard URDF but supported
  GEOM_UNKNOWN,
};

//! Stores properties of various types of visual or collision geometry which can
//! be attached to links
struct Geometry {
  //! Type of geometry for pivoting during instancing
  GeomTypes m_type{GEOM_UNKNOWN};

  double m_sphereRadius{1};

  Magnum::Vector3 m_boxSize{1, 1, 1};

  double m_capsuleRadius{1};
  double m_capsuleHeight{1};

  Magnum::Vector3 m_planeNormal{0, 0, 1};

  //! If a mesh, store the relative filepath of the asset. Note: also used to
  //! cache primitive handles for custom visual prims.
  std::string m_meshFileName;
  Magnum::Vector3 m_meshScale{1, 1, 1};

  //! If a custom material is defined, store it with the geometry. Because named
  //! materials can be overridden at multiple points in the URDF, we store the
  //! struct as-is when referenced rather than a key.
  std::shared_ptr<Material> m_localMaterial;
  bool m_hasLocalMaterial{false};

  Geometry() = default;
};

//! Stores general properties of any shape
struct Shape {
  //! If the shape originates from a file, cache the filepath
  std::string m_sourceFileLocation;
  //! Transform of this shape relative to its link
  Magnum::Matrix4 m_linkLocalFrame;
  //! Shape geometry
  Geometry m_geometry;
  //! Any custom name for this shape
  std::string m_name;
};

//! Parsed from a URDF link visual shape (<visual>)
struct VisualShape : Shape {
  //! If the shape references a named custom material, cache the name. Will also
  //! cache the material properties internally.
  std::string m_materialName;
};

//! Defines some collision shape flags
enum CollisionFlags {
  //! The shape has a custom group defined (e.g. <collision group="32">)
  HAS_COLLISION_GROUP = 1,
  //! The shape has a custom mask defined (e.g. <collision mask="32">)
  HAS_COLLISION_MASK = 2,
};

//! Parsed from a URDF link collision shape (<collision>)
struct CollisionShape : Shape {
  int m_flags{0};
  //! custom collision group (e.g. <collision>)
  int m_collisionGroup{0};
  //! custom collision mask
  int m_collisionMask{0};
  CollisionShape() = default;
};

//! Parsed from a URDF link inertia properties (<inertial>). Stores dynamic
//! properties of ths link.
struct Inertia {
  //! local transform of the link in parent joint space
  Magnum::Matrix4 m_linkLocalFrame{0.0};
  bool m_hasLinkLocalFrame{false};

  //! mass of the link (0 mass indicates unmovable, static object)
  double m_mass{0.0};

  //! inertia matrix upper triangular entries. Computed automatically if not
  //! specified.
  double m_ixx{0.0}, m_ixy{0.0}, m_ixz{0.0}, m_iyy{0.0}, m_iyz{0.0}, m_izz{0.0};

  Inertia() = default;
};

//! Parsed from a URDF joint (<joint>), connects two links.
struct Joint {
  //! Joint name
  std::string m_name;
  //! Type of the joint, used to pivot on relevant fields for instancing
  JointTypes m_type;
  //! Relative transform of the joint from its parent link
  Magnum::Matrix4 m_parentLinkToJointTransform;
  //! Name of the parent link and key in the link cache
  std::string m_parentLinkName;
  //! Name of the child link and key in the link cache
  std::string m_childLinkName;
  //! Some joints (e.g. revolute) have a local axis (e.g. X, Y, Z)
  Magnum::Vector3 m_localJointAxis;

  //! Joint limits if provided
  double m_lowerLimit{0};
  double m_upperLimit{-1};

  //! Joint force/torque limits if provided
  double m_effortLimit{0};
  double m_velocityLimit{0};

  //! Joint damping and friction. Used to generate default JointMotors for
  //! BulletArticulatedObjects.
  double m_jointDamping{0};
  double m_jointFriction{0};
  Joint() = default;
};

//! Contact info cached for links, but not currently used
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

//! Contact info cached for links, but not currently used (parsed from
//! <contact>)
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

  int m_flags{CONTACT_HAS_LATERAL_FRICTION};

  LinkContactInfo() = default;
};

//! Parsed from a URDF link (<link>). Represents a single rigid segment of the
//! articulated object.
struct Link {
  //! name of the link
  std::string m_name;
  //! dynamic properties of the link
  Inertia m_inertia;
  //! visual shapes attached to this link
  std::vector<VisualShape> m_visualArray;
  //! collision shapes attached to this link
  std::vector<CollisionShape> m_collisionArray;
  //! this link's parent link
  std::weak_ptr<Link> m_parentLink;
  //! the joint connecting this link to its parent
  std::weak_ptr<Joint> m_parentJoint;
  //! list of all link children of this link
  std::vector<std::weak_ptr<Joint>> m_childJoints;
  //! joints attaching this link to each of its children (with index
  //! correspondence to m_childJoints)
  std::vector<std::weak_ptr<Link>> m_childLinks;

  //! the index of this link in the overall articulated object
  int m_linkIndex{-2};

  //! any contact info for this link
  LinkContactInfo m_contactInfo;

  Link() = default;
};

//! Generic structure representing an articulated object parsed from a URDF file
//! independent from any physics implementation.
class Model {
 public:
  //! name of the articulated object or robot
  std::string m_name;
  //! source file this model was built from (e.g. the .urdf file)
  std::string m_sourceFile;
  //! world transform of the object root if provided
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

  //! if true, force this model to produce a fixed base instance (e.g. the root
  //! is not dynamic)
  bool m_overrideFixedBase{false};

  //! output a string to console listing the link|joint hierarchy of this model
  //! for debugging and investigation purposes
  void printKinematicChain() const;

  /**
   * @brief Get a link provided its name.
   *
   * @param linkName The link's configured name from the URDF file.
   * @return The link metadata object or nullptr if no link with provided name.
   */
  std::shared_ptr<Link> getLink(const std::string& linkName) const {
    auto linkIter = m_links.find(linkName);
    if (linkIter != m_links.end()) {
      return linkIter->second;
    }
    return nullptr;
  }

  /**
   * @brief Get a link provided its index.
   *
   * @param linkIndex The link's index.
   * @return The link metadata object or nullptr if no link with provided index.
   */
  std::shared_ptr<Link> getLink(int linkIndex) const {
    auto linkIdxToNameIter = m_linkIndicesToNames.find(linkIndex);
    if (linkIdxToNameIter != m_linkIndicesToNames.end()) {
      return getLink(linkIdxToNameIter->second);
    }
    return nullptr;
  }

  /**
   * @brief Get a parent joint of a link provided the link index.
   *
   * @param linkIndex The link's index.
   * @return The parent joint metadata object or nullptr if no link with
   * provided index.
   */
  std::shared_ptr<Joint> getJoint(int linkIndex) const {
    auto linkIdxToNameIter = m_linkIndicesToNames.find(linkIndex);
    if (linkIdxToNameIter != m_linkIndicesToNames.end()) {
      return getLink(linkIdxToNameIter->second)->m_parentJoint.lock();
    }
    return nullptr;
  }

  Model() = default;

  /**
   * @brief Set global scaling and re-scale an existing model. Modifies various
   * internal parameters.
   *
   * @param scaling The new absolute uniform scale.
   */
  void setGlobalScaling(float scaling);

  //! Get the currently configured global model scaling
  float getGlobalScaling() const { return m_globalScaling; }

  /**
   * @brief Set scaling for mass from initial values configured in URDF.
   * Modifies various internal parameters.
   *
   * @param massScaling The new absolute uniform mass scale.
   */
  void setMassScaling(float massScaling);

  /**
   * @brief Get the currently configured mass scaling of the model.
   */
  float getMassScaling() const { return m_massScaling; }

  /**
   * @brief Set the path to the render asset to attach to this URDF model.
   */
  void setRenderAsset(Cr::Containers::Optional<std::string> renderAsset) {
    m_renderAsset = std::move(renderAsset);
  }

  /**
   * @brief Get the path to the render asset to attach to this URDF model.
   */
  Cr::Containers::Optional<std::string> getRenderAsset() const {
    return m_renderAsset;
  }

  /**
   * @brief Set the semantic ID of the URDF model.
   */
  void setSemanticId(int semanticId) { m_semanticId = semanticId; }

  /**
   * @brief Get the semantic ID of this URDF model.
   */
  int getSemanticId() const { return m_semanticId; }

  /**
   * @brief Set hint to render articulated object primitives even if a render
   * asset is present.
   */
  void setRenderLinkVisualShapes(bool renderLinkVisualShapes) {
    m_renderLinkVisualShapes = renderLinkVisualShapes;
  }

  /**
   * @brief Get hint to render articulated object visual shapes as defined in
   * the URDF even if a render asset/skin is present.
   */
  bool getRenderLinkVisualShapes() const { return m_renderLinkVisualShapes; }

  /**
   * @brief This function will set the
   * @ref metadata::attributes::ArticulatedObjectAttributes used to create this model.
   */
  void setModelInitAttributes(
      metadata::attributes::ArticulatedObjectAttributes::ptr artObjAttributes);

  /**
   * @brief Gets a smart pointer reference to a copy of the user-specified
   * configuration data from a config file. Habitat does not parse or
   * process this data, but it will be available to the user via python
   * bindings for each object.
   */
  std::shared_ptr<core::config::Configuration> getUserConfiguration() const {
    return initializationAttributes_->getUserConfiguration();
  }

 protected:
  /**
   * @brief Json-based attributes defining characteristics of this model not
   * specified in the source XML/URDF. Primarily to support default user-defined
   * attributes. This data is read in from a json file with the same base name
   * as the source XML/URDF for this model but the extension ".ao_config.json".
   */
  metadata::attributes::ArticulatedObjectAttributes::ptr
      initializationAttributes_ = nullptr;

  // scaling values which can be applied to the model after parsing
  //! Global euclidean scaling applied to the model's transforms, asset scales,
  //! and prismatic joint limits. Does not affect mass.
  float m_globalScaling = 1.0;

  //! Mass scaling of the model's Link inertias.
  float m_massScaling = 1.0;

  //! Path to a render asset associated with this articulated object.
  Cr::Containers::Optional<std::string> m_renderAsset = Cr::Containers::NullOpt;

  //! Semantic ID of this model.
  int m_semanticId = 0;

  //! Forces link visual shapes to be rendered even if a render asset(skin) is
  //! present.
  bool m_renderLinkVisualShapes = false;

  //! Scale the transformation and parameters of a Shape
  void scaleShape(Shape& shape, float scale);
};  // class model

/**
 * @brief Functional class for parsing URDF files into a URDF::Model
 * representation.
 */
class Parser {
  // URDF file path of last load call
  std::string sourceFilePath_;

  /**
   * @brief Parse a transform into a Matrix4.
   *
   * @param tr The transform to fill.
   * @param xml The source xml element to parse (e.g. <origin>).
   * @return Success or failure.
   */
  bool parseTransform(Magnum::Matrix4& tr,
                      const tinyxml2::XMLElement* xml) const;

  /**
   * @brief Parse URDF link dynamic info into a datastructure.
   *
   * @param inertia The inertia datastructure to fill.
   * @param config The source xml element to parse (e.g. <inertial>).
   * @return Success or failure.
   */
  bool parseInertia(Inertia& inertia, const tinyxml2::XMLElement* config);

  /**
   * @brief Parse URDF shape geometry info into a datastructure.
   *
   * @param geom The geometry datastructure to fill.
   * @param g The source xml element to parse (e.g. <geometry>).
   * @return Success or failure.
   */
  bool parseGeometry(Geometry& geom, const tinyxml2::XMLElement* g);

  /**
   * @brief Parse URDF visual shape info into a datastructure.
   *
   * @param model The URDF::Model datastructure to fill.
   * @param visual The visual shape datastructure to fill.
   * @param config The source xml element to parse (e.g. <visual>).
   * @return Success or failure.
   */
  bool parseVisual(const std::shared_ptr<Model>& model,
                   VisualShape& visual,
                   const tinyxml2::XMLElement* config);

  /**
   * @brief Parse URDF collision shape info into a datastructure.
   *
   * @param collision The collision shape datastructure to fill.
   * @param config The source xml element to parse (e.g. <collision>).
   * @return Success or failure.
   */
  bool parseCollision(CollisionShape& collision,
                      const tinyxml2::XMLElement* config);

  /**
   * @brief Traverse the link->joint kinematic chain to cache parent->child
   * relationships and detect root/base links.
   *
   * @param model The URDF::Model datastructure to manipulate.
   * @return Success or failure.
   */
  bool initTreeAndRoot(const std::shared_ptr<Model>& model) const;

  /**
   * @brief Parse URDF material info into a datastructure.
   *
   * @param material The Material datastructure to fill.
   * @param config The source xml element to parse (e.g. <material>).
   * @return Success or failure.
   */
  bool parseMaterial(Material& material,
                     const tinyxml2::XMLElement* config) const;

  /**
   * @brief Parse joint limits from a <joint> xml element into a datastructure.
   *
   * @param joint The Joint datastructure to fill.
   * @param config The source xml element to parse (e.g. <joint>).
   * @return Success or failure.
   */
  bool parseJointLimits(Joint& joint, const tinyxml2::XMLElement* config) const;

  /**
   * @brief Parse joint dynamic info from a <joint> xml element into
   * ajsonAttributes_ datastructure.
   *
   * @param joint The Joint datastructure to fill.
   * @param config The source xml element to parse (e.g. <joint>).
   * @return Success or failure.
   */
  bool parseJointDynamics(Joint& joint,
                          const tinyxml2::XMLElement* config) const;

  /**
   * @brief Parse joint info from a <joint> xml element into a datastructure.
   *
   * @param joint The Joint datastructure to fill.
   * @param config The source xml element to parse (e.g. <joint>).
   * @return Success or failure.
   */
  bool parseJoint(Joint& joint, const tinyxml2::XMLElement* config);

  /**
   * @brief Parse link info from a <link> xml element into a datastructure.
   *
   * @param link The Link datastructure to fill.
   * @param config The source xml element to parse (e.g. <link>).
   * @return Success or failure.
   */
  bool parseLink(const std::shared_ptr<Model>&,
                 Link& link,
                 const tinyxml2::XMLElement* config);

  /**
   * @brief Parse sensor info from a into a datastructure.
   *
   * TODO: Not implemented yet, does nothing.
   *
   * @param model The URDF::Model datastructure to fill.
   * @param link The Link datastructure to fill.
   * @param joint The Joint datastructure to fill.
   * @param config The source xml element to parse.
   * @return Success or failure.
   */
  bool parseSensor(CORRADE_UNUSED const std::shared_ptr<Model>& model,
                   CORRADE_UNUSED Link& link,
                   CORRADE_UNUSED Joint& joint,
                   CORRADE_UNUSED const tinyxml2::XMLElement* config) {
    // TODO: this
    return false;
  };

  /**
   * @brief Check whether or not a given filepath exists.
   *
   * @param filename The filepath to check.
   * @return Filepath is valid or invalid.
   */
  bool validateMeshFile(std::string& filename);

 public:
  Parser() = default;

  // parse a loaded URDF string into relevant general data structures
  // return false if the string is not a valid urdf or other error causes abort
  bool parseURDF(const metadata::attributes::ArticulatedObjectAttributes::ptr&
                     artObjAttributes,
                 std::shared_ptr<Model>& model);

  // This is no longer used, instead set the urdf and physics subsystem to
  // veryverbose, i.e. export HABITAT_SIM_LOG="urdf,physics=veryverbose" bool
  // logMessages = false;
};

}  // namespace URDF
}  // namespace metadata
}  // namespace esp

#endif  // ESP_IO_URDFPARSER_H_
