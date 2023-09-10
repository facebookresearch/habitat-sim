// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

// Code adapted from Bullet3/examples/Importers/ImportURDFDemo ...

#include <Corrade/Containers/Pair.h>
#include <Corrade/Utility/DebugStl.h>
#include <Corrade/Utility/Path.h>
#include <Corrade/Utility/String.h>
#include <Magnum/Math/Quaternion.h>
#include <iostream>

#include "Corrade/Containers/Containers.h"
#include "URDFParser.h"
#include "esp/core/Logging.h"
#include "esp/io/Json.h"

#include "tinyxml2/tinyxml2.h"

// using namespace tinyxml2;
using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

namespace Cr = Corrade;
namespace Mn = Magnum;

namespace esp {
namespace metadata {
namespace URDF {

void Model::scaleShape(Shape& shape, float scale) {
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
}

void Model::setGlobalScaling(float scaling) {
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
    joint.second->m_parentLinkToJointTransform.translation() *= scaleCorrection;
    // scale prismatic joint limits
    if (joint.second->m_type == PrismaticJoint) {
      joint.second->m_lowerLimit *= double(scaleCorrection);
      joint.second->m_upperLimit *= double(scaleCorrection);
    }
  }

  m_globalScaling = scaling;
}

void Model::setMassScaling(float massScaling) {
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

void Model::setModelInitAttributes(
    metadata::attributes::ArticulatedObjectAttributes::ptr artObjAttributes) {
  initializationAttributes_ = std::move(artObjAttributes);
  // TODO these fields are redundant - can just use the attributes fields
  // directly
  m_renderAsset = initializationAttributes_->getRenderAssetHandle();
  m_semanticId = initializationAttributes_->getSemanticId();
  // TODO : Use the enum value instead of settting a boolean here

  auto renderMode = initializationAttributes_->getRenderMode();

  m_renderLinkVisualShapes =
      (renderMode ==
       metadata::attributes::ArticulatedObjectRenderMode::LinkVisuals) ||
      (renderMode == metadata::attributes::ArticulatedObjectRenderMode::Both);

}  // Model::setModelInitAttributes

bool Parser::parseURDF(
    const esp::metadata::attributes::ArticulatedObjectAttributes::ptr&
        artObjAttributes,
    std::shared_ptr<Model>& urdfModel) {
  auto filename = artObjAttributes->getURDFPath();
  // override the previous model with a fresh one
  urdfModel = std::make_shared<Model>();
  sourceFilePath_ = filename;
  urdfModel->m_sourceFile = filename;

  std::string xmlString = *Corrade::Utility::Path::readString(filename);

  XMLDocument xml_doc;
  xml_doc.Parse(xmlString.c_str());
  if (xml_doc.Error()) {
    ESP_ERROR() << "XML parse error, aborting URDF parse/load for" << filename;
    return false;
  }
  ESP_VERY_VERBOSE() << "XML parsed starting URDF parse/load.";

  const XMLElement* robot_xml = xml_doc.FirstChildElement("robot");
  if (!robot_xml) {
    ESP_ERROR() << "Expected a <robot> element. Aborting URDF parse/load for"
                << filename;
    return false;
  }

  // Get robot name
  const char* name = robot_xml->Attribute("name");
  if (!name) {
    ESP_ERROR() << "Expected a name for robot. Aborting URDF parse/load for"
                << filename;
    return false;
  }
  urdfModel->m_name = name;

  // Get all Material elements
  for (const XMLElement* material_xml =
           robot_xml->FirstChildElement("material");
       material_xml;
       material_xml = material_xml->NextSiblingElement("material")) {
    Material material;

    parseMaterial(material, material_xml);

    if (urdfModel->m_materials.count(material.m_name) == 0) {
      urdfModel->m_materials[material.m_name] =
          std::make_shared<Material>(material);
    } else {
      ESP_VERY_VERBOSE() << "W - Duplicate material";
    }
  }

  // Get all link elements including shapes
  int link_index = 0;
  for (const XMLElement* link_xml = robot_xml->FirstChildElement("link");
       link_xml; link_xml = link_xml->NextSiblingElement("link")) {
    std::shared_ptr<Link> link = std::make_shared<Link>();

    if (parseLink(urdfModel, *link, link_xml)) {
      if (urdfModel->m_links.count(link->m_name) != 0u) {
        ESP_ERROR()
            << "Link name  (" << link->m_name
            << ") is not unique, link names "
               "in the same model have to be unique. Aborting parse/load for"
            << filename;
        return false;
      } else {
        // copy model material into link material, if link has no local material
        for (size_t i = 0; i < link->m_visualArray.size(); ++i) {
          VisualShape& vis = link->m_visualArray.at(i);
          if (!vis.m_geometry.m_hasLocalMaterial &&
              !vis.m_materialName.empty()) {
            auto mat_itr = urdfModel->m_materials.find(vis.m_materialName);
            if (mat_itr != urdfModel->m_materials.end()) {
              vis.m_geometry.m_localMaterial = mat_itr->second;
            } else {
              ESP_ERROR() << "Cannot find material with name:"
                          << vis.m_materialName << ". Aborting parse/load for"
                          << filename;
            }
          }
        }

        // register the new link
        urdfModel->m_links[link->m_name] = link;
        link->m_linkIndex = link_index;
        urdfModel->m_linkIndicesToNames[link->m_linkIndex] = link->m_name;
        ++link_index;
      }
    } else {
      ESP_ERROR() << "Failed to parse link. Aborting parse/load for"
                  << filename;
      return false;
    }
  }
  if (urdfModel->m_links.empty()) {
    ESP_ERROR() << "No links found in URDF file. Aborting parse/load for"
                << filename;
    return false;
  }

  // Get all Joint elements
  for (const XMLElement* joint_xml = robot_xml->FirstChildElement("joint");
       joint_xml; joint_xml = joint_xml->NextSiblingElement("joint")) {
    std::shared_ptr<Joint> joint = std::make_shared<Joint>();

    if (parseJoint(*joint, joint_xml)) {
      if (urdfModel->m_joints.count(joint->m_name) != 0u) {
        ESP_ERROR() << "Joint" << joint->m_name
                    << "is not unique. Aborting parse/load for" << filename;
        return false;
      } else {
        urdfModel->m_joints[joint->m_name] = joint;
      }
    } else {
      ESP_ERROR() << "Joint xml is not initialized correctly. Aborting "
                     "parse/load for"
                  << filename;
      return false;
    }
  }

  // TODO: parse sensors here

  if (!initTreeAndRoot(urdfModel)) {
    return false;
  }

  // Set the creation attributes
  urdfModel->setModelInitAttributes(artObjAttributes);

  ESP_VERY_VERBOSE() << "Done parsing URDF for" << filename;
  return true;
}  // Parser::parseURDF

static bool parseColor4(Mn::Color4& color4, const std::string& vector_str) {
  color4 = Mn::Color4();  // 0 init
  std::vector<std::string> pieces =
      Corrade::Utility::String::splitWithoutEmptyParts(vector_str);
  std::vector<double> vec_parts;
  for (size_t i = 0; i < pieces.size(); ++i) {
    if (!pieces[i].empty()) {
      vec_parts.push_back(std::stod(pieces[i]));
    }
  }
  if (vec_parts.size() != 4) {
    return false;
  }
  color4 = Mn::Color4(vec_parts[0], vec_parts[1], vec_parts[2], vec_parts[3]);
  return true;
}

static bool parseVector3(Mn::Vector3& vec3,
                         const std::string& vector_str,
                         bool lastThree = false) {
  vec3 = Mn::Vector3();  // 0 init
  std::vector<std::string> pieces =
      Corrade::Utility::String::splitWithoutEmptyParts(vector_str);
  std::vector<double> vec_parts;
  for (size_t i = 0; i < pieces.size(); ++i) {
    if (!pieces[i].empty()) {
      vec_parts.push_back(std::stod(pieces[i]));
    }
  }
  if (vec_parts.size() < 3) {
    return false;
  }
  if (lastThree) {
    vec3 = Mn::Vector3(vec_parts[vec_parts.size() - 3],
                       vec_parts[vec_parts.size() - 2],
                       vec_parts[vec_parts.size() - 1]);
  } else {
    vec3 = Mn::Vector3(vec_parts[0], vec_parts[1], vec_parts[2]);
  }
  return true;
}

bool Parser::parseMaterial(Material& material, const XMLElement* config) const {
  if (!config->Attribute("name")) {
    ESP_VERY_VERBOSE() << "Material must contain a name attribute";
    return false;
  }
  material.m_name = config->Attribute("name");

  // texture
  const XMLElement* t = config->FirstChildElement("texture");
  if (t) {
    if (t->Attribute("filename")) {
      material.m_textureFilename = t->Attribute("filename");
    }
  }

  // color
  {
    const XMLElement* c = config->FirstChildElement("color");
    if (c) {
      if (c->Attribute("rgba")) {
        if (!parseColor4(material.m_matColor.m_rgbaColor,
                         c->Attribute("rgba"))) {
          ESP_VERY_VERBOSE() << "W -" << material.m_name << "has no rgba";
        }
      }
    }
  }

  {
    // specular (non-standard)
    const XMLElement* s = config->FirstChildElement("specular");
    if (s) {
      if (s->Attribute("rgb")) {
        if (!parseVector3(material.m_matColor.m_specularColor,
                          s->Attribute("rgb"))) {
        }
      }
    }
  }
  return true;
}

bool Parser::parseLink(const std::shared_ptr<Model>& model,
                       Link& link,
                       const XMLElement* config) {
  const char* linkName = config->Attribute("name");
  if (!linkName) {
    ESP_VERY_VERBOSE() << "Link with no name";
    return false;
  }
  ESP_VERY_VERBOSE() << "------------------------------------";
  ESP_VERY_VERBOSE() << "Parser::parseLink:" << linkName;
  link.m_name = linkName;

  {
    // optional 'contact' parameters
    const XMLElement* ci = config->FirstChildElement("contact");
    if (ci) {
      const XMLElement* damping_xml = ci->FirstChildElement("inertia_scaling");
      if (damping_xml) {
        if (!damping_xml->Attribute("value")) {
          ESP_VERY_VERBOSE()
              << "Link/contact: damping element must have value attribute";
          return false;
        }

        link.m_contactInfo.m_inertiaScaling =
            std::stod(damping_xml->Attribute("value"));
        link.m_contactInfo.m_flags |= CONTACT_HAS_INERTIA_SCALING;
      }
      {
        const XMLElement* friction_xml =
            ci->FirstChildElement("lateral_friction");
        if (friction_xml) {
          if (!friction_xml->Attribute("value")) {
            ESP_VERY_VERBOSE() << "Link/contact: lateral_friction "
                                  "element must have value attribute";
            return false;
          }

          link.m_contactInfo.m_lateralFriction =
              std::stod(friction_xml->Attribute("value"));
        }
      }

      {
        const XMLElement* rolling_xml =
            ci->FirstChildElement("rolling_friction");
        if (rolling_xml) {
          if (!rolling_xml->Attribute("value")) {
            ESP_VERY_VERBOSE() << "Link/contact: rolling friction "
                                  "element must have value attribute";
            return false;
          }

          link.m_contactInfo.m_rollingFriction =
              std::stod(rolling_xml->Attribute("value"));
          link.m_contactInfo.m_flags |= CONTACT_HAS_ROLLING_FRICTION;
        }
      }

      {
        const XMLElement* restitution_xml =
            ci->FirstChildElement("restitution");
        if (restitution_xml) {
          if (!restitution_xml->Attribute("value")) {
            ESP_VERY_VERBOSE() << "Link/contact: restitution "
                                  "element must have value attribute";
            return false;
          }

          link.m_contactInfo.m_restitution =
              std::stod(restitution_xml->Attribute("value"));
          link.m_contactInfo.m_flags |= CONTACT_HAS_RESTITUTION;
        }
      }

      {
        const XMLElement* spinning_xml =
            ci->FirstChildElement("spinning_friction");
        if (spinning_xml) {
          if (!spinning_xml->Attribute("value")) {
            ESP_VERY_VERBOSE() << "Link/contact: spinning friction "
                                  "element must have value attribute";
            return false;
          }

          link.m_contactInfo.m_spinningFriction =
              std::stod(spinning_xml->Attribute("value"));
          link.m_contactInfo.m_flags |= CONTACT_HAS_SPINNING_FRICTION;
        }
      }
      {
        const XMLElement* friction_anchor =
            ci->FirstChildElement("friction_anchor");
        if (friction_anchor) {
          link.m_contactInfo.m_flags |= CONTACT_HAS_FRICTION_ANCHOR;
        }
      }
      {
        const XMLElement* stiffness_xml = ci->FirstChildElement("stiffness");
        if (stiffness_xml) {
          if (!stiffness_xml->Attribute("value")) {
            ESP_VERY_VERBOSE() << "Link/contact: stiffness element "
                                  "must have value attribute";
            return false;
          }

          link.m_contactInfo.m_contactStiffness =
              std::stod(stiffness_xml->Attribute("value"));
          link.m_contactInfo.m_flags |= CONTACT_HAS_STIFFNESS_DAMPING;
        }
      }
      {
        const XMLElement* damping_xml = ci->FirstChildElement("damping");
        if (damping_xml) {
          if (!damping_xml->Attribute("value")) {
            ESP_VERY_VERBOSE() << "Link/contact: damping element "
                                  "must have value attribute";
            return false;
          }

          link.m_contactInfo.m_contactDamping =
              std::stod(damping_xml->Attribute("value"));
          link.m_contactInfo.m_flags |= CONTACT_HAS_STIFFNESS_DAMPING;
        }
      }
    }
  }

  // Inertial (optional)
  const XMLElement* i = config->FirstChildElement("inertial");
  if (i) {
    if (!parseInertia(link.m_inertia, i)) {
      ESP_VERY_VERBOSE() << "Could not parse inertial element for Link:";
      ESP_VERY_VERBOSE() << link.m_name;
      return false;
    }
    link.m_inertia.m_mass *= model->getGlobalScaling();
  } else {
    if ((strlen(linkName) == 5) && (strncmp(linkName, "world", 5)) == 0) {
      link.m_inertia.m_mass = 0.f;
      link.m_inertia.m_linkLocalFrame = Mn::Matrix4();  // Identity
      link.m_inertia.m_ixx = 0.f;
      link.m_inertia.m_iyy = 0.f;
      link.m_inertia.m_izz = 0.f;
    } else {
      ESP_VERY_VERBOSE()
          << "W - No inertial data for link:" << link.m_name
          << ", using mass=1, localinertiadiagonal = 1,1,1, identity local "
             "inertial frame";
      link.m_inertia.m_mass = 1.f * model->getMassScaling();
      link.m_inertia.m_linkLocalFrame = Mn::Matrix4();  // Identity
      link.m_inertia.m_ixx = 1.f;
      link.m_inertia.m_iyy = 1.f;
      link.m_inertia.m_izz = 1.f;
    }
  }

  // Multiple Visuals (optional)
  for (const XMLElement* vis_xml = config->FirstChildElement("visual"); vis_xml;
       vis_xml = vis_xml->NextSiblingElement("visual")) {
    VisualShape visual;

    if (parseVisual(model, visual, vis_xml)) {
      link.m_visualArray.push_back(visual);
    } else {
      ESP_VERY_VERBOSE() << "Could not parse visual element for Link:"
                         << link.m_name;
      return false;
    }
  }

  // Multiple Collisions (optional)
  for (const XMLElement* col_xml = config->FirstChildElement("collision");
       col_xml; col_xml = col_xml->NextSiblingElement("collision")) {
    CollisionShape col;

    if (parseCollision(col, col_xml)) {
      link.m_collisionArray.push_back(col);
    } else {
      ESP_VERY_VERBOSE() << "Could not parse collision element for Link:"
                         << link.m_name.c_str();
      return false;
    }
  }
  return true;
}

bool Parser::parseCollision(CollisionShape& collision,
                            const XMLElement* config) {
  collision.m_linkLocalFrame = Mn::Matrix4();  // Identity

  // Origin
  const XMLElement* o = config->FirstChildElement("origin");
  if (o) {
    if (!parseTransform(collision.m_linkLocalFrame, o))
      return false;
  }
  // Geometry
  const XMLElement* geom = config->FirstChildElement("geometry");
  if (!parseGeometry(collision.m_geometry, geom)) {
    return false;
  }

  {
    const char* group_char = config->Attribute("group");
    if (group_char) {
      collision.m_flags |= HAS_COLLISION_GROUP;
      collision.m_collisionGroup = std::stoi(group_char);
    }
  }

  {
    const char* mask_char = config->Attribute("mask");
    if (mask_char) {
      collision.m_flags |= HAS_COLLISION_MASK;
      collision.m_collisionMask = std::stoi(mask_char);
    }
  }

  const char* name_char = config->Attribute("name");
  if (name_char)
    collision.m_name = name_char;

  return true;
}

bool Parser::parseVisual(const std::shared_ptr<Model>& model,
                         VisualShape& visual,
                         const XMLElement* config) {
  visual.m_linkLocalFrame = Mn::Matrix4();  // Identity

  // Origin
  const XMLElement* o = config->FirstChildElement("origin");
  if (o) {
    if (!parseTransform(visual.m_linkLocalFrame, o))
      return false;
  }
  // Geometry
  const XMLElement* geom = config->FirstChildElement("geometry");
  if (!parseGeometry(visual.m_geometry, geom)) {
    return false;
  }

  const char* name_char = config->Attribute("name");
  if (name_char)
    visual.m_name = name_char;

  visual.m_geometry.m_hasLocalMaterial = false;

  // Material
  const XMLElement* mat = config->FirstChildElement("material");
  // todo(erwincoumans) skip materials in SDF for now (due to complexity)
  if (mat) {
    // get material name
    if (!mat->Attribute("name")) {
      ESP_VERY_VERBOSE() << "Visual material must contain a name attribute";
      return false;
    }
    visual.m_materialName = mat->Attribute("name");

    // try to parse material element in place
    const XMLElement* t = mat->FirstChildElement("texture");
    const XMLElement* c = mat->FirstChildElement("color");
    const XMLElement* s = mat->FirstChildElement("specular");
    if (t || c || s) {
      if (!visual.m_geometry.m_localMaterial) {
        // create a new material
        visual.m_geometry.m_localMaterial = std::make_shared<Material>();
      }
      if (parseMaterial(*visual.m_geometry.m_localMaterial, mat)) {
        // override if not new
        model->m_materials[visual.m_materialName] =
            visual.m_geometry.m_localMaterial;
        visual.m_geometry.m_hasLocalMaterial = true;
      }
    } else {
      auto matIter = model->m_materials.find(visual.m_materialName);
      if (matIter != model->m_materials.end()) {
        // need this to handle possible overwriting of this material name in a
        // later call.
        visual.m_geometry.m_localMaterial = matIter->second;
        visual.m_geometry.m_hasLocalMaterial = true;
      } else {
        ESP_VERY_VERBOSE() << "Warning: visual element \"" << visual.m_name
                           << "\" specified un-defined material name \""
                           << visual.m_materialName << "\".";
      }
    }
  }
  return true;
}

bool Parser::parseTransform(Mn::Matrix4& tr, const XMLElement* xml) const {
  tr = Mn::Matrix4();  // Identity

  Mn::Vector3 vec(0, 0, 0);

  const char* xyz_str = xml->Attribute("xyz");
  if (xyz_str) {
    parseVector3(vec, std::string(xyz_str));
  }

  tr.translation() = vec;

  const char* rpy_str = xml->Attribute("rpy");
  if (rpy_str != nullptr) {
    Mn::Vector3 rpy;
    if (parseVector3(rpy, std::string(rpy_str))) {
      double roll = rpy[0];
      double pitch = rpy[1];
      double yaw = rpy[2];

      double phi = roll / 2.0;
      double the = pitch / 2.0;
      double psi = yaw / 2.0;

      Mn::Quaternion orn(
          Mn::Vector3(
              sin(phi) * cos(the) * cos(psi) - cos(phi) * sin(the) * sin(psi),
              cos(phi) * sin(the) * cos(psi) + sin(phi) * cos(the) * sin(psi),
              cos(phi) * cos(the) * sin(psi) - sin(phi) * sin(the) * cos(psi)),
          cos(phi) * cos(the) * cos(psi) + sin(phi) * sin(the) * sin(psi));

      tr = Magnum::Matrix4::from(orn.normalized().toMatrix(), tr.translation());
    }
  }

  return true;
}

bool Parser::parseGeometry(Geometry& geom, const XMLElement* g) {
  if (g == nullptr)
    return false;

  const XMLElement* shape = g->FirstChildElement();
  if (!shape) {
    ESP_VERY_VERBOSE() << "Geometry tag contains no child element.";
    return false;
  }

  // const std::string type_name = shape->ValueTStr().c_str();
  const std::string type_name = shape->Value();
  if (type_name == "sphere") {
    geom.m_type = GEOM_SPHERE;

    if (!shape->Attribute("radius")) {
      ESP_VERY_VERBOSE() << "Sphere shape must have a radius attribute";
      return false;
    } else {
      geom.m_sphereRadius = std::stod(shape->Attribute("radius"));
    }
  } else if (type_name == "box") {
    geom.m_type = GEOM_BOX;
    if (!shape->Attribute("size")) {
      ESP_VERY_VERBOSE() << "Box requires a size attribute";
      return false;
    } else {
      parseVector3(geom.m_boxSize, shape->Attribute("size"));
    }
  } else if (type_name == "cylinder") {
    geom.m_type = GEOM_CYLINDER;
    geom.m_capsuleRadius = 0.1;
    geom.m_capsuleHeight = 0.1;

    if (!shape->Attribute("length") || !shape->Attribute("radius")) {
      ESP_VERY_VERBOSE()
          << "Cylinder shape must have both length and radius attributes";
      return false;
    }
    geom.m_capsuleRadius = std::stod(shape->Attribute("radius"));
    geom.m_capsuleHeight = std::stod(shape->Attribute("length"));

  } else if (type_name == "capsule") {
    geom.m_type = GEOM_CAPSULE;

    if (!shape->Attribute("length") || !shape->Attribute("radius")) {
      ESP_VERY_VERBOSE()
          << "Capsule shape must have both length and radius attributes";
      return false;
    }
    geom.m_capsuleRadius = std::stod(shape->Attribute("radius"));
    geom.m_capsuleHeight = std::stod(shape->Attribute("length"));

  } else if (type_name == "mesh") {
    geom.m_type = GEOM_MESH;
    geom.m_meshScale = Mn::Vector3(1.0, 1.0, 1.0);
    std::string fn;

    if (shape->Attribute("filename")) {
      fn = shape->Attribute("filename");
    }
    if (shape->Attribute("scale")) {
      if (!parseVector3(geom.m_meshScale, shape->Attribute("scale"))) {
        ESP_VERY_VERBOSE() << "W - Scale should be a vector3, not "
                              "single scalar. Workaround activated.";
        std::string scalar_str = shape->Attribute("scale");
        double scaleFactor = std::stod(scalar_str);
        if (scaleFactor != 0.0) {
          geom.m_meshScale = Mn::Vector3(scaleFactor);
        }
      }
    }

    if (fn.empty()) {
      ESP_VERY_VERBOSE() << "Mesh filename is empty";
      return false;
    }

    geom.m_meshFileName = fn;

    // load the mesh and modify the filename to full path to reference loaded
    // asset
    bool success = validateMeshFile(geom.m_meshFileName);

    if (!success) {
      // warning already printed
      return false;
    }
  } else {
    if (type_name == "plane") {
      geom.m_type = GEOM_PLANE;

      if (!shape->Attribute("normal")) {
        ESP_VERY_VERBOSE() << "Plane requires a normal attribute";
        return false;
      } else {
        parseVector3(geom.m_planeNormal, shape->Attribute("normal"));
      }
    } else {
      ESP_VERY_VERBOSE() << "Unknown geometry type:" << type_name;
      return false;
    }
  }

  return true;
}

bool Parser::parseInertia(Inertia& inertia, const XMLElement* config) {
  inertia.m_linkLocalFrame = Mn::Matrix4();  // Identity
  inertia.m_mass = 0.f;

  // Origin
  const XMLElement* o = config->FirstChildElement("origin");
  if (o) {
    if (!parseTransform(inertia.m_linkLocalFrame, o)) {
      return false;
    }
    inertia.m_hasLinkLocalFrame = true;
  }

  const XMLElement* mass_xml = config->FirstChildElement("mass");
  if (!mass_xml) {
    ESP_VERY_VERBOSE() << "Inertial element must have a mass element";
    return false;
  }

  if (!mass_xml->Attribute("value")) {
    ESP_VERY_VERBOSE() << "Inertial: mass element must have value attribute";
    return false;
  }

  inertia.m_mass = std::stod(mass_xml->Attribute("value"));

  const XMLElement* inertia_xml = config->FirstChildElement("inertia");
  if (!inertia_xml) {
    ESP_VERY_VERBOSE() << "Inertial element must have inertia element";
    return false;
  }

  if (!(inertia_xml->Attribute("ixx") && inertia_xml->Attribute("ixy") &&
        inertia_xml->Attribute("ixz") && inertia_xml->Attribute("iyy") &&
        inertia_xml->Attribute("iyz") && inertia_xml->Attribute("izz"))) {
    if ((inertia_xml->Attribute("ixx") && inertia_xml->Attribute("iyy") &&
         inertia_xml->Attribute("izz"))) {
      inertia.m_ixx = std::stod(inertia_xml->Attribute("ixx"));
      inertia.m_ixy = 0;
      inertia.m_ixz = 0;
      inertia.m_iyy = std::stod(inertia_xml->Attribute("iyy"));
      inertia.m_iyz = 0;
      inertia.m_izz = std::stod(inertia_xml->Attribute("izz"));
    } else {
      ESP_VERY_VERBOSE() << "Inertial: inertia element must have "
                            "ixx,ixy,ixz,iyy,iyz,izz attributes";
      return false;
    }
  } else {
    inertia.m_ixx = std::stod(inertia_xml->Attribute("ixx"));
    inertia.m_ixy = std::stod(inertia_xml->Attribute("ixy"));
    inertia.m_ixz = std::stod(inertia_xml->Attribute("ixz"));
    inertia.m_iyy = std::stod(inertia_xml->Attribute("iyy"));
    inertia.m_iyz = std::stod(inertia_xml->Attribute("iyz"));
    inertia.m_izz = std::stod(inertia_xml->Attribute("izz"));
  }

  return true;
}

bool Parser::validateMeshFile(std::string& meshFilename) {
  std::string urdfDirectory =
      sourceFilePath_.substr(0, sourceFilePath_.find_last_of('/'));

  std::string meshFilePath =
      Corrade::Utility::Path::join(urdfDirectory, meshFilename);

  bool meshSuccess = false;
  // defer asset loading to instancing time. Check asset file existence here.
  meshSuccess = Corrade::Utility::Path::exists(meshFilePath);

  if (meshSuccess) {
    // modify the meshFilename to full filepath to enable access into
    // ResourceManager assets later.
    meshFilename = meshFilePath;
  } else {
    ESP_VERY_VERBOSE() << "Mesh file \"" << meshFilePath << "\"does not exist.";
  }

  return meshSuccess;
}

bool Parser::initTreeAndRoot(const std::shared_ptr<Model>& model) const {
  // every link has children links and joints, but no parents, so we create a
  // local convenience data structure for keeping child->parent relations
  std::map<std::string, std::string> parentLinkTree;

  // loop through all joints, for every link, assign children links and children
  // joints
  for (auto itr = model->m_joints.begin(); itr != model->m_joints.end();
       ++itr) {
    auto joint = itr->second;

    std::string parent_link_name = joint->m_parentLinkName;
    std::string child_link_name = joint->m_childLinkName;
    if (parent_link_name.empty() || child_link_name.empty()) {
      ESP_VERY_VERBOSE() << "Parent link or child link is empty for joint:"
                         << joint->m_name;
      return false;
    }

    if (model->m_links.count(joint->m_childLinkName) == 0u) {
      ESP_VERY_VERBOSE() << "Cannot find child link for joint:" << joint->m_name
                         << ", child:" << joint->m_childLinkName;
      return false;
    }
    auto childLink = model->m_links.at(joint->m_childLinkName);

    if (model->m_links.count(joint->m_parentLinkName) == 0u) {
      ESP_VERY_VERBOSE() << "Cannot find parent link for joint:"
                         << joint->m_name
                         << ", parent:" << joint->m_parentLinkName;
      return false;
    }
    auto parentLink = model->m_links.at(joint->m_parentLinkName);

    childLink->m_parentLink = parentLink;

    childLink->m_parentJoint = joint;
    parentLink->m_childJoints.push_back(joint);
    parentLink->m_childLinks.push_back(childLink);
    parentLinkTree[childLink->m_name] = parentLink->m_name;
  }

  // search for children that have no parent, those are 'root'
  for (auto itr = model->m_links.begin(); itr != model->m_links.end(); ++itr) {
    auto link = itr->second;
    if (!link->m_parentLink.lock()) {
      model->m_rootLinks.push_back(link);
    }
  }

  if (model->m_rootLinks.size() > 1) {
    ESP_VERY_VERBOSE() << "W - URDF file with multiple root links found:";

    for (size_t i = 0; i < model->m_rootLinks.size(); ++i) {
      ESP_VERY_VERBOSE() << model->m_rootLinks[i]->m_name;
    }
  }

  if (model->m_rootLinks.empty()) {
    ESP_VERY_VERBOSE() << "URDF without root link found.";
    return false;
  }
  return true;
}

bool Parser::parseJointLimits(Joint& joint,
                              const tinyxml2::XMLElement* config) const {
  joint.m_lowerLimit = 0.f;
  joint.m_upperLimit = -1.f;
  joint.m_effortLimit = 0.f;
  joint.m_velocityLimit = 0.f;
  joint.m_jointDamping = 0.f;
  joint.m_jointFriction = 0.f;

  const char* lower_str = config->Attribute("lower");
  if (lower_str) {
    joint.m_lowerLimit = std::stod(lower_str);
  }

  const char* upper_str = config->Attribute("upper");
  if (upper_str) {
    joint.m_upperLimit = std::stod(upper_str);
  }

  // Get joint effort limit
  const char* effort_str = config->Attribute("effort");
  if (effort_str) {
    joint.m_effortLimit = std::stod(effort_str);
  }

  // Get joint velocity limit
  const char* velocity_str = config->Attribute("velocity");
  if (velocity_str) {
    joint.m_velocityLimit = std::stod(velocity_str);
  }

  return true;
}

bool Parser::parseJointDynamics(Joint& joint,
                                const tinyxml2::XMLElement* config) const {
  joint.m_jointDamping = 0;
  joint.m_jointFriction = 0;

  // Get joint damping
  const char* damping_str = config->Attribute("damping");
  if (damping_str) {
    joint.m_jointDamping = std::stod(damping_str);
  }

  // Get joint friction
  const char* friction_str = config->Attribute("friction");
  if (friction_str) {
    joint.m_jointFriction = std::stod(friction_str);
  }

  if (damping_str == nullptr && friction_str == nullptr) {
    ESP_VERY_VERBOSE() << "Joint dynamics element specified with no "
                          "damping and no friction";
    return false;
  }

  return true;
}

bool Parser::parseJoint(Joint& joint, const tinyxml2::XMLElement* config) {
  // Get Joint Name
  const char* name = config->Attribute("name");
  if (!name) {
    ESP_VERY_VERBOSE() << "Unnamed joint found";
    return false;
  }
  joint.m_name = name;
  joint.m_parentLinkToJointTransform = Mn::Matrix4();  // Identity

  // Get transform from Parent Link to Joint Frame
  const XMLElement* origin_xml = config->FirstChildElement("origin");
  if (origin_xml) {
    if (!parseTransform(joint.m_parentLinkToJointTransform, origin_xml)) {
      ESP_VERY_VERBOSE() << "Malformed parent origin element for joint:"
                         << joint.m_name;
      return false;
    }
  }

  // Get Parent Link
  const XMLElement* parent_xml = config->FirstChildElement("parent");
  if (parent_xml) {
    const char* pname = parent_xml->Attribute("link");
    if (!pname) {
      ESP_VERY_VERBOSE() << "No parent link name specified for "
                            "Joint link. this might be the root?"
                         << joint.m_name;
      return false;
    } else {
      joint.m_parentLinkName = std::string(pname);
    }
  }

  // Get Child Link
  const XMLElement* child_xml = config->FirstChildElement("child");
  if (child_xml) {
    const char* pname = child_xml->Attribute("link");
    if (!pname) {
      ESP_VERY_VERBOSE() << "No child link name specified for Joint link"
                         << joint.m_name;
      return false;
    } else {
      joint.m_childLinkName = std::string(pname);
    }
  }

  // Get Joint type
  const char* type_char = config->Attribute("type");
  if (!type_char) {
    ESP_VERY_VERBOSE() << "Joint" << joint.m_name
                       << "has no type, check to see if it's a reference.";
    return false;
  }

  std::string type_str = type_char;
  if (type_str == "spherical")
    joint.m_type = SphericalJoint;
  else if (type_str == "planar")
    joint.m_type = PlanarJoint;
  else if (type_str == "floating")
    joint.m_type = FloatingJoint;
  else if (type_str == "revolute")
    joint.m_type = RevoluteJoint;
  else if (type_str == "continuous")
    joint.m_type = ContinuousJoint;
  else if (type_str == "prismatic")
    joint.m_type = PrismaticJoint;
  else if (type_str == "fixed")
    joint.m_type = FixedJoint;
  else {
    ESP_VERY_VERBOSE() << "Joint" << joint.m_name
                       << "has unknown type:" << type_str;
    return false;
  }

  // Get Joint Axis
  if (joint.m_type != FloatingJoint && joint.m_type != FixedJoint) {
    // axis
    const XMLElement* axis_xml = config->FirstChildElement("axis");
    if (!axis_xml) {
      ESP_VERY_VERBOSE() << "W - urdfdom: no axis elemement for Joint, "
                            "defaulting to (1,0,0) axis"
                         << joint.m_name;
      joint.m_localJointAxis = Mn::Vector3(1, 0, 0);
    } else {
      if (axis_xml->Attribute("xyz")) {
        if (!parseVector3(joint.m_localJointAxis, axis_xml->Attribute("xyz"))) {
          ESP_VERY_VERBOSE()
              << "Malformed axis element:" << axis_xml->Attribute("xyz")
              << "for joint:" << joint.m_name;
          return false;
        }
      }
    }
  }

  // Get limit
  const XMLElement* limit_xml = config->FirstChildElement("limit");
  if (limit_xml) {
    if (!parseJointLimits(joint, limit_xml)) {
      ESP_VERY_VERBOSE() << "Could not parse limit element for joint:"
                         << joint.m_name;
      return false;
    }
  } else if (joint.m_type == RevoluteJoint) {
    ESP_VERY_VERBOSE()
        << "Joint is of type REVOLUTE but it does not specify limits:"
        << joint.m_name;
    return false;
  } else if (joint.m_type == PrismaticJoint) {
    ESP_VERY_VERBOSE() << "Joint is of type PRISMATIC without limits:"
                       << joint.m_name;
    return false;
  }

  joint.m_jointDamping = 0;
  joint.m_jointFriction = 0;

  // Get Dynamics
  const XMLElement* prop_xml = config->FirstChildElement("dynamics");
  if (prop_xml) {
    // Get joint damping
    const char* damping_str = prop_xml->Attribute("damping");
    if (damping_str) {
      joint.m_jointDamping = std::stod(damping_str);
    }

    // Get joint friction
    const char* friction_str = prop_xml->Attribute("friction");
    if (friction_str) {
      joint.m_jointFriction = std::stod(friction_str);
    }

    if (damping_str == nullptr && friction_str == nullptr) {
      ESP_VERY_VERBOSE() << "Joint dynamics element specified with "
                            "no damping and no friction";
      return false;
    }
  }

  return true;
}

void printLinkChildrenHelper(Link& link, const std::string& printPrefix = "") {
  // ESP_VERY_VERBOSE() << printPrefix<<"link "<< link.m_name;
  int childIndex = 0;
  for (auto& child : link.m_childJoints) {
    ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
        << printPrefix << "child J(" << childIndex
        << "):" << child.lock()->m_name << "->("
        << child.lock()->m_childLinkName << ")";
    ++childIndex;
  }
  childIndex = 0;
  for (auto& child : link.m_childLinks) {
    ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
        << printPrefix << "child L(" << childIndex
        << "):" << child.lock()->m_name;
    printLinkChildrenHelper(*(child.lock()), printPrefix + "  ");
    ++childIndex;
  }
}

void Model::printKinematicChain() const {
  ESP_VERY_VERBOSE()
      << "------------------------------------------------------";
  ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
      << "Model::printKinematicChain: model =" << m_name;
  int rootIndex = 0;
  for (const auto& root : m_rootLinks) {
    ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
        << "root L(" << rootIndex << "):" << root->m_name;
    printLinkChildrenHelper(*root);
    ++rootIndex;
  }
  ESP_VERY_VERBOSE()
      << "------------------------------------------------------";
}

}  // namespace URDF
}  // namespace metadata
}  // namespace esp
