// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "ObjectAttributesManager.h"
#include "AbstractObjectAttributesManager.h"

#include <Corrade/Utility/String.h>

#include <utility>

#include "esp/assets/Asset.h"
#include "esp/io/Json.h"

namespace Cr = Corrade;

namespace esp {

using assets::AssetType;

namespace metadata {

using attributes::AbstractObjectAttributes;
using attributes::ObjectAttributes;
namespace managers {

ObjectAttributes::ptr
ObjectAttributesManager::createPrimBasedAttributesTemplate(
    const std::string& primAttrTemplateHandle,
    bool registerTemplate) {
  // verify that a primitive asset with the given handle exists
  if (!this->isValidPrimitiveAttributes(primAttrTemplateHandle)) {
    ESP_ERROR(Mn::Debug::Flag::NoSpace)
        << "No primitive with handle `" << primAttrTemplateHandle
        << "` exists so cannot build physical object, so "
           "createPrimBasedAttributesTemplate for object aborted.";
    return nullptr;
  }

  // construct a ObjectAttributes based on prim handle
  auto primObjectAttributes =
      this->initNewObjectInternal(primAttrTemplateHandle, false);
  // set margin to be 0
  primObjectAttributes->setMargin(0.0);
  // make smaller as default size - prims are approx meter in size
  primObjectAttributes->setScale({0.1, 0.1, 0.1});

  // set render mesh handle
  int primType = static_cast<int>(AssetType::PRIMITIVE);
  primObjectAttributes->setRenderAssetType(primType);
  // set collision mesh/primitive handle and default for primitives to not use
  // mesh collisions
  primObjectAttributes->setCollisionAssetType(primType);
  primObjectAttributes->setUseMeshCollision(false);
  // NOTE to eventually use mesh collisions with primitive objects, a
  // collision primitive mesh needs to be configured and set in MeshMetaData
  // and CollisionMesh

  return this->postCreateRegister(std::move(primObjectAttributes),
                                  registerTemplate);
}  // ObjectAttributesManager::createPrimBasedAttributesTemplate

void ObjectAttributesManager::createDefaultPrimBasedAttributesTemplates() {
  this->undeletableObjectNames_.clear();
  // build default primtive object templates corresponding to given default
  // asset templates
  std::vector<std::string> lib =
      assetAttributesMgr_->getUndeletableObjectHandles();
  for (const std::string& elem : lib) {
    auto tmplt = createPrimBasedAttributesTemplate(elem, true);
    // save handles in list of defaults, so they are not removed
    std::string tmpltHandle = tmplt->getHandle();
    this->undeletableObjectNames_.insert(std::move(tmpltHandle));
  }
}  // ObjectAttributesManager::createDefaultPrimBasedAttributesTemplates

void ObjectAttributesManager::setValsFromJSONDoc(
    attributes::ObjectAttributes::ptr objAttributes,
    const io::JsonGenericValue& jsonConfig) {
  this->setAbstractObjectAttributesFromJson(objAttributes, jsonConfig);

  // Populate with object-specific fields found in json, if any are there.
  // object mass
  io::jsonIntoSetter<double>(jsonConfig, "mass", [objAttributes](double mass) {
    objAttributes->setMass(mass);
  });
  // linear damping
  io::jsonIntoSetter<double>(jsonConfig, "linear_damping",
                             [objAttributes](double linear_damping) {
                               objAttributes->setLinearDamping(linear_damping);
                             });
  // angular damping
  io::jsonIntoSetter<double>(
      jsonConfig, "angular_damping", [objAttributes](double angular_damping) {
        objAttributes->setAngularDamping(angular_damping);
      });
  // Use bounding box as collision object
  io::jsonIntoSetter<bool>(
      jsonConfig, "use_bounding_box_for_collision",
      [objAttributes](bool use_bounding_box_for_collision) {
        objAttributes->setBoundingBoxCollisions(use_bounding_box_for_collision);
      });
  // Join collision meshes if specified
  io::jsonIntoSetter<bool>(
      jsonConfig, "join_collision_meshes",
      [objAttributes](bool join_collision_meshes) {
        objAttributes->setJoinCollisionMeshes(join_collision_meshes);
      });

  // The object's interia matrix diagonal
  io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonConfig, "inertia", [objAttributes](const Magnum::Vector3& inertia) {
        objAttributes->setInertia(inertia);
      });

  // The object's semantic ID
  io::jsonIntoSetter<int>(jsonConfig, "semantic_id",
                          [objAttributes](int semantic_id) {
                            objAttributes->setSemanticId(semantic_id);
                          });

  // The center of mass (in the local frame of the object)
  // if COM is provided, use it for mesh shift
  bool comIsSet = io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonConfig, "COM", [objAttributes](const Magnum::Vector3& COM) {
        objAttributes->setCOM(COM);
      });
  // if com is set from json, don't compute from shape, and vice versa
  objAttributes->setComputeCOMFromShape(!comIsSet);

  // check for user defined attributes
  this->parseUserDefinedJsonVals(objAttributes, jsonConfig);

}  // ObjectAttributesManager::setValsFromJSONDoc

ObjectAttributes::ptr ObjectAttributesManager::initNewObjectInternal(
    const std::string& attributesHandle,
    bool builtFromConfig) {
  ObjectAttributes::ptr newAttributes =
      this->constructFromDefault(attributesHandle);
  bool createNewAttributes = (nullptr == newAttributes);
  if (createNewAttributes) {
    newAttributes = ObjectAttributes::create(attributesHandle);
  }
  // set the attributes source filedirectory, from the attributes name
  this->setFileDirectoryFromHandle(newAttributes);

  if (!createNewAttributes) {
    // default exists and was used to create this attributes - investigate any
    // filename fields that may have %%USE_FILENAME%% directive specified in the
    // default attributes.
    // Render asset handle
    setHandleFromDefaultTag(newAttributes,
                            newAttributes->getRenderAssetHandle(),
                            [newAttributes](const std::string& newHandle) {
                              newAttributes->setRenderAssetHandle(newHandle);
                            });
    // Collision asset handle
    setHandleFromDefaultTag(newAttributes,
                            newAttributes->getCollisionAssetHandle(),
                            [newAttributes](const std::string& newHandle) {
                              newAttributes->setCollisionAssetHandle(newHandle);
                            });
  }

  // set default render and collision asset handle
  // only set handle defaults if attributesHandle is not a config file (which
  // would never be a valid render or collision asset name).  Otherise, expect
  // handles and types to be set when config is read.
  if (!builtFromConfig) {
    if (newAttributes->getRenderAssetHandle().empty()) {
      newAttributes->setRenderAssetHandle(attributesHandle);
    }
    if (newAttributes->getCollisionAssetHandle().empty()) {
      newAttributes->setCollisionAssetHandle(attributesHandle);
    }

    // set defaults for passed render asset handles
    this->setDefaultAssetNameBasedAttributes(
        newAttributes, true, newAttributes->getRenderAssetHandle(),
        [newAttributes](int render_asset_type) {
          newAttributes->setRenderAssetType(render_asset_type);
        });
    // set defaults for passed collision asset handles
    this->setDefaultAssetNameBasedAttributes(
        newAttributes, false, newAttributes->getCollisionAssetHandle(),
        [newAttributes](int collision_asset_type) {
          newAttributes->setCollisionAssetType(collision_asset_type);
        });
  }
  return newAttributes;
}  // ObjectAttributesManager::initNewObjectInternal

// Eventually support explicitly configuring desirable defaults/file-name
// base settings.
void ObjectAttributesManager::setDefaultAssetNameBasedAttributes(
    ObjectAttributes::ptr attributes,
    bool setFrame,
    const std::string& meshHandle,
    const std::function<void(int)>& assetTypeSetter) {
  if (this->isValidPrimitiveAttributes(meshHandle)) {
    // value is valid primitive, and value is different than existing value
    assetTypeSetter(static_cast<int>(AssetType::PRIMITIVE));
  } else {
    // use unknown for object mesh types of non-primitives
    assetTypeSetter(static_cast<int>(AssetType::UNKNOWN));
  }
  if (setFrame) {
    attributes->setOrientUp({0, 1, 0});
    attributes->setOrientFront({0, 0, -1});
  }
}  // ObjectAttributesManager::setDefaultAssetNameBasedAttributes

int ObjectAttributesManager::registerObjectFinalize(
    ObjectAttributes::ptr objectTemplate,
    const std::string& objectTemplateHandle,
    bool forceRegistration) {
  if (objectTemplate->getRenderAssetHandle() == "") {
    ESP_ERROR(Mn::Debug::Flag::NoSpace)
        << "Attributes template named `" << objectTemplateHandle
        << "` does not have a valid render asset handle specified, so "
           "registration is aborted.";
    return ID_UNDEFINED;
  }

  // create a ref to the partition map of either prims or file-based objects to
  // place a ref to the object template being regsitered
  std::unordered_map<int, std::string>* mapToUse = nullptr;
  // Handles for rendering and collision assets
  std::string renderAssetHandle = objectTemplate->getRenderAssetHandle();
  std::string collisionAssetHandle = objectTemplate->getCollisionAssetHandle();

  if (this->isValidPrimitiveAttributes(renderAssetHandle)) {
    // If renderAssetHandle corresponds to valid/existing primitive attributes
    // then setRenderAssetIsPrimitive to true and set map of IDs->Names to
    // physicsSynthObjTmpltLibByID_
    objectTemplate->setRenderAssetIsPrimitive(true);
    mapToUse = &physicsSynthObjTmpltLibByID_;
  } else if (Cr::Utility::Path::exists(renderAssetHandle)) {
    // Check if renderAssetHandle is valid file name and is found in file system
    // - if so then setRenderAssetIsPrimitive to false and set map of IDs->Names
    // to physicsFileObjTmpltLibByID_ - verify file  exists
    objectTemplate->setRenderAssetIsPrimitive(false);
    mapToUse = &physicsFileObjTmpltLibByID_;
  } else if (forceRegistration) {
    // Forcing registration in case of computationaly generated assets
    ESP_WARNING(Mn::Debug::Flag::NoSpace)
        << "Render asset template handle : `" << renderAssetHandle
        << "` specified in object template with handle : `"
        << objectTemplateHandle
        << "` does not correspond to any existing file or primitive render "
           "asset.  Objects created from this template may fail.";
    objectTemplate->setRenderAssetIsPrimitive(false);
  } else {
    // If renderAssetHandle is neither valid file name nor existing primitive
    // attributes template hande, fail
    // by here always fail
    ESP_ERROR(Mn::Debug::Flag::NoSpace)
        << "Render asset template handle : `" << renderAssetHandle
        << "` specified in object template with handle : `"
        << objectTemplateHandle
        << "` does not correspond to any existing file or primitive render "
           "asset, so registration is aborted.";
    return ID_UNDEFINED;
  }

  if (this->isValidPrimitiveAttributes(collisionAssetHandle)) {
    // If collisionAssetHandle corresponds to valid/existing primitive
    // attributes then setCollisionAssetIsPrimitive to true
    objectTemplate->setCollisionAssetIsPrimitive(true);
  } else if (Cr::Utility::Path::exists(collisionAssetHandle)) {
    // Check if collisionAssetHandle is valid file name and is found in file
    // system - if so then setCollisionAssetIsPrimitive to false
    objectTemplate->setCollisionAssetIsPrimitive(false);
  } else {
    // Else, means no collision data specified, use specified render data
    ESP_DEBUG(Mn::Debug::Flag::NoSpace)
        << "Collision asset template handle :" << collisionAssetHandle
        << "specified in object template with handle :`" << objectTemplateHandle
        << "` does not correspond to any existing file or primitive render "
           "asset.  Overriding with given render asset handle :"
        << renderAssetHandle << ".";

    objectTemplate->setCollisionAssetHandle(renderAssetHandle);
    objectTemplate->setCollisionAssetIsPrimitive(
        objectTemplate->getRenderAssetIsPrimitive());
  }

  // Clear dirty flag from when asset handles are changed
  objectTemplate->setIsClean();

  // Add object template to template library
  int objectTemplateID =
      this->addObjectToLibrary(std::move(objectTemplate), objectTemplateHandle);

  if (mapToUse != nullptr) {
    mapToUse->emplace(objectTemplateID, objectTemplateHandle);
  }

  return objectTemplateID;
}  // ObjectAttributesManager::registerObjectFinalize

}  // namespace managers
}  // namespace metadata
}  // namespace esp
