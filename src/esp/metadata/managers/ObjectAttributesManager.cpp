// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "ObjectAttributesManager.h"
#include "AbstractObjectAttributesManagerBase.h"

#include <Corrade/Utility/String.h>

#include "esp/assets/Asset.h"
#include "esp/io/io.h"
#include "esp/io/json.h"

using std::placeholders::_1;
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
    LOG(ERROR)
        << "ObjectAttributesManager::createPrimBasedAttributesTemplate : No "
           "primitive with handle '"
        << primAttrTemplateHandle
        << "' exists so cannot build physical object.  Aborting.";
    return nullptr;
  }

  // construct a ObjectAttributes
  auto primObjectAttributes =
      this->initNewObjectInternal(primAttrTemplateHandle);
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

  return this->postCreateRegister(primObjectAttributes, registerTemplate);
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
    this->undeletableObjectNames_.insert(tmpltHandle);
  }
}  // ObjectAttributesManager::createDefaultPrimBasedAttributesTemplates

void ObjectAttributesManager::setValsFromJSONDoc(
    attributes::ObjectAttributes::ptr objAttributes,
    const io::JsonGenericValue& jsonConfig) {
  this->loadAbstractObjectAttributesFromJson(objAttributes, jsonConfig);

  // Populate with object-specific fields found in json, if any are there.
  // object mass
  io::jsonIntoSetter<double>(
      jsonConfig, "mass",
      std::bind(&ObjectAttributes::setMass, objAttributes, _1));
  // linear damping
  io::jsonIntoSetter<double>(
      jsonConfig, "linear_damping",
      std::bind(&ObjectAttributes::setLinearDamping, objAttributes, _1));
  // angular damping
  io::jsonIntoSetter<double>(
      jsonConfig, "angular_damping",
      std::bind(&ObjectAttributes::setAngularDamping, objAttributes, _1));
  // Use bounding box as collision object
  io::jsonIntoSetter<bool>(
      jsonConfig, "use_bounding_box_for_collision",
      std::bind(&ObjectAttributes::setBoundingBoxCollisions, objAttributes,
                _1));

  // Join collision meshes if specified
  io::jsonIntoSetter<bool>(
      jsonConfig, "join_collision_meshes",
      std::bind(&ObjectAttributes::setJoinCollisionMeshes, objAttributes, _1));

  // The object's interia matrix diagonal
  io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonConfig, "inertia",
      std::bind(&ObjectAttributes::setInertia, objAttributes, _1));

  // The center of mass (in the local frame of the object)
  // if COM is provided, use it for mesh shift
  bool comIsSet = io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonConfig, "COM",
      std::bind(&ObjectAttributes::setCOM, objAttributes, _1));
  // if com is set from json, don't compute from shape, and vice versa
  objAttributes->setComputeCOMFromShape(!comIsSet);
}  // ObjectAttributesManager::setValsFromJSONDoc

ObjectAttributes::ptr ObjectAttributesManager::initNewObjectInternal(
    const std::string& attributesHandle) {
  ObjectAttributes::ptr newAttributes =
      this->constructFromDefault(attributesHandle);
  if (nullptr == newAttributes) {
    newAttributes = ObjectAttributes::create(attributesHandle);
  }
  this->setFileDirectoryFromHandle(newAttributes);
  // set default render asset handle
  newAttributes->setRenderAssetHandle(attributesHandle);
  // set default collision asset handle
  newAttributes->setCollisionAssetHandle(attributesHandle);
  // set defaults for passed render asset handles
  this->setDefaultAssetNameBasedAttributes(
      newAttributes, true, newAttributes->getRenderAssetHandle(),
      std::bind(&AbstractObjectAttributes::setRenderAssetType, newAttributes,
                _1));
  // set defaults for passed collision asset handles
  this->setDefaultAssetNameBasedAttributes(
      newAttributes, false, newAttributes->getCollisionAssetHandle(),
      std::bind(&AbstractObjectAttributes::setCollisionAssetType, newAttributes,
                _1));

  return newAttributes;
}  // ObjectAttributesManager::initNewObjectInternal

// Eventually support explicitly configuring desirable defaults/file-name
// base settings.
void ObjectAttributesManager::setDefaultAssetNameBasedAttributes(
    ObjectAttributes::ptr attributes,
    bool setFrame,
    const std::string& meshHandle,
    std::function<void(int)> assetTypeSetter) {
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
}  // SceneAttributesManager::setDefaultAssetNameBasedAttributes

int ObjectAttributesManager::registerObjectFinalize(
    ObjectAttributes::ptr objectTemplate,
    const std::string& objectTemplateHandle) {
  if (objectTemplate->getRenderAssetHandle() == "") {
    LOG(ERROR)
        << "ObjectAttributesManager::registerObjectFinalize : "
           "Attributes template named"
        << objectTemplateHandle
        << "does not have a valid render asset handle specified. Aborting.";
    return ID_UNDEFINED;
  }

  std::map<int, std::string>* mapToUse;
  // Handles for rendering and collision assets
  std::string renderAssetHandle = objectTemplate->getRenderAssetHandle();
  std::string collisionAssetHandle = objectTemplate->getCollisionAssetHandle();

  if (this->isValidPrimitiveAttributes(renderAssetHandle)) {
    // If renderAssetHandle corresponds to valid/existing primitive attributes
    // then setRenderAssetIsPrimitive to true and set map of IDs->Names to
    // physicsSynthObjTmpltLibByID_
    objectTemplate->setRenderAssetIsPrimitive(true);
    mapToUse = &physicsSynthObjTmpltLibByID_;
  } else if (this->isValidFileName(renderAssetHandle)) {
    // Check if renderAssetHandle is valid file name and is found in file system
    // - if so then setRenderAssetIsPrimitive to false and set map of IDs->Names
    // to physicsFileObjTmpltLibByID_ - verify file  exists
    objectTemplate->setRenderAssetIsPrimitive(false);
    mapToUse = &physicsFileObjTmpltLibByID_;
  } else {
    // If renderAssetHandle is neither valid file name nor existing primitive
    // attributes template hande, fail
    // by here always fail
    LOG(ERROR)
        << "ObjectAttributesManager::registerObjectFinalize "
           ": Render asset template handle : "
        << renderAssetHandle << " specified in object template with handle : "
        << objectTemplateHandle
        << " does not correspond to any existing file or primitive render "
           "asset.  Aborting. ";
    return ID_UNDEFINED;
  }

  if (this->isValidPrimitiveAttributes(collisionAssetHandle)) {
    // If collisionAssetHandle corresponds to valid/existing primitive
    // attributes then setCollisionAssetIsPrimitive to true
    objectTemplate->setCollisionAssetIsPrimitive(true);
  } else if (this->isValidFileName(collisionAssetHandle)) {
    // Check if collisionAssetHandle is valid file name and is found in file
    // system - if so then setCollisionAssetIsPrimitive to false
    objectTemplate->setCollisionAssetIsPrimitive(false);
  } else {
    // Else, means no collision data specified, use specified render data
    LOG(INFO)
        << "ObjectAttributesManager::registerObjectFinalize "
           ": Collision asset template handle : "
        << collisionAssetHandle
        << " specified in object template with handle : "
        << objectTemplateHandle
        << " does not correspond to any existing file or primitive render "
           "asset.  Overriding with given render asset handle : "
        << renderAssetHandle << ". ";

    objectTemplate->setCollisionAssetHandle(renderAssetHandle);
    objectTemplate->setCollisionAssetIsPrimitive(
        objectTemplate->getRenderAssetIsPrimitive());
  }

  // Clear dirty flag from when asset handles are changed
  objectTemplate->setIsClean();

  // Add object template to template library
  int objectTemplateID =
      this->addObjectToLibrary(objectTemplate, objectTemplateHandle);

  mapToUse->emplace(objectTemplateID, objectTemplateHandle);

  return objectTemplateID;
}  // ObjectAttributesManager::registerObjectFinalize

}  // namespace managers
}  // namespace metadata
}  // namespace esp
