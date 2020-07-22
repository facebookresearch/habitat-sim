// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SceneAttributesManager.h"
#include "AttributesManagerBase.h"

#include "esp/io/io.h"
#include "esp/io/json.h"

namespace esp {
namespace assets {

namespace managers {

PhysicsSceneAttributes::ptr SceneAttributesManager::createAttributesTemplate(
    const std::string& sceneAttributesHandle,
    bool registerTemplate) {
  PhysicsSceneAttributes::ptr attrs;
  std::string msg;
  std::string strHandle = Cr::Utility::String::lowercase(sceneAttributesHandle);
  bool fileExists = (this->isValidFileName(sceneAttributesHandle));
  if (objectAttributesMgr_->isValidPrimitiveAttributes(sceneAttributesHandle)) {
    // if sceneAttributesHandle == some existing primitive attributes, then
    // this is a primitive-based scene (i.e. a plane) we are building
    attrs = createPrimBasedAttributesTemplate(sceneAttributesHandle,
                                              registerTemplate);
    msg = "Primitive Asset (" + sceneAttributesHandle + ") Based";

  } else if (fileExists) {
    if ((strHandle.find(".json") != std::string::npos) && fileExists) {
      // check if sceneAttributesHandle corresponds to an actual, existing json
      // scene file descriptor.
      attrs = createJSONFileBasedAttributesTemplate(sceneAttributesHandle,
                                                    registerTemplate);
      msg = "JSON File (" + sceneAttributesHandle + ") Based";
    } else {
      // if name is not json file descriptor but still appropriate file
      attrs = createFileBasedAttributesTemplate(sceneAttributesHandle,
                                                registerTemplate);
      msg = "File (" + sceneAttributesHandle + ") Based";
    }

  } else {
    // if name is not file descriptor, return default attributes.
    attrs = createDefaultAttributesTemplate(sceneAttributesHandle,
                                            registerTemplate);
    msg = "New default";
  }

  if (nullptr != attrs) {
    LOG(INFO) << msg << " scene attributes created"
              << (registerTemplate ? " and registered." : ".");
  }
  return attrs;

}  // SceneAttributesManager::createAttributesTemplate

int SceneAttributesManager::registerAttributesTemplateFinalize(
    PhysicsSceneAttributes::ptr sceneAttributesTemplate,
    const std::string& sceneAttributesHandle) {
  if (sceneAttributesTemplate->getRenderAssetHandle() == "") {
    LOG(ERROR)
        << "SceneAttributesManager::registerAttributesTemplateFinalize : "
           "Attributes template named"
        << sceneAttributesHandle
        << "does not have a valid render asset handle specified. Aborting.";
    return ID_UNDEFINED;
  }

  // Handles for rendering and collision assets
  std::string renderAssetHandle =
      sceneAttributesTemplate->getRenderAssetHandle();
  std::string collisionAssetHandle =
      sceneAttributesTemplate->getCollisionAssetHandle();

  // verify these represent legitimate assets
  if (objectAttributesMgr_->isValidPrimitiveAttributes(renderAssetHandle)) {
    // If renderAssetHandle corresponds to valid/existing primitive attributes
    // then setRenderAssetIsPrimitive to true and set map of IDs->Names to
    // physicsSynthObjTmpltLibByID_
    sceneAttributesTemplate->setRenderAssetIsPrimitive(true);
  } else if (this->isValidFileName(renderAssetHandle)) {
    // Check if renderAssetHandle is valid file name and is found in file system
    // - if so then setRenderAssetIsPrimitive to false and set map of IDs->Names
    // to physicsFileObjTmpltLibByID_ - verify file  exists
    sceneAttributesTemplate->setRenderAssetIsPrimitive(false);
  } else {
    // If renderAssetHandle is not valid file name needs to  fail
    LOG(ERROR)
        << "SceneAttributesManager::registerAttributesTemplateFinalize "
           ": Render asset template handle : "
        << renderAssetHandle << " specified in scene template with handle : "
        << sceneAttributesHandle
        << " does not correspond to any existing file or primitive render "
           "asset.  Aborting. ";
    return ID_UNDEFINED;
  }

  if (objectAttributesMgr_->isValidPrimitiveAttributes(collisionAssetHandle)) {
    // If collisionAssetHandle corresponds to valid/existing primitive
    // attributes then setCollisionAssetIsPrimitive to true
    sceneAttributesTemplate->setCollisionAssetIsPrimitive(true);
  } else if (this->isValidFileName(collisionAssetHandle)) {
    // Check if collisionAssetHandle is valid file name and is found in file
    // system - if so then setCollisionAssetIsPrimitive to false
    sceneAttributesTemplate->setCollisionAssetIsPrimitive(false);
  } else {
    // Else, means no collision data specified, use specified render data
    // Else, means no collision data specified, use specified render data
    LOG(INFO)
        << "SceneAttributesManager::registerAttributesTemplateFinalize "
           ": Collision asset template handle : "
        << collisionAssetHandle << " specified in scene template with handle : "
        << sceneAttributesHandle
        << " does not correspond to any existing file or primitive render "
           "asset.  Overriding with given render asset handle : "
        << renderAssetHandle << ". ";

    sceneAttributesTemplate->setCollisionAssetHandle(renderAssetHandle);
    sceneAttributesTemplate->setCollisionAssetIsPrimitive(
        sceneAttributesTemplate->getRenderAssetIsPrimitive());
  }
  // Clear dirty flag from when asset handles are changed
  sceneAttributesTemplate->setIsClean();

  // adds template to library, and returns either the ID of the existing
  // template referenced by sceneAttributesHandle, or the next available ID
  // if not found.
  int sceneTemplateID = this->addTemplateToLibrary(sceneAttributesTemplate,
                                                   sceneAttributesHandle);
  return sceneTemplateID;
}  // SceneAttributesManager::registerAttributesTemplate

PhysicsSceneAttributes::ptr
SceneAttributesManager::createDefaultAttributesTemplate(
    const std::string& sceneFilename,
    bool registerTemplate) {
  // Attributes descriptor for scene
  PhysicsSceneAttributes::ptr sceneAttributesTemplate =
      PhysicsSceneAttributes::create(sceneFilename);

  sceneAttributesTemplate->setRenderAssetHandle(sceneFilename);
  sceneAttributesTemplate->setCollisionAssetHandle(sceneFilename);

  if (registerTemplate) {
    int attrID = this->registerAttributesTemplate(sceneAttributesTemplate,
                                                  sceneFilename);
    if (attrID == ID_UNDEFINED) {
      // some error occurred
      return nullptr;
    }
  }
  return sceneAttributesTemplate;
}  // SceneAttributesManager::createDefaultAttributesTemplate

PhysicsSceneAttributes::ptr
SceneAttributesManager::createPrimBasedAttributesTemplate(
    const std::string& primAttrTemplateHandle,
    bool registerTemplate) {
  PhysicsSceneAttributes::ptr sceneAttributes =
      buildPrimBasedPhysObjTemplate(primAttrTemplateHandle);

  if (nullptr != sceneAttributes && registerTemplate) {
    int attrID = this->registerAttributesTemplate(sceneAttributes,
                                                  primAttrTemplateHandle);
    if (attrID == ID_UNDEFINED) {
      // some error occurred
      return nullptr;
    }
  }
  return sceneAttributes;
}  // SceneAttributesManager::createPrimBasedAttributesTemplate

PhysicsSceneAttributes::ptr
SceneAttributesManager::buildPrimBasedPhysObjTemplate(
    const std::string& primAssetHandle) {
  // verify that a primitive asset with the given handle exists
  if (!objectAttributesMgr_->isValidPrimitiveAttributes(primAssetHandle)) {
    LOG(ERROR) << "SceneAttributesManager::buildPrimBasedPhysObjTemplate : No "
                  "primitive with handle '"
               << primAssetHandle
               << "' exists so cannot build physical object.  Aborting.";
    return nullptr;
  }

  // construct a physicsSceneAttributes
  auto physicsSceneAttributes = PhysicsSceneAttributes::create(primAssetHandle);
  // set margin to be 0
  physicsSceneAttributes->setMargin(0.0);
  // make smaller as default size - prims are approx meter in size
  physicsSceneAttributes->setScale({0.1, 0.1, 0.1});

  // set render mesh handle
  physicsSceneAttributes->setRenderAssetHandle(primAssetHandle);
  // set collision mesh/primitive handle and default for primitives to not use
  // mesh collisions
  physicsSceneAttributes->setCollisionAssetHandle(primAssetHandle);
  physicsSceneAttributes->setUseMeshCollision(false);
  // NOTE to eventually use mesh collisions with primitive objects, a
  // collision primitive mesh needs to be configured and set in MeshMetaData
  // and CollisionMesh
  return physicsSceneAttributes;
}  // SceneAttributesManager::buildPrimBasedPhysObjTemplate

PhysicsSceneAttributes::ptr
SceneAttributesManager::createFileBasedAttributesTemplate(
    const std::string& sceneFilename,
    bool registerTemplate) {
  // Attributes descriptor for scene
  PhysicsSceneAttributes::ptr sceneAttributesTemplate =
      PhysicsSceneAttributes::create(sceneFilename);

  sceneAttributesTemplate->setRenderAssetHandle(sceneFilename);
  sceneAttributesTemplate->setCollisionAssetHandle(sceneFilename);

  // TODO : any specific non-json file-based parsing required

  if (registerTemplate) {
    int attrID = this->registerAttributesTemplate(sceneAttributesTemplate,
                                                  sceneFilename);
    if (attrID == ID_UNDEFINED) {
      // some error occurred
      return nullptr;
    }
  }

  return sceneAttributesTemplate;
}  // SceneAttributesManager::createFileBasedAttributesTemplate

PhysicsSceneAttributes::ptr
SceneAttributesManager::createJSONFileBasedAttributesTemplate(
    const std::string& sceneFilename,
    bool registerTemplate) {
  // Attributes descriptor for scene
  PhysicsSceneAttributes::ptr sceneAttributesTemplate =
      PhysicsSceneAttributes::create(sceneFilename);

  sceneAttributesTemplate->setRenderAssetHandle(sceneFilename);
  sceneAttributesTemplate->setCollisionAssetHandle(sceneFilename);

  // Load the scene config JSON here
  io::JsonDocument scenePhysicsConfig = io::parseJsonFile(sceneFilename);

  // TODO JSON parsing here

  if (registerTemplate) {
    int attrID = this->registerAttributesTemplate(sceneAttributesTemplate,
                                                  sceneFilename);
    if (attrID == ID_UNDEFINED) {
      // some error occurred
      return nullptr;
    }
  }

  return sceneAttributesTemplate;
}  // SceneAttributesManager::createFileBasedAttributesTemplate

// template class AttributesManager<PhysicsSceneAttributes>;

}  // namespace managers
}  // namespace assets
}  // namespace esp
