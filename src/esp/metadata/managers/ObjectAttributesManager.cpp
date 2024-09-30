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

namespace metadata {

using attributes::AbstractObjectAttributes;
using attributes::AssetType;
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
  primObjectAttributes->setRenderAssetTypeEnum(AssetType::Primitive);
  // set collision mesh/primitive handle and default for primitives to not use
  // mesh collisions
  primObjectAttributes->setCollisionAssetTypeEnum(AssetType::Primitive);
  primObjectAttributes->setUseMeshCollision(false);
  // NOTE to eventually use mesh collisions with primitive objects, a
  // collision primitive mesh needs to be configured and set in MeshMetaData
  // and CollisionMesh

  return this->postCreateRegister(std::move(primObjectAttributes),
                                  registerTemplate);
}  // ObjectAttributesManager::createPrimBasedAttributesTemplate

void ObjectAttributesManager::createDefaultPrimBasedAttributesTemplates() {
  // build default primtive object templates corresponding to given default
  // asset templates
  std::vector<std::string> lib =
      assetAttributesMgr_->getUndeletableObjectHandles();
  for (const std::string& elem : lib) {
    auto tmplt = createPrimBasedAttributesTemplate(elem, true);
    // save handles in list of defaults, so they are not removed
    std::string tmpltHandle = tmplt->getHandle();
    this->addUndeletableObjectName(std::move(tmpltHandle));
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

  // The object's inertia matrix diagonal
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
    setFilenameFromDefaultTag(newAttributes,
                              newAttributes->getRenderAssetHandle(),
                              [newAttributes](const std::string& newHandle) {
                                newAttributes->setRenderAssetHandle(newHandle);
                              });
    // Collision asset handle
    setFilenameFromDefaultTag(
        newAttributes, newAttributes->getCollisionAssetHandle(),
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
        [newAttributes](AssetType render_asset_type) {
          newAttributes->initRenderAssetTypeEnum(render_asset_type);
        });
    // set defaults for passed collision asset handles
    this->setDefaultAssetNameBasedAttributes(
        newAttributes, false, newAttributes->getCollisionAssetHandle(),
        [newAttributes](AssetType collision_asset_type) {
          newAttributes->initCollisionAssetTypeEnum(collision_asset_type);
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
    const std::function<void(AssetType)>& assetTypeSetter) {
  if (this->isValidPrimitiveAttributes(meshHandle)) {
    // value is valid primitive, and value is different than existing value
    assetTypeSetter(AssetType::Primitive);
  } else {
    // use unknown for object mesh types of non-primitives
    assetTypeSetter(AssetType::Unknown);
  }
  if (setFrame) {
    attributes->init("up", Mn::Vector3{0.0, 1.0, 0.0});
    attributes->init("front", Mn::Vector3{0.0, 0.0, -1.0});
  }
}  // ObjectAttributesManager::setDefaultAssetNameBasedAttributes

core::managedContainers::ManagedObjectPreregistration
ObjectAttributesManager::preRegisterObjectFinalize(
    ObjectAttributes::ptr objectTemplate,
    const std::string& objectTemplateHandle,
    bool forceRegistration) {
  if (objectTemplate->getRenderAssetHandle().empty()) {
    ESP_ERROR(Mn::Debug::Flag::NoSpace)
        << "Attributes template named `" << objectTemplateHandle
        << "` does not have a valid render asset handle specified, so "
           "registration is aborted.";
    return core::managedContainers::ManagedObjectPreregistration::Failed;
  }

  // Handles for rendering assets
  const std::string renderAssetHandle = objectTemplate->getRenderAssetHandle();
  const std::string renderAssetFullPath =
      objectTemplate->getRenderAssetFullPath();

  // Clear map to add to ptr from previous registration
  mapToAddTo_ = nullptr;
  // verify these represent legitimate assets
  if (this->isValidPrimitiveAttributes(renderAssetHandle)) {
    // If renderAssetHandle corresponds to valid/existing primitive attributes
    // then setRenderAssetIsPrimitive to true and set map of IDs->Names to
    // physicsSynthObjTmpltLibByID_
    objectTemplate->setRenderAssetIsPrimitive(true);
    objectTemplate->setRenderAssetFullPath(renderAssetHandle);
    mapToAddTo_ = &physicsSynthObjTmpltLibByID_;
  } else if (Cr::Utility::Path::exists(renderAssetHandle) ||
             Cr::Utility::Path::exists(renderAssetFullPath)) {
    // Check if renderAssetHandle is valid file name and is found in file system
    // - if so then setRenderAssetIsPrimitive to false and set map of IDs->Names
    // to physicsFileObjTmpltLibByID_ - verify file  exists
    objectTemplate->setRenderAssetIsPrimitive(false);
    mapToAddTo_ = &physicsFileObjTmpltLibByID_;
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
    return core::managedContainers::ManagedObjectPreregistration::Failed;
  }
  // Handles for collision assets
  const std::string collisionAssetHandle =
      objectTemplate->getCollisionAssetHandle();
  const std::string collisionAssetFullPath =
      objectTemplate->getCollisionAssetFullPath();

  if (this->isValidPrimitiveAttributes(collisionAssetHandle)) {
    // If collisionAssetHandle corresponds to valid/existing primitive
    // attributes then setCollisionAssetIsPrimitive to true
    objectTemplate->setCollisionAssetIsPrimitive(true);
    objectTemplate->setCollisionAssetFullPath(collisionAssetHandle);
  } else if (Cr::Utility::Path::exists(collisionAssetHandle) ||
             Cr::Utility::Path::exists(collisionAssetFullPath)) {
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
    // Set values to match render asset values

    objectTemplate->setCollisionAssetHandle(
        objectTemplate->getRenderAssetHandle());
    objectTemplate->setCollisionAssetFullPath(
        objectTemplate->getRenderAssetFullPath());
    objectTemplate->setCollisionAssetIsPrimitive(
        objectTemplate->getRenderAssetIsPrimitive());
  }
  // filter all paths properly so that the handles don't have filepaths and the
  // accessors are hidden fields
  this->finalizeAttrPathsBeforeRegister(objectTemplate);
  // Clear dirty flag from when asset handles are changed
  objectTemplate->setFilePathsAreClean();

  return core::managedContainers::ManagedObjectPreregistration::Success;
}  // ObjectAttributesManager::preRegisterObjectFinalize

void ObjectAttributesManager::finalizeAttrPathsBeforeRegister(
    const attributes::ObjectAttributes::ptr& objectTemplate) const {
  ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
      << "BEFORE Obj `" << objectTemplate->getHandle() << "`: Render fn `"
      << objectTemplate->getRenderAssetHandle() << "`| Collision fn `"
      << objectTemplate->getCollisionAssetHandle() << "`| file dir `"
      << objectTemplate->getFileDirectory() << "`";
  // Render asset filename filter out path and set internal reference to full
  // filepath
  const std::string renderAssetHandle = objectTemplate->getRenderAssetHandle();
  if (!this->isValidPrimitiveAttributes(renderAssetHandle)) {
    this->filterAttribsFilenames(
        objectTemplate, renderAssetHandle,
        objectTemplate->getRenderAssetFullPath(),
        [objectTemplate](const std::string& renderAsset) {
          objectTemplate->setRenderAssetHandle(renderAsset);
        },
        [objectTemplate](const std::string& renderAsset) {
          objectTemplate->setRenderAssetFullPath(renderAsset);
        });
  } else {
    // If handle refs a prim then just copy it over to full path
    objectTemplate->setRenderAssetFullPath(renderAssetHandle);
  }
  // Collision asset filename filter out path and set internal reference to
  // full filepaath
  const std::string collisionAssetHandle =
      objectTemplate->getCollisionAssetHandle();
  if (!this->isValidPrimitiveAttributes(collisionAssetHandle)) {
    this->filterAttribsFilenames(
        objectTemplate, collisionAssetHandle,
        objectTemplate->getCollisionAssetFullPath(),
        [objectTemplate](const std::string& colHndl) {
          objectTemplate->setCollisionAssetHandle(colHndl);
        },
        [objectTemplate](const std::string& colHndl) {
          objectTemplate->setCollisionAssetFullPath(colHndl);
        });
  } else {
    // If handle refs a prim then just copy it over to full path
    objectTemplate->setCollisionAssetFullPath(collisionAssetHandle);
  }

  ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
      << "AFTER Obj `" << objectTemplate->getHandle() << "`: Render fn `"
      << objectTemplate->getRenderAssetHandle() << "`| Collision fn `"
      << objectTemplate->getCollisionAssetHandle() << "`| file dir `"
      << objectTemplate->getFileDirectory() << "`";
}  // ObjectAttributesManager::finalizeAttrPathsBeforeRegister

void ObjectAttributesManager::postRegisterObjectHandling(
    int objectTemplateID,
    const std::string& objectTemplateHandle) {
  if (mapToAddTo_ != nullptr) {
    mapToAddTo_->emplace(objectTemplateID, objectTemplateHandle);
  }
}  // ObjectAttributesManager::postRegisterObjectHandling

}  // namespace managers
}  // namespace metadata
}  // namespace esp
