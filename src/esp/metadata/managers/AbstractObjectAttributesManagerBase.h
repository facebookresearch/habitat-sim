// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_MANAGERS_ABSTRACTOBJECTATTRIBUTESMANAGERBASE_H_
#define ESP_METADATA_MANAGERS_ABSTRACTOBJECTATTRIBUTESMANAGERBASE_H_

/** @file
 * @brief Class Template @ref
 * esp::metadata::managers::AbstractObjectAttributesManager
 */

#include "esp/metadata/attributes/ObjectAttributes.h"

#include "esp/metadata/MetadataUtils.h"
#include "esp/metadata/managers/AttributesManagerBase.h"

namespace Cr = Corrade;

namespace esp {
namespace metadata {
namespace managers {

/**
 * @brief Class template defining responsibilities and functionality for
 * managing @ref esp::metadata::attributes::AbstractObjectAttributes constructs.
 * @tparam T the type of managed attributes a particular specialization
 * of this class works with.  Must inherit from @ref
 * esp::metadata::attributes::AbstractObjectAttributes.
 */
template <class T, core::ManagedObjectAccess Access>
class AbstractObjectAttributesManager : public AttributesManager<T, Access> {
 public:
  static_assert(std::is_base_of<attributes::AbstractObjectAttributes, T>::value,
                "AbstractObjectAttributesManager :: Managed object type must "
                "be derived from AbstractObjectAttributes");

  typedef std::shared_ptr<T> AbsObjAttrPtr;

  AbstractObjectAttributesManager(const std::string& attrType,
                                  const std::string& JSONTypeExt)
      : AttributesManager<T, Access>::AttributesManager(
            (attrType + " Template"),
            JSONTypeExt) {}
  ~AbstractObjectAttributesManager() override = default;

  /**
   * @brief Creates an instance of an object or stage template. The passed
   * string should be either a file name or a reference to a primitive asset
   * template that should be used in the construction of the object or stage;
   * any other strings will result in a new default template being created.
   *
   * If a template exists with this handle, this existing template will be
   * overwritten with the newly created one if registerTemplate is true.
   *
   * @param attributesTemplateHandle the origin of the desired template to be
   * created, either a file name or an existing primitive asset template. If
   * this is neither a recognized file name nor the handle of an existing
   * primitive asset, a new default template will be created.
   * @param registerTemplate whether to add this template to the library.
   * If the user is going to edit this template, this should be false - any
   * subsequent editing will require re-registration. Defaults to true. If
   * specified as true, then this function returns a copy of the registered
   * template.
   * @return a reference to the desired template.
   */
  AbsObjAttrPtr createObject(const std::string& attributesTemplateHandle,
                             bool registerTemplate = true) override;

  /**
   * @brief Creates an instance of an object or stage template described by
   * passed string, which should be a reference to an existing primitive asset
   * template to be used in the construction of the object or stage (as render
   * and collision mesh). It returns existing instance if there is one, and
   * nullptr if fails.
   *
   * @param primAttrTemplateHandle The handle to an existing primitive asset
   * template. Fails if does not exist.
   * @param registerTemplate whether to add this template to the library.
   * If the user is going to edit this template, this should be false - any
   * subsequent editing will require re-registration. Defaults to true.
   * @return a reference to the desired stage template, or nullptr if fails.
   */
  virtual AbsObjAttrPtr createPrimBasedAttributesTemplate(
      const std::string& primAttrTemplateHandle,
      bool registerTemplate = true) = 0;

 protected:
  //======== Common JSON import functions ========

  /**
   * @brief Populate an existing @ref
   * metadata::attributes::AbstractObjectAttributes from a JSON config.  Also
   * will populate render mesh and collision mesh handles with value(s)
   * specified in JSON.  If one is blank will use other for both.
   *
   * @param attributes the attributes to populate with
   * @param jsonDoc JSON document to parse
   * @return an appropriately cast attributes pointer with base class fields
   * filled in.
   */
  AbsObjAttrPtr loadAbstractObjectAttributesFromJson(
      AbsObjAttrPtr attributes,
      const io::JsonGenericValue& jsonDoc);

  //======== Internally accessed functions ========
  /**
   * @brief Only used by @ref
   * esp::metadata::attributes::AbstractObjectAttributes derived-attributes. Set
   * the asset type and mesh asset filename from json file. If mesh asset
   * filename has changed in json, but type has not been specified in json,
   * re-run file-path-driven configuration to get asset type and possibly
   * orientation frame, if appropriate.
   *
   * @param attributes The AbstractObjectAttributes object to be populated
   * @param jsonDoc The json document
   * @param jsonMeshTypeTag The string tag denoting the desired mesh type in the
   * json.
   * @param jsonMeshHandleTag The string for the mesh asset handle.
   * @param assetName [in/out] On entry this is old mesh handle, on exit is
   * new mesh handle, or empty.
   * @param meshTypeSetter Function pointer to the appropriate mesh type setter
   * in the Attributes object.
   * @return Either the previously set asset handle or a new one based on what
   * was read from JSON config document.
   */
  std::string setJSONAssetHandleAndType(
      AbsObjAttrPtr attributes,
      const io::JsonGenericValue& jsonDoc,
      const char* jsonMeshTypeTag,
      const char* jsonMeshHandleTag,
      std::string& assetName,
      const std::function<void(int)>& meshTypeSetter);

  /**
   * @brief Perform asset-name-based attributes initialization. This is to
   * take the place of the AssetInfo::fromPath functionality, and is only
   * intended to provide default values and other help if certain mistakes
   * are made by the user, such as specifying an asset handle in json but not
   * specifying the asset type corresponding to that handle.  These settings
   * should not restrict anything, only provide defaults.
   *
   * @param attributes The AbstractObjectAttributes object to be configured
   * @param setFrame whether the frame should be set or not (only for render
   * assets in scenes)
   * @param assetName Mesh Handle to check.
   * @param meshTypeSetter Setter for mesh type.
   */
  virtual void setDefaultAssetNameBasedAttributes(
      AbsObjAttrPtr attributes,
      bool setFrame,
      const std::string& assetName,
      std::function<void(int)> meshTypeSetter) = 0;

  // ======== Typedefs and Instance Variables ========

 public:
  ESP_SMART_POINTERS(AbstractObjectAttributesManager<T, Access>)

};  // class AbstractObjectAttributesManager<T>

/////////////////////////////
// Class Template Method Definitions

template <class T, core::ManagedObjectAccess Access>
auto AbstractObjectAttributesManager<T, Access>::createObject(
    const std::string& attributesTemplateHandle,
    bool registerTemplate) -> AbsObjAttrPtr {
  AbsObjAttrPtr attrs;
  std::string msg;
  if (this->isValidPrimitiveAttributes(attributesTemplateHandle)) {
    // if attributesTemplateHandle == some existing primitive attributes, then
    // this is a primitive-based object we are building
    attrs = this->createPrimBasedAttributesTemplate(attributesTemplateHandle,
                                                    registerTemplate);
    msg = "Primitive Asset (" + attributesTemplateHandle + ") Based";
  } else {
    attrs = this->createFromJsonOrDefaultInternal(attributesTemplateHandle, msg,
                                                  registerTemplate);

  }  // if this is prim else
  if (nullptr != attrs) {
    ESP_DEBUG() << msg << this->objectType_ << "attributes created"
                << (registerTemplate ? "and registered." : ".");
  }
  return attrs;

}  // AbstractObjectAttributesManager<T>::createObject

template <class T, core::ManagedObjectAccess Access>
auto AbstractObjectAttributesManager<T, Access>::
    loadAbstractObjectAttributesFromJson(AbsObjAttrPtr attributes,
                                         const io::JsonGenericValue& jsonDoc)
        -> AbsObjAttrPtr {
  // scale
  io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonDoc, "scale", [attributes](const Magnum::Vector3& scale) {
        attributes->setScale(scale);
      });

  // collision asset size
  io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonDoc, "collision_asset_size",
      [attributes](const Magnum::Vector3& collision_asset_size) {
        attributes->setCollisionAssetSize(collision_asset_size);
      });
  // margin
  io::jsonIntoSetter<double>(jsonDoc, "margin", [attributes](double margin) {
    attributes->setMargin(margin);
  });
  // initialize with collisions on/off
  io::jsonIntoSetter<bool>(jsonDoc, "is_collidable",
                           [attributes](bool is_collidable) {
                             attributes->setIsCollidable(is_collidable);
                           });

  // load the friction coefficient
  io::jsonIntoSetter<double>(
      jsonDoc, "friction_coefficient",
      [attributes](double friction_coefficient) {
        attributes->setFrictionCoefficient(friction_coefficient);
      });

  // load the restitution coefficient
  io::jsonIntoSetter<double>(
      jsonDoc, "restitution_coefficient",
      [attributes](double restitution_coefficient) {
        attributes->setRestitutionCoefficient(restitution_coefficient);
      });

  // if object or stage will be forced to be flat shaded
  io::jsonIntoSetter<bool>(
      jsonDoc, "force_flat_shading", [attributes](bool force_flat_shading) {
        attributes->setForceFlatShading(force_flat_shading);
      });

  // units to meters
  io::jsonIntoSetter<double>(jsonDoc, "units_to_meters",
                             [attributes](double units_to_meters) {
                               attributes->setUnitsToMeters(units_to_meters);
                             });

  // load object/scene specific up orientation
  io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonDoc, "up",
      [attributes](const Magnum::Vector3& up) { attributes->setOrientUp(up); });

  // load object/scene specific front orientation
  io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonDoc, "front", [attributes](const Magnum::Vector3& front) {
        attributes->setOrientFront(front);
      });

  // 4. parse render and collision mesh filepaths
  // current value - also place holder for json read result
  std::string rTmpFName = attributes->getRenderAssetHandle();
  // is true if mesh name is found in JSON and different than current value
  std::string rndrFName = setJSONAssetHandleAndType(
      attributes, jsonDoc, "render_asset_type", "render_asset", rTmpFName,
      [attributes](int render_asset_type) {
        attributes->setRenderAssetType(render_asset_type);
      });

  // current value - also place holder for json read result
  std::string cTmpFName = attributes->getCollisionAssetHandle();
  // is true if mesh name is found in JSON and different than current value
  std::string colFName = setJSONAssetHandleAndType(
      attributes, jsonDoc, "collision_asset_type", "collision_asset", cTmpFName,
      [attributes](int collision_asset_type) {
        attributes->setCollisionAssetType(collision_asset_type);
      });
  // use non-empty result if either result is empty
  attributes->setRenderAssetHandle(rndrFName == "" ? colFName : rndrFName);
  attributes->setCollisionAssetHandle(colFName == "" ? rndrFName : colFName);

  // check if primitive collision mesh
  auto colAssetHandle = attributes->getCollisionAssetHandle();
  if (this->isValidPrimitiveAttributes(colAssetHandle)) {
    // value is valid primitive, and value is different than existing value
    attributes->setCollisionAssetType(
        static_cast<int>(esp::assets::AssetType::PRIMITIVE));
    attributes->setUseMeshCollision(false);
  } else {
    // TODO eventually remove this, but currently non-prim collision mesh must
    // be UNKNOWN
    attributes->setCollisionAssetType(
        static_cast<int>(esp::assets::AssetType::UNKNOWN));
    attributes->setUseMeshCollision(true);
  }

  // set attributes shader type to use.  This may be overridden by a scene
  // instance specification.
  const std::string shaderTypeVal = getShaderTypeFromJsonDoc(jsonDoc);
  // if a known shader type val is specified in json, set that value for the
  // attributes, overriding constructor defaults.  Do not overwrite anything for
  // unknown
  if (shaderTypeVal !=
      getShaderTypeName(attributes::ObjectInstanceShaderType::Unknown)) {
    attributes->setShaderType(shaderTypeVal);
  }

  return attributes;
}  // AbstractObjectAttributesManager<AbsObjAttrPtr>::createObjectAttributesFromJson

template <class T, core::ManagedObjectAccess Access>
std::string
AbstractObjectAttributesManager<T, Access>::setJSONAssetHandleAndType(
    AbsObjAttrPtr attributes,
    const io::JsonGenericValue& jsonDoc,
    const char* jsonMeshTypeTag,
    const char* jsonMeshHandleTag,
    std::string& assetName,
    const std::function<void(int)>& meshTypeSetter) {
  std::string propertiesFileDirectory = attributes->getFileDirectory();
  // save current file name
  std::string oldFName(assetName);
  // clear var to get new value - if returns true use this as new value
  assetName = "";
  // Map a json string value to its corresponding AssetType if found and cast to
  // int, based on AbstractObjectAttributes::AssetTypeNamesMap mappings.
  // Casts an int of the esp::AssetType enum value if found and understood,
  // 0 (esp::assets::AssetType::UNKNOWN) if found but not understood, and
  //-1 if tag is not found in json.
  int typeVal = -1;
  std::string tmpVal = "";
  if (io::readMember<std::string>(jsonDoc, jsonMeshTypeTag, tmpVal)) {
    // tag was found, perform check
    std::string strToLookFor = Cr::Utility::String::lowercase(tmpVal);
    if (T::AssetTypeNamesMap.count(strToLookFor)) {
      typeVal = static_cast<int>(T::AssetTypeNamesMap.at(strToLookFor));
    } else {
      ESP_WARNING() << "<" << Magnum::Debug::nospace << this->objectType_
                    << Magnum::Debug::nospace
                    << "> : Value in json @ tag :" << jsonMeshTypeTag << ": `"
                    << tmpVal
                    << "` does not map to a valid "
                       "AbstractObjectAttributes::AssetTypeNamesMap value, so "
                       "defaulting mesh type to AssetType::UNKNOWN.";
      typeVal = static_cast<int>(esp::assets::AssetType::UNKNOWN);
    }
    // value found so override current value, otherwise do not.
    meshTypeSetter(typeVal);
  }  // if type is found in json.  If not typeVal is -1

  // Read json for new mesh handle if present
  if (io::readMember<std::string>(jsonDoc, jsonMeshHandleTag, assetName)) {
    // value is specified in json doc
    if ((this->isValidPrimitiveAttributes(assetName)) &&
        (oldFName != assetName)) {
      // if mesh name is specified and different than old value,
      // perform name-specific mesh-type config.
      setDefaultAssetNameBasedAttributes(attributes, false, assetName,
                                         meshTypeSetter);
    } else {
      // is not valid primitive, assume valid file name
      assetName =
          Cr::Utility::Directory::join(propertiesFileDirectory, assetName);
      if ((typeVal == -1) && (oldFName != assetName)) {
        // if file name is different, and type val has not been specified,
        // perform name-specific mesh type config do not override orientation -
        // should be specified in json.
        setDefaultAssetNameBasedAttributes(attributes, false, assetName,
                                           meshTypeSetter);
      }
    }  // value is valid prim and exists, else value is valid file and exists
    return assetName;
  }
  // handle value is not present in JSON or is specified but does not change
  return oldFName;
}  // AbstractObjectAttributesManager<AbsObjAttrPtr>::setJSONAssetHandleAndType

}  // namespace managers
}  // namespace metadata
}  // namespace esp
#endif  // ESP_METADATA_MANAGERS_ABSTRACTOBJECTATTRIBUTESMANAGERBASE_H_
