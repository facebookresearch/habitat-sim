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

#include "esp/metadata/managers/AttributesManagerBase.h"

namespace Cr = Corrade;

namespace esp {
namespace metadata {
namespace managers {
namespace Attrs = esp::metadata::attributes;
/**
 * @brief Class template defining responsibilities and functionality for
 * managing @ref esp::metadata::attributes::AbstractObjectAttributes constructs.
 * @tparam T the type of managed attributes a particular specialization
 * of this class works with.  Must inherit from @ref
 * esp::metadata::attributes::AbstractObjectAttributes.
 */
template <class T>
class AbstractObjectAttributesManager : public AttributesManager<T> {
 public:
  static_assert(std::is_base_of<Attrs::AbstractObjectAttributes, T>::value,
                "AbstractObjectAttributesManager :: Managed object type must "
                "be derived from AbstractObjectAttributes");

  typedef std::shared_ptr<T> AbsObjAttrPtr;

  AbstractObjectAttributesManager(const std::string& attrType)
      : AttributesManager<T>::AttributesManager(attrType) {}
  virtual ~AbstractObjectAttributesManager() = default;

 protected:
  //======== Common JSON import functions ========

  /**
   * @brief Create either an object or a stage attributes from a json config.
   * Since both object attributes and stage attributes inherit from @ref
   * esp::metadata::attributes::AbstractObjectAttributes, the functionality to
   * populate these fields from json can be shared.  Also will will populate
   * render mesh and collision mesh handles in object and stage attributes with
   * value(s) specified in json.  If one is blank will use other for both.
   *
   * @param filename name of json descriptor file
   * @param jsonDoc json document to parse
   * @return an appropriately cast attributes pointer with base class fields
   * filled in.
   */
  AbsObjAttrPtr createObjectAttributesFromJson(const std::string& filename,
                                               const io::JsonDocument& jsonDoc);

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
   * @return Whether the render asset name was specified in the json and should
   * be set from assetName variable.
   */
  bool setJSONAssetHandleAndType(AbsObjAttrPtr attributes,
                                 const io::JsonDocument& jsonDoc,
                                 const char* jsonMeshTypeTag,
                                 const char* jsonMeshHandleTag,
                                 std::string& assetName,
                                 std::function<void(int)> meshTypeSetter);

  //======== Internally accessed functions ========

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
  ESP_SMART_POINTERS(AbstractObjectAttributesManager<AbsObjAttrPtr>)

};  // namespace managers

/////////////////////////////
// Class Template Method Definitions

template <class T>
auto AbstractObjectAttributesManager<T>::createObjectAttributesFromJson(
    const std::string& configFilename,
    const io::JsonDocument& jsonDoc) -> AbsObjAttrPtr {
  AbsObjAttrPtr attributes = this->initNewObjectInternal(configFilename);

  using std::placeholders::_1;

  // scale
  io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonDoc, "scale",
      std::bind(&Attrs::AbstractObjectAttributes::setScale, attributes, _1));

  // collision asset size
  io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonDoc, "collision asset size",
      std::bind(&Attrs::AbstractObjectAttributes::setCollisionAssetSize,
                attributes, _1));
  // margin
  io::jsonIntoSetter<double>(
      jsonDoc, "margin",
      std::bind(&Attrs::AbstractObjectAttributes::setMargin, attributes, _1));

  // load the friction coefficient
  io::jsonIntoSetter<double>(
      jsonDoc, "friction coefficient",
      std::bind(&Attrs::AbstractObjectAttributes::setFrictionCoefficient,
                attributes, _1));

  // load the restitution coefficient
  io::jsonIntoSetter<double>(
      jsonDoc, "restitution coefficient",
      std::bind(&Attrs::AbstractObjectAttributes::setRestitutionCoefficient,
                attributes, _1));

  // if object will be flat or phong shaded
  io::jsonIntoSetter<bool>(
      jsonDoc, "requires lighting",
      std::bind(&Attrs::AbstractObjectAttributes::setRequiresLighting,
                attributes, _1));

  // units to meters
  io::jsonIntoSetter<double>(
      jsonDoc, "units to meters",
      std::bind(&Attrs::AbstractObjectAttributes::setUnitsToMeters, attributes,
                _1));

  // load object/scene specific up orientation
  io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonDoc, "up",
      std::bind(&Attrs::AbstractObjectAttributes::setOrientUp, attributes, _1));

  // load object/scene specific front orientation
  io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonDoc, "front",
      std::bind(&Attrs::AbstractObjectAttributes::setOrientFront, attributes,
                _1));

  // 4. parse render and collision mesh filepaths
  std::string rndrFName = "";
  // current value - also place holder for json read result
  std::string rTmpFName = attributes->getRenderAssetHandle();
  // is true if mesh name is found in JSON and different than current value
  if (setJSONAssetHandleAndType(
          attributes, jsonDoc, "render mesh type", "render mesh", rTmpFName,
          std::bind(&Attrs::AbstractObjectAttributes::setRenderAssetType,
                    attributes, _1))) {
    // set asset name to be what was read in json
    rndrFName = rTmpFName;
  }

  std::string colFName = "";
  // current value - also place holder for json read result
  std::string cTmpFName = attributes->getCollisionAssetHandle();
  // is true if mesh name is found in JSON and different than current value
  if (setJSONAssetHandleAndType(
          attributes, jsonDoc, "collision mesh type", "collision mesh",
          cTmpFName,
          std::bind(&Attrs::AbstractObjectAttributes::setCollisionAssetType,
                    attributes, _1))) {
    // set asset name to be what was read in json
    colFName = cTmpFName;
  }

  // use non-empty result if either result is empty
  attributes->setRenderAssetHandle(rndrFName.compare("") == 0 ? colFName
                                                              : rndrFName);
  attributes->setCollisionAssetHandle(colFName.compare("") == 0 ? rndrFName
                                                                : colFName);

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
  return attributes;
}  // AbstractObjectAttributesManager<AbsObjAttrPtr>::createObjectAttributesFromJson

template <class T>
bool AbstractObjectAttributesManager<T>::setJSONAssetHandleAndType(
    AbsObjAttrPtr attributes,
    const io::JsonDocument& jsonDoc,
    const char* jsonMeshTypeTag,
    const char* jsonMeshHandleTag,
    std::string& assetName,
    std::function<void(int)> meshTypeSetter) {
  std::string propertiesFileDirectory = attributes->getFileDirectory();
  // save current file name
  const std::string oldFName(assetName);
  // clear var to get new value - if returns true use this as new value
  assetName = "";
  // Map a json string value to its corresponding AssetType if found and cast to
  // int, based on AbstractObjectAttributes::AssetTypeNamesMap mappings.
  // Casts an int of the esp::AssetType enum value if found and understood,
  // 0 (esp::assets::AssetType::UNKNOWN) if found but not understood, and
  //-1 if tag is not found in json.
  int typeVal = -1;
  std::string tmpVal = "";
  if (io::jsonIntoVal<std::string>(jsonDoc, jsonMeshTypeTag, tmpVal)) {
    // tag was found, perform check
    std::string strToLookFor = Cr::Utility::String::lowercase(tmpVal);
    if (Attrs::AbstractObjectAttributes::AssetTypeNamesMap.count(tmpVal)) {
      typeVal = static_cast<int>(
          Attrs::AbstractObjectAttributes::AssetTypeNamesMap.at(tmpVal));
    } else {
      LOG(WARNING)
          << "AbstractObjectAttributesManager::setJSONAssetHandleAndType : "
             "Value in json @ tag : "
          << jsonMeshTypeTag << " : `" << tmpVal
          << "` does not map to a valid "
             "AbstractObjectAttributes::AssetTypeNamesMap value, so "
             "defaulting mesh type to AssetType::UNKNOWN.";
      typeVal = static_cast<int>(esp::assets::AssetType::UNKNOWN);
    }
    // value found so override current value, otherwise do not.
    meshTypeSetter(typeVal);
  }  // if type is found in json.  If not typeVal is -1

  // Read json for new mesh handle
  if (io::jsonIntoVal<std::string>(jsonDoc, jsonMeshHandleTag, assetName)) {
    // value is specified in json doc
    if ((this->isValidPrimitiveAttributes(assetName)) &&
        (oldFName.compare(assetName) != 0)) {
      // if mesh name is specified and different than old value,
      // perform name-specific mesh-type config.
      setDefaultAssetNameBasedAttributes(attributes, false, assetName,
                                         meshTypeSetter);
    } else {
      // is not valid primitive, check if valid file name
      assetName =
          Cr::Utility::Directory::join(propertiesFileDirectory, assetName);
      if ((typeVal == -1) && (oldFName.compare(assetName) != 0)) {
        // if file name is different, and type val has not been specified,
        // perform name-specific mesh type config do not override orientation -
        // should be specified in json.
        setDefaultAssetNameBasedAttributes(attributes, false, assetName,
                                           meshTypeSetter);
      }
    }  // value is valid prim and exists, else value is valid file and exists
    return true;
  }  // value is present in json
  return false;
}  // AbstractObjectAttributesManager<AbsObjAttrPtr>::setJSONAssetHandleAndType

}  // namespace managers
}  // namespace metadata
}  // namespace esp
#endif  // ESP_METADATA_MANAGERS_ABSTRACTOBJECTATTRIBUTESMANAGERBASE_H_
