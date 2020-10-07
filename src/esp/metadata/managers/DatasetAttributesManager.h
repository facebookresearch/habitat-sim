// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_MANAGERS_DATASETATTRIBUTEMANAGER_H_
#define ESP_METADATA_MANAGERS_DATASETATTRIBUTEMANAGER_H_

#include "PhysicsAttributesManager.h"

#include "AttributesManagerBase.h"
#include "esp/metadata/attributes/DatasetAttributes.h"

namespace esp {
namespace metadata {

namespace managers {
class DatasetAttributesManager
    : public AttributesManager<attributes::DatasetAttributes> {
 public:
  DatasetAttributesManager(PhysicsAttributesManager::ptr physicsAttributesMgr)
      : AttributesManager<attributes::DatasetAttributes>::AttributesManager(
            "Dataset",
            "dataset_config.json"),
        physicsAttributesManager_(physicsAttributesMgr) {
    buildCtorFuncPtrMaps();
  }

  /**
   * @brief copy current @ref esp::sim::SimulatorConfiguration driven values,
   * such as file paths, to make them available for stage attributes defaults.
   *
   * @param datasetName the name of the dataset to apply these to.
   * @param filepaths the map of file paths from the configuration object
   * @param lightSetup the config-specified light setup
   * @param frustrumCulling whether or not (semantic) stage should be
   * partitioned for culling.
   */
  void setCurrCfgVals(const std::string& datasetName,
                      const std::map<std::string, std::string>& filepaths,
                      const std::string& lightSetup,
                      bool frustrumCulling) {
    if (this->getObjectLibHasHandle(datasetName)) {
      auto dataset =
          this->getObjectInternal<attributes::DatasetAttributes>(datasetName);
      dataset->setCurrCfgVals(filepaths, lightSetup, frustrumCulling);
    } else {
      LOG(ERROR) << "DatasetAttributesManager::setCurrCfgVals : No "
                 << objectType_ << " managed object with handle " << datasetName
                 << "exists. Aborting";
    }
  }  // StageAttributesManager::setCurrCfgVals

  /**
   * @brief Creates an instance of a dataset template described by passed
   * string. For dataset templates, this a file name.
   *
   * If a template exists with this handle, this existing template will be
   * overwritten with the newly created one if registerTemplate is true.
   *
   * @param attributesTemplateHandle the origin of the desired dataset template
   * to be created.
   * @param registerTemplate whether to add this template to the library.
   * If the user is going to edit this template, this should be false - any
   * subsequent editing will require re-registration. Defaults to true. If
   * specified as true, then this function returns a copy of the registered
   * template.
   * @return a reference to the desired template.
   */
  attributes::DatasetAttributes::ptr createObject(
      const std::string& attributesTemplateHandle,
      bool registerTemplate = true) override;

  /**
   * @brief Method to take an existing attributes and set its values from passed
   * json config file.
   * @param attribs (out) an existing attributes to be modified.
   * @param jsonConfig json document to parse
   */
  void setValsFromJSONDoc(attributes::DatasetAttributes::ptr attribs,
                          const io::JsonGenericValue& jsonConfig) override;

  /**
   * @brief This will set the current physics manager attributes that is
   * governing the world that this DatasetAttributesManager's datasets will be
   * created in.  This is used so that upon creation of new
   * esp::metadata::attributes::DatasetAttributes, PhysicsManagerAttributes
   * defaults can be set in the esp::metadata::attributes::DatasetAttributes
   * before any scene-specific values are set.
   *
   * @param handle The string handle referencing the @ref
   * esp::metadata::attributes::PhysicsManagerAttributes governing the current
   * @ref esp::physics::PhysicsManager.
   */
  void setCurrPhysicsManagerAttributesHandle(const std::string& handle) {
    physicsManagerAttributesHandle_ = handle;
    for (auto& val : this->objectLibrary_) {
      auto dataset =
          this->getObjectInternal<attributes::DatasetAttributes>(val.first);
      dataset->setPhysicsManagerHandle(handle);
    }
  }  // DatasetAttributesManager::setCurrPhysicsManagerAttributesHandle

 protected:
  /**
   * @brief Handle reading a JSON sub-cell with in the dataset_config.JSON file,
   * using the passed attributesManager for the dataset being processed.
   * @tparam the type of the attributes manager.
   * @param jsonCell The sub cell in the json document being processed.
   * @param attrMgr The dataset's attributes manager.
   */
  template <typename U>
  void readDatasetJSONCell(const char* tag,
                           const io::JsonGenericValue& jsonConfig,
                           const U& attrMgr);

  /**
   * @brief Used Internally.  Create and configure newly-created dataset
   * attributes with any default values, before any specific values are set.
   *
   * @param handleName handle name to be assigned to dataset attributes
   */
  attributes::DatasetAttributes::ptr initNewObjectInternal(
      const std::string& handleName) override;

  /**
   * @brief This method will perform any necessary updating that is
   * attributesManager-specific upon template removal, such as removing a
   * specific template handle from the list of file-based template handles in
   * ObjectAttributesManager.  This should only be called internally.
   *
   * @param templateID the ID of the template to remove
   * @param templateHandle the string key of the attributes desired.
   */

  void updateObjectHandleLists(
      CORRADE_UNUSED int templateID,
      CORRADE_UNUSED const std::string& templateHandle) override {}

  /**
   * @brief Any dataset-attributes-specific resetting that needs to happen on
   * reset.
   */
  void resetFinalize() override {}

  /**
   * @brief Add a @ref std::shared_ptr<attributesType> object to the
   * @ref objectLibrary_.  Verify that render and collision handles have been
   * set properly.  We are doing this since these values can be modified by the
   * user.
   *
   * @param datasetAttributes The attributes template.
   * @param datasetAttributesHandle The key for referencing the template in the
   * @ref objectLibrary_.
   * @return The index in the @ref objectLibrary_ of object
   * template.
   */
  int registerObjectFinalize(
      attributes::DatasetAttributes::ptr datasetAttributes,
      const std::string& datasetAttributesHandle) override;

  /**
   * @brief This function will assign the appropriately configured function
   * pointer for the copy constructor as required by
   * AttributesManager<PhysicsSceneAttributes::ptr>
   */
  void buildCtorFuncPtrMaps() override {
    this->copyConstructorMap_["DatasetAttributes"] =
        &DatasetAttributesManager::createObjectCopy<
            attributes::DatasetAttributes>;
  }  // PhysicsAttributesManager::buildCtorFuncPtrMaps

  /**
   * @brief This function is meaningless for this manager's ManagedObjects.
   * @param handle Ignored.
   * @return false
   */
  virtual bool isValidPrimitiveAttributes(
      CORRADE_UNUSED const std::string& handle) override {
    return false;
  }

  /**
   * @brief Name of currently used physicsManagerAttributes
   */
  std::string physicsManagerAttributesHandle_ = "";

  /**
   * @brief Reference to PhysicsAttributesManager to give access to default
   * physics manager attributes settings when
   * esp::metadata::attributes::DatasetAttributes are created within Dataset.
   */
  PhysicsAttributesManager::ptr physicsAttributesManager_ = nullptr;

 public:
  ESP_SMART_POINTERS(DatasetAttributesManager)
};  // class DatasetAttributesManager
}  // namespace managers
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_MANAGERS_DATASETATTRIBUTEMANAGER_H_
