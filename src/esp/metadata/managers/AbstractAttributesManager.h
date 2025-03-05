// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_MANAGERS_ABSTRACTATTRIBUTESMANAGER_H_
#define ESP_METADATA_MANAGERS_ABSTRACTATTRIBUTESMANAGER_H_

/** @file
 * @brief Class Template @ref esp::metadata::managers::AbstractAttributesManager
 */

#include "esp/metadata/attributes/AbstractAttributes.h"

#include "esp/core/managedContainers/ManagedFileBasedContainer.h"
#include "esp/io/Io.h"

namespace Cr = Corrade;
namespace CrPath = Cr::Utility::Path;

namespace esp {
namespace core {
namespace managedContainers {
enum class ManagedObjectAccess;
class ManagedFileBasedContainerBase;
}  // namespace managedContainers
}  // namespace core
namespace metadata {
namespace managers {

using core::config::Configuration;
using core::managedContainers::ManagedFileBasedContainer;
using core::managedContainers::ManagedObjectAccess;

/**
 * @brief Class template defining responsibilities and functionality for
 * managing @ref esp::metadata::attributes::AbstractAttributes constructs.
 * @tparam T the type of managed attributes a particular specialization
 * of this class works with.  Must inherit from @ref
 * esp::metadata::attributes::AbstractAttributes.
 * @tparam Access Whether the default access (getters) for this
 * container provides copies of the objects held, or the actual objects
 * themselves.
 */
template <class T, ManagedObjectAccess Access>
class AbstractAttributesManager : public ManagedFileBasedContainer<T, Access> {
 public:
  static_assert(
      std::is_base_of<attributes::AbstractAttributes, T>::value,
      "AbstractAttributesManager :: Managed object type must be derived from "
      "AbstractAttributes");

  typedef std::shared_ptr<T> AttribsPtr;

  /**
   * @brief Construct an attributes manager to manage shared pointers of
   * attributes of type T.
   * @param attrType A string describing the type of attributes, for
   * @param JSONTypeExt The attributes JSON file extension, which must be of the
   * form 'XXXXXX.json', where XXXXXX represents the sub extension specific to
   * the managed type of attributes (i.e. "stage_config.json" for configurations
   * describing stages).
   */
  AbstractAttributesManager(const std::string& attrType,
                            const std::string& JSONTypeExt)
      : ManagedFileBasedContainer<T, Access>::ManagedFileBasedContainer(
            attrType,
            JSONTypeExt) {}
  ~AbstractAttributesManager() override = default;

  /**
   * @brief Load all file-based templates given string list of template file
   * locations.
   *
   * This will take the list of file names specified and load the referenced
   * templates.  It is assumed these files are JSON files currently.
   * @param tmpltFilenames list of file names of templates
   * @param saveAsDefaults Set these templates as un-deletable from library.
   * @return vector holding IDs of templates that have been added
   */
  std::vector<int> loadAllFileBasedTemplates(
      const std::vector<std::string>& tmpltFilenames,
      bool saveAsDefaults);

  /**
   * @brief Load file-based templates for all @ref JSONTypeExt_
   * files from the provided file or directory path.
   *
   * This will take the passed @p path string and either treat it as a file
   * name or a directory, depending on what is found in the filesystem. If @p
   * path does not end with @ref JSONTypeExt_, it will append this and check to
   * see if such a file exists, and load it. It will also check if @p path
   * exists as a directory, and if so will perform a shallow search to find any
   * files ending in @ref JSONTypeExt_ and load those that are found.
   *
   * @param path A global path to configuration files or a directory containing
   * such files.
   * @param saveAsDefaults Set the templates loaded as undeleteable default
   * templates.
   * @return A list of template indices for loaded valid configs
   */
  std::vector<int> loadAllJSONConfigsFromPath(const std::string& path,
                                              bool saveAsDefaults = false) {
    return this->loadAllTemplatesFromPathAndExt(path, this->JSONTypeExt_,
                                                saveAsDefaults);
  }

  /**
   * @brief Load file-based templates for all @p extType files from the provided
   * file or directory path.
   *
   * This will take the passed @p path string and either treat it as a file
   * name or a directory, depending on what is found in the filesystem. If @p
   * path does not end with @p extType, it will append this and check to
   * see if such a file exists, and load it. It will also check if @p path
   * exists as a directory, and if so will perform a shallow search to find any
   * files ending in @p extType and load those that are found.
   *
   * @param path A global path to configuration files or a directory containing
   * such files.
   * @param extType The extension of files to be attempted to be loaded as
   * templates.
   * @param saveAsDefaults Set the templates loaded as undeleteable default
   * templates.
   * @return A list of template indices for loaded valid configs
   */
  std::vector<int> loadAllTemplatesFromPathAndExt(const std::string& path,
                                                  const std::string& extType,
                                                  bool saveAsDefaults = false);

  /**
   * @brief This builds a list of paths to this type of attributes's JSON Config
   * files from the passed @p jsonPaths array element.  It then will load all
   * the configs it finds at each path.
   * @param configDir The directory to use as a root to search in - may be
   * different than the config dir already listed in this manager.
   * @param jsonPaths The json array element
   */

  void buildJSONCfgPathsFromJSONAndLoad(const std::string& configDir,
                                        const io::JsonGenericValue& jsonPaths) {
    this->buildAttrSrcPathsFromJSONAndLoad(configDir, this->JSONTypeExt_,
                                           jsonPaths);
  }

  /**
   * @brief This builds a list of paths to the @p extType files to use to
   * construct templates derived from the passed @p jsonPaths array element.  It
   * then will load all the configs it finds at each path.
   * @param configDir The directory to use as a root to search in - may be
   * different than the config dir already listed in this manager.
   * @param extType The extension of files to be attempted to be loaded as
   * templates.
   * @param jsonPaths The json array element
   */
  void buildAttrSrcPathsFromJSONAndLoad(const std::string& configDir,
                                        const std::string& extType,
                                        const io::JsonGenericValue& jsonPaths);

  /**
   * @brief Parse passed JSON Document for @ref
   * esp::metadata::attributes::AbstractAttributes. It always returns a
   * valid @ref esp::metadata::attributes::AbstractAttributes shared
   * pointer.
   *
   * @param templateName The desired name for this @ref
   * esp::metadata::attributes::AbstractAttributes template.
   * @param jsonConfig json document to parse
   * @return a reference to the desired template.
   */
  AttribsPtr buildObjectFromJSONDoc(
      const std::string& templateName,
      const io::JsonGenericValue& jsonConfig) override {
    // Construct a ObjectAttributes and populate with any
    // AbstractObjectAttributes fields found in json.
    auto attributes = this->initNewObjectInternal(templateName, true);
    // set the values for this attributes from the json config.
    this->setValsFromJSONDoc(attributes, jsonConfig);
    return attributes;
  }  // AbstractAttributesManager<T, Access>::buildObjectFromJSONDoc

  /**
   * @brief Method to take an existing attributes and set its values from passed
   * json config file.
   * @param attribs (out) an existing attributes to be modified.
   * @param jsonConfig json document to parse
   */
  virtual void setValsFromJSONDoc(AttribsPtr attribs,
                                  const io::JsonGenericValue& jsonConfig) = 0;

  /**
   * @brief This function takes the json block specifying user-defined values
   * and parses it into the passed existing attributes.
   * @param attribs (out) an existing attributes to be modified.
   * @param jsonConfig json document to parse
   * @return true if tag is found, of appropriate configuration, and holds
   * actual values.
   */
  bool parseUserDefinedJsonVals(
      const attributes::AbstractAttributes::ptr& attribs,
      const io::JsonGenericValue& jsonConfig) const {
    return this->parseSubconfigJsonVals("user_defined", attribs, jsonConfig);
  }  // AbstractAttributesManager<T, Access>::parseUserDefinedJsonVals

  /**
   * @brief This function takes the passed json block @p jsonConfig, looks for
   * @p subGroupName , and if found in json document and referencing an
   * appropriately formatted object, will load the found values into a
   * subconfig, keyed by
   * @p subGroupName , placed into @p attribs ' subconfig tree.
   * @param subGroupName The name of the target subgroup in the json document.
   * Will also be the key of the resultant subconfig.
   * @param attribs (out) an existing attributes to be modified.
   * @param jsonConfig json document to parse
   * @return true if tag is found, of appropriate configuration, and holds
   * actual values.
   */
  bool parseSubconfigJsonVals(
      const std::string& subGroupName,
      const attributes::AbstractAttributes::ptr& attribs,
      const io::JsonGenericValue& jsonConfig) const;

  /**
   * @brief Returns actual attributes handle containing @p attrName as a
   * substring, or the empty string if none exists.
   * Does a substring search, and returns first value found.
   * @param attrName name to be used as searching substring in @p attrMgr
   * @return actual name of attributes in attrMgr, or empty string if does not
   * exist.
   */
  inline std::string getFullAttrNameFromStr(const std::string& attrName) const {
    if (this->getObjectLibHasHandle(attrName)) {
      return attrName;
    }
    auto handleList = this->getObjectHandlesBySubstring(attrName);
    if (!handleList.empty()) {
      return handleList[0];
    }
    return "";
  }  // getFullAttrNameFromStr

  /**
   * @brief This function will be called to finalize attributes' paths before
   * registration, moving fully qualified paths to the appropriate hidden
   * attribute fields. This can also be called without registration to make sure
   * the paths specified in an attributes are properly configured.
   * @param attributes The attributes to be filtered.
   */
  virtual void finalizeAttrPathsBeforeRegister(
      const AttribsPtr& attributes) const = 0;

 protected:
  /**
   * @brief Called internally right before attribute registration. Filepaths
   * in the json configs for Habitat SceneDatasets are specified relative to
   * the location of the config in the file hierarchy. As the config is loaded,
   * these filepaths will be fully qualified with absolute paths so that they
   * can be easily found and consumed on command. However, saving fully
   * qualified filepaths to JSON configs is undesirable.
   *
   * This function will examine the @p curRelPathName to see that it is a
   * relative path in relation to the @p attributes' set filepath, and if not it
   * will strip out the path information and resave the name to @p rePathSetter.
   * It will query the passed @p curFQPathName to see that it is a fully
   * qualified path to a file that exists, and if not it will add the
   * attributes' filepath and resave it to @p fqPathSetter.
   *
   *
   * It is assumed that what is passed to this function always referneces a
   * filepath, so if, for example, this is called on a render asset filepath,
   * it is assumed that that filepath does not reference a primitive
   * (non-file-based) asset.
   *
   * @param attributes The Configuration-backed attributes owning the filepath
   * in question.
   * @param curRelPathName The currently set relative path filepath in the
   * attributes
   * @param curFQPathName The currently set fully qualified filepath in the
   * attributes
   * @param relPathSetter The relative filepath setter in the attributes
   * @param fqPathSetter The fully qualified filepath setter in the attributes
   */
  void filterAttribsFilenames(
      const AttribsPtr& attributes,
      const std::string& curRelPathName,
      const std::string& curFQPathName,
      const std::function<void(const std::string&)>& relPathSetter,
      const std::function<void(const std::string&)>& fqPathSetter) const;

  /**
   * @brief Called intenrally from createObject.  This will create either a
   * file based AbstractAttributes or a default one based on whether the
   * passed file name exists and has appropriate string tag/extension for @ref
   * esp::metadata::attributes::AbstractAttributes.
   *
   * @param filename the file holding the configuration of the object
   * @param msg reference to progress message
   * @param registerObj whether the new object should be registered in library
   * @return the created @ref esp::metadata::attributes::AbstractAttributes.
   */
  AttribsPtr createFromJsonOrDefaultInternal(const std::string& filename,
                                             std::string& msg,
                                             bool registerObj);

  /**
   * @brief This function is accessed by various attributes managers during
   * loading. It provides a generic handler to look for a particular @p jsonTag
   * in the given @p jsonConfig document, and if it is present it will attempt
   * to find the value in the passed string-keyed @p mapToUse of enum config
   * values. If the string is present it will return the string, otherwise it
   * will return an empty string. The consumer of the function is responsible
   * for only setting a config value if the string is not empty. The enum class
   * should have the first value defined as Unspecified and given the value
   * ID_UNDEFINED.
   * @tparam Type of enum class referenced by @p mapToUse
   * @param jsonConfig The source json document
   * @param jsonTag The field to look for. If present expected to hold a string
   * @param mapName String map name, to be displayed if the value found in json
   * is not found in @p mapToUse
   * @param saveUnspecified Whether to save the 'unspecified' string in value
   * setter if no value is specified in json.
   * @param mapToUse String-keyed map of enum values supported for the field
   * that corresponds to @p jsonTag. See attributes::AttributesEnumMaps.h/cpp.
   * @param valueSetter The config function to use to set the value
   */
  template <class M>
  void setEnumStringFromJsonDoc(
      const io::JsonGenericValue& jsonConfig,
      const char* jsonTag,
      const std::string& mapName,
      bool saveUnspecified,
      const std::map<std::string, M>& mapToUse,
      const std::function<void(const std::string&)>& valueSetter);

  /**
   * @brief Set a filename attribute to hold the appropriate data if the
   * existing attribute's given path contains the sentinel tag value defined at
   * @ref esp::metadata::CONFIG_NAME_AS_ASSET_FILENAME. This will be specified in
   * the Scene Dataset configuration file in the "default_attributes" tag for
   * any attributes which consume file names to specify that the name specified
   * as the instanced attributes should also be used to build the name of the
   * specified asset. The tag value will be replaced by the attributes object's
   * simplified handle, or if unable to be found, with an empty string.
   *
   * If the given data does not contain the @ref esp::metadata::CONFIG_NAME_AS_ASSET_FILENAME
   * tag, this will attempt to find the file referenced in @p srcAssetFilename
   * directly, first as it is given, and then by prefixing it with the current
   * attributes' source file directory. If found it will call @p filenameSetter
   * with the successful string.
   *
   * This will only be called from the specified manager's initNewObjectInternal
   * function, where the attributes is initially built from a default attributes
   * (if such an attributes exists), since it is only within the default
   * attributes that the tag in question would be specified.
   *
   * @param attributes The AbstractAttributes being worked with.
   * @param srcAssetFilename The given asset's stored filename to be queried for
   * the specified tag. If the tag exists, replace it with the simplified handle
   * given by the attributes (hence copy). If this DNE on disk, add file
   * directory.
   * @param filenameSetter The function to set the filename appropriately for
   * the given asset.
   * @return Whether or not the final value residing within the attribute's
   * asset filename exists or not.
   */
  bool setFilenameFromDefaultTag(
      const AttribsPtr& attributes,
      const std::string& srcAssetFilename,
      const std::function<void(const std::string&)>& filenameSetter);

  /**
   * @brief Set an attribute that holds another attribute's handle to hold the
   * appropriate data if the existing attribute's given path contains the
   * sentinel tag value defined at
   * @ref esp::metadata::CONFIG_NAME_AS_ASSET_FILENAME. This will be used in
   * the Scene Dataset configuration file in the "default_attributes" tag for
   * any attributes which consume attributes handle names to specify that the
   * name given as the instanced attributes should also be used to build the
   * name of the specified asset's handle. The tag value will be replaced by the
   * attributes object's simplified handle.
   *
   * This will only be called from the specified manager's initNewObjectInternal
   * function, where the attributes is initially built from a default attributes
   * (if such an attributes exists).
   * @param attributes The AbstractAttributes being worked with.
   * @param srcAssetHandle The given asset's stored asset handle to be queried
   * for the wildcard tag. If the tag exists, replace it with the simplified
   * handle given by the attributes (hence copy). If this DNE on disk, add file
   * directory.
   * @param handleSetter The function to set the handle appropriately for
   * the given asset.
   * @return Whether the sentinel tag was found or not in the handle
   * specification.
   */
  bool setAttributesHandleFromDefaultTag(
      const AttribsPtr& attributes,
      const std::string& srcAssetHandle,
      const std::function<void(const std::string&)>& handleSetter);

 public:
  ESP_SMART_POINTERS(AbstractAttributesManager<T, Access>)

};  // class AbstractAttributesManager

/////////////////////////////
// Class Template Method Definitions
template <class T, ManagedObjectAccess Access>
std::vector<int>
AbstractAttributesManager<T, Access>::loadAllFileBasedTemplates(
    const std::vector<std::string>& paths,
    bool saveAsDefaults) {
  std::vector<int> templateIndices(paths.size(), ID_UNDEFINED);
  if (paths.size() > 0) {
    std::string dir = CrPath::path(paths[0]);
    ESP_DEBUG() << "Loading" << paths.size() << "" << this->objectType_
                << "templates found in" << dir;
    for (uint32_t i = 0; i < paths.size(); ++i) {
      auto attributesFilename = paths[i];
      ESP_VERY_VERBOSE() << "Load" << this->objectType_
                         << "template:" << CrPath::filename(attributesFilename);
      auto tmplt = this->createObject(attributesFilename, true);
      // If failed to load, do not attempt to modify further
      if (tmplt == nullptr) {
        continue;
      }
      // save handles in list of defaults, so they are not removed, if desired.
      if (saveAsDefaults) {
        std::string tmpltHandle = tmplt->getHandle();
        this->addUndeletableObjectName(std::move(tmpltHandle));
      }
      templateIndices[i] = tmplt->getID();
    }
  }
  ESP_DEBUG(Mn::Debug::Flag::NoSpace)
      << "<" << this->objectType_
      << "> : Loaded file-based templates: " << std::to_string(paths.size());
  return templateIndices;
}  // AbstractAttributesManager<T, Access>::loadAllObjectTemplates

template <class T, ManagedObjectAccess Access>
std::vector<int>
AbstractAttributesManager<T, Access>::loadAllTemplatesFromPathAndExt(
    const std::string& path,
    const std::string& extType,
    bool saveAsDefaults) {
  namespace Dir = Cr::Utility::Path;
  std::vector<std::string> paths;
  std::vector<int> templateIndices;
  ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
      << "<" << this->objectType_ << "> : Searching for files at path: `"
      << path << "` with `" << extType << "` files";
  // Check if directory
  const bool dirExists = Dir::isDirectory(path);
  if (dirExists) {
    ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
        << "Searching " << this->objectType_ << " library directory: `" << path
        << "` for `" << extType << "` files";
    for (auto& file : *Dir::list(path, Dir::ListFlag::SortAscending)) {
      std::string absoluteSubfilePath = Dir::join(path, file);
      if (Cr::Utility::String::endsWith(absoluteSubfilePath, extType)) {
        paths.push_back(absoluteSubfilePath);
      }
    }
  } else {
    ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
        << "Parsing " << this->objectType_ << " path: `" << path
        << "` to see if valid `" << extType << "` file.";
    // not a directory, perhaps a file
    std::string attributesFilepath =
        this->convertFilenameToPassedExt(path, extType);
    const bool fileExists = Dir::exists(attributesFilepath);

    if (fileExists) {
      paths.push_back(attributesFilepath);
    } else {  // neither a directory or a file
      ESP_WARNING(Mn::Debug::Flag::NoSpace)
          << "<" << this->objectType_ << "> : Parsing `" << extType
          << "` files : Cannot find `" << path << "` as directory or `"
          << attributesFilepath << "` as config file, so template load failed.";
      return templateIndices;
    }  // if fileExists else
  }    // if dirExists else

  // build templates from aggregated paths
  templateIndices = this->loadAllFileBasedTemplates(paths, saveAsDefaults);

  return templateIndices;
}  // AbstractAttributesManager<T, Access>::loadAllTemplatesFromPathAndExt

template <class T, ManagedObjectAccess Access>
void AbstractAttributesManager<T, Access>::buildAttrSrcPathsFromJSONAndLoad(
    const std::string& configDir,
    const std::string& extType,
    const io::JsonGenericValue& filePaths) {
  // configDir must end with an os sep (i.e. '/')
  for (rapidjson::SizeType i = 0; i < filePaths.Size(); ++i) {
    if (!filePaths[i].IsString()) {
      ESP_WARNING(Mn::Debug::Flag::NoSpace)
          << "<" << this->objectType_
          << "> : Invalid path value in file path array element @ idx" << i
          << ". Skipping.";
      continue;
    }
    const char* fileString = filePaths[i].GetString();
    // The full filepath inferred by the given absolute config directory and the
    // passed relative file paths
    std::string dsFilePath = CrPath::join(configDir, fileString);

    ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
        << "<" << this->objectType_ << "> : Config dir : " << configDir
        << " : filePaths[" << i << "] : " << fileString
        << " | Constructed File Path : " << dsFilePath;

    // Search in derived full filepath
    std::vector<std::string> globPaths = io::globDirs(dsFilePath);
    if (globPaths.size() > 0) {
      for (const auto& globPath : globPaths) {
        // load all object templates available as configs in dsFilePath
        ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
            << "<" << this->objectType_ << "> : Glob path result for"
            << dsFilePath << ":" << globPath;
        this->loadAllTemplatesFromPathAndExt(globPath, extType, true);
      }
    } else {
      ESP_WARNING(Mn::Debug::Flag::NoSpace)
          << "<" << this->objectType_ << "> : No Glob path result found for `"
          << dsFilePath << "` so unable to load templates from that path.";
    }
  }
  ESP_DEBUG(Mn::Debug::Flag::NoSpace)
      << "<" << this->objectType_ << "> : " << std::to_string(filePaths.Size())
      << " paths specified in JSON doc for " << this->objectType_
      << " configuration files.";
}  // AbstractAttributesManager<T, Access>::buildAttrSrcPathsFromJSONAndLoad

template <class T, ManagedObjectAccess Access>
auto AbstractAttributesManager<T, Access>::createFromJsonOrDefaultInternal(
    const std::string& filename,
    std::string& msg,
    bool registerObj) -> AttribsPtr {
  AttribsPtr attrs;
  // Modify the passed filename to have the format of a legitimate
  // configuration file for this Attributes by changing the extension
  std::string jsonAttrFileName =
      (Cr::Utility::String::endsWith(filename, this->JSONTypeExt_)
           ? filename
           : this->getFormattedJSONFileName(filename));
  // Check if this configuration file exists and if so use it to build
  // attributes
  bool jsonFileExists = CrPath::exists(jsonAttrFileName);
  ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
      << "<" << this->objectType_ << ">: Proposing JSON name `"
      << jsonAttrFileName << "` from original name `" << filename
      << "`| This file" << (jsonFileExists ? " exists." : " does not exist.");
  if (jsonFileExists) {
    // configuration file exists with requested name, use to build Attributes
    attrs = this->createObjectFromJSONFile(jsonAttrFileName, registerObj);
    if (ESP_LOG_LEVEL_ENABLED(logging::LoggingLevel::Debug)) {
      msg = "JSON Configuration File `" + jsonAttrFileName + "` based";
    }
  } else {
    // An existing, valid configuration file could not be found using the
    // passed filename. Currently non-JSON filenames are used to create new,
    // default attributes.
    attrs = this->createDefaultObject(filename, registerObj);
    // check if original filename is an actual object
    bool fileExists = CrPath::exists(filename);
    // if filename passed is name of some kind of asset, or if it was not
    // found
    if (fileExists) {
      // Built from non-json filename (i.e. glb file)
      attrs->setActualFilename(filename);
      this->setFileDirectoryFromFilePath(attrs);
      // only populate msg if appropriate logging level is enabled
      if (ESP_LOG_LEVEL_ENABLED(logging::LoggingLevel::Debug)) {
        msg = "File `" + filename +
              "` exists but is not a recognized config filename extension, so "
              "new default";
      }
    } else {
      // only populate msg if appropriate logging level is enabled
      if (ESP_LOG_LEVEL_ENABLED(logging::LoggingLevel::Debug)) {
        msg = "File `" + filename + "` not found, so new default";
      }
    }
  }
  return attrs;
}  // AbstractAttributesManager<T, Access>::createFromJsonFileOrDefaultInternal

template <class T, ManagedObjectAccess Access>
bool AbstractAttributesManager<T, Access>::parseSubconfigJsonVals(
    const std::string& subGroupName,
    const attributes::AbstractAttributes::ptr& attribs,
    const io::JsonGenericValue& jsonConfig) const {
  // check for subGroupName tagged-json value and verify it is an object
  io::JsonGenericValue::ConstMemberIterator jsonIter =
      jsonConfig.FindMember(subGroupName.c_str());
  if (jsonIter != jsonConfig.MemberEnd()) {
    if (jsonIter->value.IsObject() || jsonIter->value.IsArray()) {
      // get pointer to new or existing subGroupName-specified subgroup
      // configuration
      std::shared_ptr<Configuration> subGroupPtr =
          attribs->editSubconfig<Configuration>(subGroupName);

      // get json object referenced by tag subGroupName
      const io::JsonGenericValue& jsonObj = jsonIter->value;

      // count number of valid sub-config settings found
      int numConfigSettings = subGroupPtr->loadFromJson(jsonObj);

      return (numConfigSettings > 0);
    } else {
      ESP_WARNING(Mn::Debug::Flag::NoSpace)
          << "<" << this->objectType_
          << "> : " << attribs->getSimplifiedHandle()
          << " configuration specifies `" << subGroupName
          << "` JSON subgroup but its format is unable to be deciphered as a "
             "subconfiguration (i.e. it is not a JSON object or array), so no `"
          << subGroupName << "` configuration values will be loaded.";
      return false;
    }
  }  // if has requested tag
  return false;
}  // AbstractAttributesManager<T, Access>::parseSubconfigJsonVals

template <class T, ManagedObjectAccess Access>
bool AbstractAttributesManager<T, Access>::setFilenameFromDefaultTag(
    const AttribsPtr& attributes,
    const std::string& srcAssetFilename,
    const std::function<void(const std::string&)>& filenameSetter) {
  if (srcAssetFilename.empty()) {
    return false;
  }
  // First check if tag references a file that already exists on disk and is
  // able to be found
  if (CrPath::exists(srcAssetFilename)) {
    // set filename with verified filepath
    filenameSetter(srcAssetFilename);
    return true;
  }
  const auto loc = srcAssetFilename.find(CONFIG_NAME_AS_ASSET_FILENAME);
  if (loc != std::string::npos) {
    std::string tempStr(srcAssetFilename);
    // sentinel tag is found - replace tag with simplified handle of
    // attributes and use filenameSetter and return whether this file exists or
    // not.
    tempStr.replace(loc, strlen(CONFIG_NAME_AS_ASSET_FILENAME),
                    attributes->getSimplifiedHandle());
    if (CrPath::exists(tempStr)) {
      // replace the component of the string containing the tag with the base
      // filename/handle, and verify it exists. Otherwise, clear it.
      filenameSetter(tempStr);
      return true;
    }
    tempStr = CrPath::join(attributes->getFileDirectory(), tempStr);
    if (CrPath::exists(tempStr)) {
      // replace the component of the string containing the tag with the base
      // filename/handle, and verify it exists. Otherwise, clear it.
      filenameSetter(tempStr);
      return true;
    }
    // out of options, clear out the wild-card default so that init-based
    // default is derived and used.
    filenameSetter("");
    return false;
  }
  // no sentinel tag found - check if existing non-empty field exists.
  std::string tempStr =
      CrPath::join(attributes->getFileDirectory(), srcAssetFilename);
  if (CrPath::exists(tempStr)) {
    // path-prefixed filename exists on disk, so set as filename
    filenameSetter(tempStr);
    return true;
  }
  // No file exists with passed name, either as relative path or as
  // wild-card-equipped relative path
  return false;
}  // AbstractAttributesManager<T, Access>::setFilenameFromDefaultTag

template <class T, ManagedObjectAccess Access>
bool AbstractAttributesManager<T, Access>::setAttributesHandleFromDefaultTag(
    const AttribsPtr& attributes,
    const std::string& srcAssetHandle,
    const std::function<void(const std::string&)>& handleSetter) {
  if (srcAssetHandle.empty()) {
    return false;
  }
  std::string tempStr(srcAssetHandle);
  const auto loc = srcAssetHandle.find(CONFIG_NAME_AS_ASSET_FILENAME);
  if (loc != std::string::npos) {
    // sentinel tag is found - replace tag with simplified handle of
    // attributes and use filenameSetter and return whether this file exists or
    // not.
    auto simpleHandle = attributes->getSimplifiedHandle();
    if (simpleHandle == "NONE") {
      tempStr = "";
    } else {
      tempStr.replace(loc, strlen(CONFIG_NAME_AS_ASSET_FILENAME), simpleHandle);
    }
    // DOES NOT VERIFY THAT HANDLE REFERENCES ACTUAL OBJECT
    handleSetter(tempStr);
    return true;
  }
  return false;
}  // AbstractAttributesManager<T, Access>::setAttributesHandleFromDefaultTag

template <class T, ManagedObjectAccess Access>
void AbstractAttributesManager<T, Access>::filterAttribsFilenames(
    const AttribsPtr& attributes,
    const std::string& curRelPathName,
    const std::string& curFQPathName,
    const std::function<void(const std::string&)>& relPathSetter,
    const std::function<void(const std::string&)>& fqPathSetter) const {
  // Get substitute pathnames if either target is empty
  const std::string curFullyQualifiedPathName =
      (curFQPathName.empty() ? curRelPathName : curFQPathName);
  const std::string curRelativePathName =
      (curRelPathName.empty() ? curFullyQualifiedPathName : curRelPathName);
  // Verbose Debug string - only build if we have appropriate logging enabled.
  std::string dispString{""};
  if (ESP_LOG_LEVEL_ENABLED(logging::LoggingLevel::VeryVerbose)) {
    dispString = Cr::Utility::formatString(
        "AttrHandle `{}` class :`{}`|Arg RelPathname `{}`|Arg FQPathName "
        ":`{}`|Rel proposal :`{}`|FQ proposal :`{}`",
        attributes->getHandle(), attributes->getClassKey(), curRelPathName,
        curFQPathName, curRelativePathName, curFullyQualifiedPathName);
  }
  // If both relative and fully qualified paths are empty, skip further
  // processing.
  if (curRelativePathName.empty() && curFullyQualifiedPathName.empty()) {
    ESP_VERY_VERBOSE() << "BOTH RELATIVE AND FQ PATHS ARE EMPTY Skipping: "
                       << dispString;
    return;
  }

  // Initialize potentially empty field.
  relPathSetter(curRelativePathName);
  // Check if expected relative filepath is accessible on disk (therefore is
  // fully qualified)
  bool relPathNameExists = CrPath::exists(curRelativePathName);
  if (relPathNameExists) {
    if (ESP_LOG_LEVEL_ENABLED(logging::LoggingLevel::VeryVerbose)) {
      Cr::Utility::formatInto(dispString, dispString.size(),
                              "|Rel proposal is FOUND/FQ");
    }
    // Set the fully qualified value to be the curRelativePathName since it
    // exists in the filesystem.
    fqPathSetter(curRelativePathName);
  } else {
    // initialize fully-qualified name if empty
    fqPathSetter(curFullyQualifiedPathName);
  }

  // Get the attributes filepath that our desired filepath should be
  // relative to. If this is empty or unknown then we have no path to set the
  // target configuration path relative to and so we will preserve whatever is
  // currently set as the path in the target relative field. This potentially
  // fully qualified path will also be saved to file should the owning
  // attributes be saved.
  const std::string attrFilepath = attributes->getFileDirectory();
  if (attrFilepath.empty()) {
    ESP_VERY_VERBOSE() << "EMPTY FILEPATH Skipping : " << dispString;
    // if filepath is empty, do nothing more.
    return;
  }

  // Verify non-empty attributes filepath exists in the filesystem
  if (CrPath::isDirectory(attrFilepath)) {
    if (ESP_LOG_LEVEL_ENABLED(logging::LoggingLevel::VeryVerbose)) {
      Cr::Utility::formatInto(dispString, dispString.size(),
                              "|attributes Filepath :`{}`", attrFilepath);
    }

    // If expected relative filepath is accessible on disk, and therefore is
    // fully qualified, then make relative to attributes filepath.
    if (relPathNameExists) {
      // Get new path relative to attrFilepath
      const std::string newRelFilepath =
          io::getPathRelativeToAbsPath(curRelativePathName, attrFilepath);
      // save the new relative filepath
      relPathSetter(newRelFilepath);
      if (ESP_LOG_LEVEL_ENABLED(logging::LoggingLevel::VeryVerbose)) {
        Cr::Utility::formatInto(dispString, dispString.size(),
                                "|New Rel proposal :`{}`", newRelFilepath);
      }
    }
  }  // if attributes dir is set properly
  else {
    // This means the attributes filepath is not empty but is not found on
    // disk either, probably indicative of a much bigger issue.
    if (ESP_LOG_LEVEL_ENABLED(logging::LoggingLevel::VeryVerbose)) {
      Cr::Utility::formatInto(dispString, dispString.size(),
                              "|ATTR PATH DOES NOT EXIST IN SYSTEM :`{}` ",
                              attrFilepath);
    }
  }

  ESP_VERY_VERBOSE() << dispString;
}  // AbstractAttributesManager<T, Access>::filterAttribsFilenames

template <class T, ManagedObjectAccess Access>
template <class M>
void AbstractAttributesManager<T, Access>::setEnumStringFromJsonDoc(
    const io::JsonGenericValue& jsonConfig,
    const char* jsonTag,
    const std::string& mapName,
    bool saveUnspecified,
    const std::map<std::string, M>& mapToUse,
    const std::function<void(const std::string&)>& valueSetter) {
  std::string tmpStrVal = "";
  if (io::readMember<std::string>(jsonConfig, jsonTag, tmpStrVal)) {
    std::string strToLookFor = Cr::Utility::String::lowercase(tmpStrVal);
    // If unspecified, do not set config value unless from instance attributes
    if (strToLookFor != "unspecified") {
      auto found = mapToUse.find(strToLookFor);
      if (found != mapToUse.end()) {
        // only override attributes default value if new value is valid
        valueSetter(strToLookFor);
      } else {
        ESP_WARNING(Mn::Debug::Flag::NoSpace)
            << "`" << jsonTag << "` value in JSON : `" << tmpStrVal
            << "` does not map to a valid `" << mapName
            << "` value, so not setting/overriding default `" << jsonTag
            << "` value.";
      }
    } else if (saveUnspecified) {
      // only true if writing to an instance attributes and config holds
      // unspecified value. If writing to an object/stage/articulated object
      // config, just keep default value.
      // TODO default all instance attributes to be unspecified for the
      // supported config values. Then this would be unnecessary
      valueSetter("unspecified");
    }
  }
}  // AbstractAttributesManager<T, Access>::setEnumStringFromJsonDoc

}  // namespace managers
}  // namespace metadata
}  // namespace esp
#endif  // ESP_METADATA_MANAGERS_ABSTRACTATTRIBUTESMANAGER_H_
