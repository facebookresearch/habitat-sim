// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_MANAGERS_ATTRIBUTESMANAGERBASE_H_
#define ESP_METADATA_MANAGERS_ATTRIBUTESMANAGERBASE_H_

/** @file
 * @brief Class Template @ref esp::metadata::managers::AttributesManager
 */

#include "esp/metadata/attributes/AttributesBase.h"

#include "esp/core/managedContainers/ManagedFileBasedContainer.h"
#include "esp/io/io.h"

namespace Cr = Corrade;

namespace esp {
namespace core {
enum class ManagedObjectAccess;
class ManagedFileBasedContainerBase;
}  // namespace core
namespace metadata {
namespace managers {

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
template <class T, core::ManagedObjectAccess Access>
class AttributesManager
    : public esp::core::ManagedFileBasedContainer<T, Access> {
 public:
  static_assert(std::is_base_of<attributes::AbstractAttributes, T>::value,
                "AttributesManager :: Managed object type must be derived from "
                "AbstractAttributes");

  typedef std::shared_ptr<T> AttribsPtr;

  AttributesManager(const std::string& attrType, const std::string& JSONTypeExt)
      : esp::core::ManagedFileBasedContainer<T, Access>::
            ManagedFileBasedContainer(attrType),
        JSONTypeExt_(JSONTypeExt) {}
  ~AttributesManager() override = default;

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
   * @brief Check if currently configured primitive asset template library has
   * passed handle.
   * @param handle String name of primitive asset attributes desired
   * @return whether handle exists or not in asset attributes library
   */
  virtual bool isValidPrimitiveAttributes(const std::string& handle) = 0;

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
  }  // AttributesManager<T, Access>::buildObjectFromJSONDoc

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
      const io::JsonGenericValue& jsonConfig) const;

  /**
   * @brief Return a properly formated JSON file name for the attributes
   * managed by this manager.  This will change the extension to the
   * appropriate json extension.
   * @param filename The original filename
   * @return a candidate JSON file name for the attributes managed by this
   * manager.
   */
  std::string getFormattedJSONFileName(const std::string& filename) {
    return this->convertFilenameToPassedExt(filename, this->JSONTypeExt_);
  }

 protected:
  /**
   * @brief Called intenrally from createObject.  This will create either a
   * file based AbstractAttributes or a default one based on whether the
   * passed file name exists and has appropriate string tag/extension for @ref
   * esp::metadata::attributes::AbstractAttributes.
   *
   * @param filename the file holding the configuration of the object
   * @param msg reference to progress message
   * @param registerObj whether the new object should be registered in library
   * @return the create @ref esp::metadata::attributes::AbstractAttributes.
   */
  AttribsPtr createFromJsonOrDefaultInternal(const std::string& filename,
                                             std::string& msg,
                                             bool registerObj);

  // ======== Typedefs and Instance Variables ========
  /**
   * @brief The string extension for json files for this manager's attributes
   * types
   */
  const std::string JSONTypeExt_;

 public:
  ESP_SMART_POINTERS(AttributesManager<T, Access>)

};  // class AttributesManager

/////////////////////////////
// Class Template Method Definitions
template <class T, core::ManagedObjectAccess Access>
std::vector<int> AttributesManager<T, Access>::loadAllFileBasedTemplates(
    const std::vector<std::string>& paths,
    bool saveAsDefaults) {
  std::vector<int> templateIndices(paths.size(), ID_UNDEFINED);
  if (paths.size() > 0) {
    std::string dir = Cr::Utility::Directory::path(paths[0]);
    LOG(INFO) << "<" << this->objectType_
              << ">::loadAllFileBasedTemplates : Loading " << paths.size()
              << " " << this->objectType_ << " templates found in " << dir;
    for (int i = 0; i < paths.size(); ++i) {
      auto attributesFilename = paths[i];
      LOG(INFO) << "::loadAllFileBasedTemplates : Load " << this->objectType_
                << " template: "
                << Cr::Utility::Directory::filename(attributesFilename);
      auto tmplt = this->createObject(attributesFilename, true);
      // If failed to load, do not attempt to modify further
      if (tmplt == nullptr) {
        continue;
      }
      // save handles in list of defaults, so they are not removed, if desired.
      if (saveAsDefaults) {
        std::string tmpltHandle = tmplt->getHandle();
        this->undeletableObjectNames_.insert(tmpltHandle);
      }
      templateIndices[i] = tmplt->getID();
    }
  }
  LOG(INFO) << "<" << this->objectType_
            << ">::loadAllFileBasedTemplates : Loaded file-based templates: "
            << std::to_string(paths.size());
  return templateIndices;
}  // AttributesManager<T, Access>::loadAllObjectTemplates

template <class T, core::ManagedObjectAccess Access>
std::vector<int> AttributesManager<T, Access>::loadAllTemplatesFromPathAndExt(
    const std::string& path,
    const std::string& extType,
    bool saveAsDefaults) {
  namespace Dir = Cr::Utility::Directory;
  std::vector<std::string> paths;
  std::vector<int> templateIndices;

  // Check if directory
  const bool dirExists = Dir::isDirectory(path);
  if (dirExists) {
    LOG(INFO) << "<" << this->objectType_
              << ">::loadAllTemplatesFromPathAndExt <" << extType
              << "> : Parsing " << this->objectType_
              << " library directory: " + path + " for \'" + extType +
                     "\' files";
    for (auto& file : Dir::list(path, Dir::Flag::SortAscending)) {
      std::string absoluteSubfilePath = Dir::join(path, file);
      if (Cr::Utility::String::endsWith(absoluteSubfilePath, extType)) {
        paths.push_back(absoluteSubfilePath);
      }
    }
  } else {
    // not a directory, perhaps a file
    std::string attributesFilepath =
        this->convertFilenameToPassedExt(path, extType);
    const bool fileExists = Dir::exists(attributesFilepath);

    if (fileExists) {
      paths.push_back(attributesFilepath);
    } else {  // neither a directory or a file
      LOG(WARNING) << "<" << this->objectType_
                   << ">::loadAllTemplatesFromPathAndExt : Parsing "
                   << this->objectType_ << " : Cannot find " << path
                   << " as directory or " << attributesFilepath
                   << " as config file. Aborting parse.";
      return templateIndices;
    }  // if fileExists else
  }    // if dirExists else

  // build templates from aggregated paths
  templateIndices = this->loadAllFileBasedTemplates(paths, saveAsDefaults);

  return templateIndices;
}  // AttributesManager<T, Access>::loadAllTemplatesFromPathAndExt

template <class T, core::ManagedObjectAccess Access>
void AttributesManager<T, Access>::buildAttrSrcPathsFromJSONAndLoad(
    const std::string& configDir,
    const std::string& extType,
    const io::JsonGenericValue& filePaths) {
  for (rapidjson::SizeType i = 0; i < filePaths.Size(); ++i) {
    if (!filePaths[i].IsString()) {
      LOG(ERROR) << "::buildAttrSrcPathsFromJSONAndLoad : "
                    "Invalid path "
                    "value in file path array element @ idx "
                 << i << ". Skipping.";
      continue;
    }
    std::string absolutePath =
        Cr::Utility::Directory::join(configDir, filePaths[i].GetString());
    std::vector<std::string> globPaths = io::globDirs(absolutePath);
    if (globPaths.size() > 0) {
      for (const auto& globPath : globPaths) {
        // load all object templates available as configs in absolutePath
        LOG(WARNING) << "Glob path result for " << absolutePath << " : "
                     << globPath;
        this->loadAllTemplatesFromPathAndExt(globPath, extType, true);
      }
    } else {
      LOG(WARNING) << "No Glob path result for " << absolutePath;
    }
  }
  LOG(INFO) << "<" << this->objectType_
            << ">::buildAttrSrcPathsFromJSONAndLoad : "
            << std::to_string(filePaths.Size())
            << " paths specified in JSON doc for " << this->objectType_
            << " templates.";
}  // AttributesManager<T, Access>::buildAttrSrcPathsFromJSONAndLoad

template <class T, core::ManagedObjectAccess Access>
auto AttributesManager<T, Access>::createFromJsonOrDefaultInternal(
    const std::string& filename,
    std::string& msg,
    bool registerObj) -> AttribsPtr {
  AttribsPtr attrs;
  // Modify the passed filename to have the format of a legitimate
  // configuration file for this Attributes by changing the extension
  std::string jsonAttrFileName =
      (Cr::Utility::String::endsWith(filename, this->JSONTypeExt_)
           ? filename
           : getFormattedJSONFileName(filename));
  // Check if this configuration file exists and if so use it to build
  // attributes
  bool jsonFileExists = (this->isValidFileName(jsonAttrFileName));
  LOG(INFO) << "<" << this->objectType_
            << ">::createFromJsonOrDefaultInternal : Proposing JSON name : "
            << jsonAttrFileName << " from original name : " << filename
            << " | This file "
            << (jsonFileExists ? " exists." : " does not exist.");
  if (jsonFileExists) {
    // configuration file exists with requested name, use to build Attributes
    attrs = this->createObjectFromJSONFile(jsonAttrFileName, registerObj);
    msg = "JSON Configuration File (" + jsonAttrFileName + ") based";
  } else {
    // An existing, valid configuration file could not be found using the
    // passed filename. Currently non-JSON filenames are used to create new,
    // default attributes.
    attrs = this->createDefaultObject(filename, registerObj);
    // check if original filename is an actual object
    bool fileExists = (this->isValidFileName(filename));
    // if filename passed is name of some kind of asset, or if it was not
    // found
    if (fileExists) {
      msg = "File (" + filename +
            ") exists but is not a recognized config filename extension, so "
            "new default";
    } else {
      msg = "File (" + filename + ") not found, so new default";
    }
  }
  return attrs;
}  // AttributesManager<T, Access>::createFromJsonFileOrDefaultInternal

template <class T, core::ManagedObjectAccess Access>
bool AttributesManager<T, Access>::parseUserDefinedJsonVals(
    const attributes::AbstractAttributes::ptr& attribs,
    const io::JsonGenericValue& jsonConfig) const {
  // check for user defined attributes
  if (jsonConfig.HasMember("user_defined")) {
    if (!jsonConfig["user_defined"].IsObject()) {
      LOG(WARNING) << "<" << this->objectType_
                   << ">::parseUserDefinedJsonVals : "
                   << attribs->getSimplifiedHandle()
                   << " attributes specifies user_defined attributes but they "
                      "are not of the correct format. Skipping.";
      return false;
    } else {
      const auto& userObj = jsonConfig["user_defined"];
      // count number of valid user config settings found
      int numConfigSettings = 0;
      // jsonConfig is the json object referenced by the tag "user_defined" in
      // the original config file.  By here it is guaranteed to be a json
      // object.
      for (rapidjson::Value::ConstMemberIterator it = userObj.MemberBegin();
           it != userObj.MemberEnd(); ++it) {
        // for each key, attempt to parse
        const std::string key = it->name.GetString();
        const auto& obj = it->value;
        // increment, assuming is valid object
        ++numConfigSettings;
        if (obj.IsFloat()) {
          attribs->setUserConfigValue(key, obj.GetFloat());
        } else if (obj.IsDouble()) {
          attribs->setUserConfigValue(key, obj.GetDouble());
        } else if (obj.IsNumber()) {
          attribs->setUserConfigValue(key, obj.Get<int>());
        } else if (obj.IsString()) {
          attribs->setUserConfigValue(key, obj.GetString());
        } else if (obj.IsBool()) {
          attribs->setUserConfigValue(key, obj.GetBool());
        } else if (obj.IsArray() && obj.Size() > 0 && obj[0].IsNumber()) {
          // numeric vector or quaternion
          if (obj.Size() == 3) {
            Magnum::Vector3 val{};
            if (io::fromJsonValue(obj, val)) {
              attribs->setUserConfigValue(key, val);
            }
          } else if (obj.Size() == 4) {
            // assume is quaternion
            Magnum::Quaternion val{};
            if (io::fromJsonValue(obj, val)) {
              attribs->setUserConfigValue(key, val);
            }
          } else {
            // decrement count for key:obj due to not being handled vector
            --numConfigSettings;
            // TODO support numeric array in JSON
            LOG(WARNING)
                << "<" << this->objectType_
                << ">::parseUserDefinedJsonVals : For "
                << attribs->getSimplifiedHandle()
                << " attributes, user_defined config cell in JSON document "
                   "contains key "
                << key
                << " referencing an unsupported numeric array of length : "
                << obj.Size() << " so skipping.";
          }
        } else {
          // TODO support other types?
          // decrement count for key:obj due to not being handled type
          --numConfigSettings;
          LOG(WARNING)
              << "<" << this->objectType_
              << ">::parseUserDefinedJsonVals : For "
              << attribs->getSimplifiedHandle()
              << " attributes, user_defined config cell in JSON document "
                 "contains key "
              << key
              << " referencing an unknown/unparsable value, so skipping this "
                 "key.";
        }
      }
      // whether or not any valid configs were found
      return (numConfigSettings > 0);
    }
  }  // if has user_defined tag
  return false;
}  // AttributesManager<T, Access>::parseUserDefinedJsonVals

}  // namespace managers
}  // namespace metadata
}  // namespace esp
#endif  // ESP_METADATA_MANAGERS_ATTRIBUTESMANAGERBASE_H_
