// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SemanticAttributes.h"
namespace esp {
namespace metadata {
namespace attributes {

/////////////////////////////////
// SemanticVolumeAttributes

SemanticVolumeAttributes::SemanticVolumeAttributes(const std::string& handle)
    : AbstractAttributes("SemanticVolumeAttributes", handle) {
  // Initialize values
  init("floor_height", 0.0);
  init("extrusion_height", 2.5);

}  // SemanticVolumeAttributes ctor

std::string SemanticVolumeAttributes::getObjectInfoHeaderInternal() const {
  std::string res =
      "Name,Label,Floor Height,Extrusion Height,Min Bounds,Max Bounds,";
  const auto& polyLoop = getPolyLoop();
  for (uint32_t iter = 0; iter < polyLoop.size(); ++iter) {
    Cr::Utility::formatInto(res, res.size(), "Poly Vert {} XYZ,",
                            std::to_string(iter));
  }
  return res;
}  // namespace attributes

std::string SemanticVolumeAttributes::getObjectInfoInternal() const {
  std::string res = Cr::Utility::formatString(
      "{},{},{},{},{},{},", getHandle(), getAsString("label"),
      getAsString("floor_height"), getAsString("extrusion_height"),
      getAsString("min_bounds"), getAsString("max_bounds"));
  Cr::Utility::formatInto(res, res.size(), "[");
  const auto& polyLoop = getPolyLoop();
  for (const auto& pt : polyLoop) {
    Cr::Utility::formatInto(res, res.size(), "[{} {} {}],", pt.x(), pt.y(),
                            pt.z());
  }
  Cr::Utility::formatInto(res, res.size(), "],");

  return res;
}  // SemanticVolumeAttributes::getObjectInfoInternal()

void SemanticVolumeAttributes::writeValuesToJson(
    io::JsonGenericValue& jsonObj,
    io::JsonAllocator& allocator) const {
  // map "handle" to "name" key in json - this SemanticVolumeAttributes' handle
  // is its unique name
  writeValueToJson("handle", "name", jsonObj, allocator);
  writeValueToJson("label", jsonObj, allocator);
  writeValueToJson("floor_height", jsonObj, allocator);
  writeValueToJson("extrusion_height", jsonObj, allocator);
  writeValueToJson("min_bounds", jsonObj, allocator);
  writeValueToJson("max_bounds", jsonObj, allocator);

}  // SemanticVolumeAttributes::writeValuesToJson

/////////////////////////////////
// SemanticAttributes
SemanticAttributes::SemanticAttributes(const std::string& handle)
    : AbstractAttributes("SemanticAttributes", handle) {
  // set empty defaults for handles
  init("semantic_descriptor_filename", "");
  setHidden("__semanticDescriptorFullPath", "");
  init("semantic_asset", "");
  setHidden("__semanticAssetFullPath", "");
  init("semantic_up", Mn::Vector3{0.0, 1.0, 0.0});
  init("semantic_front", Mn::Vector3{0.0, 0.0, -1.0});
  init("use_semantic_frame", false);
  // setting default for semantic assets having semantically painted textures to
  // false
  init("has_semantic_textures", false);
  // 4 corresponds to AssetType::InstanceMesh
  initSemanticAssetTypeEnum(AssetType::InstanceMesh);
  // get refs to internal subconfigs for semantic region attributes
  regionAnnotationConfig_ = editSubconfig<Configuration>("region_annotations");
}  // SemanticAttributes ctor

SemanticAttributes::SemanticAttributes(const SemanticAttributes& otr)
    : AbstractAttributes(otr),
      availableRegionInstIDs_(otr.availableRegionInstIDs_) {
  // get refs to internal subconfigs for semantic region attributes
  regionAnnotationConfig_ = editSubconfig<Configuration>("region_annotations");
  copySubconfigIntoMe<SemanticVolumeAttributes>(otr.regionAnnotationConfig_,
                                                regionAnnotationConfig_);
}  // SemanticAttributes copy ctor

SemanticAttributes::SemanticAttributes(SemanticAttributes&& otr) noexcept
    : AbstractAttributes(std::move(static_cast<AbstractAttributes&&>(otr))),
      availableRegionInstIDs_(std::move(otr.availableRegionInstIDs_)) {
  // get refs to internal subconfigs for semantic region attributes
  // original data were moved over so should retain full derived class
  regionAnnotationConfig_ = editSubconfig<Configuration>("region_annotations");
}  // SemanticAttributes move ctor

void SemanticAttributes::writeValuesToJson(io::JsonGenericValue& jsonObj,
                                           io::JsonAllocator& allocator) const {
  if (getSemanticDescriptorFilename() != "") {
    writeValueToJson("semantic_descriptor_filename", jsonObj, allocator);
  }
  if (getSemanticAssetHandle() != "") {
    writeValueToJson("semantic_asset", jsonObj, allocator);
    if (getHasSemanticTextures()) {
      writeValueToJson("has_semantic_textures", jsonObj, allocator);
    }
    if (getUseSpecifiedSemanticFrame()) {
      writeValueToJson("semantic_up", jsonObj, allocator);
      writeValueToJson("semantic_front", jsonObj, allocator);
    }
  }
}  // SemanticAttributes::writeValuesToJson

void SemanticAttributes::writeSubconfigsToJson(
    io::JsonGenericValue& jsonObj,
    io::JsonAllocator& allocator) const {
  // build list of JSON objs from subconfigs describing Semantic Region
  // Attributes

  auto regionCfgIterPair = regionAnnotationConfig_->getSubconfigIterator();
  io::JsonGenericValue regionInstArray(rapidjson::kArrayType);
  for (auto& cfgIter = regionCfgIterPair.first;
       cfgIter != regionCfgIterPair.second; ++cfgIter) {
    regionInstArray.PushBack(cfgIter->second->writeToJsonObject(allocator),
                             allocator);
  }
  jsonObj.AddMember("region_annotations", regionInstArray, allocator);

  // iterate through other subconfigs using standard handling
  // do not resave Region Attributes instances
  auto cfgIterPair = getSubconfigIterator();
  for (auto& cfgIter = cfgIterPair.first; cfgIter != cfgIterPair.second;
       ++cfgIter) {
    if (cfgIter->first == "region_annotations") {
      continue;
    }
    // only save if subconfig has entries
    if (cfgIter->second->getNumEntries() > 0) {
      rapidjson::GenericStringRef<char> name{cfgIter->first.c_str()};
      io::JsonGenericValue subObj =
          cfgIter->second->writeToJsonObject(allocator);
      jsonObj.AddMember(name, subObj, allocator);
    } else {
      ESP_VERY_VERBOSE()
          << "Unitialized/empty Subconfig in Configuration @ key ["
          << cfgIter->first
          << "], so nothing will be written to JSON for this key.";
    }
  }  // iterate through all subconfigurations

}  // SemanticAttributes::writeSubconfigsToJson

SemanticAttributes& SemanticAttributes::operator=(
    const SemanticAttributes& otr) {
  if (this != &otr) {
    this->AbstractAttributes::operator=(otr);
    availableRegionInstIDs_ = otr.availableRegionInstIDs_;
    // get refs to internal subconfigs for semantic region attributes
    regionAnnotationConfig_ =
        editSubconfig<Configuration>("region_annotations");
    copySubconfigIntoMe<SemanticVolumeAttributes>(otr.regionAnnotationConfig_,
                                                  regionAnnotationConfig_);
  }
  return *this;
}  // SemanticAttributes copy assignment

SemanticAttributes& SemanticAttributes::operator=(
    SemanticAttributes&& otr) noexcept {
  availableRegionInstIDs_ = std::move(otr.availableRegionInstIDs_);
  this->AbstractAttributes::operator=(static_cast<AbstractAttributes&&>(otr));
  // original data were moved over so should retain full derived class
  regionAnnotationConfig_ = editSubconfig<Configuration>("region_annotations");
  return *this;
}  // SemanticAttributes move assignment

std::string SemanticAttributes::getObjectInfoInternal() const {
  // Semantic description info base-level values
  // std::string res = "";
  // TODO do this for any SemanticAttributes level values
  std::string res = Cr::Utility::formatString(
      "\nSemantic Scene Descriptor Filename,Semantic Mesh Asset,\n{},{}\n",
      getSemanticDescriptorFilename(), getSemanticAssetHandle());

  int iter = 0;
  // region annotation instance info
  const auto& regionInstances = getRegionInstances();
  for (const auto& regionInst : regionInstances) {
    if (iter == 0) {
      ++iter;
      Cr::Utility::formatInto(res, res.size(), "Region Annotation Info :\n{}\n",
                              regionInst->getObjectInfoHeader());
    }
    Cr::Utility::formatInto(res, res.size(), "{}\n",
                            regionInst->getObjectInfo());
  }

  Cr::Utility::formatInto(res, res.size(),
                          "End of data for Semantic Attributes {}\n",
                          getSimplifiedHandle());
  return res;
}  // SemanticAttributes::getObjectInfoInternal
}  // namespace attributes
}  // namespace metadata
}  // namespace esp
