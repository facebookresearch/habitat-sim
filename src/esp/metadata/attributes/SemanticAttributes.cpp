// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SemanticAttributes.h"
namespace esp {
namespace metadata {
namespace attributes {

/////////////////////////////////
// SemanticRegionAttributes

SemanticRegionAttributes::SemanticRegionAttributes(const std::string& handle)
    : AbstractAttributes("SemanticRegionAttributes", handle) {
  // Initialize values
  set("floor_height", 0.0);
  set("extrusion_height", 2.5);

}  // SemanticRegionAttributes ctor

std::string SemanticRegionAttributes::getObjectInfoHeaderInternal() const {
  std::string res =
      "Name,Label,Floor Height,Extrusion Height,Min Bounds,Max Bounds,";
  int iter = 0;
  for (const auto& it : polyLoop_) {
    Cr::Utility::formatInto(res, res.size(), "Poly Vert {},",
                            std::to_string(iter++));
  }
  return res;
}

std::string SemanticRegionAttributes::getObjectInfoInternal() const {
  std::string res = Cr::Utility::formatString(
      "{},{},{},{},{},{},", getAsString("handle"), getAsString("label"),
      getAsString("floor_height"), getAsString("extrusion_height"),
      getAsString("min_bounds"), getAsString("max_bounds"));
  Cr::Utility::formatInto(res, res.size(), "[");
  for (const Magnum::Vector3& pt : polyLoop_) {
    Cr::Utility::formatInto(res, res.size(), "[{} {} {}],", pt.x(), pt.y(),
                            pt.z());
  }
  Cr::Utility::formatInto(res, res.size(), "],");

  return res;
}  // SemanticRegionAttributes::getObjectInfoInternal()

void SemanticRegionAttributes::writeValuesToJson(
    io::JsonGenericValue& jsonObj,
    io::JsonAllocator& allocator) const {
  // map "handle" to "name" key in json - this SemanticRegionAttributes' handle
  // is its unique name
  writeValueToJson("handle", "name", jsonObj, allocator);
  writeValueToJson("label", jsonObj, allocator);
  writeValueToJson("floor_height", jsonObj, allocator);
  writeValueToJson("extrusion_height", jsonObj, allocator);
  writeValueToJson("min_bounds", jsonObj, allocator);
  writeValueToJson("max_bounds", jsonObj, allocator);
  // write out poly loop point array.
  if (!polyLoop_.empty()) {
    io::addMember(jsonObj, "poly_loop", polyLoop_, allocator);
  }

}  // SemanticRegionAttributes::writeValuesToJson

/////////////////////////////////
// SemanticAttributes
SemanticAttributes::SemanticAttributes(const std::string& handle)
    : AbstractAttributes("SemanticAttributes", handle) {
  // get refs to internal subconfigs for semantic region attributes
  regionAnnotationConfig_ = editSubconfig<Configuration>("region_annotations");
}  // SemanticAttributes ctor

SemanticAttributes::SemanticAttributes(const SemanticAttributes& otr)
    : AbstractAttributes(otr),
      availableRegionInstIDs_(otr.availableRegionInstIDs_) {
  // get refs to internal subconfigs for semantic region attributes
  regionAnnotationConfig_ = editSubconfig<Configuration>("region_annotations");
  copySubconfigIntoMe<SemanticRegionAttributes>(otr.regionAnnotationConfig_,
                                                regionAnnotationConfig_);
}  // SemanticAttributes copy ctor

SemanticAttributes::SemanticAttributes(SemanticAttributes&& otr) noexcept
    : AbstractAttributes(std::move(static_cast<AbstractAttributes>(otr))),
      availableRegionInstIDs_(std::move(otr.availableRegionInstIDs_)) {
  // get refs to internal subconfigs for semantic region attributes
  // original data were moved over so should retain full derived class
  regionAnnotationConfig_ = editSubconfig<Configuration>("region_annotations");
}  // SemanticAttributes move ctor

void SemanticAttributes::writeValuesToJson(io::JsonGenericValue& jsonObj,
                                           io::JsonAllocator& allocator) const {
  // TODO : handle all root level attributes defined for SemanticAttributes
  // objects
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
    copySubconfigIntoMe<SemanticRegionAttributes>(otr.regionAnnotationConfig_,
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
  std::string res = "";
  // TODO do this for any SemanticAttributes level values
  // std::string res = Cr::Utility::formatString(
  //     "\nlabel1,label2,\n{},{}\n",
  //     getValue1(),getValue2());

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