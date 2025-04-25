// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "PrimitiveAssetAttributes.h"

namespace esp {
namespace metadata {
namespace attributes {

AbstractPrimitiveAttributes::AbstractPrimitiveAttributes(
    bool isWireframe,
    int primObjType,
    const std::string& primObjClassName,
    const std::string& attributesClassKey)
    : AbstractAttributes(attributesClassKey, "") {
  // clear handle that was set in base class constructor
  AbstractAttributes::setHandle("");
  setIsWireframe(isWireframe);
  setPrimObjType(primObjType);
  setPrimObjClassName(primObjClassName);
  setFileDirectory("none");
  // initialize so empty values are present
  set("halfLength", 0.0);
  set("segments", 0);
  set("rings", 0);
  if (!isWireframe) {  // solid only
    // do not call setters since they call buildHandle, which does not
    // exist here - is abstract in base class
    set("textureCoordinates", false);
    set("tangents", false);
  }
  set("materialStr", "default");
  materialConfig_ = editSubconfig<Configuration>("materialDefinition");
}  // ctor

AbstractPrimitiveAttributes::AbstractPrimitiveAttributes(
    const AbstractPrimitiveAttributes& otr)
    : AbstractAttributes(otr) {
  materialConfig_ = editSubconfig<Configuration>("materialDefinition");

  copySubconfigIntoMe<Configuration>(otr.materialConfig_, materialConfig_);
}

AbstractPrimitiveAttributes::AbstractPrimitiveAttributes(
    AbstractPrimitiveAttributes&& otr)
    : AbstractAttributes(std::move(static_cast<AbstractAttributes>(otr))) {
  materialConfig_ = editSubconfig<Configuration>("materialDefinition");
}

AbstractPrimitiveAttributes& AbstractPrimitiveAttributes::operator=(
    const AbstractPrimitiveAttributes& otr) {
  if (this != &otr) {
    this->AbstractAttributes::operator=(otr);
    materialConfig_ = editSubconfig<Configuration>("materialDefinition");
    copySubconfigIntoMe<Configuration>(otr.materialConfig_, materialConfig_);
  }
  return *this;
}

AbstractPrimitiveAttributes& AbstractPrimitiveAttributes::operator=(
    AbstractPrimitiveAttributes&& otr) {
  this->AbstractAttributes::operator=(static_cast<AbstractAttributes&&>(otr));
  copySubconfigIntoMe<Configuration>(otr.materialConfig_, materialConfig_);
  return *this;
}

CapsulePrimitiveAttributes::CapsulePrimitiveAttributes(
    bool isWireframe,
    int primObjType,
    const std::string& primObjClassName)
    : AbstractPrimitiveAttributes(isWireframe,
                                  primObjType,
                                  primObjClassName,
                                  "CapsulePrimitiveAttributes") {
  set("cylinderRings", 1);
  if (!isWireframe) {  // solid
    set("hemisphereRings", 4);
    set("segments", 12);
    set("halfLength", 0.75);
  } else {  // wireframe
    set("hemisphereRings", 8);
    set("segments", 16);
    set("halfLength", 1.0);
  }
  buildHandle();  // build handle based on config
}  // CapsulePrimitiveAttributes

bool CapsulePrimitiveAttributes::parseStringIntoConfigDetail(
    const std::string& configString) {
  bool hemiRingSet =
      setIntFromConfigKey("_hemiRings_", configString,
                          [this](int val) { setHemisphereRings(val); });
  bool cylRingSet = setIntFromConfigKey(
      "_cylRings_", configString, [this](int val) { setCylinderRings(val); });
  bool segmentSet = setIntFromConfigKey(
      "_segments_", configString, [this](int val) { setNumSegments(val); });
  auto halflenSet = setDoubleFromConfigKey(
      "_halfLen_", configString, [this](double val) { setHalfLength(val); });

  if (!getIsWireframe()) {
    setUseTextureCoords(getBoolForConfigKey("_useTexCoords_", configString));
    setUseTangents(getBoolForConfigKey("_useTangents_", configString));
  }
  return hemiRingSet && cylRingSet && segmentSet && halflenSet;
}  // CapsulePrimitiveAttributes::parseStringIntoConfigDetail(

ConePrimitiveAttributes::ConePrimitiveAttributes(
    bool isWireframe,
    int primObjType,
    const std::string& primObjClassName)
    : AbstractPrimitiveAttributes(isWireframe,
                                  primObjType,
                                  primObjClassName,
                                  "ConePrimitiveAttributes") {
  set("halfLength", 1.25);

  if (!isWireframe) {  // solid
    set("rings", 1);
    set("segments", 12);
    set("capEnd", true);
  } else {  // wireframe
    set("segments", 32);
  }
  buildHandle();  // build handle based on config
}  // ConePrimitiveAttributes

bool ConePrimitiveAttributes::parseStringIntoConfigDetail(
    const std::string& configString) {
  bool segmentSet = setIntFromConfigKey(
      "_segments_", configString, [this](int val) { setNumSegments(val); });
  auto halflenSet = setDoubleFromConfigKey(
      "_halfLen_", configString, [this](double val) { setHalfLength(val); });
  bool ringSet = true;
  if (!getIsWireframe()) {
    ringSet = setIntFromConfigKey("_rings_", configString,
                                  [this](int val) { setNumRings(val); });
    setUseTextureCoords(getBoolForConfigKey("_useTexCoords_", configString));
    setUseTangents(getBoolForConfigKey("_useTangents_", configString));
    setCapEnd(getBoolForConfigKey("_capEnd_", configString));
  }
  return segmentSet && halflenSet && ringSet;

}  // ConePrimitiveAttributes::parseStringIntoConfigDetail(

CylinderPrimitiveAttributes::CylinderPrimitiveAttributes(
    bool isWireframe,
    int primObjType,
    const std::string& primObjClassName)
    : AbstractPrimitiveAttributes(isWireframe,
                                  primObjType,
                                  primObjClassName,
                                  "CylinderPrimitiveAttributes") {
  set("rings", 1);
  set("halfLength", 1.0);

  if (!isWireframe) {  // solid
    set("segments", 12);
    set("capEnds", true);
  } else {  // wireframe
    set("segments", 32);
  }
  buildHandle();  // build handle based on config
}  // CylinderPrimitiveAttributes

bool CylinderPrimitiveAttributes::parseStringIntoConfigDetail(
    const std::string& configString) {
  bool ringSet = setIntFromConfigKey("_rings_", configString,
                                     [this](int val) { setNumRings(val); });
  bool segmentSet = setIntFromConfigKey(
      "_segments_", configString, [this](int val) { setNumSegments(val); });
  auto halflenSet = setDoubleFromConfigKey(
      "_halfLen_", configString, [this](double val) { setHalfLength(val); });

  if (!getIsWireframe()) {
    setUseTextureCoords(getBoolForConfigKey("_useTexCoords_", configString));
    setUseTangents(getBoolForConfigKey("_useTangents_", configString));
    setCapEnds(getBoolForConfigKey("_capEnds_", configString));
  }
  return ringSet && halflenSet && segmentSet;
}  // CylinderPrimitiveAttributes::parseStringIntoConfigDetail(

UVSpherePrimitiveAttributes::UVSpherePrimitiveAttributes(
    bool isWireframe,
    int primObjType,
    const std::string& primObjClassName)
    : AbstractPrimitiveAttributes(isWireframe,
                                  primObjType,
                                  primObjClassName,
                                  "UVSpherePrimitiveAttributes") {
  if (!isWireframe) {  // solid
    set("rings", 8);
    set("segments", 16);
  } else {  // wireframe
    set("rings", 16);
    set("segments", 32);
  }
  buildHandle();  // build handle based on config
}  // UVSpherePrimitiveAttributes

bool UVSpherePrimitiveAttributes::parseStringIntoConfigDetail(
    const std::string& configString) {
  std::ostringstream oHndlStrm;

  bool ringSet = setIntFromConfigKey("_rings_", configString,
                                     [this](int val) { setNumRings(val); });
  bool segmentSet = setIntFromConfigKey(
      "_segments_", configString, [this](int val) { setNumSegments(val); });
  if (!getIsWireframe()) {
    setUseTextureCoords(getBoolForConfigKey("_useTexCoords_", configString));
    setUseTangents(getBoolForConfigKey("_useTangents_", configString));
  }
  return ringSet && segmentSet;
}  // UVSpherePrimitiveAttributes::parseStringIntoConfigDetail(

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
