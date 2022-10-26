// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "PrimitiveAssetAttributes.h"

namespace esp {
namespace metadata {
namespace attributes {

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
