// Copyright (c) Facebook, Inc. and its affiliates.
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
  setCylinderRings(1);
  if (!isWireframe) {  // solid
    setHemisphereRings(4);
    setNumSegments(12);
    setHalfLength(0.75);
  } else {  // wireframe
    setHemisphereRings(8);
    setNumSegments(16);
    setHalfLength(1.0);
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
  setHalfLength(1.25);

  if (!isWireframe) {  // solid
    setNumRings(1);
    setNumSegments(12);
    setCapEnd(true);
  } else {  // wireframe
    setNumSegments(32);
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
  setNumRings(1);
  setHalfLength(1.0);

  if (!isWireframe) {  // solid
    setNumSegments(12);
    setCapEnds(true);
  } else {  // wireframe
    setNumSegments(32);
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
    setNumRings(8);
    setNumSegments(16);
  } else {  // wireframe
    setNumRings(16);
    setNumSegments(32);
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
