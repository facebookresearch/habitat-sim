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
}  // PhysicsCapsulePrimAttributes

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
}  // PhysicsConePrimAttributes

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
}  // PhysicsCylinderPrimAttributes

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
}  // PhysicsUVSpherePrimAttributes

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
