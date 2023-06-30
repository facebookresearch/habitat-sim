// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_ATTRIBUTES_PRIMITIVEASSETATTRIBUTES_H_
#define ESP_METADATA_ATTRIBUTES_PRIMITIVEASSETATTRIBUTES_H_

#include "AttributesBase.h"

namespace esp {
namespace metadata {
namespace attributes {

///////////////////////////////////
// primitive object attributes

//! attributes describing primitve render/collision objects - abstract class
//! without pure virtual methods
class AbstractPrimitiveAttributes : public AbstractAttributes {
 public:
  AbstractPrimitiveAttributes(bool isWireframe,
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
    if (!isWireframe) {  // solid
      // do not call setters since they call buildHandle, which does not
      // exist - is abstract in base class
      set("textureCoordinates", false);
      set("tangents", false);
    }

  }  // ctor

  // necessary since abstract
  ~AbstractPrimitiveAttributes() override = default;

  // handle is set internally based on attributes configuration
  // setting externally is prohibited
  void setHandle(const std::string&) override {}

  bool getIsWireframe() const { return get<bool>("isWireframe"); }

  // only solid prims can use texture coords
  void setUseTextureCoords(bool useTextureCoords) {
    if (!getIsWireframe()) {  // check if solid
      set("textureCoordinates", useTextureCoords);
      buildHandle();  // build handle based on config
    }
  }
  bool getUseTextureCoords() const { return get<bool>("textureCoordinates"); }

  // only solid prims have option to use tangents
  void setUseTangents(bool tangents) {
    if (!getIsWireframe()) {  // check if solid
      set("tangents", tangents);
      buildHandle();  // build handle based on config
    }
  }
  bool getUseTangents() const { return get<bool>("tangents"); }

  // only circular prims set number of rings - NOTE : capsule sets rings
  // separately for hemispheres and cylinder
  // set virtual so cannot be deleted in capsule attributes
  void setNumRings(int rings) {
    set("rings", rings);
    buildHandle();  // build handle based on config
  }
  int getNumRings() const { return get<int>("rings"); }

  void setNumSegments(int segments) {
    set("segments", segments);
    buildHandle();  // build handle based on config
  }
  int getNumSegments() const { return get<int>("segments"); }
  // capsule, cone and cylinder use halfLength
  void setHalfLength(double halfLength) {
    set("halfLength", halfLength);
    buildHandle();
  }
  double getHalfLength() const { return get<double>("halfLength"); }

  std::string getPrimObjClassName() const {
    return get<std::string>("primObjClassName");
  }

  /**
   * @brief The integer representation of the @ref esp::metadata::PrimObjTypes
   * this primitive represents,
   */
  int getPrimObjType() const { return get<int>("primObjType"); }
  /**
   * @brief This will determine if the stated template has the required
   * quantities needed to instantiate a primitive properly of desired type.
   * AbstractPrimitiveAttributes is never valid for any primitive - should be
   * overridden in prim-specific class.
   * @return whether or not the template holds valid data for desired primitive
   * type.
   */
  virtual bool isValidTemplate() { return false; }

  /**
   * @brief Handle for primitive attribute-based templates should reflect
   * the parameters used to construct the primitive, and so should only be set
   * internally or when relevant values are set manually.
   */
  void buildHandle() {
    std::ostringstream oHndlStrm;
    oHndlStrm << getPrimObjClassName() << buildHandleDetail();
    set("handle", oHndlStrm.str());
  }

  /**
   * @brief This will parse the passed candidate object template handle, treated
   * as a configuration string, and populate the appropriate values.
   * @param configString The configuration string to parse.
   * @return Whether the parsing has succeeded or not (might fail with
   * inappropriate formatting for object type.)
   */
  bool parseStringIntoConfig(const std::string& configString) {
    bool success = parseStringIntoConfigDetail(configString);
    if (success) {
      // if parsed successfully, make sure the object's handle contains the
      // original config string
      return (this->getHandle().find(configString) != std::string::npos);
    }
    return success;
  }

  /**
   * @brief PrimitiveAssetAttributes handles are already simplified, and embed
   * no path info.
   */
  std::string getSimplifiedHandle() const override { return getHandle(); }

 protected:
  /**
   * @brief Retrieve a comma-separated string holding the header values for the
   * info returned for this managed object, type-specific.
   */
  std::string getObjectInfoHeaderInternal() const override {
    // Handle already encodes all relevant info
    return ",";
  }

  /**
   * @brief Retrieve a comma-separated informational string about the contents
   * of this managed object.
   */
  std::string getObjectInfoInternal() const override {
    // Handle already encodes all relevant info
    return ", ";
  }

  /**
   * @brief Verifies that val is larger than, and a multiple of, divisor
   * div
   * @param val the value to check
   * @param div the divsior (value to verify is greater than and a multiple of)
   * - will be either 2 or 4 for primitives value checking
   * @return whether check passes
   */
  bool isValueMultipleOfDivisor(int val, int div) {
    return (val >= div) && (val % div == 0);
  }

  // helper for handle construction
  std::string getBoolDispStr(bool val) const {
    return (val ? "true" : "false");
  }

  // helper for parseStringIntoConfig process
  std::string getValueForConfigKey(const std::string& key,
                                   const std::string& configStr) {
    std::size_t keyLoc = configStr.find(key);
    if (keyLoc == std::string::npos) {
      ESP_WARNING() << "Key" << key << "not found in configStr" << configStr
                    << ", so retrieving empty string.";
      return "";
    }
    std::size_t keyLen = key.length(), keyEnd = keyLoc + keyLen;
    return configStr.substr(keyEnd, configStr.find('_', keyEnd) - keyEnd);
  }
  bool getBoolForConfigKey(const std::string& key,
                           const std::string& configStr) {
    std::string res = getValueForConfigKey(key, configStr);
    return (res.find("true") != std::string::npos);
  }

  bool setIntFromConfigKey(const std::string& key,
                           const std::string& configStr,
                           const std::function<void(int)>& setter) {
    const std::string conv = getValueForConfigKey(key, configStr);
    try {
      setter(stoi(conv));
      return true;
    } catch (...) {
      ESP_WARNING() << "Failed due to -" << conv << "- value for key -" << key
                    << "- in format string -" << configStr
                    << "- not being recognized as an int.";
      return false;
    }
  }

  bool setDoubleFromConfigKey(const std::string& key,
                              const std::string& configStr,
                              const std::function<void(double)>& setter) {
    const std::string conv = getValueForConfigKey(key, configStr);
    try {
      setter(stod(conv));
      return true;
    } catch (...) {
      ESP_WARNING() << "Failed due to -" << conv << "- value for key -" << key
                    << "- in format string -" << configStr
                    << "- not being recognized as a double.";
      return false;
    }
  }

  virtual std::string buildHandleDetail() = 0;

  virtual bool parseStringIntoConfigDetail(const std::string& configString) = 0;

 private:
  // Should never change, only set by ctor
  void setPrimObjClassName(const std::string& primObjClassName) {
    set("primObjClassName", primObjClassName);
  }

  // Should never change, only set by ctor
  void setPrimObjType(int primObjType) { set("primObjType", primObjType); }

  // not used to construct prim mesh, so setting this does not require
  // modification to handle.  Should never change, only set by ctor
  void setIsWireframe(bool isWireframe) { set("isWireframe", isWireframe); }

 public:
  ESP_SMART_POINTERS(AbstractPrimitiveAttributes)
};  // class AbstractPrimitiveAttributes

//! attributes describing primitive capsule objects
class CapsulePrimitiveAttributes : public AbstractPrimitiveAttributes {
 public:
  CapsulePrimitiveAttributes(bool isWireframe,
                             int primObjType,
                             const std::string& primObjClassName);

  void setHemisphereRings(int hemisphereRings) {
    set("hemisphereRings", hemisphereRings);
    buildHandle();  // build handle based on config
  }
  int getHemisphereRings() const { return get<int>("hemisphereRings"); }

  void setCylinderRings(int cylinderRings) {
    set("cylinderRings", cylinderRings);
    buildHandle();  // build handle based on config
  }
  int getCylinderRings() const { return get<int>("cylinderRings"); }

  /**
   * @brief This will determine if the stated template has the required
   * quantities needed to instantiate a primitive properly of desired type
   * @return whether or not the template holds valid data for desired
   * primitive type
   */
  bool isValidTemplate() override {
    bool wfCheck =
        ((getIsWireframe() && isValueMultipleOfDivisor(getNumSegments(), 4)) ||
         (!getIsWireframe() && getNumSegments() > 2));

    return (getCylinderRings() > 0 && getHemisphereRings() > 0 && wfCheck &&
            getHalfLength() > 0);
  }

 protected:
  std::string buildHandleDetail() override {
    std::ostringstream oHndlStrm;
    oHndlStrm << "_hemiRings_" << getHemisphereRings() << "_cylRings_"
              << getCylinderRings() << "_segments_" << getNumSegments()
              << "_halfLen_" << getHalfLength();
    if (!getIsWireframe()) {
      oHndlStrm << "_useTexCoords_" << getBoolDispStr(getUseTextureCoords())
                << "_useTangents_" << getBoolDispStr(getUseTangents());
    }
    return oHndlStrm.str();
  }  // buildHandleDetail

  bool parseStringIntoConfigDetail(const std::string& configString) override;

 public:
  ESP_SMART_POINTERS(CapsulePrimitiveAttributes)
};  // class CapsulePrimitiveAttributes

class ConePrimitiveAttributes : public AbstractPrimitiveAttributes {
 public:
  ConePrimitiveAttributes(bool isWireframe,
                          int primObjType,
                          const std::string& primObjClassName);

  // only solid cones can have end capped
  void setCapEnd(bool capEnd) {
    set("capEnd", capEnd);
    buildHandle();  // build handle based on config
  }
  bool getCapEnd() const { return get<bool>("capEnd"); }

  /**
   * @brief This will determine if the stated template has the required
   * quantities needed to instantiate a primitive properly of desired type
   * @return whether or not the template holds valid data for desired
   * primitive type
   */
  bool isValidTemplate() override {
    bool wfCheck =
        ((getIsWireframe() && isValueMultipleOfDivisor(getNumSegments(), 4)) ||
         (!getIsWireframe() && getNumSegments() > 2 && getNumRings() > 0));

    return (getHalfLength() > 0 && wfCheck);
  }

 protected:
  std::string buildHandleDetail() override {
    std::ostringstream oHndlStrm;
    oHndlStrm << "_segments_" << getNumSegments() << "_halfLen_"
              << getHalfLength();
    if (!getIsWireframe()) {
      oHndlStrm << "_rings_" << getNumRings() << "_useTexCoords_"
                << getBoolDispStr(getUseTextureCoords()) << "_useTangents_"
                << getBoolDispStr(getUseTangents()) << "_capEnd_"
                << getBoolDispStr(getCapEnd());
    }
    return oHndlStrm.str();
  }  // buildHandleDetail

  bool parseStringIntoConfigDetail(const std::string& configString) override;

 public:
  ESP_SMART_POINTERS(ConePrimitiveAttributes)
};  // class ConePrimitiveAttributes

class CubePrimitiveAttributes : public AbstractPrimitiveAttributes {
 public:
  CubePrimitiveAttributes(bool isWireframe,
                          int primObjType,
                          const std::string& primObjClassName)
      : AbstractPrimitiveAttributes(isWireframe,
                                    primObjType,
                                    primObjClassName,
                                    "CubePrimitiveAttributes") {
    buildHandle();  // build handle based on config
  }

  /**
   * @brief This will determine if the stated template has the required
   * quantities needed to instantiate a primitive properly of desired type.
   * Cube primitives require no values and so this attributes is always valid.
   * @return whether or not the template holds valid data for desired
   * primitive type
   */
  bool isValidTemplate() override { return true; }

 protected:
  std::string buildHandleDetail() override { return ""; }

  bool parseStringIntoConfigDetail(
      CORRADE_UNUSED const std::string& configString) override {
    return true;
  }

 public:
  ESP_SMART_POINTERS(CubePrimitiveAttributes)
};  // class CubePrimitiveAttributes

class CylinderPrimitiveAttributes : public AbstractPrimitiveAttributes {
 public:
  CylinderPrimitiveAttributes(bool isWireframe,
                              int primObjType,
                              const std::string& primObjClassName);

  // only solid culinders can have ends capped
  void setCapEnds(bool capEnds) {
    set("capEnds", capEnds);
    buildHandle();  // build handle based on config
  }
  bool getCapEnds() const { return get<bool>("capEnds"); }

  /**
   * @brief This will determine if the stated template has the required
   * quantities needed to instantiate a primitive properly of desired type
   * @return whether or not the template holds valid data for desired
   * primitive type
   */
  bool isValidTemplate() override {
    bool wfCheck =
        ((getIsWireframe() && isValueMultipleOfDivisor(getNumSegments(), 4)) ||
         (!getIsWireframe() && getNumSegments() > 2));
    return getNumRings() > 0 && getHalfLength() > 0 && wfCheck;
  }

 protected:
  std::string buildHandleDetail() override {
    std::ostringstream oHndlStrm;
    oHndlStrm << "_rings_" << getNumRings() << "_segments_" << getNumSegments()
              << "_halfLen_" << getHalfLength();
    if (!getIsWireframe()) {
      oHndlStrm << "_useTexCoords_" << getBoolDispStr(getUseTextureCoords())
                << "_useTangents_" << getBoolDispStr(getUseTangents())
                << "_capEnds_" << getBoolDispStr(getCapEnds());
    }
    return oHndlStrm.str();
  }  // buildHandleDetail

  bool parseStringIntoConfigDetail(const std::string& configString) override;

 public:
  ESP_SMART_POINTERS(CylinderPrimitiveAttributes)
};  // class CylinderPrimitiveAttributes

class IcospherePrimitiveAttributes : public AbstractPrimitiveAttributes {
 public:
  // note there is no magnum primitive implementation of a wireframe icosphere
  IcospherePrimitiveAttributes(bool isWireframe,
                               int primObjType,
                               const std::string& primObjClassName)
      : AbstractPrimitiveAttributes(isWireframe,
                                    primObjType,
                                    primObjClassName,
                                    "IcospherePrimitiveAttributes") {
    // setting manually because wireframe icosphere does not currently support
    // subdiv > 1 and setSubdivisions checks for wireframe
    set("subdivisions", 1);
    buildHandle();  // build handle based on config
  }
  // only solid icospheres will support subdivision - wireframes default to 1
  void setSubdivisions(int subdivisions) {
    if (!getIsWireframe()) {
      set("subdivisions", subdivisions);
      buildHandle();  // build handle based on config
    }
  }
  int getSubdivisions() const { return get<int>("subdivisions"); }

  /**
   * @brief This will determine if the stated template has the required
   * quantities needed to instantiate a primitive properly of desired type
   * @return whether or not the template holds valid data for desired
   * primitive type
   */
  bool isValidTemplate() override {
    return (getIsWireframe() || (!getIsWireframe() && getSubdivisions() >= 0));
  }

 protected:
  std::string buildHandleDetail() override {
    std::ostringstream oHndlStrm;
    // wireframe subdivision currently does not change
    // but think about the possibilities.
    oHndlStrm << "_subdivs_" << getSubdivisions();
    return oHndlStrm.str();
  }  // buildHandleDetail

  bool parseStringIntoConfigDetail(const std::string& configString) override {
    bool subDivsSet = setIntFromConfigKey(
        "_subdivs_", configString, [this](int val) { setSubdivisions(val); });

    return subDivsSet;
  }

 public:
  ESP_SMART_POINTERS(IcospherePrimitiveAttributes)
};  // class IcospherePrimitiveAttributes

class UVSpherePrimitiveAttributes : public AbstractPrimitiveAttributes {
 public:
  UVSpherePrimitiveAttributes(bool isWireframe,
                              int primObjType,
                              const std::string& primObjClassName);

  /**
   * @brief This will determine if the stated template has the required
   * quantities needed to instantiate a primitive properly of desired type
   * @return whether or not the template holds valid data for desired
   * primitive type
   */
  bool isValidTemplate() override {
    return ((getIsWireframe() &&
             isValueMultipleOfDivisor(getNumSegments(), 4) &&
             isValueMultipleOfDivisor(getNumRings(), 2)) ||
            (!getIsWireframe() && getNumRings() > 1 && getNumSegments() > 2));
  }

 protected:
  std::string buildHandleDetail() override {
    std::ostringstream oHndlStrm;
    oHndlStrm << "_rings_" << getNumRings() << "_segments_" << getNumSegments();
    if (!getIsWireframe()) {
      oHndlStrm << "_useTexCoords_" << getBoolDispStr(getUseTextureCoords())
                << "_useTangents_" << getBoolDispStr(getUseTangents());
    }
    return oHndlStrm.str();
  }  // buildHandleDetail

  bool parseStringIntoConfigDetail(const std::string& configString) override;

 public:
  ESP_SMART_POINTERS(UVSpherePrimitiveAttributes)
};  // class UVSpherePrimitiveAttributes

///////////////////////////////////////
// end primitive object attributes

}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_PRIMITIVEASSETATTRIBUTES_H_
