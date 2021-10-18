// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_ATTRIBUTES_ATTRIBUTESENUMMAPS_H_
#define ESP_METADATA_ATTRIBUTES_ATTRIBUTESENUMMAPS_H_

#include "esp/core/Esp.h"
#include "esp/gfx/LightSetup.h"
namespace esp {
namespace asset {
enum class AssetType;
}
namespace physics {
enum class MotionType;
}
namespace metadata {
namespace attributes {

/**
 * @brief This enum class defines the possible shader options for rendering
 * instances of objects or stages in Habitat-sim.
 */
enum class ObjectInstanceShaderType {
  /**
   * Represents the user not specifying which shader type choice to use. Resort
   * to defaults for object type.
   */
  Unspecified = ID_UNDEFINED,
  /**
   * Override any config-specified or default shader-type values to use the
   * material-specified shader.
   */
  Material,
  /**
   * Refers to flat shading, pure color and no lighting.  This is often used for
   * textured objects
   */
  Flat,
  /**
   * Refers to phong shading with pure diffuse color.
   */
  Phong,
  /**
   * Refers to using a shader built with physically-based rendering models.
   */
  PBR,
  /**
   * End cap value - no shader type enums should be defined past this enum.
   */
  EndShaderType,
};

/**
 * @brief This enum class describes whether an object instance position is
 * relative to its COM or the asset's local origin.  Depending on this value, we
 * may take certain actions when instantiating a scene described by a scene
 * instance. For example, scene instances exported from Blender will have no
 * conception of an object's configured COM, and so will require adjustment to
 * translations to account for COM location when the object is placed*/
enum class SceneInstanceTranslationOrigin {
  /**
   * @brief Default value - in the case of object instances, this means use the
   * specified scene instance default; in the case of a scene instance, this
   * means do not correct for COM.
   */
  Unknown = ID_UNDEFINED,
  /**
   * @brief Indicates scene instance objects were placed without knowledge of
   * their COM location, and so need to be corrected when placed in scene in
   * Habitat. For example, they were exported from an external editor like
   * Blender.
   */
  AssetLocal,
  /**
   * @brief Indicates scene instance objects' location were recorded at their
   * COM location, and so do not need correction.  For example they were
   * exported from Habitat-sim.
   */
  COM,
  /**
   * End cap value - no instance translation origin type enums should be defined
   * past this enum.
   */
  EndTransOrigin,
};

/**
 * @brief Constant map to provide mappings from string tags to @ref
 * ObjectInstanceShaderType values.  This will be used to map values set
 * in json for translation origin to @ref ObjectInstanceShaderType.  Keys
 * must be lowercase.
 */
const extern std::map<std::string, ObjectInstanceShaderType> ShaderTypeNamesMap;

/**
 * @brief This method will convert a @ref ObjectInstanceShaderType value to the
 * string key that maps to it in the ShaderTypeNamesMap
 */
std::string getShaderTypeName(ObjectInstanceShaderType shaderTypeVal);

/**
 * @brief Constant map to provide mappings from string tags to @ref
 * SceneInstanceTranslationOrigin values.  This will be used to map values set
 * in json for translation origin to @ref SceneInstanceTranslationOrigin.  Keys
 * must be lowercase.
 */
const extern std::map<std::string, SceneInstanceTranslationOrigin>
    InstanceTranslationOriginMap;
/**
 * @brief This method will convert a @ref SceneInstanceTranslationOrigin value
 * to the string key that maps to it in the InstanceTranslationOriginMap
 */
std::string getTranslationOriginName(
    SceneInstanceTranslationOrigin translationOrigin);

/**
 * @brief Constant static map to provide mappings from string tags to @ref
 * esp::gfx::LightType values.  This will be used to map values set in json
 * for light type to @ref esp::gfx::LightType.  Keys must be lowercase - will
 * support any case values in JSON.
 */
const extern std::map<std::string, esp::gfx::LightType> LightTypeNamesMap;
/**
 * @brief This method will convert a @ref esp::gfx::LightType value to the
 * string key it maps to in the LightTypeNamesMap
 */
std::string getLightTypeName(esp::gfx::LightType lightTypeEnum);

/**
 * @brief Constant static map to provide mappings from string tags to @ref
 * esp::gfx::LightPositionModel values.  This will be used to map values set
 * in json to specify what translations are measured from for a lighting
 * instance.
 */
const extern std::map<std::string, esp::gfx::LightPositionModel>
    LightPositionNamesMap;
/**
 * @brief This method will convert a @ref esp::gfx::LightPositionModel value to
 * the string key it maps to in the LightPositionNamesMap
 */
std::string getLightPositionModelName(
    esp::gfx::LightPositionModel lightPositionEnum);

/**
 * @brief Constant static map to provide mappings from string tags to @ref
 * esp::physics::MotionType values.  This will be used to map values set in
 * json for mesh type to @ref esp::physics::MotionType.  Keys must be
 * lowercase.
 */
const extern std::map<std::string, esp::physics::MotionType> MotionTypeNamesMap;

/**
 * @brief This method will convert a @ref esp::gfx::LightPositionModel value to
 * the string key it maps to in the LightPositionNamesMap
 */
std::string getMotionTypeName(esp::physics::MotionType motionTypeEnum);

}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_ATTRIBUTESENUMMAPS_H_
