// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_ATTRIBUTES_ATTRIBUTESENUMMAPS_H_
#define ESP_METADATA_ATTRIBUTES_ATTRIBUTESENUMMAPS_H_

#include "esp/core/Esp.h"
#include "esp/gfx/LightSetup.h"
#include "esp/sensor/FisheyeSensor.h"
#include "esp/sensor/Sensor.h"

namespace esp {
namespace physics {
enum class MotionType;
}
namespace metadata {
namespace attributes {

/**
 * @brief Supported Asset types
 */
enum class AssetType {
  Unknown,
  Mp3dMesh,
  InstanceMesh,
  Navmesh,
  Primitive,
  EndAssetType,
};

/**
 * @brief This enum class defines possible options for the type of joint that
 * connects the base of the Articulated Object to the world (the root 6 dofs)
 */

enum class ArticulatedObjectBaseType {
  /**
   * @brief Represents the user not specifying the type of base/root joint.
   * Resorts to any previously known/set value.
   */
  Unspecified = ID_UNDEFINED,
  /**
   * @brief The Articulated Object is joined to the world with a free joint and
   * is free to move around in the world.
   */
  Free,
  /**
   * @brief The Articulated Object is connected to the world with a fixed joint
   * at a specific location in the world and is unable to move within the world.
   */
  Fixed,
  /**
   * @brief End cap value - no Articulated Object base type enums should be
   * defined at or past this enum.
   */
  EndAOBaseType,
};

/**
 * @brief This enum class defines the source of the inertia values to use for
 * the Articulated Object.
 */
enum class ArticulatedObjectInertiaSource {
  /**
   * @brief Represents the user not specifying the source of the inertia values
   * to use. Resorts to any previously known/set value.
   */
  Unspecified = ID_UNDEFINED,
  /**
   * @brief Use inertia values computed from the collision shapes when the model
   * is loaded. This is usually more stable and is the default value.
   */
  Computed,
  /**
   * @brief Use the inertia values specified in the URDF file.
   */
  URDF,
  /**
   * @brief End cap value - no Articulated Object intertia source enums should
   * be defined at or past this enum.
   */
  EndAOInertiaSource,
};

/**
 * @brief This enum class defines how the links in the Articulated Object should
 * be ordered.
 */
enum class ArticulatedObjectLinkOrder {
  /**
   * @brief Represents the user not specifying which link ordering to use.
   * Resorts to any previously known/set value
   */
  Unspecified = ID_UNDEFINED,
  /**
   * @brief Use the link order specified in the source URDF file.
   */
  URDFOrder,
  /**
   * @brief Use the link order derived from a tree traversal of the Articulated
   * Object.
   */
  TreeTraversal,
  /**
   * @brief End cap value - no Articulated Object link order enums should be
   * defined at or past this enum.
   */
  EndAOLinkOrder,

};
/**
 * @brief This enum class defines the possible options for what will be rendered
 * for a particular Articulated Object.
 *
 */
enum class ArticulatedObjectRenderMode {
  /**
   * @brief Represents the user not specifying which rendering mode to use.
   * Resorts to any previously known/set value
   */
  Unspecified = ID_UNDEFINED,
  /**
   * @brief Render the Articulated Object using its skin if it has one,
   * otherwise render it using the urdf-defined link meshes/primitives.
   */
  Default,
  /**
   * @brief Render the Articulated Object using its skin.
   */
  Skin,
  /**
   * @brief Render the Articulated Object using urdf-defined meshes/primitives
   * to represent each link.
   */
  LinkVisuals,
  /**
   * @brief Do not render the Articulated Object.
   */
  None,
  /**
   * @brief Render the Articulated Object using both the skin and the
   * urdf-defined link meshes/primitives.
   */
  Both,
  /**
   * @brief End cap value - no Articulated Object render mode enums should be
   * defined at or past this enum.
   */
  EndAORenderMode,
};

/**
 * @brief This enum class defines the possible shader options for rendering
 * instances of objects or stages in Habitat-sim.
 */
enum class ObjectInstanceShaderType {
  /**
   * @brief Represents the user not specifying which shader type choice to use.
   * Resort to defaults for object type.
   */
  Unspecified = ID_UNDEFINED,
  /**
   * @brief Override any config-specified or default shader-type values to use
   * the material-specified shader.
   */
  Material,
  /**
   * @brief Refers to flat shading, pure color and no lighting. This is often
   * used for textured objects
   */
  Flat,
  /**
   * @brief Refers to phong shading with pure diffuse color.
   */
  Phong,
  /**
   * @brief Refers to using a shader built with physically-based rendering
   * models.
   */
  PBR,
  /**
   * @brief End cap value - no shader type enums should be defined at or past
   * this enum.
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
   * @brief End cap value - no instance translation origin type enums should be
   * defined at or past this enum.
   */
  EndTransOrigin,
};

/**
 * @brief Constant static map to provide mappings from string tags to
 * @ref AssetType values.  This will be used to map string values
 * used for mesh type to @ref AssetType.  Keys must be lowercase.
 */
const extern std::map<std::string, AssetType> AssetTypeNamesMap;

/**
 * @brief Get a string name representing the specified @ref
 * AssetType enum value.
 */
std::string getAssetTypeName(AssetType meshTypeEnum);

/**
 * @brief Constant map to provide mappings from string tags to @ref
 * ArticulatedObjectBaseType values. This will be used to map string values
 * used for AO base_type to @ref ArticulatedObjectBaseType. Keys must be
 * lowercase.
 */
const extern std::map<std::string, ArticulatedObjectBaseType> AOBaseTypeMap;

/**
 * @brief This method will convert a @ref ArticulatedObjectBaseType value to the
 * string key that maps to it in the AOBaseTypeMap
 */
std::string getAOBaseTypeName(ArticulatedObjectBaseType aoBaseType);

/**
 * @brief Constant map to provide mappings from string tags to @ref
 * ArticulatedObjectInertiaSource values. This will be used to map string
 * values used for AO inertia_source to @ref ArticulatedObjectInertiaSource. Keys
 * must be lowercase.
 */
const extern std::map<std::string, ArticulatedObjectInertiaSource>
    AOInertiaSourceMap;

/**
 * @brief This method will convert a @ref ArticulatedObjectInertiaSource value to the
 * string key that maps to it in the AOInertiaSourceMap
 */
std::string getAOInertiaSourceName(
    ArticulatedObjectInertiaSource aoInertiaSource);

/**
 * @brief Constant map to provide mappings from string tags to @ref
 * ArticulatedObjectLinkOrder values. This will be used to map string values
 * used for AO link_order to @ref ArticulatedObjectLinkOrder. Keys
 * must be lowercase.
 */
const extern std::map<std::string, ArticulatedObjectLinkOrder> AOLinkOrderMap;

/**
 * @brief This method will convert a @ref ArticulatedObjectLinkOrder value to the
 * string key that maps to it in the AOLinkOrderMap
 */
std::string getAOLinkOrderName(ArticulatedObjectLinkOrder aoLinkOrder);

/**
 * @brief Constant map to provide mappings from string tags to @ref
 * ArticulatedObjectRenderMode values. This will be used to map string values
 * used for AO render_mode to @ref ArticulatedObjectRenderMode. Keys must be
 * lowercase.
 */
const extern std::map<std::string, ArticulatedObjectRenderMode>
    AORenderModesMap;

/**
 * @brief This method will convert a @ref ArticulatedObjectRenderMode value to the
 * string key that maps to it in the AORenderModesMap
 */
std::string getAORenderModeName(ArticulatedObjectRenderMode aoRenderMode);

/**
 * @brief Constant map to provide mappings from string tags to @ref
 * ObjectInstanceShaderType values. This will be used to map string values used
 * for shader type to @ref ObjectInstanceShaderType. Keys must be lowercase.
 */
const extern std::map<std::string, ObjectInstanceShaderType> ShaderTypeNamesMap;

/**
 * @brief This method will convert a @ref ObjectInstanceShaderType value to the
 * string key that maps to it in the ShaderTypeNamesMap
 */
std::string getShaderTypeName(ObjectInstanceShaderType shaderTypeVal);

/**
 * @brief Constant map to provide mappings from string tags to @ref
 * SceneInstanceTranslationOrigin values. This will be used to map string values
 * used for translation origin to @ref SceneInstanceTranslationOrigin. Keys
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
 * esp::gfx::LightType values. This will be used to map string values used
 * for light type to @ref esp::gfx::LightType. Keys must be lowercase - will
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
 * esp::gfx::LightPositionModel values. This will be used to map string values
 * used to specify what translations are measured from for a lighting
 * instance placement. Keys must be lowercase.
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
 * esp::physics::MotionType values. This will be used to map string values used
 * for motion type to @ref esp::physics::MotionType. Keys must be lowercase.
 */
const extern std::map<std::string, esp::physics::MotionType> MotionTypeNamesMap;

/**
 * @brief This method will convert a @ref esp::physics::MotionType value to
 * the string key it maps to in the MotionTypeNamesMap
 */
std::string getMotionTypeName(esp::physics::MotionType motionTypeEnum);

/**
 * @brief Constant static map to provide mappings from string tags to @ref
 * esp::sensor::SensorType values. This will be used to map string values used
 * for sensor type to @ref esp::sensor::SensorType. Keys must be lowercase.
 */
const extern std::map<std::string, esp::sensor::SensorType> SensorTypeNamesMap;

/**
 * @brief This method will convert a @ref esp::sensor::SensorType value to
 * the string key it maps to in the SensorTypeNamesMap
 */
std::string getSensorTypeName(esp::sensor::SensorType sensorTypeEnum);

/**
 * @brief Constant static map to provide mappings from string tags to @ref
 * esp::sensor::SensorSubType values. This will be used to map string values
 * used for sensor sub type to @ref esp::sensor::SensorSubType. Keys must be
 * lowercase.
 */
const extern std::map<std::string, esp::sensor::SensorSubType>
    SensorSubTypeNamesMap;

/**
 * @brief This method will convert a @ref esp::sensor::SensorSubType value to
 * the string key it maps to in the SensorSubTypeNamesMap
 */
std::string getSensorSubTypeName(esp::sensor::SensorSubType sensorSubTypeEnum);
/**
 * @brief Constant static map to provide mappings from string tags to @ref
 * esp::sensor::SemanticSensorTarget values. This will be used to map string
 * values used for sensor sub type to @ref esp::sensor::SemanticSensorTarget.
 * Keys must be lowercase.
 */
const extern std::map<std::string, esp::sensor::SemanticSensorTarget>
    SemanticSensorTargetMap;

/**
 * @brief This method will convert a @ref esp::sensor::SemanticSensorTarget value to
 * the string key it maps to in the SensorSubTypeNamesMap
 */
std::string getSemanitcSensorTargetName(
    esp::sensor::SemanticSensorTarget semanticSensorTargetEnum);

/**
 * @brief Constant static map to provide mappings from string tags to @ref
 * esp::sensor::FisheyeSensorModelType values. This will be used to map string
 * values used for to @ref esp::sensor::FisheyeSensorModelType. Keys must be
 * lowercase.
 */
const extern std::map<std::string, esp::sensor::FisheyeSensorModelType>
    FisheyeSensorModelTypeMap;

/**
 * @brief This method will convert a @ref esp::sensor::FisheyeSensorModelType value to
 * the string key it maps to in the FisheyeSensorModelTypeMap
 */
std::string getFisheyeSensorModelTypeName(
    esp::sensor::FisheyeSensorModelType fisheyeSensorModelTypeEnum);

}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_ATTRIBUTESENUMMAPS_H_
