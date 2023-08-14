// Copyright (c) Meta Platforms, Inc. and its affiliates.
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
 * @brief This enum class defines possible options for the type of joint that
 * connects the base of the Articulated Object to the world (the root 6 dofs)
 */

enum class ArticulatedObjectBaseType {
  /**
   * Represents the user not specifying the type of base/root joint.
   * Resorts to any previously known/set value.
   */
  Unspecified = ID_UNDEFINED,
  /**
   * The Articulated Object is joined to the world with a free joint and
   * is free to move around in the world.
   */
  Free,
  /**
   * The Articulated Object is connected to the world with a fixed joint
   * at a specific location in the world and is unable to move within the world.
   */
  Fixed,
  /**
   * End cap value - no Articulated Object base type enums should be defined
   * at or past this enum.
   */
  EndAOBaseType,
};

/**
 * @brief This enum class defines the source of the interia values to use for
 * the Articulated Object.
 */
enum class ArticulatedObjectInertiaSource {
  /**
   * Represents the user not specifying the source of the inertia values
   * to use. Resorts to any previously known/set value.
   */
  Unspecified = ID_UNDEFINED,
  /**
   * Use inertia values computed from the collision shapes when the model
   * is loaded. This is usually more stable and is the default value.
   */
  Computed,
  /**
   * Use the interia values specified in the URDF file.
   */
  URDF,
  /**
   * End cap value - no Articulated Object intertia source enums should be
   * defined at or past this enum.
   */
  EndAOInertiaSource,
};

/**
 * @brief This enum class defines how the links in the Articulated Object should
 * be ordered.
 */
enum class ArticulatedObjectLinkOrder {
  /**
   * Represents the user not specifying which link ordering to use. Resorts
   * to any previously known/set value
   */
  Unspecified = ID_UNDEFINED,
  /**
   * Use the link order specified in the source URDF file.
   */
  URDFOrder,
  /**
   * Use the link order derived from a tree traversal of the Articulated Object.
   */
  TreeTraversal,
  /**
   * End cap value - no Articulated Object link order enums should be
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
   * Represents the user not specifying which rendering mode to use. Resorts
   * to any previously known/set value
   */
  Unspecified = ID_UNDEFINED,
  /**
   * Render the Articulated Object using its skin if it has one, otherwise
   * render it using the urdf-defined link meshes/primitives.
   */
  Default,
  /**
   * Render the Articulated Object using its skin.
   */
  Skin,
  /**
   * Render the Articulated Object using urdf-defined meshes/primitives to
   * respresent each link.
   */
  LinkVisuals,
  /**
   *
   */
  None,
  /**
   * Render the Articulated Object using both the skin and the urdf-defined link
   * meshes/primitives.
   */
  Both,
  /**
   * End cap value - no Articulated Object render mode enums should be defined
   * at or past this enum.
   */
  EndAORenderMode,
};

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
   * Refers to flat shading, pure color and no lighting. This is often used for
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
   * End cap value - no shader type enums should be defined at or past this
   * enum.
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
   * at or past this enum.
   */
  EndTransOrigin,
};

/**
 * @brief Constant map to provide mappings from string tags to @ref
 * ArticulatedObjectBaseType values. This will be used to map values set
 * in json for AO base_type to @ref ArticulatedObjectBaseType. Keys
 * must be lowercase.
 */
const extern std::map<std::string, ArticulatedObjectBaseType> AOBaseTypeMap;

/**
 * @brief This method will convert a @ref ArticulatedObjectBaseType value to the
 * string key that maps to it in the AOBaseTypeMap
 */
std::string getAOBaseTypeName(ArticulatedObjectBaseType aoBaseType);

/**
 * @brief Constant map to provide mappings from string tags to @ref
 * ArticulatedObjectInertiaSource values. This will be used to map values set
 * in json for AO inertia_source to @ref ArticulatedObjectInertiaSource. Keys
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
 * ArticulatedObjectLinkOrder values. This will be used to map values set
 * in json for AO link_order to @ref ArticulatedObjectLinkOrder. Keys
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
 * ArticulatedObjectRenderMode values. This will be used to map values set
 * in json for AO render_mode to @ref ArticulatedObjectRenderMode. Keys
 * must be lowercase.
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
 * ObjectInstanceShaderType values. This will be used to map values set
 * in json for shader type to @ref ObjectInstanceShaderType. Keys
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
 * SceneInstanceTranslationOrigin values. This will be used to map values set
 * in json for translation origin to @ref SceneInstanceTranslationOrigin. Keys
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
 * esp::gfx::LightType values. This will be used to map values set in json
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
 * esp::gfx::LightPositionModel values. This will be used to map values set
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
 * esp::physics::MotionType values. This will be used to map values set in
 * json for mesh type to @ref esp::physics::MotionType. Keys must be
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
