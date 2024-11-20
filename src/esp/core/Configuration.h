// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_CORE_CONFIGURATION_H_
#define ESP_CORE_CONFIGURATION_H_

#include <Corrade/Utility/Configuration.h>
#include <Corrade/Utility/Debug.h>
#include <Corrade/Utility/FormatStl.h>
#include <Magnum/Magnum.h>
#include <string>
#include <unordered_map>

#include "esp/core/Check.h"
#include "esp/core/Esp.h"
#include "esp/io/Json.h"

namespace Cr = Corrade;
namespace Mn = Magnum;

namespace esp {
namespace core {

namespace config {

constexpr int CONFIG_VAL_SIZE = 8;  // 2 * 4(DWORDs)

/**
 * @brief This enum lists every type of value that can be currently stored
 * directly in an @ref esp::core::config::Configuration.  All supported types
 * should have entries in this enum class. All pointer-backed types (i.e. data
 * larger than ConfigValue::_data array sizee) should have their enums placed
 * after @p _storedAsAPointer tag. All non-trivial types should have their enums
 * placed below @p _nonTrivialTypes tag. Any small, trivially copyable types
 * should be placed before @p _storedAsAPointer tag.
 */
enum class ConfigValType {
  /**
   * @brief Unknown type
   */
  Unknown = ID_UNDEFINED,
  /**
   * @brief boolean type
   */
  Boolean,
  /**
   * @brief integer type
   */
  Integer,
  /**
   * @brief Magnum::Rad angle type
   */
  MagnumRad,
  /**
   * @brief Magnum::Deg angle type
   */
  MagnumDeg,
  /**
   * @brief double type
   */
  Double,

  /**
   * @brief Magnum::Vector2 type
   */
  MagnumVec2,

  /**
   * @brief Magnum::Vector2i type
   */
  MagnumVec2i,

  // Types stored as a pointer.  All non-trivial types must also be placed after
  // this marker.
  _storedAsAPointer,
  /**
   * @brief Magnum::Vector3 type. All types of size greater than _data size must
   * be placed after this marker, either before or after String, depending on if
   * they are trivially copyable or not.
   */
  MagnumVec3 = _storedAsAPointer,
  /**
   * @brief Magnum::Vector4 type
   */
  MagnumVec4,
  /**
   * @brief Magnum::Quaternion type
   */
  MagnumQuat,
  /**
   * @brief Magnum::Matrix3 (3x3) type
   */
  MagnumMat3,
  /**
   * @brief Magnum::Matrix4 (4x4) type
   */
  MagnumMat4,

  // These types are not trivially copyable. All non-trivial types are by
  // default stored as pointers in ConfigValue::_data array.
  _nonTrivialTypes,
  /**
   * @brief All enum values of nontrivial types must be added after @p String .
   */
  String = _nonTrivialTypes,
};  // enum class ConfigValType

/**
 * @brief This enum lists the bit flags describing the various states @ref ConfigValue
 * can be in or characteristics they may have. These mask the higher order DWORD
 * of _typeAndFlags, and so begin on bit 32 of 0-63.
 */
enum ConfigValStatus : uint64_t {

  /**
   * @brief Whether or not the @ref ConfigValue was set with a default/initializing
   * value. This flag is used, for example, to govern whether a value should be
   * written to file or not - if the value was set only with a program-governed
   * default value, it should not be written to file.
   */
  isDefault = 1ULL << 32,

  /**
   * @brief Specifies that this @ref ConfigValue is hidden and used as an internally
   * tracked/managed variable, not part of the user-accessible metadata itself.
   * These variables should never be written to file or displayed except for
   * debugging purposes.
   */
  isHidden = 1ULL << 33,

  /**
   * @brief Specifies that this @ref ConfigValue is translated from a string value
   * to some other value/representation, such as an enum, intended to provide
   * constrained values and string representations. These types of values are
   * stored as strings and translated (i.e. cast) to the appropriate Enums on
   * access by the extending class for the @ref Configuration. They are saved
   * to file as a string that is mapped from the enum value.
   *
   * NOTE : Any ConfigValues specified as @p isTranslated need
   * to have their Json read handled by the consuming class, as they may not be
   * properly read from or written to file otherwise.
   */
  isTranslated = 1ULL << 34,

  /**
   * @brief This @ref ConfigValue requires manual fuzzy comparison (i.e. floating
   * point scalar type) using the Magnum::Math::equal method. Magnum types
   * already perform fuzzy comparison.
   */
  reqsFuzzyComparison = 1ULL << 35,
};  // enum class ConfigValStatus

/**
 * @brief Retrieve a string description of the passed @ref ConfigValType enum
 * value.
 */
std::string getNameForStoredType(const ConfigValType& value);

/**
 * @brief Quick check to see if type is stored as a pointer in the data or not
 * (i.e. the type is trivially copyable or not)
 */
constexpr bool isConfigValTypePointerBased(ConfigValType type) {
  return static_cast<int>(type) >=
         static_cast<int>(ConfigValType::_storedAsAPointer);
}
/**
 * @brief Quick check to see if type is trivially copyable or not.
 */
constexpr bool isConfigValTypeNonTrivial(ConfigValType type) {
  return static_cast<int>(type) >=
         static_cast<int>(ConfigValType::_nonTrivialTypes);
}

/**
 * @brief Function template to return whether the value requires fuzzy
 * comparison or not.
 */
template <typename T>
constexpr bool useFuzzyComparisonFor() {
  // Default for all types is no.
  return false;
}

/**
 * @brief Specify that @ref ConfigValType::Double scalar floating point values require fuzzy comparison
 */
template <>
constexpr bool useFuzzyComparisonFor<double>() {
  return true;
}

/**
 * @brief Function template to return type enum for specified type. All
 * supported types should have a specialization of this function handling their
 * type to @ref ConfigValType enum tags mapping.
 */
template <typename T>
constexpr ConfigValType configValTypeFor() {
  static_assert(sizeof(T) == 0,
                "unknown/unsupported type specified for ConfigValue");
  return ConfigValType::Unknown;
}

/**
 * @brief Returns @ref ConfigValType::Boolean type enum for specified type
 */
template <>
constexpr ConfigValType configValTypeFor<bool>() {
  return ConfigValType::Boolean;
}
/**
 * @brief Returns @ref ConfigValType::Integer type enum for specified type
 */
template <>
constexpr ConfigValType configValTypeFor<int>() {
  return ConfigValType::Integer;
}

/**
 * @brief Returns @ref ConfigValType::MagnumRad type enum for specified type
 */
template <>
constexpr ConfigValType configValTypeFor<Mn::Rad>() {
  return ConfigValType::MagnumRad;
}

/**
 * @brief Returns @ref ConfigValType::MagnumDeg type enum for specified type
 */
template <>
constexpr ConfigValType configValTypeFor<Mn::Deg>() {
  return ConfigValType::MagnumDeg;
}

/**
 * @brief Returns @ref ConfigValType::Double type enum for specified type
 */
template <>
constexpr ConfigValType configValTypeFor<double>() {
  return ConfigValType::Double;
}

/**
 * @brief Returns @ref ConfigValType::String type enum for specified type
 */
template <>
constexpr ConfigValType configValTypeFor<std::string>() {
  return ConfigValType::String;
}

/**
 * @brief Returns @ref ConfigValType::MagnumVec2 type enum for specified type
 */
template <>
constexpr ConfigValType configValTypeFor<Mn::Vector2>() {
  return ConfigValType::MagnumVec2;
}

/**
 * @brief Returns @ref ConfigValType::MagnumVec2i type enum for specified type
 */
template <>
constexpr ConfigValType configValTypeFor<Mn::Vector2i>() {
  return ConfigValType::MagnumVec2i;
}

/**
 * @brief Returns @ref ConfigValType::MagnumVec3 type enum for specified type
 */
template <>
constexpr ConfigValType configValTypeFor<Mn::Vector3>() {
  return ConfigValType::MagnumVec3;
}

/**
 * @brief Returns @ref ConfigValType::MagnumVec3 type enum for specified type
 */
template <>
constexpr ConfigValType configValTypeFor<Mn::Color3>() {
  return ConfigValType::MagnumVec3;
}

/**
 * @brief Returns @ref ConfigValType::MagnumVec4 type enum for specified type
 */
template <>
constexpr ConfigValType configValTypeFor<Mn::Vector4>() {
  return ConfigValType::MagnumVec4;
}

/**
 * @brief Returns @ref ConfigValType::MagnumVec4 type enum for specified type
 */
template <>
constexpr ConfigValType configValTypeFor<Mn::Color4>() {
  return ConfigValType::MagnumVec4;
}

/**
 * @brief Returns @ref ConfigValType::MagnumMat3 type enum for specified type
 */
template <>
constexpr ConfigValType configValTypeFor<Mn::Matrix3>() {
  return ConfigValType::MagnumMat3;
}

/**
 * @brief Returns @ref ConfigValType::MagnumMat4 type enum for specified type
 */
template <>
constexpr ConfigValType configValTypeFor<Mn::Matrix4>() {
  return ConfigValType::MagnumMat4;
}

/**
 * @brief Returns @ref ConfigValType::MagnumQuat type enum for specified type
 */
template <>
constexpr ConfigValType configValTypeFor<Mn::Quaternion>() {
  return ConfigValType::MagnumQuat;
}

/**
 * @brief Stream operator to support display of @ref ConfigValType enum tags
 */
Mn::Debug& operator<<(Mn::Debug& debug, const ConfigValType& value);

/**
 * @brief This class uses an anonymous tagged union to store values of different
 * types, as well as providing access to the values in a type safe manner.
 */
class ConfigValue {
 private:
  /**
   * @brief This field holds various state flags in the higher 8 bytes and the
   * type of the data represented in this @ref ConfigValue in the lower 8 bytes.
   */
  uint64_t _typeAndFlags{0x00000000FFFFFFFF};

  /**
   * @brief The data this @ref ConfigValue holds.
   * Aligns to individual 8-byte bounds. The _typeAndFlags is 8 bytes, and 8
   * bytes for data.
   */
  alignas(8) char _data[CONFIG_VAL_SIZE] = {0};

  /**
   * @brief Copy the passed @p val into this @ref ConfigValue.  If this @ref
   * ConfigValue's type is not pointer-based, this will call the appropriate
   * copy handler for the type.
   * @param val source val to copy into this config
   */
  void copyValueFrom(const ConfigValue& val);

  /**
   * @brief Move the passed @p val into this ConfigVal. If this @ref
   * ConfigValue's type is not pointer-based, this will call the appropriate
   * move handler for the type.
   * @param val source val to copy into this config
   */
  void moveValueFrom(ConfigValue&& val);

  /**
   * @brief Delete the current value. If this @ref
   * ConfigValue's type is not pointer-based, this will call the appropriate
   * destructor handler for the type.
   */
  void deleteCurrentValue();

  /**
   * @brief These functions specify how the set and get functions should perform
   * depending on whether or not the value is pointer-pased.
   */

  /**
   * @brief Set the type component of @p _typeAndFlags to the passed @p type
   * value, preserving the existing state of the flags component of
   * @p _typeAndFlags .
   */
  inline void setType(const ConfigValType& type) {
    // Clear out type component, retaining flags
    _typeAndFlags &= 0xFFFFFFFF00000000;
    // set new type component, preserving flags state via the mask
    // (ConfigValType::for Unknown)
    _typeAndFlags |= static_cast<uint64_t>(type) & 0x00000000FFFFFFFF;
  }

  /**
   * @brief Set this ConfigVal to a new value. Type is stored as a Pointer.
   */
  template <typename T>
  EnableIf<isConfigValTypePointerBased(configValTypeFor<T>()), void>
  setInternalTyped(const T& value) {
    T** tmpDst = reinterpret_cast<T**>(_data);
    *tmpDst = new T(value);
  }
  /**
   * @brief Set this ConfigVal to a new value. Type is stored directly in
   * buffer.
   */
  template <typename T>
  EnableIf<!isConfigValTypePointerBased(configValTypeFor<T>()), void>
  setInternalTyped(const T& value) {
    new (_data) T(value);
  }

  /**
   * @brief Get this ConfigVal's value. For Types that are stored in _data as a
   * Pointer.
   */
  template <typename T>
  EnableIf<isConfigValTypePointerBased(configValTypeFor<T>()), const T&>
  getInternalTyped() const {
    auto val = [&]() {
      return *reinterpret_cast<const T* const*>(this->_data);
    };
    return *val();
  }

  /**
   * @brief Get this ConfigVal's value. For Types that are stored directly in
   * buffer.
   */
  template <typename T>
  EnableIf<!isConfigValTypePointerBased(configValTypeFor<T>()), const T&>
  getInternalTyped() const {
    auto val = [&]() { return reinterpret_cast<const T*>(this->_data); };
    return *val();
  }

  /**
   * @brief Set the passed @p value as the data for this @ref ConfigValue, while also setting the appropriate type.
   * @tparam The type of the @p value being set. Must be a handled type as specified by @ref ConfigValType.
   * @param value The value to store in this @ref ConfigValue
   */
  template <typename T>
  void setInternal(const T& value) {
    deleteCurrentValue();
    // This will blow up at compile time if given type is not supported
    setType(configValTypeFor<T>());
    // These asserts are checking the integrity of the support for T's type, and
    // will fire if conditions are not met.

    // This fails if we added a new type into @ref ConfigValType enum improperly
    // (trivial type added after entry ConfigValType::_nonTrivialTypes, or
    // vice-versa)
    static_assert(isConfigValTypeNonTrivial(configValTypeFor<T>()) !=
                      std::is_trivially_copyable<T>::value,
                  "Something's incorrect about enum placement for added type "
                  "(type is not trivially copyable, or vice-versa)");

    // This verifies that any values that are too large to be stored directly
    // (or are already specified as non-trivial) are pointer based, while those
    // that are trivial and small are stored directly,
    //
    static_assert(
        ((sizeof(T) <= sizeof(_data)) ||
         isConfigValTypePointerBased(configValTypeFor<T>())),
        "ConfigValue's internal storage is too small for added type!");
    // This fails if a new type was added whose alignment does not match
    // internal storage alignment
    static_assert(
        alignof(T) <= alignof(ConfigValue),
        "ConfigValue's internal storage improperly aligned for added type!");

    //_data should be destructed at this point, construct a new value
    setInternalTyped(value);
    // set whether this type requires fuzzy comparison or not
    setReqsFuzzyCompare(useFuzzyComparisonFor<T>());

  }  // ConfigValue::setInternal

  /**
   * @brief Returns true if this and otr's _data arrays are not pointers to the
   * same data, or are not pointerbased types.
   */
  inline bool isSafeToDeconstruct(const ConfigValue& otr) const {
    // Don't want to clobber otr's data when we decon this if pointer backed
    // type and each ConfigValue's _data arrays are equal (when cast to
    // pointers)
    return !isConfigValTypePointerBased(getType()) ||
           !(reinterpret_cast<const void*>(_data) ==
             reinterpret_cast<const void*>(otr._data));
  }

 public:
  /**
   * @brief Constructor
   */
  ConfigValue() = default;
  /**
   * @brief Copy Constructor
   */
  ConfigValue(const ConfigValue& otr);
  /**
   * @brief Move Constructor
   */
  ConfigValue(ConfigValue&& otr) noexcept;
  ~ConfigValue();

  /**
   * @brief Copy assignment
   */
  ConfigValue& operator=(const ConfigValue& otr);

  /**
   * @brief Move assignment
   */
  ConfigValue& operator=(ConfigValue&& otr) noexcept;

  /**
   * @brief Whether this @ref ConfigValue is valid.
   * @return Whether or not the specified type of this @ref ConfigValue is known.
   */
  bool isValid() const { return getType() != ConfigValType::Unknown; }

  /**
   * @brief Write this @ref ConfigValue to an appropriately configured json object.
   */
  io::JsonGenericValue writeToJsonObject(io::JsonAllocator& allocator) const;

  /**
   * @brief Set the passed @p value as the data for this @ref ConfigValue, while
   * also setting the appropriate type.
   * @tparam The type of the @p value being set. Must be a handled type as
   * specified by @ref ConfigValType.
   * @param value The value to store in this @ref ConfigValue
   * @param isTranslated Whether this value is translated before use or not.
   */
  template <typename T>
  void set(const T& value, bool isTranslated = false) {
    setInternal(value);
    // set default value state to false
    setDefaultVal(false);
    setHiddenVal(false);
    setTranslatedVal(isTranslated);
  }

  /**
   * @brief Set the passed @p value as the data for this @ref ConfigValue, while
   * also setting the appropriate type. This ConfigValue is hidden, to be used
   * internally and not expected to be exposed to user interaction except for
   * potentially debugging.
   * @tparam The type of the @p value being set. Must be a handled type as
   * specified by @ref ConfigValType.
   * @param value The value to store in this @ref ConfigValue
   * @param isTranslated Whether this value is translated before use or not.
   */
  template <typename T>
  void setHidden(const T& value, bool isTranslated = false) {
    setInternal(value);
    // set default value state to false
    setDefaultVal(false);
    setHiddenVal(true);
    setTranslatedVal(isTranslated);
  }

  /**
   * @brief Set the passed @p value as the programmatic default/initialization
   * data for this @ref ConfigValue, while also setting the appropriate type. This value
   * will not be saved to file unless it is changed by file read or user input.
   * @tparam The type of the @p value being set. Must be a handled type as
   * specified by @ref ConfigValType.
   * @param value The value to store in this @ref ConfigValue
   * @param isTranslated Whether this value is translated before use or not.
   */
  template <typename T>
  void init(const T& value, bool isTranslated = false) {
    setInternal(value);
    // set default value state to true
    setDefaultVal(true);
    setHiddenVal(false);
    setTranslatedVal(isTranslated);
  }

  /**
   * @brief Retrieve an appropriately cast copy of the data stored in this @ref ConfigValue
   * @tparam The type the data should be cast as.
   */
  template <typename T>
  const T& get() const {
    ESP_CHECK(getType() == configValTypeFor<T>(),
              "Attempting to access ConfigValue of" << getType() << "with type"
                                                    << configValTypeFor<T>());
    return getInternalTyped<T>();
  }

  /**
   * @brief Returns the current type of this @ref ConfigValue
   */
  inline ConfigValType getType() const {
    return static_cast<ConfigValType>(_typeAndFlags & 0xFFFFFFFF);
  }

  /**
   * @brief Get whether the flags specified by @p mask are true or not
   */
  inline bool getState(const uint64_t mask) const {
    return (_typeAndFlags & mask) == mask;
  }
  /**
   * @brief Set the flags specified by @p mask to be @p val
   */
  inline void setState(const uint64_t mask, bool val) {
    if (val) {
      _typeAndFlags |= mask;
    } else {
      _typeAndFlags &= ~mask;
    }
  }

  /**
   * @brief Check whether this ConfigVal was set as a programmatic default value
   * or was set from metadata or other intentional sources. We may not wish to
   * write this value to file if it was only populated with a default value
   * programmatically.
   */
  inline bool isDefaultVal() const {
    return getState(ConfigValStatus::isDefault);
  }
  /**
   * @brief Set whether this ConfigVal was set as a programmatic default value
   * or was set from metadata or other intentional sources. We may not wish to
   * write this value to file if it was only populated with a default value
   * programmatically.
   */
  inline void setDefaultVal(bool isDefault) {
    setState(ConfigValStatus::isDefault, isDefault);
  }

  /**
   * @brief Check whether this ConfigVal is an internal value used
   * programmatically in conjunction with other data in the owning
   * Configuration. These values probably should never be written to file or
   * shared with the user except for debugging purposes.
   */
  inline bool isHiddenVal() const {
    return getState(ConfigValStatus::isHidden);
  }
  /**
   * @brief Set whether this ConfigVal is an internal value used
   * programmatically in conjunction with other data in the owning
   * Configuration. These values probably should never be written to file or
   * shared with the user except for debugging purposes.
   */
  inline void setHiddenVal(bool isHidden) {
    setState(ConfigValStatus::isHidden, isHidden);
  }

  /**
   * @brief Check whether this ConfigVal is translated before use, for example
   * to an enum. This is generally to limit values and provide context and
   * meaning to the values it holds. These ConfigVals are stored intnerally in a
   * Configuration as strings but are consumed as their translated value (i.e
   * enums). Because of this, they need custom handling to be read from file.
   *
   *  NOTE : Any ConfigValues specified as @p isTranslated need to have their
   * Json read handled by the consuming class, as they will not be necessarily
   * properly read from file otherwise.
   */
  inline bool isTranslated() const {
    return getState(ConfigValStatus::isTranslated);
  }
  /**
   * @brief Set whether this ConfigVal is translated before use, for example
   * to an enum. This is generally to limit values and provide context and
   * meaning to the values it holds. These ConfigVals are stored intnerally in a
   * Configuration as strings but are consumed as their translated value (i.e
   * enums). Because of this, they need custom handling to be read from file.
   *
   *  NOTE : Any ConfigValues specified as @p isTranslated need to have their
   * Json read handled by the consuming class, as they will not be necessarily
   * properly read from file otherwise.
   */
  inline void setTranslatedVal(bool isTranslated) {
    setState(ConfigValStatus::isTranslated, isTranslated);
  }

  /**
   * @brief Check whether this ConfigVal requires a fuzzy comparison for
   * equality (i.e. for a scalar double).
   *
   * The comparisons for such a type
   * should use Magnum::Math::equal to be consistent with similarly configured
   * magnum types.
   */
  inline bool reqsFuzzyCompare() const {
    return getState(ConfigValStatus::reqsFuzzyComparison);
  }
  /**
   * @brief Check whether this ConfigVal requires a fuzzy comparison for
   * equality (i.e. for a scalar double).
   *
   * The comparisons for such a type
   * should use Magnum::Math::equal to be consistent with similarly configured
   * magnum types.
   */
  inline void setReqsFuzzyCompare(bool fuzzyCompare) {
    setState(ConfigValStatus::reqsFuzzyComparison, fuzzyCompare);
  }

  /**
   * @brief Whether or not this @ref ConfigValue should be written to file during
   * common execution. The reason we may not want to do this might be that the
   * value was only set to a programmatic default value, and not set
   * intentionally from the source file or user input; we also do not want to
   * write any internal/hidden values to files.
   */
  bool shouldWriteToFile() const { return !(isDefaultVal() || isHiddenVal()); }

  /**
   * @brief Retrieve a string representation of the data held in this @ref
   * ConfigValue
   */
  std::string getAsString() const;

  /**
   * @brief Copy this @ref ConfigValue into the passed @ref Corrade::Utility::ConfigurationGroup
   */
  bool putValueInConfigGroup(const std::string& key,
                             Cr::Utility::ConfigurationGroup& cfg) const;

  /**
   * @brief Comparison
   */
  friend bool operator==(const ConfigValue& a, const ConfigValue& b);
  /**
   * @brief Inequality Comparison
   */
  friend bool operator!=(const ConfigValue& a, const ConfigValue& b);

  ESP_SMART_POINTERS(ConfigValue)
};  // class ConfigValue

/**
 * @brief provide debug stream support for @ref ConfigValue
 */
Mn::Debug& operator<<(Mn::Debug& debug, const ConfigValue& value);

/**
 * @brief This class holds Configuration data in a map of ConfigValues, and
 * also supports nested Configurations via a map of smart pointers to this
 * type.
 */
class Configuration {
 public:
  /**
   * @brief Convenience typedef for the value map
   */
  typedef std::unordered_map<std::string, ConfigValue> ValueMapType;
  /**
   * @brief Convenience typedef for the subConfiguration map
   */
  typedef std::map<std::string, std::shared_ptr<Configuration>> ConfigMapType;

  /**
   * @brief Constructor
   */
  Configuration() = default;

  /**
   * @brief Copy Constructor
   */
  Configuration(const Configuration& otr)
      : configMap_(), valueMap_(otr.valueMap_) {
    for (const auto& entry : otr.configMap_) {
      configMap_[entry.first] = std::make_shared<Configuration>(*entry.second);
    }
  }  // copy ctor

  /**
   * @brief Move Constructor
   */
  Configuration(Configuration&& otr) noexcept
      : configMap_(std::move(otr.configMap_)),
        valueMap_(std::move(otr.valueMap_)) {}  // move ctor

  // virtual destructor set so that pybind11 recognizes attributes inheritance
  // from Configuration to be polymorphic
  virtual ~Configuration() = default;

  /**
   * @brief Copy Assignment.
   */
  Configuration& operator=(const Configuration& otr);

  /**
   * @brief Move Assignment.
   */
  Configuration& operator=(Configuration&& otr) noexcept = default;

  // ****************** Getters ******************
  /**
   * @brief Get @ref ConfigValue specified by key, or empty ConfigValue if DNE.
   *
   * @param key The key of the value desired to be retrieved.
   * @return ConfigValue specified by key. If none exists, will be empty
   * ConfigValue, with type @ref ConfigValType::Unknown
   */
  ConfigValue get(const std::string& key) const {
    ValueMapType::const_iterator mapIter = valueMap_.find(key);
    if (mapIter != valueMap_.end()) {
      return mapIter->second;
    }
    ESP_WARNING() << "Key :" << key << "not present in Configuration";
    return {};
  }

  /**
   * @brief Get value specified by @p key and expected to be type @p T and
   * return it if it exists and is appropriate type.  Otherwise throw a
   * warning and return a default value.
   * @tparam The type of the value desired
   * @param key The key of the value desired to be retrieved.
   * @return The value held at @p key, expected to be type @p T .  If not
   * found, or not of expected type, gives an error message and returns a
   * default value.
   */
  template <typename T>
  T get(const std::string& key) const {
    ValueMapType::const_iterator mapIter = valueMap_.find(key);
    const ConfigValType desiredType = configValTypeFor<T>();
    if (mapIter != valueMap_.end() &&
        (mapIter->second.getType() == desiredType)) {
      return mapIter->second.get<T>();
    }
    ESP_ERROR() << "Key :" << key << "not present in Configuration as"
                << getNameForStoredType(desiredType);
    return {};
  }

  // ****************** Value Status ******************

  /**
   * @brief Return the @ref ConfigValType enum representing the type of the
   * value referenced by the passed @p key or @ref ConfigValType::Unknown
   * if unknown/unspecified.
   */
  ConfigValType getType(const std::string& key) const {
    ValueMapType::const_iterator mapIter = valueMap_.find(key);
    if (mapIter != valueMap_.end()) {
      return mapIter->second.getType();
    }
    ESP_ERROR() << "Key :" << key << "not present in Configuration.";
    return ConfigValType::Unknown;
  }

  /**
   * @brief Returns whether or not the @ref ConfigValue specified
   * by @p key is a default/initialization value or was intentionally set.
   */
  bool isDefaultVal(const std::string& key) const {
    ValueMapType::const_iterator mapIter = valueMap_.find(key);
    if (mapIter != valueMap_.end()) {
      return mapIter->second.isDefaultVal();
    }
    ESP_ERROR() << "Key :" << key << "not present in Configuration.";
    return false;
  }

  /**
   * @brief Returns whether or not the @ref ConfigValue specified
   * by @p key is a hidden value intended to be be only used internally.
   */
  bool isHiddenVal(const std::string& key) const {
    ValueMapType::const_iterator mapIter = valueMap_.find(key);
    if (mapIter != valueMap_.end()) {
      return mapIter->second.isHiddenVal();
    }
    ESP_ERROR() << "Key :" << key << "not present in Configuration.";
    return false;
  }

  /**
   * @brief Returns whether or not the @ref ConfigValue specified
   * by @p key is a translated value, meaning a string that corresponds to, and
   * is translated into, an enum value for consumption.
   */
  bool isTranslated(const std::string& key) const {
    ValueMapType::const_iterator mapIter = valueMap_.find(key);
    if (mapIter != valueMap_.end()) {
      return mapIter->second.isTranslated();
    }
    ESP_ERROR() << "Key :" << key << "not present in Configuration.";
    return false;
  }

  // ****************** String Conversion ******************

  /**
   * @brief This method will look for the provided key, and return a string
   * holding the object, if it is found in one of this Configuration's maps
   */
  std::string getAsString(const std::string& key) const {
    ValueMapType::const_iterator mapIter = valueMap_.find(key);
    if (mapIter != valueMap_.end()) {
      return mapIter->second.getAsString();
    }
    std::string retVal = Cr::Utility::formatString(
        "Key {} does not represent a valid value in this Configuration.", key);
    ESP_WARNING() << retVal;
    return retVal;
  }

  // ****************** Key List Retrieval ******************

  /**
   * @brief Retrieve list of keys present in this @ref Configuration's
   * valueMap_. Subconfigs are not included.
   * @return a vector of strings representing the subconfig keys for this
   * Configuration.
   */
  std::vector<std::string> getKeys(bool sorted = false) const {
    std::vector<std::string> keys;
    keys.reserve(valueMap_.size());
    for (const auto& entry : valueMap_) {
      keys.push_back(entry.first);
    }
    if (sorted) {
      std::sort(keys.begin(), keys.end());
    }
    return keys;
  }

  /**
   * @brief This function returns this @ref Configuration's subconfig keys.
   * @param sorted whether the keys should be sorted or not.
   * @return a vector of strings representing the subconfig keys for this
   * Configuration.
   */
  std::vector<std::string> getSubconfigKeys(bool sorted = false) const {
    std::vector<std::string> keys;
    keys.reserve(configMap_.size());
    for (const auto& entry : configMap_) {
      keys.push_back(entry.first);
    }
    if (sorted) {
      std::sort(keys.begin(), keys.end());
    }
    return keys;
  }

  /**
   * @brief Retrieve a list of all the keys in this @ref Configuration pointing
   * to values of passed type @p storedType.
   * @param storedType The desired type of value whose key should be returned.
   * @param sorted whether the keys should be sorted or not.
   * @return vector of string keys pointing to values of desired @p storedType
   */
  std::vector<std::string> getKeysByType(ConfigValType storedType,
                                         bool sorted = false) const {
    std::vector<std::string> keys;
    // reserve space for all keys
    keys.reserve(valueMap_.size());
    unsigned int count = 0;
    for (const auto& entry : valueMap_) {
      if (entry.second.getType() == storedType) {
        ++count;
        keys.push_back(entry.first);
      }
    }
    if (valueMap_.size() > count) {
      keys.shrink_to_fit();
    }
    if (sorted) {
      std::sort(keys.begin(), keys.end());
    }
    return keys;
  }

  // ****************** Setters ******************

  /**
   * @brief Save the passed @p value using specified @p key
   * @tparam The type of the value to be saved.
   * @param key The key to assign to the passed value.
   * @param value The value to save at given @p key
   */
  template <typename T>
  void set(const std::string& key, const T& value) {
    valueMap_[key].set<T>(value);
  }
  /**
   * @brief Save the passed @p value char* as a string to the Configuration at
   * the passed @p key.
   * @param key The key to assign to the passed value.
   * @param value The char* to save at given @p key as a string.
   */
  void set(const std::string& key, const char* value) {
    valueMap_[key].set<std::string>(std::string(value));
  }

  /**
   * @brief Save the passed float @p value as a double using the specified @p
   * key .
   * @param key The key to assign to the passed value.
   * @param value The float value to save at given @p key as a double.
   */
  void set(const std::string& key, float value) {
    valueMap_[key].set<double>(static_cast<double>(value));
  }

  /**
   * @brief Save the passed string @p value using specified @p key as an
   * translated/enum-backed value. This value will be translated by the consumer
   * of this Configuration to be an enum constant and so should not use
   * the generic Configuration file read functionality but rather an
   * implementation that will verify the validity of the input.
   * @param key The key to assign to the passed value.
   * @param value The string value to save at given @p key
   */
  void setTranslated(const std::string& key, const char* value) {
    valueMap_[key].set<std::string>(std::string(value), true);
  }
  /**
   * @brief Save the passed string @p value using specified @p key as an
   * translated/enum-backed value. This value will be translated by the consumer
   * of this Configuration to be an enum constant and so should not use
   * the generic Configuration file read functionality but rather an
   * implementation that will verify the validity of the input.
   * @param key The key to assign to the passed value.
   * @param value The string value to save at given @p key
   */
  void setTranslated(const std::string& key, const std::string& value) {
    valueMap_[key].set<std::string>(value, true);
  }

  /**
   * @brief Save the passed @p value using specified @p key as a hidden value,
   * to be used internally but not saved or exposed to the user except for debug
   * purposes.
   * @tparam The type of the value to be saved.
   * @param key The key to assign to the passed value.
   * @param value The value to save at given @p key
   */
  template <typename T>
  void setHidden(const std::string& key, const T& value) {
    valueMap_[key].setHidden<T>(value);
  }
  /**
   * @brief Save the passed @p value char* as a string to the Configuration at
   * the passed @p key as a hidden value,  to be used internally but not saved
   * or exposed to the user except for debug purposes.
   * @param key The key to assign to the passed value.
   * @param value The char* to save at given @p key as a string.
   */
  void setHidden(const std::string& key, const char* value) {
    valueMap_[key].setHidden<std::string>(std::string(value));
  }

  /**
   * @brief Save the passed float @p value as a double using the specified
   * @p key as a hidden value, to be used internally but not saved or exposed to
   * the user except for debug purposes.
   * @param key The key to assign to the passed value.
   * @param value The float value to save at given @p key as a double.
   */
  void setHidden(const std::string& key, float value) {
    valueMap_[key].setHidden<double>(static_cast<double>(value));
  }

  /**
   * @brief Save the passed string @p value using specified @p key as an
   * translated/enum-backed valuethat is also a hidden value, to be used
   * internally but not saved or exposed to the user except for debug
   * purposes.This value will be translated by the consumer of this
   * Configuration to be an enum constant and so should not use the generic
   * Configuration file read functionality but rather an implementation that
   * will verify the validity of the input.
   * @param key The key to assign to the passed value.
   * @param value The string value to save at given @p key
   */
  void setHiddenTranslated(const std::string& key, const char* value) {
    valueMap_[key].setHidden<std::string>(std::string(value), true);
  }

  /**
   * @brief Save the passed string @p value using specified @p key as an
   * translated/enum-backed valuethat is also a hidden value, to be used
   * internally but not saved or exposed to the user except for debug
   * purposes.This value will be translated by the consumer of this
   * Configuration to be an enum constant and so should not use the generic
   * Configuration file read functionality but rather an implementation that
   * will verify the validity of the input.
   * @param key The key to assign to the passed value.
   * @param value The string value to save at given @p key
   */
  void setHiddenTranslated(const std::string& key, const std::string& value) {
    valueMap_[key].setHidden<std::string>(value, true);
  }

  /**
   * @brief Save the passed @p value using specified @p key as a
   * programmatically set initial value (will not be written to file if this
   * Configuration is saved unless it is changed via a file read or user input).
   * @tparam The type of the value to be saved.
   * @param key The key to assign to the passed value.
   * @param value The value to save at given @p key
   */
  template <typename T>
  void init(const std::string& key, const T& value) {
    valueMap_[key].init<T>(value);
  }
  /**
   * @brief Save the passed @p value char* as a string to the Configuration at
   * the passed @p key as a programmatically set initial value (will not be
   * written to file if this Configuration is saved unless it is changed via a
   * file read or user input).
   * @param key The key to assign to the passed value.
   * @param value The char* to save at given @p key as a string.
   */
  void init(const std::string& key, const char* value) {
    valueMap_[key].init<std::string>(std::string(value));
  }

  /**
   * @brief Save the passed float @p value as a double using the specified
   * @p key as a programmatically set initial value (will not be written to file
   * if this Configuration is saved unless it is changed via a file read or user
   * input).
   * @param key The key to assign to the passed value.
   * @param value The float value to save at given @p key as a double.
   */
  void init(const std::string& key, float value) {
    valueMap_[key].init<double>(static_cast<double>(value));
  }

  /**
   * @brief Save the passed string @p value using specified @p key as a
   * programmatically set initial value (will not be written to file if this
   * Configuration is saved unless it is changed via a file read or user input)
   * of the translated/enum-backed value. This value will be translated by the
   * consumer of this Configuration to be an enum constant and so should not use
   * the generic Configuration file read functionality but rather an
   * implementation that will verify the validity of the input.
   * @param key The key to assign to the passed value.
   * @param value The string value to save at given @p key
   */
  void initTranslated(const std::string& key, const char* value) {
    valueMap_[key].init<std::string>(std::string(value), true);
  }

  /**
   * @brief Save the passed string @p value using specified @p key as a
   * programmatically set initial value (will not be written to file if this
   * Configuration is saved unless it is changed via a file read or user input)
   * of the translated/enum-backed value. This value will be translated by the
   * consumer of this Configuration to be an enum constant and so should not use
   * the generic Configuration file read functionality but rather an
   * implementation that will verify the validity of the input.
   * @param key The key to assign to the passed value.
   * @param value The string value to save at given @p key
   */
  void initTranslated(const std::string& key, const std::string& value) {
    valueMap_[key].init<std::string>(value, true);
  }

  // ****************** Value removal ******************

  /**
   * @brief Remove value specified by @p key and return it if it exists.
   * Otherwise throw a warning and return a default value.
   * @param key The key of the value desired to be retrieved/removed.
   * @return The erased value, held at @p key if found.  If not found, or not
   * of expected type, gives a warning and returns a default value.
   */

  ConfigValue remove(const std::string& key) {
    ValueMapType::const_iterator mapIter = valueMap_.find(key);
    if (mapIter != valueMap_.end()) {
      valueMap_.erase(mapIter);
      return mapIter->second;
    }
    ESP_WARNING() << "Key :" << key << "not present in Configuration";
    return {};
  }

  /**
   * @brief Remove value specified by @p key and expected to be type @p T and
   * return it if it exists and is appropriate type.  Otherwise throw a
   * warning and return a default value.
   * @tparam The type of the value desired
   * @param key The key of the value desired to be retrieved/removed.
   * @return The erased value, held at @p key and expected to be type @p T ,
   * if found.  If not found, or not of expected type, gives a warning and
   * returns a default value.
   */
  template <typename T>
  T remove(const std::string& key) {
    ValueMapType::const_iterator mapIter = valueMap_.find(key);
    const ConfigValType desiredType = configValTypeFor<T>();
    if (mapIter != valueMap_.end() &&
        (mapIter->second.getType() == desiredType)) {
      valueMap_.erase(mapIter);
      return mapIter->second.get<T>();
    }
    ESP_WARNING() << "Key :" << key << "not present in Configuration as"
                  << getNameForStoredType(desiredType);
    return {};
  }

  /**
   * @brief Remove all values from this Configuration of the specified type.
   */
  template <typename T>
  void removeAllOfType() {
    const ConfigValType desiredType = configValTypeFor<T>();
    for (auto mapIter = valueMap_.cbegin(), nextIter = mapIter;
         mapIter != valueMap_.cend(); mapIter = nextIter) {
      ++nextIter;
      if (mapIter->second.getType() == desiredType) {
        valueMap_.erase(mapIter);
      }
    }
  }

  // ************************* Queries **************************

  /**
   * @brief Return number of value and subconfig entries in this
   * Configuration. This only counts each subConfiguration entry as a single
   * entry.
   */
  int getNumEntries() const { return configMap_.size() + valueMap_.size(); }

  /**
   * @brief Return total number of value and subconfig entries held by this
   * Configuration and all its subconfigs.
   */
  int getConfigTreeNumEntries() const {
    int num = getNumEntries();
    for (const auto& subConfig : configMap_) {
      num += subConfig.second->getConfigTreeNumEntries();
    }
    return num;
  }
  /**
   * @brief Return number of subconfig entries in this Configuration. This
   * only counts each subConfiguration entry as a single entry.
   */
  int getNumSubconfigs() const { return configMap_.size(); }

  /**
   * @brief Return size of entire subconfig tree (i.e. total number of
   * subconfigs nested under this Configuration.)
   */
  int getConfigTreeNumSubconfigs() const {
    int num = configMap_.size();
    for (const auto& subConfig : configMap_) {
      num += subConfig.second->getConfigTreeNumSubconfigs();
    }
    return num;
  }

  /**
   * @brief Returns number of values in this Configuration.
   */
  int getNumValues() const { return valueMap_.size(); }

  /**
   * @brief Returns number of non-hidden values in this Configuration. This is
   * necessary for determining whether or not configurations are "effectively"
   * equal, where they contain the same data but may vary in number
   * internal-use-only fields.
   */
  int getNumVisibleValues() const {
    int numVals = 0;
    for (const auto& val : valueMap_) {
      if (!val.second.isHiddenVal()) {
        numVals += 1;
      }
    }
    return numVals;
  }

  /**
   * @brief Return total number of values held by this Configuration and all
   * its subconfigs.
   */
  int getConfigTreeNumValues() const {
    int num = valueMap_.size();
    for (const auto& subConfig : configMap_) {
      num += subConfig.second->getConfigTreeNumValues();
    }
    return num;
  }

  /**
   * @brief Returns whether this @ref Configuration has the passed @p key as a
   * non-Configuration value. Does not check subConfigurations.
   */
  bool hasValue(const std::string& key) const {
    return valueMap_.count(key) > 0;
  }

  /**
   * @brief Whether passed @p key references a @ref ConfigValue of passed @ref ConfigValType @p desiredType
   * @param key The key to check the type of.
   * @param desiredType the @ref ConfigValType to compare the value's type to
   * @return Whether @p key references a value that is of @p desiredType.
   */
  bool hasKeyToValOfType(const std::string& key,
                         ConfigValType desiredType) const {
    ValueMapType::const_iterator mapIter = valueMap_.find(key);
    return (mapIter != valueMap_.end() &&
            (mapIter->second.getType() == desiredType));
  }

  /**
   * @brief Checks if passed @p key is contained in this Configuration.
   * Returns a list of nested subConfiguration keys, in order, to the
   * Configuration where the key was found, ending in the requested @p key.
   * If list is empty, @p key was not found.
   * @param key The key to look for
   * @return A breadcrumb list to where the value referenced by @p key
   * resides. An empty list means the value was not found.
   */
  std::vector<std::string> findValue(const std::string& key) const;

  /**
   * @brief Builds and returns @ref Corrade::Utility::ConfigurationGroup
   * holding the values in this esp::core::config::Configuration.
   *
   * @return a reference to a Configuration group for this Configuration
   * object.
   */
  Cr::Utility::ConfigurationGroup getConfigGroup() const {
    Cr::Utility::ConfigurationGroup cfg{};
    putAllValuesInConfigGroup(cfg);
    return cfg;
  }

  /**
   * @brief This method will build a map of the keys of all the config values
   * this Configuration holds and the types of each of these values.
   */
  std::unordered_map<std::string, ConfigValType> getValueTypes() const {
    std::unordered_map<std::string, ConfigValType> res{};
    res.reserve(valueMap_.size());
    for (const auto& elem : valueMap_) {
      res[elem.first] = elem.second.getType();
    }
    return res;
  }

  // ****************** SubConfiguration accessors ******************

  /**
   * @brief return if passed key corresponds to a subconfig in this
   * Configuration
   */
  bool hasSubconfig(const std::string& key) const {
    ConfigMapType::const_iterator mapIter = configMap_.find(key);
    return (mapIter != configMap_.end());
  }

  /**
   * @brief return if passed @p subConfig exists in this Configuration's
   * subconfig map. Does not compare hidden ConfigValues.
   */
  template <typename T>
  bool hasSubconfig(const std::shared_ptr<T>& subConfig) const {
    static_assert(std::is_base_of<Configuration, T>::value,
                  "Configuration : Desired subconfig must be derived from "
                  "core::config::Configuration");
    auto cfgIterPair = getSubconfigIterator();
    for (auto& cfgIter = cfgIterPair.first; cfgIter != cfgIterPair.second;
         ++cfgIter) {
      if (*(cfgIter->second) == *subConfig) {
        return true;
      }
    }
    return false;
  }  // hasSubconfig

  /**
   * @brief Templated subconfig copy getter. Retrieves a shared pointer to a
   * copy of the subConfig @ref esp::core::config::Configuration that has the
   * passed @p cfgName .
   *
   * @tparam Type to return. Must inherit from @ref
   * esp::core::config::Configuration
   * @param cfgName The name of the Configuration to retrieve.
   * @return A pointer to a copy of the Configuration having the requested
   * name, cast to the appropriate type, or nullptr if not found.
   */
  template <typename T>
  std::shared_ptr<T> getSubconfigCopy(const std::string& cfgName) const {
    static_assert(std::is_base_of<Configuration, T>::value,
                  "Configuration : Desired subconfig must be derived from "
                  "core::config::Configuration");
    auto configIter = configMap_.find(cfgName);
    if (configIter != configMap_.end()) {
      // if exists return copy, so that consumers can modify it freely
      return std::make_shared<T>(
          *std::static_pointer_cast<T>(configIter->second));
    }
    return nullptr;
  }

  /**
   * @brief return pointer to read-only sub-Configuration of given @p cfgName.
   * Will fail if Configuration with given name dne.
   * @param cfgName The name of the desired Configuration.
   */
  std::shared_ptr<const Configuration> getSubconfigView(
      const std::string& cfgName) const {
    auto configIter = configMap_.find(cfgName);
    CORRADE_ASSERT(configIter != configMap_.end(),
                   "SubConfiguration with name "
                       << cfgName << " not found in Configuration.",
                   nullptr);
    // if exists return actual object
    return configIter->second;
  }

  /**
   * @brief Templated Version. Retrieves the stored shared pointer to the
   * subConfig @ref esp::core::config::Configuration that has the passed @p cfgName
   * , cast to the specified type. This will create a shared pointer to a new
   * sub-Configuration if none exists and return it, cast to specified type.
   *
   * Use this function when you wish to modify this Configuration's
   * subgroup, possibly creating it in the process.
   * @tparam The type to cast the @ref esp::core::config::Configuration to. Type
   * is checked to verify that it inherits from Configuration.
   * @param cfgName The name of the Configuration to edit.
   * @return The actual pointer to the Configuration having the requested
   * name, cast to the specified type.
   */
  template <typename T>
  std::shared_ptr<T> editSubconfig(const std::string& cfgName) {
    static_assert(std::is_base_of<Configuration, T>::value,
                  "Configuration : Desired subconfig must be derived from "
                  "core::config::Configuration");
    // retrieve existing (or create new) subgroup, with passed name
    return std::static_pointer_cast<T>(
        addOrEditSubgroup<T>(cfgName).first->second);
  }

  /**
   * @brief move specified subgroup config into configMap at desired name.
   * Will replace any subConfiguration at given name without warning if
   * present.
   * @param cfgName The name of the subConfiguration to add
   * @param configPtr A pointer to a subConfiguration to add.
   */
  template <typename T>
  void setSubconfigPtr(const std::string& cfgName,
                       std::shared_ptr<T>& configPtr) {
    static_assert(std::is_base_of<Configuration, T>::value,
                  "Configuration : Desired subconfig must be derived from "
                  "core::config::Configuration");

    configMap_[cfgName] = std::move(configPtr);
  }  // setSubconfigPtr

  /**
   * @brief Removes and returns the named subconfig. If not found, returns an
   * empty subconfig with a warning.
   * @param cfgName The name of the subConfiguration to delete
   * @return a shared pointer to the removed subConfiguration.
   */
  std::shared_ptr<Configuration> removeSubconfig(const std::string& cfgName) {
    ConfigMapType::const_iterator mapIter = configMap_.find(cfgName);
    if (mapIter != configMap_.end()) {
      configMap_.erase(mapIter);
      return mapIter->second;
    }
    ESP_WARNING() << "Name :" << cfgName
                  << "not present in map of subConfigurations.";
    return {};
  }

  /**
   * @brief Retrieve the number of entries held by the subconfig with the
   * given name
   * @param cfgName The name of the subconfig to query. If not found, returns 0
   * with a warning.
   * @return The number of entries in the named subconfig
   */
  int getSubconfigNumEntries(const std::string& cfgName) const {
    auto configIter = configMap_.find(cfgName);
    if (configIter != configMap_.end()) {
      return configIter->second->getNumEntries();
    }
    ESP_WARNING() << "No Subconfig found named :" << cfgName;
    return 0;
  }

  /**
   * @brief Retrieve the number of entries held by the subconfig with the
   * given @p cfgName , recursing subordinate subconfigs
   * @param cfgName The name of the subconfig to query. If not found, returns 0
   * with a warning.
   * @return The number of entries in the named subconfig, including all
   * subconfigs
   */
  int getSubconfigTreeNumEntries(const std::string& cfgName) const {
    auto configIter = configMap_.find(cfgName);
    if (configIter != configMap_.end()) {
      return configIter->second->getConfigTreeNumEntries();
    }
    ESP_WARNING() << "No Subconfig found named :" << cfgName;
    return 0;
  }

  /**
   * @brief Merges Configuration pointed to by @p src into this
   * Configuration, including all subconfigs. Passed config overwrites
   * existing data in this config.
   * @param src The source of Configuration data we wish to merge into this
   * Configuration.
   */
  void overwriteWithConfig(const std::shared_ptr<const Configuration>& src);

  /**
   * @brief Performs the opposite operation to @ref Configuration::overwriteWithConfig.
   * All values and subconfigs in the passed Configuration will be removed from
   * this config unless the data they hold is different. Any empty subconfigs
   * will be removed as well.
   *
   * @param src The source of Configuration data we wish to prune from this
   * Configuration.
   */

  void filterFromConfig(const std::shared_ptr<const Configuration>& src);

  /**
   * @brief Returns a const iterator across the map of values.
   */
  std::pair<ValueMapType::const_iterator, ValueMapType::const_iterator>
  getValuesIterator() const {
    return std::make_pair(valueMap_.cbegin(), valueMap_.cend());
  }

  /**
   * @brief Returns a const iterator across the map of subConfigurations.
   */
  std::pair<ConfigMapType::const_iterator, ConfigMapType::const_iterator>
  getSubconfigIterator() const {
    return std::make_pair(configMap_.cbegin(), configMap_.cend());
  }

  /**
   * @brief Get all values of passed type held within subconfig specified by
   * given tag @p subCfgName and place the values into a vector.
   *
   * This assumes the  subconfig's values of the specified type are all intended
   * to be considered as part of a vector of data.
   * @tparam The type of data within the specified subconfig we wish to
   * retrieve.
   * @param subCfgName The handle of the
   */
  template <typename T>
  std::vector<T> getSubconfigValsOfTypeInVector(
      const std::string& subCfgName) const {
    const ConfigValType desiredType = configValTypeFor<T>();
    auto configIter = configMap_.find(subCfgName);
    if (configIter == configMap_.end()) {
      ESP_WARNING() << "No Subconfig found named :" << subCfgName;
      return {};
    }
    const auto subCfg = configIter->second;
    const auto& subCfgTags = subCfg->getKeysByType(desiredType, true);
    std::vector<T> res;
    res.reserve(subCfgTags.size());
    for (const auto& tag : subCfgTags) {
      res.emplace_back(subCfg->get<T>(tag));
    }
    return res;
  }  // getSubconfigValsOfTypeInVector

  /**
   * @brief Set all values from vector of passed type into subconfig specified
   * by given tag @p subCfgName as key-value pairs where the key is the index in
   * the vector as a string (to preserve vector ordering). This function will
   * remove all existing values of specified type from the subconfig before
   * adding the values specified in the given map.
   *
   * This assumes the  subconfig's values of the specified type are all intended
   * to be considered as part of a vector of data.
   * @tparam The type of data within the specified subConfig we wish to
   * set from the passed vector.
   * @param subCfgName The handle of the desired subconfig to add the values to.
   * @param values The vector of values of type @p T to add to the specified
   * subconfig.
   */
  template <typename T>
  void setSubconfigValsOfTypeInVector(const std::string& subCfgName,
                                      const std::vector<T>& values) {
    auto subCfg = editSubconfig<Configuration>(subCfgName);
    // remove existing values in subconfig of specified type
    subCfg->removeAllOfType<T>();
    // add new values, building string key from index in values array of each
    // value.
    for (std::size_t i = 0; i < values.size(); ++i) {
      const std::string& key = Cr::Utility::formatString("{:.03d}", i);
      subCfg->set(key, values[i]);
    }
  }  // setSubconfigValsOfTypeInVector

  // ==================== load from and save to json =========================

  /**
   * @brief Load values into this Configuration from the passed @p jsonObj.
   * Will recurse for subConfigurations.
   * @param jsonObj The JSON object to read from for the data for this
   * Configuration.
   * @return The number of fields successfully read and populated.
   */
  int loadFromJson(const io::JsonGenericValue& jsonObj);

  /**
   * @brief Build and return a json object holding the values and nested
   * objects holding the subconfigs of this Configuration.
   */
  io::JsonGenericValue writeToJsonObject(io::JsonAllocator& allocator) const;

  /**
   * @brief Populate a json object with all the first-level values held in
   * this Configuration.  May be overridden to handle special cases for
   * root-level Configuration of Attributes classes derived from
   * Configuration.
   */
  virtual void writeValuesToJson(io::JsonGenericValue& jsonObj,
                                 io::JsonAllocator& allocator) const;

  /**
   * @brief Populate a json object with all the data from the
   * subConfigurations, held in json sub-objects, for this Configuration.
   */
  virtual void writeSubconfigsToJson(io::JsonGenericValue& jsonObj,
                                     io::JsonAllocator& allocator) const;

  /**
   * @brief Take the passed @p key and query the config value for that key,
   * writing it to @p jsonName within the passed @p jsonObj.
   * @param key The key of the data in the Configuration
   * @param jsonName The tag to use in the json file
   * @param jsonObj The json object to write to
   * @param allocator The json allocator to use to build the json object
   */
  void writeValueToJson(const char* key,
                        const char* jsonName,
                        io::JsonGenericValue& jsonObj,
                        io::JsonAllocator& allocator) const {
    auto cfgVal = get(key);
    if (cfgVal.shouldWriteToFile()) {
      writeValueToJsonInternal(cfgVal, jsonName, jsonObj, allocator);
    }
  }

  /**
   * @brief Take the passed @p key and query the config value for that key,
   * writing it to tag with @p key as name within the passed @p jsonObj.
   * @param key The key of the data in the Configuration
   * @param jsonObj The json object to write to
   * @param allocator The json allocator to use to build the json object
   */
  void writeValueToJson(const char* key,
                        io::JsonGenericValue& jsonObj,
                        io::JsonAllocator& allocator) const {
    auto cfgVal = get(key);
    if (cfgVal.shouldWriteToFile()) {
      writeValueToJsonInternal(cfgVal, key, jsonObj, allocator);
    }
  }

  /**
   * @brief Return all the values in this cfg in a formatted string.
   * Subconfigs will be displaced by a tab.
   * @param newLineStr The string to put at the end of each newline. As
   * subconfigs are called, add a tab to this.
   */
  std::string getAllValsAsString(const std::string& newLineStr = "\n") const;

  /**
   * @brief Find the subConfiguration with the given handle and rekey it such
   * that all the value entries it contains are keyed by sequential numeric
   * strings. These keys will preserve the order of the original keys.
   *
   * NOTE : all values regardless of type in this subconfig will be rekeyed.
   * This function is primarily intended to be used on subconfigs holding a
   * collection of the same type of date, to facilitate accessing this data as a
   * sorted list.
   *
   * @param subconfigKey The key of this Configuration's subconfig whose values'
   * keys we wish to have rekeyed.
   * @return the number of values whose keys have been changed.
   */
  int rekeySubconfigValues(const std::string& subconfigKey);

  /**
   * @brief Rekey all the value entries in this Configuration such that all
   * their keys are sequential numeric strings that presever the order of their
   * original strings.
   */
  int rekeyAllValues();

  /**
   * @brief Clear all key-value pairs from this Configuration's valueMap
   */
  void _clearAllValues() { valueMap_.clear(); }

 protected:
  /**
   * @brief Friend function.  Checks if passed @p key is contained in @p
   * config. Returns the highest level where @p key was found
   * @param config The Configuration to search for passed key
   * @param key The key to look for
   * @param parentLevel The parent level to the current iteration.  If
   * iteration finds @p key, it will return parentLevel+1
   * @param breadcrumb [out] List of keys to subconfigs to get to value.
   * Always should end with @p key.
   * @return The level @p key was found. 0 if not found (so can be treated
   * as bool)
   */
  static int findValueInternal(const Configuration& config,
                               const std::string& key,
                               int parentLevel,
                               std::vector<std::string>& breadcrumb);

  /**
   * @brief Populate the passed cfg with all the values this map holds, along
   * with the values any subgroups/sub-Configs it may hold
   */
  void putAllValuesInConfigGroup(Cr::Utility::ConfigurationGroup& cfg) const {
    // put ConfigVal values in map
    for (const auto& entry : valueMap_) {
      entry.second.putValueInConfigGroup(entry.first, cfg);
    }

    for (const auto& subConfig : configMap_) {
      const auto name = subConfig.first;
      auto* cfgSubGroup = cfg.addGroup(name);
      subConfig.second->putAllValuesInConfigGroup(*cfgSubGroup);
    }
  }

  /**
   * @brief Retrieves named subgroup; if no subgroup with given name exists this
   * will make one.
   * @param name Name of desired existing or new subgroup.
   * @return The resultant pair after attempting to emplace a Configuration at
   * the requested location given by @p name. Consists of an iterator to the
   * Configuration, and a boolean value that denotes whether this is a new
   * Configuration or it existed already.
   */
  template <typename T>
  std::pair<ConfigMapType::iterator, bool> addOrEditSubgroup(
      const std::string& name) {
    static_assert(std::is_base_of<Configuration, T>::value,
                  "Configuration : Desired subconfig must be derived from "
                  "core::config::Configuration");
    // Attempt to insert an empty pointer
    auto result = configMap_.emplace(name, std::shared_ptr<T>{});
    // If name not already present (insert succeeded) then add new
    // Configuration
    if (result.second) {
      result.first->second = std::make_shared<T>();
    }
    return result;
  }

 private:
  /**
   * @brief Write the passed @p configValue to a json object tagged with
   * @p jsonName within the passed @p jsonObj . By here we have already verified
   * that this object should be written to Json (i.e. checks for being hidden or
   * only programmatically initialized have occurred already).
   * @param configValue The @ref ConfigValue we want to write to the json file.
   * @param jsonName The tag to use in the json file
   * @param jsonObj The json object to write to
   * @param allocator The json allocator to use to build the json object
   */
  void writeValueToJsonInternal(const ConfigValue& configValue,
                                const char* jsonName,
                                io::JsonGenericValue& jsonObj,
                                io::JsonAllocator& allocator) const;

  /**
   * @brief Process passed json object into this Configuration, using passed
   * key.
   *
   * @param numVals number of values/configs loaded so far
   * @param key key to use to search @p jsonObj and also to set value or
   * subconfig within this Configuration.
   * @return the number of total fields successfully loaded after this
   * function executes.
   */
  int loadOneConfigFromJson(int numVals,
                            const std::string& key,
                            const io::JsonGenericValue& jsonObj);

  /**
   * @brief Process passed json array into this Configuration.
   *
   * @param jsonObj The json object being treated as an array
   * @return the number of elements loaded into this configuration from the
   * source json array.
   */
  int loadFromJsonArray(const io::JsonGenericValue& jsonObj);

  /**
   * @brief Map to hold Configurations as subgroups
   */
  ConfigMapType configMap_{};

  /**
   * @brief Map that holds all config values
   */
  ValueMapType valueMap_{};

 public:
  /**
   * @brief Comparison - Ignores ConfigValues specified as hidden
   */
  friend bool operator==(const Configuration& a, const Configuration& b);
  /**
   * @brief Inequality Comparison - Ignores ConfigValues specified as hidden
   */
  friend bool operator!=(const Configuration& a, const Configuration& b);

  ESP_SMART_POINTERS(Configuration)
};  // class Configuration

/**
 * @brief provide debug stream support for a @ref Configuration
 */
Mn::Debug& operator<<(Mn::Debug& debug, const Configuration& value);

template <>
std::vector<float> Configuration::getSubconfigValsOfTypeInVector(
    const std::string& subCfgName) const;

template <>
void Configuration::setSubconfigValsOfTypeInVector(
    const std::string& subCfgName,
    const std::vector<float>& values);

/**
 * @brief Retrieves a shared pointer to a copy of the subConfig @ref
 * esp::core::config::Configuration that has the passed @p name . This will
 * create a pointer to a new sub-Configuration if none exists already with
 * that name, but will not add this Configuration to this Configuration's
 * internal storage.
 *
 * @param name The name of the Configuration to retrieve.
 * @return A pointer to a copy of the Configuration having the requested
 * name, or a pointer to an empty Configuration.
 */

template <>
std::shared_ptr<Configuration> Configuration::getSubconfigCopy<Configuration>(
    const std::string& name) const;
/**
 * @brief Retrieve a shared pointer to the actual subConfiguration given by @p
 * name, or a new subConfiguration with that name, if none exists.
 *
 * @param name The name of the desired subConfiguration
 * @return A pointer to the Configuration having the requested
 * name, or a pointer to an empty Configuration.
 */
template <>
std::shared_ptr<Configuration> Configuration::editSubconfig<Configuration>(
    const std::string& name);

/**
 * @brief Save the passed @ref Configuration pointed to by @p configPtr at location specified by @p name
 * @param name The name to save the subConfiguration by
 * @param configPtr A pointer to the @ref Configuration to save with the given @p name .
 */
template <>
void Configuration::setSubconfigPtr<Configuration>(
    const std::string& name,
    std::shared_ptr<Configuration>& configPtr);

}  // namespace config
}  // namespace core
}  // namespace esp

#endif  // ESP_CORE_CONFIGURATION_H_
