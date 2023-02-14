// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_CORE_ESP_H_
#define ESP_CORE_ESP_H_

/** @file */

#include <map>
#include <memory>
#include <ostream>
#include <tuple>
#include <vector>

#include "esp/core/Logging.h"
#include "esp/core/Spimpl.h"
#include "esp/core/configure.h"

//! core simulator namespace
namespace esp {

// smart pointers macro
#define ESP_SMART_POINTERS(...)                                         \
 public:                                                                \
  typedef std::shared_ptr<__VA_ARGS__> ptr;                             \
  typedef std::unique_ptr<__VA_ARGS__> uptr;                            \
  typedef std::shared_ptr<const __VA_ARGS__> cptr;                      \
  typedef std::unique_ptr<const __VA_ARGS__> ucptr;                     \
  template <typename... Targs>                                          \
  static inline ptr create(Targs&&... args) {                           \
    return std::make_shared<__VA_ARGS__>(std::forward<Targs>(args)...); \
  }                                                                     \
  template <typename... Targs>                                          \
  static inline uptr create_unique(Targs&&... args) {                   \
    return std::make_unique<__VA_ARGS__>(std::forward<Targs>(args)...); \
  }

/**
 * shim function helper for derived class of std::enable_shared_from_this<Base>
 */
template <typename Base>
inline std::shared_ptr<Base> shared_from_base(
    std::enable_shared_from_this<Base>* base) {
  return base->shared_from_this();
}

/**
 * shared_from_this access for inheriting classes of Base classes that inherit
 * std::enable_shared_from_this<Base>
 */
template <typename Derived>
inline std::shared_ptr<Derived> shared_from(Derived* derived) {
  return std::static_pointer_cast<Derived>(shared_from_base(derived));
}

// pimpl macro backed by unique_ptr pointer
#define ESP_UNIQUE_PTR_PIMPL() \
 protected:                    \
  struct Impl;                 \
  spimpl::unique_impl_ptr<Impl> pimpl_;

// pimpl macro backed by shared_ptr pointer
#define ESP_SHARED_PTR_PIMPL() \
 protected:                    \
  struct Impl;                 \
  spimpl::impl_ptr<Impl> pimpl_;

// convenience macros with combined smart pointers and pimpl members
#define ESP_SMART_POINTERS_WITH_UNIQUE_PIMPL(T) \
  ESP_SMART_POINTERS(T)                         \
  ESP_UNIQUE_PTR_PIMPL()
#define ESP_SMART_POINTERS_WITH_SHARED_PIMPL(T) \
  ESP_SMART_POINTERS(T)                         \
  ESP_SHARED_PTR_PIMPL()

/** @brief Returned on failed creation or lookup of an ID. */
constexpr int ID_UNDEFINED = -1;

/** @brief Undefined or invalid attribute in physics property query. */
constexpr double PHYSICS_ATTR_UNDEFINED = -1.0;

static const double NO_TIME = 0.0;

/**
 * @brief The @ref esp::gfx::ShaderManager key for @ref esp::gfx::LightInfo
 * which has no lights
 */
constexpr char NO_LIGHT_KEY[] = "no_lights";

/**
 *@brief The @ref esp::gfx::ShaderManager key for the default @ref
 *esp::gfx::LightInfo
 */
constexpr char DEFAULT_LIGHTING_KEY[] = "";

/**
 *@brief The @ref esp::gfx::ShaderManager key for the default @ref
 *esp::gfx::MaterialInfo
 */
constexpr char DEFAULT_MATERIAL_KEY[] = "";

/**
 *@brief The @ref esp::gfx::ShaderManager key for full ambient white @ref
 *esp::gfx::MaterialInfo used for primitive wire-meshes
 */
constexpr char WHITE_MATERIAL_KEY[] = "ambient_white";

/**
 *@brief The @ref ShaderManager key for @ref MaterialInfo with per-vertex
 * object ID
 */
constexpr char PER_VERTEX_OBJECT_ID_MATERIAL_KEY[] = "per_vertex_object_id";

template <typename T>
inline bool equal(const std::vector<std::shared_ptr<T>>& a,
                  const std::vector<std::shared_ptr<T>>& b) {
  return a.size() == b.size() &&
         std::equal(
             a.begin(), a.end(), b.begin(),
             [](const std::shared_ptr<T>& v1,
                const std::shared_ptr<T>& v2) -> bool { return *v1 == *v2; });
}

// NB: This logic ONLY works on std::map as the keys are ordered
// Same logic will NOT work for std::unordered_map
template <typename K, typename V>
inline bool equal(const std::map<K, std::shared_ptr<V>>& a,
                  const std::map<K, std::shared_ptr<V>>& b) {
  return a.size() == b.size() &&
         std::equal(a.begin(), a.end(), b.begin(),
                    [](const auto& p1, const auto& p2) -> bool {
                      return p1.first == p2.first &&
                             ((*p1.second) == (*p2.second));
                    });
}

}  // namespace esp

#endif  // ESP_CORE_ESP_H_
