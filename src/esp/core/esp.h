// Copyright (c) Facebook, Inc. and its affiliates.
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

// Eigen has an enum that clashes with X11 Success define
#ifdef Success
#undef Success
#endif
#include <Eigen/Core>
#include <Eigen/Geometry>
// #include <Eigen/StdVector>

#include "esp/core/configure.h"
#include "esp/core/logging.h"
#include "esp/core/spimpl.h"

namespace Eigen {
typedef Matrix<uint8_t, 3, 1> Vector3uc;
typedef Matrix<uint32_t, 3, 1> Vector3ui;
typedef Matrix<uint8_t, 4, 1> Vector4uc;
typedef Matrix<uint32_t, 4, 1> Vector4ui;
typedef Matrix<uint64_t, 4, 1> Vector4ul;

typedef Matrix<float, Dynamic, Dynamic, RowMajor> RowMatrixXf;

//! Eigen JSON string format specification
static const IOFormat kJsonFormat(StreamPrecision,
                                  DontAlignCols,
                                  ",",   // coef separator
                                  ",",   // row separator
                                  "",    // row prefix
                                  "",    // col prefix
                                  "[",   // mat prefix
                                  "]");  // mat suffix

//! Write Eigen matrix types into ostream in JSON string format
template <typename T, int numRows, int numCols>
std::ostream& operator<<(std::ostream& os,
                         const Matrix<T, numRows, numCols>& matrix) {
  return os << matrix.format(kJsonFormat);
}

//! Write Eigen map into ostream in JSON string format
template <typename T>
std::ostream& operator<<(std::ostream& os, const Map<T>& m) {
  return os << m.format(kJsonFormat);
}

}  // namespace Eigen

// EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector2f)
// EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector3f)
// EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector4f)
// EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector2d)
// EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector3d)
// EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector4d)
// EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector2i)
// EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector3i)
// EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector4i)
// EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3f)
// EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix4f)
// EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3d)
// EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix4d)
// EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector4uc)

//! core simulator namespace
namespace esp {

// basic types
typedef Eigen::Vector2f vec2f;
typedef Eigen::Vector3f vec3f;
typedef Eigen::Vector4f vec4f;
typedef Eigen::Vector2d vec2d;
typedef Eigen::Vector3d vec3d;
typedef Eigen::Vector4d vec4d;
typedef Eigen::Vector2i vec2i;
typedef Eigen::Vector3i vec3i;
typedef Eigen::Vector4i vec4i;
typedef Eigen::Matrix3f mat3f;
typedef Eigen::Matrix4f mat4f;
typedef Eigen::Matrix3d mat3d;
typedef Eigen::Matrix4d mat4d;
typedef Eigen::Quaternionf quatf;
typedef Eigen::Vector3uc vec3uc;
typedef Eigen::Vector3ui vec3ui;
typedef Eigen::Vector4uc vec4uc;
typedef Eigen::Vector4ui vec4ui;
typedef Eigen::Vector4i vec4i;
typedef Eigen::Vector4ul vec4ul;
typedef Eigen::VectorXi vecXi;
typedef Eigen::AlignedBox3f box3f;

//! Write box3f into ostream in JSON string format
inline std::ostream& operator<<(std::ostream& os, const box3f& bbox) {
  return os << "{min:" << bbox.min() << ",max:" << bbox.max() << "}";
}

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
