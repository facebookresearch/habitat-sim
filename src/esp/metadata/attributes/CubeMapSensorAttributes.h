
// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_ATTRIBUTES_ABSTRACTCUBEMAPSENSORATTRIBUTES_H_
#define ESP_METADATA_ATTRIBUTES_ABSTRACTCUBEMAPSENSORATTRIBUTES_H_

#include "AbstractVisualSensorAttributes.h"

namespace esp {
namespace metadata {
namespace attributes {
class AbstractCubeMapSensorAttributes : public AbstractVisualSensorAttributes {
 public:
  /**
   * @brief Class to suppoort creating CubeMap-based sensors by providing common
   * attributes.
   */
  AbstractCubeMapSensorAttributes(const std::string& classKey,
                                  const std::string& handle);

  /**
   * @brief Set whether to use a user-specified CubeMap size. Otherwise, should
   * be the smallest of the x or y resolution values.
   */
  void setUseSpecifiedCubeMapSize(bool _useCubeMapSize) {
    setHidden("__useSpecifiedCubeMapSize", _useCubeMapSize);
  }
  /**
   * @brief Get whether to use a user-specified CubeMap size. Otherwise, should
   * be the smallest of the x or y resolution values.
   */
  bool getUseSpecifiedCubeMapSize() const {
    return get<bool>("__useSpecifiedCubeMapSize");
  }

  /**
   * @brief Set user-specified CubeMap Size. -1 means ignore this field and use
   * the max dimension of the resolution as the size
   */
  void setCubeMapSize(int cubemap_size) {
    set("cubemap_size", cubemap_size);
    setUseSpecifiedCubeMapSize(true);
  }

  /**
   * @brief Get user-specified CubeMap Size. -1 means ignore this field and use
   * the max dimension of the resolution as the size
   */
  int getCubeMapSize() const { return get<int>("cubemap_size"); }

 protected:
  /**
   * @brief Write CubeMap Sensor-specific values to json object
   */
  void writeVisualSensorValuesToJsonInternal(
      io::JsonGenericValue& jsonObj,
      io::JsonAllocator& allocator) const override;

  virtual void writeCubeMapSensorValuesToJsonInternal(
      CORRADE_UNUSED io::JsonGenericValue& jsonObj,
      CORRADE_UNUSED io::JsonAllocator& allocator) const {};
  /**
   * @brief get AbstractCubeMapSensorAttributes-specific info header
   */
  std::string getAbstractVisualSensorInfoHeaderInternal() const override;

  /**
   * @brief get AbstractCubeMapSensorAttributes specific info for csv string
   */
  std::string getAbstractVisualSensorInfoInternal() const override;

  /**
   * @brief get AbstractCubeMapSensorAttributes-child class info header
   */
  virtual std::string getCubeMapSensorInfoHeaderInternal() const { return ""; }

  /**
   * @brief get AbstractCubeMapSensorAttributes-child class info for csv string
   */
  virtual std::string getCubeMapSensorInfoInternal() const { return ""; };

 public:
  ESP_SMART_POINTERS(AbstractCubeMapSensorAttributes)

};  // class AbstractCubeMapSensorAttributes

// AbstractCubeMapSensorAttributes Child classes

/**
 * @brief Class holding attributes for CubeMap-based Equirectangular sensor
 */
class EquirectangularSensorAttributes : public AbstractCubeMapSensorAttributes {
 public:
  explicit EquirectangularSensorAttributes(const std::string& handle = "");

 protected:
  /**
   * @brief Write EquirectangularSensorAttributes Sensor-specific values to json
   * object
   */
  void writeCubeMapSensorValuesToJsonInternal(
      io::JsonGenericValue& jsonObj,
      io::JsonAllocator& allocator) const override;

  /**
   * @brief get EquirectangularSensorAttributes-specific info header
   */
  std::string getCubeMapSensorInfoHeaderInternal() const override;

  /**
   * @brief get EquirectangularSensorAttributes specific info for csv string
   */
  std::string getCubeMapSensorInfoInternal() const override;

 public:
  ESP_SMART_POINTERS(EquirectangularSensorAttributes)
};

/**
 * @brief Class holding attributes for CubeMap-based Fisheye sensor
 */
class FisheyeSensorAttributes : public AbstractCubeMapSensorAttributes {
 public:
  explicit FisheyeSensorAttributes(const std::string& handle = "");

  /**
   * @brief Set the fisheye sensor focal length, fx, fy, the distance between
   * the pinhole and the image plane. In practice, fx and fy can differ for a
   * number of reasons. See details here:
   * http://ksimek.github.io/2013/08/13/intrinsic/
   * Both values must be positive to be legal
   */
  void setFocalLength(const Magnum::Vector2& focal_length) {
    set("focal_length", focal_length);
  }

  /**
   * @brief Get the fisheye sensor focal length, fx, fy, the distance between
   * the pinhole and the image plane. In practice, fx and fy can differ for a
   * number of reasons. See details here:
   * http://ksimek.github.io/2013/08/13/intrinsic/
   * Both values must be positive to be legal
   */
  Magnum::Vector2 getFocalLength() const {
    return get<Magnum::Vector2>("focal_length");
  }

  /**
   * @brief Set whether to use specified Principal Point Offset. If false, the
   * Principle Point Offset will be placed in the middle of the image.
   */
  void setUsePrincipalPointOffset(bool use_specified_ppo) {
    setHidden("__useSpecifiedPPO", use_specified_ppo);
  }
  /**
   * @brief Get whether to use specified Principal Point Offset. If false, the
   * Principle Point Offset will be placed in the middle of the image.
   */
  bool getUsePrincipalPointOffset() const {
    return get<bool>("__useSpecifiedPPO");
  }

  /**
   * @brief Set the Principal Point Offset in pixel, cx, cy, location of the
   * principal point relative to the image plane's origin.
   */
  void setPrincipalPointOffset(const Magnum::Vector2& principle_point_offset) {
    set<Magnum::Vector2>("principle_point_offset", principle_point_offset);
    setUsePrincipalPointOffset(true);
  }

  /**
   * @brief Get the Principal Point Offset in pixel, cx, cy, location of the
   * principal point relative to the image plane's origin. If not specified use
   * center of image.
   */
  Magnum::Vector2 getPrincipalPointOffset() const {
    if (getUsePrincipalPointOffset()) {
      return get<Magnum::Vector2>("principle_point_offset");
    }
    // If not specified then should be center of given resolution
    return Magnum::Vector2(getResolution()) * 0.5;
  }

  /**
   * @brief Set the alpha value specifiec to the "double sphere" fisheye camera
   * model. see details (value ranges) in: Vladyslav Usenko, Nikolaus Demmel and
   * Daniel Cremers: The Double Sphere Camera Model, The International
   * Conference on 3D Vision (3DV), 2018
   */
  void setDoubleSphereAlpha(float ds_alpha) { set("ds_alpha", ds_alpha); }

  /**
   * @brief Get the alpha value specifiec to the "double sphere" fisheye camera
   * model. see details (value ranges) in: Vladyslav Usenko, Nikolaus Demmel and
   * Daniel Cremers: The Double Sphere Camera Model, The International
   * Conference on 3D Vision (3DV), 2018
   */
  float getDoubleSphereAlpha() const {
    return static_cast<float>(get<double>("ds_alpha"));
  }

  /**
   * @brief Set the xi value specifiec to the "double sphere" fisheye camera
   * model. see details (value ranges) in: Vladyslav Usenko, Nikolaus Demmel and
   * Daniel Cremers: The Double Sphere Camera Model, The International
   * Conference on 3D Vision (3DV), 2018
   */
  void setDoubleSphereXi(float ds_xi) { set("ds_xi", ds_xi); }

  /**
   * @brief Get the xi value specifiec to the "double sphere" fisheye camera
   * model. see details (value ranges) in: Vladyslav Usenko, Nikolaus Demmel and
   * Daniel Cremers: The Double Sphere Camera Model, The International
   * Conference on 3D Vision (3DV), 2018
   */
  float getDoubleSphereXi() const {
    return static_cast<float>(get<double>("ds_xi"));
  }

 protected:
  /**
   * @brief Write FisheyeSensorAttributes Sensor-specific values to json
   * object
   */
  void writeCubeMapSensorValuesToJsonInternal(
      io::JsonGenericValue& jsonObj,
      io::JsonAllocator& allocator) const override;

  /**
   * @brief get FisheyeSensorAttributes-specific info header
   */
  std::string getCubeMapSensorInfoHeaderInternal() const override;

  /**
   * @brief get FisheyeSensorAttributes specific info for csv string
   */
  std::string getCubeMapSensorInfoInternal() const override;

 public:
  ESP_SMART_POINTERS(FisheyeSensorAttributes)
};

}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_ABSTRACTCUBEMAPSENSORATTRIBUTES_H_