
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
   * @brief Populate this AbstractCubeMapSensorAttributes from an appropriate @ref sensor::SensorSpec.
   * @todo Remove when SensorSpecs are removed
   *
   */
  void populateWithSensorSpec(const sensor::SensorSpec::ptr& spec) override;

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
   * @brief Set user-specified CubeMap Size. 0 means ignore this field and use
   * the min dimension of the resolution as the size
   */
  void setCubeMapSize(int cubemap_size) {
    set("cubemap_size", cubemap_size);
    setUseSpecifiedCubeMapSize(true);
  }

  void clearCubeMapSize() {
    set("cubemap_size", 0);
    setUseSpecifiedCubeMapSize(false);
  }

  /**
   * @brief Get user-specified CubeMap Size. 0 means ignore this field and use
   * the max dimension of the resolution as the size
   */
  int getCubeMapSize() const { return get<int>("cubemap_size"); }

  /**
   * @brief Get the actual cube map size to use when constructing the cube map -
   * either the user's specified value or the minimum resolution dimension. This
   * should not be saved, and is just offered as a convenience accessor.
   */
  int getCubeMapSizeToUse() const {
    if (getUseSpecifiedCubeMapSize()) {
      return get<int>("cubemap_size");
    }
    // If no cubemap size is specified, use the minimum resolution dimension
    return getResolution().min();
  }

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
  /**
   * @brief Populate this EquirectangularSensorAttributes from an appropriate @ref sensor::SensorSpec.
   * @todo Remove when SensorSpecs are removed
   *
   */
  void populateWithSensorSpec(const sensor::SensorSpec::ptr& spec) override;

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
};  // class EquirectangularSensorAttributes

/**
 * @brief Class holding attributes for CubeMap-based Fisheye sensor
 */
class FisheyeSensorAttributes : public AbstractCubeMapSensorAttributes {
 public:
  explicit FisheyeSensorAttributes(const std::string& handle = "");
  /**
   * @brief Populate this FisheyeSensorAttributes from an appropriate @ref sensor::SensorSpec.
   * @todo Remove when SensorSpecs are removed
   *
   */
  void populateWithSensorSpec(const sensor::SensorSpec::ptr& spec) override;
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
    // TODO both values should always be > 0
    set<Magnum::Vector2>("principle_point_offset", principle_point_offset);
    setUsePrincipalPointOffset(true);
  }

  /**
   * @brief Clear out any specified Principal Point Offsets that might have been
   * set.
   */
  void clearPrincipalPointOffset() {
    set<Magnum::Vector2>("principle_point_offset", Magnum::Vector2(0.0, 0.0));
    setUsePrincipalPointOffset(false);
  }
  /**
   * @brief Get the Principal Point Offset - the pixel, cx, cy, location of the
   * principal point relative to the image plane's origin. If not specified use
   * center of the sensor image.
   */
  Magnum::Vector2 getPrincipalPointOffset() const {
    return get<Magnum::Vector2>("principle_point_offset");
  }
  /**
   * @brief Get the Actual Principal Point Offset to use; pixel, cx, cy,
   * location of the principal point relative to the image plane's origin. This
   * value is what should be used to create the sensor - if not specified, use
   * the halfway point of the resolution specified (i.e. the center of the
   * sensor image).
   */
  Magnum::Vector2 getPrincipalPointOffsetToUse() const {
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
};  // class FisheyeSensorAttributes

}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_ABSTRACTCUBEMAPSENSORATTRIBUTES_H_
