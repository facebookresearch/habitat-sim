// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_ATTRIBUTES_OBJECTATTRIBUTES_H_
#define ESP_METADATA_ATTRIBUTES_OBJECTATTRIBUTES_H_

#include "AbstractObjectAttributes.h"

namespace esp {
namespace metadata {
namespace attributes {

/**
 * @brief Specific Attributes instance describing a rigid object, constructed
 * with a default set of object-specific required attributes.
 */
class ObjectAttributes : public AbstractObjectAttributes {
 public:
  explicit ObjectAttributes(const std::string& handle = "");
  // center of mass (COM)
  void setCOM(const Magnum::Vector3& com) { set("COM", com); }
  Magnum::Vector3 getCOM() const { return get<Magnum::Vector3>("COM"); }

  // whether com is provided or not
  void setComputeCOMFromShape(bool computeCOMFromShape) {
    set("compute_COM_from_shape", computeCOMFromShape);
  }
  bool getComputeCOMFromShape() const {
    return get<bool>("compute_COM_from_shape");
  }

  void setMass(double mass) { set("mass", mass); }
  double getMass() const { return get<double>("mass"); }

  // inertia diagonal
  void setInertia(const Magnum::Vector3& inertia) { set("inertia", inertia); }
  Magnum::Vector3 getInertia() const { return get<Magnum::Vector3>("inertia"); }

  void setLinearDamping(double linearDamping) {
    set("linear_damping", linearDamping);
  }
  double getLinearDamping() const { return get<double>("linear_damping"); }

  void setAngularDamping(double angularDamping) {
    set("angular_damping", angularDamping);
  }
  double getAngularDamping() const { return get<double>("angular_damping"); }

  // if true override other settings and use render mesh bounding box as
  // collision object
  void setBoundingBoxCollisions(bool useBoundingBoxForCollision) {
    set("use_bounding_box_for_collision", useBoundingBoxForCollision);
  }
  bool getBoundingBoxCollisions() const {
    return get<bool>("use_bounding_box_for_collision");
  }

  // if true join all mesh components of an asset into a unified collision
  // object
  void setJoinCollisionMeshes(bool joinCollisionMeshes) {
    set("join_collision_meshes", joinCollisionMeshes);
  }
  bool getJoinCollisionMeshes() const {
    return get<bool>("join_collision_meshes");
  }

  void setSemanticId(int semanticId) { set("semantic_id", semanticId); }

  uint32_t getSemanticId() const { return get<int>("semantic_id"); }

 protected:
  /**
   * @brief Write object-specific values to json object
   */
  void writeValuesToJsonInternal(io::JsonGenericValue& jsonObj,
                                 io::JsonAllocator& allocator) const override;

  /**
   * @brief get AbstractObject specific info header
   */
  std::string getAbstractObjectInfoHeaderInternal() const override {
    return "Mass,COM XYZ,I XX YY ZZ,Angular Damping,"
           "Linear Damping,Semantic ID";
  }
  /**
   * @brief get AbstractObject specific info for csv string
   */
  std::string getAbstractObjectInfoInternal() const override;

 public:
  ESP_SMART_POINTERS(ObjectAttributes)

};  // class ObjectAttributes

}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_OBJECTATTRIBUTES_H_
