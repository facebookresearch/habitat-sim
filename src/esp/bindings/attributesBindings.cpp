// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/bindings.h"

#include <Magnum/Magnum.h>
#include <Magnum/Python.h>

#include "esp/assets/Attributes.h"

namespace py = pybind11;
using py::literals::operator""_a;

namespace esp {
namespace assets {

void initAttributesBindings(py::module& m) {
  // ==== AbstractPhysicsAttributes ====
  py::class_<AbstractPhysicsAttributes, esp::core::Configuration,
             AbstractPhysicsAttributes::ptr>(m, "AbstractPhysicsAttributes")
      .def(py::init(&AbstractPhysicsAttributes::create<>))
      .def(py::init(&AbstractPhysicsAttributes::create<const std::string&>));

  // ==== PhysicsObjectAttributes ====
  py::class_<PhysicsObjectAttributes, AbstractPhysicsAttributes,
             PhysicsObjectAttributes::ptr>(m, "PhysicsObjectAttributes")
      .def(py::init(&PhysicsObjectAttributes::create<>))
      .def(py::init(&PhysicsObjectAttributes::create<const std::string&>))
      .def("set_com", &PhysicsObjectAttributes::setCOM, "com"_a)
      .def("get_com", &PhysicsObjectAttributes::getCOM)
      .def("set_margin", &PhysicsObjectAttributes::setMargin, "margin"_a)
      .def("get_margin", &PhysicsObjectAttributes::getMargin)
      .def("set_mass", &PhysicsObjectAttributes::setMass, "mass"_a)
      .def("get_mass", &PhysicsObjectAttributes::getMass)
      .def("set_inertia", &PhysicsObjectAttributes::setInertia, "inertia"_a)
      .def("get_inertia", &PhysicsObjectAttributes::getInertia)
      .def("set_scale", &PhysicsObjectAttributes::setScale, "scale"_a)
      .def("get_scale", &PhysicsObjectAttributes::getScale)
      .def("set_friction_coefficient",
           &PhysicsObjectAttributes::setFrictionCoefficient,
           "friction_coefficient"_a)
      .def("get_friction_coefficient",
           &PhysicsObjectAttributes::getFrictionCoefficient)
      .def("set_restitution_coefficient",
           &PhysicsObjectAttributes::setRestitutionCoefficient,
           "restitution_coefficient"_a)
      .def("get_restitution_coefficient",
           &PhysicsObjectAttributes::getRestitutionCoefficient)
      .def("set_linear_damping", &PhysicsObjectAttributes::setLinearDamping,
           "linear_damping"_a)
      .def("get_linear_damping", &PhysicsObjectAttributes::getLinearDamping)
      .def("set_angular_damping", &PhysicsObjectAttributes::setAngularDamping,
           "angular_damping"_a)
      .def("get_angular_damping", &PhysicsObjectAttributes::getAngularDamping)
      .def("set_origin_handle", &PhysicsObjectAttributes::setOriginHandle,
           "origin_handle"_a)
      .def("get_origin_handle", &PhysicsObjectAttributes::getOriginHandle)
      .def("set_render_asset_handle",
           &PhysicsObjectAttributes::setRenderAssetHandle,
           "render_asset_handle"_a)
      .def("get_render_asset_handle",
           &PhysicsObjectAttributes::getRenderAssetHandle)
      .def("set_collision_asset_handle",
           &PhysicsObjectAttributes::setCollisionAssetHandle,
           "collision_asset_handle"_a)
      .def("get_collision_asset_handle",
           &PhysicsObjectAttributes::getCollisionAssetHandle)
      .def("set_bounding_box_collisions",
           &PhysicsObjectAttributes::setBoundingBoxCollisions,
           "use_bounding_box_for_collision"_a)
      .def("get_bounding_box_collisions",
           &PhysicsObjectAttributes::getBoundingBoxCollisions)
      .def("set_join_collision_meshes",
           &PhysicsObjectAttributes::setJoinCollisionMeshes,
           "join_collision_meshes"_a)
      .def("get_join_collision_meshes",
           &PhysicsObjectAttributes::getJoinCollisionMeshes)
      .def("set_requires_lighting",
           &PhysicsObjectAttributes::setRequiresLighting, "requires_lighting"_a)
      .def("get_requires_lighting",
           &PhysicsObjectAttributes::getRequiresLighting);
}

}  // namespace assets
}  // namespace esp
