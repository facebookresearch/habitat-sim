// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/bindings.h"

#include "esp/assets/ResourceManager.h"
#include "esp/core//random.h"
#include "esp/core/Check.h"
#include "esp/core/Configuration.h"
#include "esp/core/RigidState.h"

namespace py = pybind11;
using py::literals::operator""_a;

namespace esp {

void initEspBindings(py::module& m) {
  // ==== box3f ====
  py::class_<box3f>(m, "BBox")
      .def_property_readonly("sizes", &box3f::sizes)
      .def_property_readonly("center", &box3f::center);
#ifdef ESP_BUILD_WITH_VHACD
  py::class_<assets::ResourceManager::VHACDParameters,
             assets::ResourceManager::VHACDParameters::ptr>(m,
                                                            "VHACDParameters")
      .def(py::init(&assets::ResourceManager::VHACDParameters::create<>))
      .def_readwrite(
          "resolution", &assets::ResourceManager::VHACDParameters::m_resolution,
          R"(Maximum number of voxels generated during the voxelization stage (default=100,000, range=10,000-16,000,000).)")
      .def_readwrite(
          "max_num_vertices_per_ch",
          &assets::ResourceManager::VHACDParameters::m_maxNumVerticesPerCH,
          R"(Controls the maximum number of triangles per convex-hull (default=64, range=4-1024).)")
      .def_readwrite(
          "max_convex_hulls",
          &assets::ResourceManager::VHACDParameters::m_maxConvexHulls,
          R"(Maximum number of convex hulls to produce.)")
      .def_readwrite(
          "concavity", &assets::ResourceManager::VHACDParameters::m_concavity,
          R"(Maximum allowed concavity (default=0.0025, range=0.0-1.0).)")
      .def_readwrite(
          "plane_downsampling",
          &assets::ResourceManager::VHACDParameters::m_planeDownsampling,
          R"(Controls the granularity of the search for the \"best\" clipping plane (default=4, range=1-16).)")
      .def_readwrite(
          "convex_hull_downsampling",
          &assets::ResourceManager::VHACDParameters::m_convexhullDownsampling,
          R"(Controls the precision of the convex-hull generation process during the clipping plane selection stage (default=4, range=1-16).)")
      .def_readwrite(
          "alpha", &assets::ResourceManager::VHACDParameters::m_alpha,
          R"(Controls the bias toward clipping along symmetry planes (default=0.05, range=0.0-1.0).)")
      .def_readwrite(
          "beta", &assets::ResourceManager::VHACDParameters::m_beta,
          R"(Controls the bias toward clipping along revolution axes (default=0.05, range=0.0-1.0).)")
      .def_readwrite(
          "pca", &assets::ResourceManager::VHACDParameters::m_pca,
          R"(Enable/disable normalizing the mesh before applying the convex decomposition (default=0, range={0,1}).)")
      .def_readwrite(
          "mode", &assets::ResourceManager::VHACDParameters::m_mode,
          R"(0: voxel-based approximate convex decomposition, 1: tetrahedron-based approximate convex decomposition (default=0, range={0,1}.)")
      .def_readwrite(
          "min_volume_per_ch",
          &assets::ResourceManager::VHACDParameters::m_minVolumePerCH,
          R"(Controls the adaptive sampling of the generated convex-hulls (default=0.0001, range=0.0-0.01).)")
      .def_readwrite(
          "convex_hull_approximation",
          &assets::ResourceManager::VHACDParameters::m_convexhullApproximation,
          R"(Enable/disable approximation when computing convex-hulls (default=1, range={0,1}.)");
#endif
}

namespace core {

void initCoreBindings(py::module& m) {
  py::class_<Configuration, Configuration::ptr>(m, "ConfigurationGroup")
      .def(py::init(&Configuration::create<>))
      .def("get_bool", &Configuration::getBool)
      .def("get_string", &Configuration::getString)
      .def("get_int", &Configuration::getInt)
      .def("get_double", &Configuration::getDouble)
      .def("get_vec3", &Configuration::getVec3)
      .def("get", &Configuration::getString)
      .def("set", &Configuration::set<std::string>)
      .def("set", &Configuration::set<int>)
      .def("set", &Configuration::set<double>)
      .def("set", &Configuration::set<bool>)
      .def("set", &Configuration::set<Magnum::Vector3>)
      .def("add_string_to_group", &Configuration::addStringToGroup)
      .def("get_string_group", &Configuration::getStringGroup)
      .def("has_value", &Configuration::hasValue)
      .def("remove_value", &Configuration::removeValue);

  // ==== struct RigidState ===
  py::class_<RigidState, RigidState::ptr>(m, "RigidState")
      .def(py::init(&RigidState::create<>))
      .def(py::init(&RigidState::create<const Magnum::Quaternion&,
                                        const Magnum::Vector3&>))
      .def_readwrite("rotation", &RigidState::rotation)
      .def_readwrite("translation", &RigidState::translation);

  py::class_<Random, Random::ptr>(m, "Random")
      .def(py::init(&Random::create<>))
      .def("seed", &Random::seed)
      .def("uniform_float_01", &Random::uniform_float_01)
      .def("uniform_float", &Random::uniform_float)
      .def("uniform_int", py::overload_cast<>(&Random::uniform_int))
      .def("uniform_int", py::overload_cast<int, int>(&Random::uniform_int))
      .def("uniform_uint", &Random::uniform_uint)
      .def("normal_float_01", &Random::normal_float_01);
}

}  // namespace core
}  // namespace esp

PYBIND11_MODULE(habitat_sim_bindings, m) {
  m.attr("cuda_enabled") =
#ifdef ESP_BUILD_WITH_CUDA
      true;
#else
      false;
#endif

  m.attr("vhacd_enabled") =
#ifdef ESP_BUILD_WITH_VHACD
      true;
#else
      false;
#endif

  /* This function pointer is used by ESP_CHECK(). If it's null, it
     std::abort()s, if not, it calls it to cause a Python AssertionError */
  esp::core::throwInPython = [](const char* const message) {
    PyErr_SetString(PyExc_AssertionError, message);
    throw pybind11::error_already_set{};
  };

  m.import("magnum.scenegraph");

  py::bind_map<std::map<std::string, std::string>>(m, "MapStringString");

  // NOTE(msb) These need to be run in dependency order.
  // TODO(msb) gfx, scene, and sensor should not cross-depend
  // TODO(msb) sim and sensor should not cross-depend
  esp::initEspBindings(m);
  esp::core::initCoreBindings(m);
  esp::metadata::initAttributesBindings(m);
  esp::metadata::initMetadataMediatorBindings(m);
  esp::metadata::managers::initAttributesManagersBindings(m);
  esp::geo::initGeoBindings(m);
  esp::physics::initPhysicsBindings(m);
  esp::scene::initSceneBindings(m);
  esp::gfx::initGfxBindings(m);
  esp::gfx::replay::initGfxReplayBindings(m);
  esp::sensor::initSensorBindings(m);
  esp::nav::initShortestPathBindings(m);
  esp::sim::initSimBindings(m);
}
