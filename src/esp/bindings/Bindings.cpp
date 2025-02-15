// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/Bindings.h"

#include "esp/assets/ResourceManager.h"
#include "esp/core/Check.h"
#include "esp/core/Configuration.h"
#include "esp/core/Esp.h"
#include "esp/core/RigidState.h"
#include "esp/gfx/Renderer.h"

namespace py = pybind11;
using py::literals::operator""_a;

PYBIND11_MODULE(habitat_sim_bindings, m) {
  m.attr("cuda_enabled") =
#ifdef ESP_BUILD_WITH_CUDA
      true;
#else
      false;
#endif

  m.attr("built_with_bullet") =
#ifdef ESP_BUILD_WITH_BULLET
      true;
#else
      false;
#endif

  m.attr("audio_enabled") =
#ifdef ESP_BUILD_WITH_AUDIO
      true;
#else
      false;
#endif
  m.attr("stage_id") = esp::RIGID_STAGE_ID;

  /* This function pointer is used by ESP_CHECK(). If it's null, it
     std::abort()s, if not, it calls it to cause a Python AssertionError */
  esp::core::throwInPython = [](const char* const message) {
    PyErr_SetString(PyExc_AssertionError, message);
    throw pybind11::error_already_set{};
  };

  py::module_::import("magnum.scenegraph");

  py::bind_map<std::map<std::string, std::string>>(m, "MapStringString");

  esp::core::config::initConfigBindings(m);
  esp::core::initCoreBindings(m);
  esp::geo::initGeoBindings(m);

  // To address circular references, we build certain class binding before
  // other bindings that reference it, and then complete its definition that
  // includes references to those classes.
  auto pySceneNode = esp::scene::createSceneNodeBind(m);
  auto pyRenderCamera = esp::gfx::createRenderCameraBind(m);
  auto pyRenderer = esp::gfx::createRendererBind(m);
  esp::gfx::initRenderTargetBind(m);
  // Sensor depends on SceneNode, RenderCamera and RenderTarget bindings
  esp::sensor::initSensorBindings(m);
  esp::gfx::initGfxBindings(m, pyRenderCamera);

  esp::gfx::replay::initGfxReplayBindings(m);
  // We pass the created scene node class binding to the initialization function
  // to complete its definition
  esp::scene::initSceneBindings(m, pySceneNode);
  esp::nav::initShortestPathBindings(m);
  esp::sim::initSimConfigBindings(m);
  esp::sim::initRenderInstanceHelperBindings(m);
  esp::metadata::initAttributesBindings(m);
  esp::metadata::managers::initAttributesManagersBindings(m);
  esp::metadata::initMetadataMediatorBindings(m);
  // These depend on SceneNode bindings
  esp::physics::initPhysicsBindings(m);
  esp::physics::initPhysicsObjectBindings(m);
  esp::physics::initPhysicsWrapperManagerBindings(m);
  esp::sim::initSimBindings(m);
  // Renderer relies on simulator class bindings
  esp::gfx::finalInitRenderer(pyRenderer);
}
