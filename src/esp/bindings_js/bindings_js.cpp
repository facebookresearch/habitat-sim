// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <emscripten/bind.h>

namespace em = emscripten;

#include "esp/gfx/Simulator.h"

using namespace esp;
using namespace esp::core;
using namespace esp::geo;
using namespace esp::gfx;
using namespace esp::nav;
using namespace esp::scene;
using namespace esp::sensor;

SceneConfiguration* SimulatorConfiguration_getScene(SimulatorConfiguration& config) 
  { return &config.scene; }

EMSCRIPTEN_BINDINGS(habitat_sim_bindings_js) {

  em::class_<SceneConfiguration>("SceneConfiguration")
      .smart_ptr_constructor("SceneConfiguration",
                             &SceneConfiguration::create<>)
      .property("dataset", &SceneConfiguration::dataset)
      .property("id", &SceneConfiguration::id)
      .property("filepaths", &SceneConfiguration::filepaths)
      .property("scene_up_dir", &SceneConfiguration::sceneUpDir)
      .property("scene_front_dir", &SceneConfiguration::sceneFrontDir)
      .property("scene_scale_unit", &SceneConfiguration::sceneScaleUnit);

  em::class_<SimulatorConfiguration>("SimulatorConfiguration")
      .smart_ptr_constructor("SimulatorConfiguration",
                             &SimulatorConfiguration::create<>)
      .function("getScene", &SimulatorConfiguration_getScene, em::allow_raw_pointers())
      .property("scene", &SimulatorConfiguration::scene)
      .property("default_agent_id", &SimulatorConfiguration::defaultAgentId)
      .property("default_camera_uuid",
                &SimulatorConfiguration::defaultCameraUuid)
      .property("gpu_device_id", &SimulatorConfiguration::gpuDeviceId)
      .property("width", &SimulatorConfiguration::width)
      .property("height", &SimulatorConfiguration::height)
      .property("compress_textures", &SimulatorConfiguration::compressTextures);

  em::class_<Simulator>("Simulator")
      .smart_ptr_constructor("Simulator",
                             &Simulator::create<const SimulatorConfiguration&>)
      // .function("get_active_scene_graph", &Simulator::getActiveSceneGraph)
      // .function("get_active_semantic_scene_graph", &Simulator::getActiveSemanticSceneGraph)
      // .property("semantic_scene", [] (Simulator& self) { return &self.getSemanticScene(); } )
      // .property("renderer", &Simulator::getRenderer)
         .function("seed", &Simulator::seed)
      // .function("reconfigure", &Simulator::reconfigure)
         .function("reset", &Simulator::reset);
      ;
}
