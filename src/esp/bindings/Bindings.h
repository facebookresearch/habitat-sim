// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_BINDINGS_BINDINGS_H_
#define ESP_BINDINGS_BINDINGS_H_

#include <Magnum/SceneGraph/PythonBindings.h>
#include <pybind11/pybind11.h>
#include "esp/bindings/OpaqueTypes.h"

namespace esp {

namespace core {

/**
 * @brief Specify core bindings
 */
void initCoreBindings(pybind11::module& m);
namespace config {
/**
 * @brief Specify bindings for @ref esp::core::config::Configuration
 */
void initConfigBindings(pybind11::module& m);
}  // namespace config
}  // namespace core

namespace geo {
/**
 * @brief Specify bindings for @ref esp::geo::OBB and @ref esp::geo::Ray
 */
void initGeoBindings(pybind11::module& m);
}  // namespace geo

namespace gfx {

/**
 * @brief Create pybind class for RenderCamera, and partially define bindings.
 * Bindings should be completed in @ref initGfxBindings, once dependent
 * bindings have been created (i.e. SceneNode)
 */
pybind11::class_<RenderCamera,
                 Magnum::SceneGraph::PyFeature<RenderCamera>,
                 Magnum::SceneGraph::Camera3D,
                 Magnum::SceneGraph::PyFeatureHolder<RenderCamera>>
createRenderCameraBind(pybind11::module& m);

class Renderer;

/**
 * @brief Create pybind class for Renderer, and partially define bindings.
 * Bindings should be completed in @ref initGfxBindings, once dependent
 * bindings have been created (i.e. SceneNode)
 */
pybind11::class_<esp::gfx::Renderer, std::shared_ptr<Renderer>>
createRendererBind(pybind11::module& m);

/**
 * @brief Finalize Renderer bindings definitions after sim bindings class
 * defined.
 */
void finalInitRenderer(
    pybind11::class_<Renderer, std::shared_ptr<Renderer>>& renderer);

/**
 * @brief Specify bindings for RenderTarget. Done separately so that it can be
 * performed before Sensor bindings are defined, which depend on it.
 */
void initRenderTargetBind(pybind11::module& m);

/**
 * @brief Specify bindings for constructs in esp::gfx namespace
 */
void initGfxBindings(
    pybind11::module& m,
    pybind11::class_<RenderCamera,
                     Magnum::SceneGraph::PyFeature<RenderCamera>,
                     Magnum::SceneGraph::Camera3D,
                     Magnum::SceneGraph::PyFeatureHolder<RenderCamera>>&
        renderCamera);

namespace replay {
/**
 * @brief Specify bindings for constructs in esp::gfx::replay namespace
 */
void initGfxReplayBindings(pybind11::module& m);
}  // namespace replay
}  // namespace gfx

namespace metadata {
/**
 * @brief Specify bindings for attributes in esp::metadata::attributes namespace
 */
void initAttributesBindings(pybind11::module& m);
/**
 * @brief Specify bindings for @ref esp::metadata::MetadataMediator
 */
void initMetadataMediatorBindings(pybind11::module& m);

namespace managers {
/**
 * @brief Specify bindings for attributes in esp::metadata::managers namespace
 */
void initAttributesManagersBindings(pybind11::module& m);
}  // namespace managers
}  // namespace metadata

namespace nav {
/**
 * @brief Specify bindings for @ref esp::nav::HitRecord , @ref esp::nav::ShortestPath ,
 * @ref esp::nav::MultiGoalShortestPath, @ref esp::nav::NavMeshSettings, @ref esp::nav::PathFinder, and
 * @ref esp::nav::GreedyGeodesicFollowerImpl
 */
void initShortestPathBindings(pybind11::module& m);
}  // namespace nav

namespace physics {
/**
 * @brief Specify bindings for @ref esp::physics::VelocityControl ,
 * @ref esp::physics::JointMotorSettings ,
 * @ref esp::physics::RigidConstraintSettings ,
 * @ref esp::physics::RayHitInfo ,
 * @ref esp::physics::RaycastResults ,
 * @ref esp::physics::ContactPointData ,
 * and @ref esp::physics::CollisionGroupHelper
 */
void initPhysicsBindings(pybind11::module& m);

/**
 * @brief Specify bindings for the various esp::physics wrapper objects.
 */
void initPhysicsObjectBindings(pybind11::module& m);

/**
 * @brief Specify bindings for the various esp::physics wrapper object managers.
 */
void initPhysicsWrapperManagerBindings(pybind11::module& m);
}  // namespace physics

namespace scene {

pybind11::class_<
    esp::scene::SceneNode,
    Magnum::SceneGraph::PyObject<esp::scene::SceneNode>,
    Magnum::SceneGraph::Object<
        Magnum::SceneGraph::BasicTranslationRotationScalingTransformation3D<
            float>>,
    Magnum::SceneGraph::PyObjectHolder<esp::scene::SceneNode>>
createSceneNodeBind(pybind11::module& m);

/**
 * @brief Specify bindings for @ref esp::scene::SceneNode , @ref esp::scene::SceneGraph ,
 * @ref esp::scene::SceneManager , @ref esp::scene::SemanticCategory ,
 * @ref esp::scene::Mp3dObjectCategory , @ref esp::scene::Mp3dRegionCategory
 * @ref esp::scene::SemanticObject , @ref esp::scene::SemanticRegion ,
 * @ref esp::scene::SemanticLevel , @ref esp::scene::SemanticScene
 */
void initSceneBindings(
    pybind11::module& m,
    pybind11::class_<
        esp::scene::SceneNode,
        Magnum::SceneGraph::PyObject<esp::scene::SceneNode>,
        Magnum::SceneGraph::Object<
            Magnum::SceneGraph::BasicTranslationRotationScalingTransformation3D<
                float>>,
        Magnum::SceneGraph::PyObjectHolder<esp::scene::SceneNode>>&
        pySceneNode);
}  // namespace scene

namespace sensor {
/**
 * @brief Specify bindings for various esp::sensor classes.
 */
void initSensorBindings(pybind11::module& m);
}  // namespace sensor

namespace sim {
/**
 * @brief Specify bindings for @ref esp::sim::Simulator and @ref esp::sim::AbstractReplayRenderer ,
 */
void initSimBindings(pybind11::module& m);
/**
 * @brief Specify bindings for @ref esp::sim::SimulatorConfiguration and
 * @ref esp::sim::ReplayRendererConfiguration
 */
void initSimConfigBindings(pybind11::module& m);
void initRenderInstanceHelperBindings(pybind11::module& m);

}  // namespace sim

}  // namespace esp

#endif  // ESP_BINDINGS_BINDINGS_H_
