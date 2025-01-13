// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/Bindings.h"

#include <Magnum/Magnum.h>
#include <Magnum/SceneGraph/SceneGraph.h>

#include <Magnum/PythonBindings.h>
#include <Magnum/SceneGraph/PythonBindings.h>
#include "esp/scene/Mp3dSemanticScene.h"
#include "esp/scene/SceneGraph.h"
#include "esp/scene/SceneManager.h"
#include "esp/scene/SceneNode.h"
#include "esp/scene/SemanticScene.h"
namespace py = pybind11;
using py::literals::operator""_a;

namespace Mn = Magnum;
namespace esp {
namespace scene {

py::class_<esp::scene::SceneNode,
           Magnum::SceneGraph::PyObject<esp::scene::SceneNode>,
           Magnum::SceneGraph::Object<
               Magnum::SceneGraph::
                   BasicTranslationRotationScalingTransformation3D<float>>,
           Magnum::SceneGraph::PyObjectHolder<esp::scene::SceneNode>>
createSceneNodeBind(py::module& m) {
  // ==== SceneNode ====
  py::class_<esp::scene::SceneNode,
             Magnum::SceneGraph::PyObject<esp::scene::SceneNode>,
             Magnum::SceneGraph::Object<
                 Magnum::SceneGraph::
                     BasicTranslationRotationScalingTransformation3D<float>>,
             Magnum::SceneGraph::PyObjectHolder<esp::scene::SceneNode>>
      pySceneNode(m, "SceneNode", R"(
      SceneNode: a node in the scene graph.

      Cannot apply a smart pointer to a SceneNode object.
      You can "create it and forget it".
      Simulator backend will handle the memory.)");

  py::class_<SceneGraph>(m, "SceneGraph")
      .def(py::init())
      .def("get_root_node",
           py::overload_cast<>(&SceneGraph::getRootNode, py::const_),
           R"(
            Get the root node of the scene graph.

            User can specify transformation of the root node w.r.t. the world
            frame. (const function) PYTHON DOES NOT GET OWNERSHIP)",
           py::return_value_policy::reference)
      .def("get_root_node", py::overload_cast<>(&SceneGraph::getRootNode),
           R"(
            Get the root node of the scene graph.

            User can specify transformation of the root node w.r.t. the world
            frame. PYTHON DOES NOT GET OWNERSHIP)",
           py::return_value_policy::reference);

  return pySceneNode;
}  // createSceneNodeBind

void initSceneBindings(
    py::module& m,
    py::class_<esp::scene::SceneNode,
               Magnum::SceneGraph::PyObject<esp::scene::SceneNode>,
               Magnum::SceneGraph::Object<
                   Magnum::SceneGraph::
                       BasicTranslationRotationScalingTransformation3D<float>>,
               Magnum::SceneGraph::PyObjectHolder<esp::scene::SceneNode>>&
        pySceneNode) {
  // ==== SceneGraph ====

  // !!Warning!!
  // CANNOT apply smart pointers to "SceneNode" or ANY its descendant classes,
  // namely, any class whose instance can be a node in the scene graph. Reason:
  // Memory will be automatically handled in simulator (C++ backend). Using
  // smart pointers on scene graph node from Python code, will claim the
  // ownership and eventually free its resources, which leads to "duplicated
  // deallocation", and thus memory corruption.

  // ==== enum SceneNodeType ====
  py::enum_<SceneNodeType>(m, "SceneNodeType")
      .value("EMPTY", SceneNodeType::Empty)
      .value("SENSOR", SceneNodeType::Sensor)
      .value("AGENT", SceneNodeType::Agent)
      .value("CAMERA", SceneNodeType::Camera)
      .value("OBJECT", SceneNodeType::Object);

  pySceneNode
      .def(py::init_alias<std::reference_wrapper<SceneNode>>(),
           R"(Constructor: creates a scene node, and sets its parent.)")
      .def_property("type", &SceneNode::getType, &SceneNode::setType)
      .def_property("semantic_id", &SceneNode::getSemanticId,
                    &SceneNode::setSemanticId)
      .def_property(
          "object_semantic_id", &SceneNode::getBaseObjectId,
          &SceneNode::setBaseObjectId,
          R"(This node's owning object's ID, for instance-based semantics)")
      .def_property(
          "drawable_semantic_id", &SceneNode::getDrawableId,
          &SceneNode::setDrawableId,
          R"(This node's drawable's ID, for instance-based semantics)")
      .def(
          "create_child", [](SceneNode& self) { return &self.createChild(); },
          R"(Creates a child node, and sets its parent to the current node.)")
      .def("set_parent", &SceneNode::setParent,
           R"(Sets parent to parentNode, and updates ancestors' SensorSuites)")
      .def(
          "compute_cumulative_bb", &SceneNode::computeCumulativeBB,
          R"(Recursively compute the approximate axis aligned bounding boxes of the SceneGraph sub-tree rooted at this node.)")
      .def_property_readonly(
          "cumulative_bb", &SceneNode::getCumulativeBB,
          R"(The approximate axis aligned bounding box of the SceneGraph sub-tree rooted at this node.)")
      .def_property_readonly(
          "mesh_bb", &SceneNode::getMeshBB,
          R"(The axis aligned bounding box of the mesh drawables attached to this node.)")
      .def_property_readonly(
          "absolute_translation",
          py::overload_cast<>(&SceneNode::absoluteTranslation))
      .def_property_readonly(
          "absolute_translation",
          py::overload_cast<>(&SceneNode::absoluteTranslation, py::const_))
      .def_property_readonly("node_sensor_suite",
                             &SceneNode::getNodeSensorSuite,
                             R"(Get node SensorSuite of this SceneNode)")
      .def_property_readonly("subtree_sensor_suite",
                             &SceneNode::getSubtreeSensorSuite,
                             R"(Get subtree SensorSuite of this SceneNode)")
      .def_property_readonly("node_sensors", &SceneNode::getNodeSensors,
                             R"(Get node sensors of this SceneNode)")
      .def_property_readonly("subtree_sensors", &SceneNode::getSubtreeSensors,
                             R"(Get subtree sensors of this SceneNode)");

  // ==== SceneManager ====
  py::class_<SceneManager>(m, "SceneManager")
      .def("init_scene_graph", &SceneManager::initSceneGraph,
           R"(
          Initialize a new scene graph, and return its ID.)")
      .def("get_scene_graph",
           py::overload_cast<int>(&SceneManager::getSceneGraph),
           R"(
             Get the scene graph by scene graph ID.

             PYTHON DOES NOT GET OWNERSHIP)",
           "sceneGraphID"_a, py::return_value_policy::reference)
      .def("get_scene_graph",
           py::overload_cast<int>(&SceneManager::getSceneGraph, py::const_),
           R"(
             Get the scene graph by scene graph ID.

             PYTHON DOES NOT GET OWNERSHIP)",
           "sceneGraphID"_a, py::return_value_policy::reference);

  // ==== SemanticCategory ====
  py::class_<SemanticCategory, SemanticCategory::ptr>(m, "SemanticCategory")
      .def("index", &SemanticCategory::index, "mapping"_a = "")
      .def("name", &SemanticCategory::name, "mapping"_a = "");

  // === Mp3dObjectCategory ===
  py::class_<Mp3dObjectCategory, SemanticCategory, Mp3dObjectCategory::ptr>(
      m, "Mp3dObjectCategory")
      .def("index", &Mp3dObjectCategory::index, "mapping"_a = "")
      .def("name", &Mp3dObjectCategory::name, "mapping"_a = "");

  // === Mp3dRegionCategory ===
  py::class_<Mp3dRegionCategory, SemanticCategory, Mp3dRegionCategory::ptr>(
      m, "Mp3dRegionCategory")
      .def("index", &Mp3dRegionCategory::index, "mapping"_a = "")
      .def("name", &Mp3dRegionCategory::name, "mapping"_a = "");

  // === Polyloop-based Semantic Region Category ===
  py::class_<LoopRegionCategory, SemanticCategory, LoopRegionCategory::ptr>(
      m, "LoopRegionCategory")
      .def("index", &LoopRegionCategory::index, "mapping"_a = "")
      .def("name", &LoopRegionCategory::name, "mapping"_a = "");

  // These are (cyclically) referenced by multiple classes below, define
  // the classes first so pybind has the type definition available when binding
  // functions
  py::class_<SemanticObject, SemanticObject::ptr> semanticObject(
      m, "SemanticObject");
  py::class_<CCSemanticObject, SemanticObject, CCSemanticObject::ptr>
      ccSemanticObject(
          m, "CCSemanticObject",
          "This class exists to facilitate semantic object data access for "
          "bboxes derived from connected component analysis.");
  py::class_<SemanticRegion, SemanticRegion::ptr> semanticRegion(
      m, "SemanticRegion");

  // ==== SemanticLevel ====
  py::class_<SemanticLevel, SemanticLevel::ptr>(m, "SemanticLevel")
      .def_property_readonly("id", &SemanticLevel::id)
      .def_property_readonly("aabb", &SemanticLevel::aabb)
      .def_property_readonly("regions", &SemanticLevel::regions)
      .def_property_readonly("objects", &SemanticLevel::objects);

  // ==== SemanticRegion ====
  semanticRegion
      .def_property_readonly(
          "id", &SemanticRegion::id,
          "The ID of the region, either as the region's unique name, or of the "
          "form ``<level_id>_<region_id>``")
      .def_property_readonly("level", &SemanticRegion::level)
      .def_property_readonly("aabb", &SemanticRegion::aabb)
      .def_property_readonly("category", &SemanticRegion::category,
                             "The semantic category of the region")
      .def_property_readonly("objects", &SemanticRegion::objects,
                             "All objects in the region")
      .def_property_readonly("poly_loop_points",
                             &SemanticRegion::getPolyLoopPoints,
                             "The points making up the polyloop for this "
                             "region, coplanar and parallel to the floor.")
      .def_property_readonly(
          "volume_edges", &SemanticRegion::getVisEdges,
          "The edges, as pairs of points, that determine "
          "the boundaries of the region. For visualizations.")
      .def_property_readonly("floor_height", &SemanticRegion::getFloorHeight,
                             "The height above the x-z plane for the floor of "
                             "the semantic region.")
      .def_property_readonly("extrusion_height",
                             &SemanticRegion::getExtrusionHeight,
                             "The height of the extrusion above the floor.")
      .def("contains", &SemanticRegion::contains, "point"_a,
           "Check whether the given point is contained in the given region.");

  // ==== SemanticObject ====
  semanticObject
      .def_property_readonly("id", &SemanticObject::id,
                             "The ID of the object, of the form "
                             "``<level_id>_<region_id>_<object_id>``")
      .def_property_readonly("semantic_id", &SemanticObject::semanticID)
      .def_property_readonly("region", &SemanticObject::region)
      .def_property_readonly("aabb", &SemanticObject::aabb)
      .def_property_readonly("obb", &SemanticObject::obb)
      .def_property_readonly("category", &SemanticObject::category);

  // ==== CCSemanticObject =====
  ccSemanticObject
      .def_property_readonly("num_src_verts", &CCSemanticObject::getNumSrcVerts,
                             "The number of vertices in the connected "
                             "component making up this semantic object.")
      .def_property_readonly("vert_set", &CCSemanticObject::getVertSet,
                             "A set of the vertices in the connected component "
                             "making up this semantic object.");

  // ==== SemanticScene ====
  py::class_<SemanticScene, SemanticScene::ptr>(m, "SemanticScene")
      .def(py::init(&SemanticScene::create<>))
      .def_static(
          "load_mp3d_house",
          [](const std::string& filename, SemanticScene& scene,
             const Mn::Vector4& rotation) {
            // numpy doesn't have a quaternion equivalent, use vec4
            // instead
            return SemanticScene::loadMp3dHouse(filename, scene,
                                                {rotation.xyz(), rotation.w()});
          },
          R"(
        Loads a SemanticScene from a Matterport3D House format file into passed
        `SemanticScene`.
      )",
          "file"_a, "scene"_a, "rotation"_a)
      .def_property_readonly("aabb", &SemanticScene::aabb)
      .def_property_readonly("categories", &SemanticScene::categories,
                             "All semantic categories in the scene")
      .def_property_readonly("levels", &SemanticScene::levels,
                             "All levels in the scene")
      .def_property_readonly("regions", &SemanticScene::regions,
                             "All regions in the scene")
      .def_property_readonly("objects", &SemanticScene::objects,
                             "All object in the scene")
      .def_property_readonly("semantic_index_map",
                             &SemanticScene::getSemanticIndexMap)
      .def("semantic_index_to_object_index",
           &SemanticScene::semanticIndexToObjectIndex)
      .def("get_regions_for_point", &SemanticScene::getRegionsForPoint,
           R"(Compute all SemanticRegions which contain the point and return a
             list of indices for the regions in this SemanticScene.)",
           "point"_a)
      .def("get_weighted_regions_for_point",
           &SemanticScene::getWeightedRegionsForPoint,
           R"("Find all SemanticRegions which contain the point and return a
              sorted list of tuple pairs of the region index and a score of that
              region, derived as 1 - (region_area/ttl_region_area) where ttl_region_area is the area of all the regions containing
              the point, so that smaller regions are weighted higher. If only
              one region contains the passed point, its weight will be 1.)",
           "point"_a)
      .def("get_regions_for_points", &SemanticScene::getRegionsForPoints,
           R"("Compute SemanticRegion containment for a set of points. Return a
              sorted list of tuple pairs with each containing region index and
              the percentage of points contained by that region. In the case of nested
              regions, points are considered belonging to every region the point is
              found in.)",
           "points"_a);
}

}  // namespace scene
}  // namespace esp
