// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/OpaqueTypes.h"

namespace py = pybind11;
using namespace py::literals;

#include "esp/core/Configuration.h"
#include "esp/geo/OBB.h"
#include "esp/gfx/RenderCamera.h"
#include "esp/gfx/Renderer.h"
#include "esp/gfx/Simulator.h"
#include "esp/nav/PathFinder.h"
#include "esp/scene/AttachedObject.h"
#include "esp/scene/Mp3dSemanticScene.h"
#include "esp/scene/ObjectControls.h"
#include "esp/scene/SceneGraph.h"
#include "esp/scene/SceneNode.h"
#include "esp/scene/SemanticScene.h"
#include "esp/scene/SuncgSemanticScene.h"
#include "esp/sensor/PinholeCamera.h"
#include "esp/sensor/Sensor.h"

using namespace esp;
using namespace esp::core;
using namespace esp::geo;
using namespace esp::gfx;
using namespace esp::nav;
using namespace esp::scene;
using namespace esp::sensor;

void initShortestPathBindings(py::module& m);
void initGeoBindings(py::module& m);

PYBIND11_MODULE(habitat_sim_bindings, m) {
  initGeoBindings(m);

  py::bind_map<std::map<std::string, std::string>>(m, "MapStringString");

  py::class_<Configuration, Configuration::ptr>(m, "Configuration")
      .def(py::init(&Configuration::create<>))
      .def("getBool", &Configuration::getBool)
      .def("getString", &Configuration::getString)
      .def("getInt", &Configuration::getInt)
      .def("getFloat", &Configuration::getFloat)
      .def("get", &Configuration::getString)
      .def("set", &Configuration::set<std::string>)
      .def("set", &Configuration::set<int>)
      .def("set", &Configuration::set<float>)
      .def("set", &Configuration::set<bool>);

  // !!Warning!!
  // CANNOT apply smart pointers to "SceneNode" or ANY its descendant classes,
  // namely, any class whose instance can be a node in the scene graph. Reason:
  // Memory will be automatically handled in simulator (C++ backend). Using
  // smart pointers on scene graph node from Python code, will claim the
  // ownership and eventually free its resources, which leads to "duplicated
  // deallocation", and thus memory corruption.

  // ==== SceneNode ====
  py::class_<scene::SceneNode>(m, "SceneNode", R"(
      SceneNode: a node in the scene graph.
      Cannot apply a smart pointer to a SceneNode object.
      You can "create it and forget it".
      Simulator backend will handle the memory.)")
      .def(py::init<scene::SceneNode&>(),
           R"(Constructor: creates a scene node, and sets its parent.
           PYTHON DOES NOT GET OWNERSHIP)",
           pybind11::return_value_policy::reference)
      .def("set_parent", &scene::SceneNode::setParent, R"(
        Sets the parent scene node
      )")
      .def("create_child", &scene::SceneNode::createChild,
           R"(Creates a child node, and sets its parent to the current node.
        PYTHON DOES NOT GET OWNERSHIP)",
           pybind11::return_value_policy::reference)
      .def("transformation", &SceneNode::getTransformation, R"(
        Returns transformation relative to parent :py:class:`SceneNode`
      )")
      .def("absolute_transformation", &SceneNode::getAbsoluteTransformation, R"(
        Returns absolute transformation matrix (relative to root :py:class:`SceneNode`)
      )")
      .def(
          "rotation",
          [](const SceneNode& self) { return self.getRotation().coeffs(); },
          R"()")
      .def("rotation",
           [](SceneNode& self, const Eigen::Ref<const esp::vec4f> coeffs) {
             self.setRotation(Eigen::Map<const quatf>(coeffs.data()));
           })
      .def("absolute_position", &SceneNode::getAbsolutePosition, R"(
        Returns absolute position (relative to root :py:class:`SceneNode`)
      )")
      // .def("absolute_orientation_z", &SceneNode::absoluteOrientationZ, R"(
      //   Returns absolute Z-axis orientation (relative to root
      //   :py:class:`SceneNode`)
      // )")
      .def("set_transformation",
           py::overload_cast<const Eigen::Ref<const mat4f>>(
               &SceneNode::setTransformation),
           R"(
        Set transformation relative to parent :py:class:`SceneNode`
      )",
           "transformation"_a)
      .def("set_transformation",
           py::overload_cast<const Eigen::Ref<const vec3f>,
                             const Eigen::Ref<const vec3f>,
                             const Eigen::Ref<const vec3f>>(
               &SceneNode::setTransformation),
           R"(
        Set this :py:class:`SceneNode`'s matrix to look at target from given position
        and orientation towards target. Semantics is same as gluLookAt.
      )",
           "position"_a, "target"_a, "up"_a)
      .def("set_translation", &SceneNode::setTranslation, R"(
        Sets translation relative to parent :py:class:`SceneNode` to given translation
      )",
           "vector"_a)
      .def("reset_transformation", &SceneNode::resetTransformation, R"(
        Reset transform relative to parent :py:class:`SceneNode` to identity
      )")
      // .def("transform", &SceneNode::transform, R"(
      //   Transform relative to parent :py:class:`SceneNode` by given transform
      // )",
      //      "transformation"_a)
      // .def("transformLocal", &SceneNode::transform, R"(
      //   Transform relative to parent :py:class:`SceneNode` by given transform
      //   in local frame. Applies given transform before any other
      //   transformation.
      // )",
      //      "transformation"_a)
      .def("translate", &SceneNode::translate, R"(
        Translate relative to parent :py:class:`SceneNode` by given translation
      )",
           "vector"_a)
      .def("translate_local", &SceneNode::translateLocal, R"(
        Translate relative to parent :py:class:`SceneNode` by given translation
        in local frame. Applies given translation before any other translation.
      )",
           "vector"_a)
      .def("rotate", &SceneNode::rotate, R"(
        Rotate relative to parent :py:class:`SceneNode` by given angleInRad
        radians around given normalizedAxis.
      )",
           "angleInRad"_a, "normalizedAxis"_a)
      .def("rotate_local", &SceneNode::rotateLocal, R"(
        Rotate relative to parent :py:class:`SceneNode` by given angleInRad
        radians around given normalizedAxis in local frame. Applies given
        rotation before any other rotation.
      )",
           "angleInRad"_a, "normalizedAxis"_a)
      .def("rotateX", &SceneNode::rotateX, R"(
        Rotate arounx X axis relative to parent :py:class:`SceneNode` by
        in local frame. Applies given rotation before any other rotation
      )",
           "angleInRad"_a)
      .def("rotateX_local", &SceneNode::rotateXLocal, R"(
        Rotate relative to parent :py:class:`SceneNode` by given angleInRad
        radians around X axis in local frame. Applies given rotation before
        any other rotation.
      )",
           "angleInRad"_a)
      .def("rotateY", &SceneNode::rotateY, R"(
        Rotate arounx Y axis relative to parent :py:class:`SceneNode` by
        in local frame. Applies given rotation before any other rotation
      )",
           "angleInRad"_a)
      .def("rotateY_local", &SceneNode::rotateYLocal, R"(
        Rotate relative to parent :py:class:`SceneNode` by given angleInRad
        radians around X axis in local frame. Applies given rotation before
        any other rotation.
      )",
           "angleInRad"_a)
      .def("rotateZ", &SceneNode::rotateZ, R"(
        Rotate arounx Y axis relative to parent :py:class:`SceneNode` by
        in local frame. Applies given rotation before any other rotation
      )",
           "angleInRad"_a)
      .def("rotateZ_local", &SceneNode::rotateZLocal, R"(
        Rotate relative to parent :py:class:`SceneNode` by given angleInRad
        radians around X axis in local frame. Applies given rotation before
        any other rotation.
      )",
           "angleInRad"_a)
      .def("normalize", [](SceneNode& self) {
        self.setRotation(self.getRotation().normalized());
      });

  // ==== enum AttachedObjectType ====
  py::enum_<AttachedObjectType>(m, "AttachedObjectType")
      .value("NONE", AttachedObjectType::NONE)
      .value("SENSOR", AttachedObjectType::SENSOR)
      .value("AGENT", AttachedObjectType::AGENT)
      .value("CAMERA", AttachedObjectType::CAMERA);

  // ==== AttachedObject ====
  // An object that is attached to a scene node.
  // Such object can be Agent, Sensor, Camera etc.
  py::class_<AttachedObject, AttachedObject::ptr>(m, "AttachedObject",
                                                  R"(
      AttachedObject: An object that is attached to a scene node.
      Such object can be Agent, Sensor, Camera etc.
      )")
      // create a new attached object, which is NOT attached to any scene node
      // the initial status is invalid, namely, isValid will return false
      .def(py::init(&AttachedObject::create<>))
      // input: the scene node this object is going to attach to
      // the initial status is valid, since the object is attached to a node
      .def(py::init(&AttachedObject::create<scene::SceneNode&>))

      // NOTE:
      // please always call this function to check the status
      // in order to avoid runtime errors
      .def_property_readonly(
          "is_valid", &AttachedObject::isValid,
          R"(Returns true if the object is being attached to a scene node.)")
      .def("attach", &AttachedObject::attach,
           R"(Attaches the object to an existing scene node.)", "sceneNode"_a)
      .def("detach", &AttachedObject::detach,
           R"(Detached the object and the scene node)")
      .def("get_scene_node", &AttachedObject::getSceneNode,
           R"(get the scene node (pointer) being attached to (can be nullptr)
              PYTHON DOES NOT GET OWNERSHIP)",
           pybind11::return_value_policy::reference)
      .def_property("object_type", &AttachedObject::getObjectType,
                    &AttachedObject::setObjectType)

      // ---- functions related to rigid body transformation ----
      // Please check "isValid" before using it.
      .def("get_transformation", &AttachedObject::getTransformation, R"(
        If it is valid, returns transformation relative to parent scene node.
      )")
      .def("get_absolute_transformation",
           &AttachedObject::getAbsoluteTransformation, R"(
        If it is valid, returns absolute transformation matrix
      )")
      .def("get_absolute_position", &AttachedObject::getAbsolutePosition, R"(
        If it is valid, returns absolute position w.r.t. world coordinate frame
      )")
      .def(
          "get_rotation",
          [](const AttachedObject& self) {
            return self.getRotation().coeffs();
          },
          R"()")
      .def("set_rotation",
           [](AttachedObject& self, const Eigen::Ref<const esp::vec4f> coeffs) {
             self.setRotation(Eigen::Map<const quatf>(coeffs.data()));
           })
      .def(
          "set_transformation",
          py::overload_cast<const Eigen::Ref<const mat4f>>(
              &AttachedObject::setTransformation),
          R"(If it is valid, sets transformation relative to parent scene node.)",
          "transformation"_a)
      .def("set_transformation",
           py::overload_cast<const Eigen::Ref<const vec3f>,
                             const Eigen::Ref<const vec3f>,
                             const Eigen::Ref<const vec3f>>(
               &AttachedObject::setTransformation),
           R"(
             Set the position and orientation of the object by setting
             this :py:class:`AttachedObject`'s matrix to look at target from given position
             and orientation towards target. Semantics is same as gluLookAt.
             )",
           "position"_a, "target"_a, "up"_a)
      .def("set_translation", &AttachedObject::setTranslation, R"(
        If it is valid, sets translation relative to parent scene node
      )",
           "vector"_a)
      .def("reset_transformation", &AttachedObject::resetTransformation, R"(
        If it is valid, reset transform relative to parent scene node to identity
      )")
      .def("translate", &AttachedObject::translate, R"(
        If it is valid, Translate relative to parent scene node by given translation
      )",
           "vector"_a)
      .def("translateLocal", &AttachedObject::translateLocal, R"(
        If it is valid, Translate relative to parent scene node by given translation
        in local frame. Applies given translation before any other translation.
      )",
           "vector"_a)
      .def("rotate", &AttachedObject::rotate, R"(
        If it is valid, rotate relative to parent scene node by given angleInRad
        radians around given normalizedAxis.
      )",
           "angleInRad"_a, "normalizedAxis"_a)
      .def("rotateLocal", &AttachedObject::rotateLocal, R"(
        If it is valid, rotate relative to parent scene node by given angleInRad
        radians around given normalizedAxis in local frame. Applies given
        rotation before any other rotation.
      )",
           "angleInRad"_a, "normalizedAxis"_a)
      .def("rotateX", &AttachedObject::rotateX, R"(
        If it is valid, rotate arounx X axis relative to parent scene node by
        in local frame. Applies given rotation before any other rotation
      )",
           "angleInRad"_a)
      .def("rotateXLocal", &AttachedObject::rotateXLocal, R"(
        If it is valid, rotate relative to parent scene node by given angleInRad
        radians around X axis in local frame. Applies given rotation before
        any other rotation.
      )",
           "angleInRad"_a)
      .def("rotateY", &AttachedObject::rotateY, R"(
        If it is valid, rotate arounx Y axis relative to parent scene node by
        in local frame. Applies given rotation before any other rotation
      )",
           "angleInRad"_a)
      .def("rotateYLocal", &AttachedObject::rotateYLocal, R"(
        If it is valid, rotate relative to parent scene node by given angleInRad
        radians around X axis in local frame. Applies given rotation before
        any other rotation.
      )",
           "angleInRad"_a)
      .def("rotateZ", &AttachedObject::rotateZ, R"(
        If it is valid, rotate arounx Y axis relative to parent scene node by
        in local frame. Applies given rotation before any other rotation
      )",
           "angleInRad"_a)
      .def("rotateZLocal", &AttachedObject::rotateZLocal, R"(
        If it is valid, rotate relative to parent scene node by given angleInRad
        radians around X axis in local frame. Applies given rotation before
        any other rotation.
      )",
           "angleInRad"_a);

  // ==== RenderCamera (subclass of AttachedObject) ====
  py::class_<RenderCamera, AttachedObject, RenderCamera::ptr>(
      m, "Camera",
      R"(RenderCamera: subclass of AttachedObject.
      The object of this class is a camera attached
      to the scene node for rendering.)")
      .def(py::init(&RenderCamera::create<>))
      .def(py::init(&RenderCamera::create<SceneNode&, const vec3f&,
                                          const vec3f&, const vec3f&>))
      .def("setProjectionMatrix", &RenderCamera::setProjectionMatrix, R"(
        Set this :py:class:`Camera`'s projection matrix.
      )",
           "width"_a, "height"_a, "znear"_a, "zfar"_a, "hfov"_a)
      .def("getProjectionMatrix", &RenderCamera::getProjectionMatrix, R"(
        Get this :py:class:`Camera`'s projection matrix.
      )")
      .def("getCameraMatrix", &RenderCamera::getCameraMatrix, R"(
        Get this :py:class:`Camera`'s camera matrix.
      )");

  // ==== SceneGraph ====
  py::class_<scene::SceneGraph>(m, "SceneGraph")
      .def(py::init())
      .def("get_root_node",
           py::overload_cast<>(&scene::SceneGraph::getRootNode, py::const_),
           R"(
            Get the root node of the scene graph. User can specify transformation
            of the root node w.r.t. the world frame. (const function)
            PYTHON DOES NOT GET OWNERSHIP)",
           pybind11::return_value_policy::reference)
      .def("get_root_node",
           py::overload_cast<>(&scene::SceneGraph::getRootNode),
           R"(
            Get the root node of the scene graph. User can specify transformation
            of the root node w.r.t. the world frame.
            PYTHON DOES NOT GET OWNERSHIP)",
           pybind11::return_value_policy::reference)
      .def("set_default_render_camera_parameters",
           &scene::SceneGraph::setDefaultRenderCamera,
           R"(
            Set transformation and the projection matrix to the default render camera.
            The camera will have the same absolute transformation
            as the target scene node after the operation.)",
           "targetSceneNode"_a)
      .def("get_default_render_camera",
           &scene::SceneGraph::getDefaultRenderCamera,
           R"(
            Get the default camera stored in scene graph for rendering.
            PYTHON DOES NOT GET OWNERSHIP)",
           pybind11::return_value_policy::reference);

  // ==== SceneManager ====
  py::class_<scene::SceneManager>(m, "SceneManager")
      .def("init_scene_graph", &scene::SceneManager::initSceneGraph,
           R"(
          Initialize a new scene graph, and return its ID.)")
      .def("get_scene_graph",
           py::overload_cast<int>(&scene::SceneManager::getSceneGraph),
           R"(
             Get the scene graph by scene graph ID.
             PYTHON DOES NOT GET OWNERSHIP)",
           "sceneGraphID"_a, pybind11::return_value_policy::reference)
      .def("get_scene_graph",
           py::overload_cast<int>(&scene::SceneManager::getSceneGraph,
                                  py::const_),
           R"(
             Get the scene graph by scene graph ID.
             PYTHON DOES NOT GET OWNERSHIP)",
           "sceneGraphID"_a, pybind11::return_value_policy::reference);

  // ==== box3f ====
  py::class_<box3f>(m, "BBox")
      .def_property_readonly("sizes", &box3f::sizes)
      .def_property_readonly("center", &box3f::center);

  // ==== OBB ====
  py::class_<OBB>(m, "OBB")
      .def_property_readonly("center", &OBB::center)
      .def_property_readonly("sizes", &OBB::sizes)
      .def_property_readonly("half_extents", &OBB::halfExtents)
      .def_property_readonly(
          "rotation", [](const OBB& self) { return self.rotation().coeffs(); });

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

  // === SuncgObjectCategory ===
  py::class_<SuncgObjectCategory, SemanticCategory, SuncgObjectCategory::ptr>(
      m, "SuncgObjectCategory")
      .def("index", &SuncgObjectCategory::index, "mapping"_a = "")
      .def("name", &SuncgObjectCategory::name, "mapping"_a = "");

  // === SuncgRegionCategory ===
  py::class_<SuncgRegionCategory, SemanticCategory, SuncgRegionCategory::ptr>(
      m, "SuncgRegionCategory")
      .def("index", &SuncgRegionCategory::index, "mapping"_a = "")
      .def("name", &SuncgRegionCategory::name, "mapping"_a = "");

  // ==== SemanticLevel ====
  py::class_<SemanticLevel, SemanticLevel::ptr>(m, "SemanticLevel")
      .def_property_readonly("id", &SemanticLevel::id)
      .def_property_readonly("aabb", &SemanticLevel::aabb)
      .def_property_readonly("regions", &SemanticLevel::regions)
      .def_property_readonly("objects", &SemanticLevel::objects);

  // ==== SemanticRegion ====
  py::class_<SemanticRegion, SemanticRegion::ptr>(m, "SemanticRegion")
      .def_property_readonly("id", &SemanticRegion::id)
      .def_property_readonly("level", &SemanticRegion::level)
      .def_property_readonly("aabb", &SemanticRegion::aabb)
      .def_property_readonly("category", &SemanticRegion::category)
      .def_property_readonly("objects", &SemanticRegion::objects);

  // ==== SuncgSemanticRegion ====
  py::class_<SuncgSemanticRegion, SemanticRegion, SuncgSemanticRegion::ptr>(
      m, "SuncgSemanticRegion")
      .def_property_readonly("id", &SuncgSemanticRegion::id)
      .def_property_readonly("level", &SuncgSemanticRegion::level)
      .def_property_readonly("aabb", &SuncgSemanticRegion::aabb)
      .def_property_readonly("category", &SuncgSemanticRegion::category)
      .def_property_readonly("objects", &SuncgSemanticRegion::objects);

  // ==== SemanticObject ====
  py::class_<SemanticObject, SemanticObject::ptr>(m, "SemanticObject")
      .def_property_readonly("id", &SemanticObject::id)
      .def_property_readonly("region", &SemanticObject::region)
      .def_property_readonly("aabb", &SemanticObject::aabb)
      .def_property_readonly("obb", &SemanticObject::obb)
      .def_property_readonly("category", &SemanticObject::category);

  // ==== SuncgSemanticObject ====
  py::class_<SuncgSemanticObject, SemanticObject, SuncgSemanticObject::ptr>(
      m, "SuncgSemanticObject")
      .def_property_readonly("id", &SuncgSemanticObject::id)
      .def_property_readonly("region", &SuncgSemanticObject::region)
      .def_property_readonly("aabb", &SuncgSemanticObject::aabb)
      .def_property_readonly("obb", &SuncgSemanticObject::obb)
      .def_property_readonly("category", &SuncgSemanticObject::category);

  // ==== SemanticScene ====
  py::class_<SemanticScene, SemanticScene::ptr>(m, "SemanticScene")
      .def(py::init(&SemanticScene::create<>))
      .def_static("load_mp3d_house", &SemanticScene::loadMp3dHouse, R"(
        Loads a SemanticScene from a Matterport3D House format file into passed
        :py:class:`SemanticScene`'.
      )",
                  "file"_a, "scene"_a, "rotation"_a)
      .def_property_readonly("aabb", &SemanticScene::aabb)
      .def_property_readonly("categories", &SemanticScene::categories)
      .def_property_readonly("levels", &SemanticScene::levels)
      .def_property_readonly("regions", &SemanticScene::regions)
      .def_property_readonly("objects", &SemanticScene::objects)
      .def_property_readonly("semantic_index_map",
                             &SemanticScene::getSemanticIndexMap)
      .def("semantic_index_to_object_index",
           &SemanticScene::semanticIndexToObjectIndex);

  // ==== ObjectControls ====
  py::class_<ObjectControls, ObjectControls::ptr>(m, "ObjectControls")
      .def(py::init(&ObjectControls::create<>))
      .def("action", &ObjectControls::action, R"(
        Take action using this :py:class:`ObjectControls`.
      )",
           "object"_a, "name"_a, "amount"_a, "apply_filter"_a = true);

  // ==== Renderer ====
  py::class_<Renderer, Renderer::ptr>(m, "Renderer")
      .def(py::init(&Renderer::create<int, int>))
      .def("set_size", &Renderer::setSize, R"(Set the size of the canvas)",
           "width"_a, "height"_a)
      .def(
          "readFrameRgba",
          [](Renderer& self,
             Eigen::Ref<Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic,
                                      Eigen::RowMajor>>& img) {
            self.readFrameRgba(img.data());
          },
          py::arg("img").noconvert(),
          R"(
      Reads RGBA frame into passed img in uint8 byte format.

      Parameters
      ----------
      img: numpy.ndarray[uint8[m, n], flags.writeable, flags.c_contiguous]
           Numpy array array to populate with frame bytes.
           Memory is NOT allocated to this array.
           Assume that ``m = height`` and ``n = width * 4``.
      )")
      .def("draw",
           py::overload_cast<sensor::Sensor&, scene::SceneGraph&>(
               &Renderer::draw),
           R"(Draw given scene using the visual sensor)", "visualSensor"_a,
           "scene"_a)
      .def("draw",
           py::overload_cast<gfx::RenderCamera&, scene::SceneGraph&>(
               &Renderer::draw),
           R"(Draw given scene using the camera)", "camera"_a, "scene"_a)
      .def(
          "readFrameDepth",
          [](Renderer& self,
             Eigen::Ref<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic,
                                      Eigen::RowMajor>>& img) {
            self.readFrameDepth(img.data());
          },
          py::arg("img").noconvert(), R"()")
      .def(
          "readFrameObjectId",
          [](Renderer& self,
             Eigen::Ref<Eigen::Matrix<uint32_t, Eigen::Dynamic, Eigen::Dynamic,
                                      Eigen::RowMajor>>& img) {
            self.readFrameObjectId(img.data());
          },
          py::arg("img").noconvert(), R"()");

  // TODO fill out other SensorTypes
  // ==== enum SensorType ====
  py::enum_<SensorType>(m, "SensorType")
      .value("NONE", SensorType::NONE)
      .value("COLOR", SensorType::COLOR)
      .value("DEPTH", SensorType::DEPTH)
      .value("SEMANTIC", SensorType::SEMANTIC);

  // ==== SensorSpec ====
  py::class_<SensorSpec, SensorSpec::ptr>(m, "SensorSpec")
      .def(py::init(&SensorSpec::create<>))
      .def_readwrite("uuid", &SensorSpec::uuid)
      .def_readwrite("sensor_type", &SensorSpec::sensorType)
      .def_readwrite("sensor_subtype", &SensorSpec::sensorSubtype)
      .def_readwrite("parameters", &SensorSpec::parameters)
      .def_readwrite("position", &SensorSpec::position)
      .def_readwrite("orientation", &SensorSpec::orientation)
      .def_readwrite("resolution", &SensorSpec::resolution)
      .def_readwrite("channels", &SensorSpec::channels)
      .def_readwrite("encoding", &SensorSpec::encoding)
      .def_readwrite("observation_space", &SensorSpec::observationSpace)
      .def("__eq__",
           [](const SensorSpec& self, const SensorSpec& other) -> bool {
             return self == other;
           })
      .def("__neq__",
           [](const SensorSpec& self, const SensorSpec& other) -> bool {
             return self != other;
           });

  // ==== Observation ====
  py::class_<Observation, Observation::ptr>(m, "Observation");

  // ==== Sensor (subclass of AttachedObject) ====
  py::class_<Sensor, AttachedObject, Sensor::ptr>(m, "Sensor")
      .def(py::init(&Sensor::create<const SensorSpec::ptr&>))
      .def("specification", &Sensor::specification)
      .def("set_transformation_from_spec", &Sensor::setTransformationFromSpec)
      .def("is_visual_sensor", &Sensor::isVisualSensor)
      .def("get_observation", &Sensor::getObservation);

  // ==== PinholeCamera (subclass of Sensor) ====
  py::class_<sensor::PinholeCamera, sensor::Sensor, sensor::PinholeCamera::ptr>(
      m, "PinholeCamera")
      // initialized, not attach to any scene node, status: "invalid"
      .def(py::init(&PinholeCamera::create<const sensor::SensorSpec::ptr&>))
      // initialized, attached to pinholeCameraNode, status: "valid"
      .def(py::init(&PinholeCamera::create<const sensor::SensorSpec::ptr&,
                                           scene::SceneNode&>))
      .def("set_projection_matrix", &sensor::PinholeCamera::setProjectionMatrix,
           R"(Set the width, height, near, far, and hfov,
          stored in pinhole camera to the render camera.)");

  // ==== SensorSuite ====
  py::class_<SensorSuite, SensorSuite::ptr>(m, "SensorSuite")
      .def(py::init(&SensorSuite::create<>))
      .def("add", &SensorSuite::add)
      .def("get", &SensorSuite::get, R"(get the sensor by id)");

  // ==== SceneConfiguration ====
  py::class_<SceneConfiguration, SceneConfiguration::ptr>(m,
                                                          "SceneConfiguration")
      .def(py::init(&SceneConfiguration::create<>))
      .def_readwrite("dataset", &SceneConfiguration::dataset)
      .def_readwrite("id", &SceneConfiguration::id)
      .def_readwrite("filepaths", &SceneConfiguration::filepaths)
      .def_readwrite("scene_up_dir", &SceneConfiguration::sceneUpDir)
      .def_readwrite("scene_front_dir", &SceneConfiguration::sceneFrontDir)
      .def_readwrite("scene_scale_unit", &SceneConfiguration::sceneScaleUnit)
      .def(
          "__eq__",
          [](const SceneConfiguration& self,
             const SceneConfiguration& other) -> bool { return self == other; })
      .def("__neq__",
           [](const SceneConfiguration& self, const SceneConfiguration& other)
               -> bool { return self != other; });

  // ==== SimulatorConfiguration ====
  py::class_<SimulatorConfiguration, SimulatorConfiguration::ptr>(
      m, "SimulatorConfiguration")
      .def(py::init(&SimulatorConfiguration::create<>))
      .def_readwrite("scene", &SimulatorConfiguration::scene)
      .def_readwrite("default_agent_id",
                     &SimulatorConfiguration::defaultAgentId)
      .def_readwrite("default_camera_uuid",
                     &SimulatorConfiguration::defaultCameraUuid)
      .def_readwrite("gpu_device_id", &SimulatorConfiguration::gpuDeviceId)
      .def_readwrite("width", &SimulatorConfiguration::width)
      .def_readwrite("height", &SimulatorConfiguration::height)
      .def_readwrite("compress_textures",
                     &SimulatorConfiguration::compressTextures)
      .def("__eq__",
           [](const SimulatorConfiguration& self,
              const SimulatorConfiguration& other) -> bool {
             return self == other;
           })
      .def("__neq__",
           [](const SimulatorConfiguration& self,
              const SimulatorConfiguration& other) -> bool {
             return self != other;
           });

  initShortestPathBindings(m);

  // ==== Simulator ====
  py::class_<Simulator, Simulator::ptr>(m, "Simulator")
      .def(py::init(&Simulator::create<const SimulatorConfiguration&>))
      .def("get_active_scene_graph", &Simulator::getActiveSceneGraph,
           R"(PYTHON DOES NOT GET OWNERSHIP)",
           pybind11::return_value_policy::reference)
      .def("get_active_semantic_scene_graph",
           &Simulator::getActiveSemanticSceneGraph,
           R"(PYTHON DOES NOT GET OWNERSHIP)",
           pybind11::return_value_policy::reference)
      .def_property_readonly("semantic_scene", &Simulator::getSemanticScene)
      .def_property_readonly("renderer", &Simulator::getRenderer)
      .def("seed", &Simulator::seed, R"()", "new_seed"_a)
      .def("reconfigure", &Simulator::reconfigure, R"()", "configuration"_a)
      .def("reset", &Simulator::reset, R"()");
}
