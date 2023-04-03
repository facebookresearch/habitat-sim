// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/Bindings.h"

#include <Magnum/PythonBindings.h>
#include <Magnum/SceneGraph/PythonBindings.h>

#include "esp/gfx/replay/Player.h"
#include "esp/gfx/replay/ReplayManager.h"

namespace py = pybind11;
using py::literals::operator""_a;

namespace esp {
namespace gfx {
namespace replay {

void initGfxReplayBindings(py::module& m) {
  py::class_<Player, Player::ptr>(m, "Player")
      .def("get_num_keyframes", &Player::getNumKeyframes,
           R"(Get the currently-set keyframe, or -1 if no keyframe is set.)")

      .def(
          "set_keyframe_index", &Player::setKeyframeIndex,
          R"(Set a keyframe by index, or pass -1 to clear the currently-set keyframe.)")

      .def("get_keyframe_index", &Player::getKeyframeIndex,
           R"(Get the number of keyframes read from file.)")

      .def(
          "get_user_transform",
          [](Player& self, const std::string& name) {
            Magnum::Vector3 translation;
            Magnum::Quaternion rotation;
            bool found = self.getUserTransform(name, &translation, &rotation);
            return found ? py::cast<py::object>(
                               py::make_tuple(translation, rotation))
                         : py::cast<py::object>(Py_None);
          },
          R"(Get a previously-added user transform. See also ReplayManager.add_user_transform_to_keyframe.)")

      .def(
          "close", &Player::close,
          R"(Unload all keyframes. The Player is unusable after it is closed.)");

  py::class_<ReplayManager, ReplayManager::ptr>(m, "ReplayManager")
      .def(
          "save_keyframe",
          [](ReplayManager& self) {
            if (!self.getRecorder()) {
              throw std::runtime_error(
                  "replay save not enabled. See "
                  "SimulatorConfiguration.enable_gfx_replay_save.");
            }
            self.getRecorder()->saveKeyframe();
          },
          R"(Save a render keyframe. A render keyframe can be loaded later and used to draw observations.)")

      .def(
          "extract_keyframe",
          [](ReplayManager& self) {
            if (!self.getRecorder()) {
              throw std::runtime_error(
                  "replay save not enabled. See "
                  "SimulatorConfiguration.enable_gfx_replay_save.");
            }
            return esp::gfx::replay::Recorder::keyframeToString(
                self.getRecorder()->extractKeyframe());
          },
          R"(Extract the current keyframe as a JSON-formatted string.)")

      .def(
          "add_user_transform_to_keyframe",
          [](ReplayManager& self, const std::string& name,
             const Magnum::Vector3& translation,
             const Magnum::Quaternion& rotation) {
            if (!self.getRecorder()) {
              throw std::runtime_error(
                  "replay save not enabled. See "
                  "SimulatorConfiguration.enable_gfx_replay_save.");
            }
            self.getRecorder()->addUserTransformToKeyframe(name, translation,
                                                           rotation);
          },
          R"(Add a user transform to the current render keyframe. It will get stored with the keyframe and will be available later upon loading the keyframe.)")

      .def(
          "write_saved_keyframes_to_file",
          [](ReplayManager& self, const std::string& filepath) {
            if (!self.getRecorder()) {
              throw std::runtime_error(
                  "replay save not enabled. See "
                  "SimulatorConfiguration.enable_gfx_replay_save.");
            }
            self.getRecorder()->writeSavedKeyframesToFile(filepath);
          },
          R"(Write all saved keyframes to a file, then discard the keyframes.)")

      .def(
          "write_saved_keyframes_to_string",
          [](ReplayManager& self) {
            if (!self.getRecorder()) {
              throw std::runtime_error(
                  "replay save not enabled. See "
                  "SimulatorConfiguration.enable_gfx_replay_save.");
            }
            return self.getRecorder()->writeSavedKeyframesToString();
          },
          R"(Write all saved keyframes to a string, then discard the keyframes.)")

      .def(
          "write_incremental_saved_keyframes_to_string_array",
          [](ReplayManager& self) {
            if (!self.getRecorder()) {
              throw std::runtime_error(
                  "replay save not enabled. See "
                  "SimulatorConfiguration.enable_gfx_replay_save.");
            }
            return self.getRecorder()
                ->writeIncrementalSavedKeyframesToStringArray();
          },
          R"(Write all saved keyframes to individual strings. See Recorder.h for details.)")

      .def("read_keyframes_from_file", &ReplayManager::readKeyframesFromFile,
           R"(Create a Player object from a replay file.)");
}

}  // namespace replay
}  // namespace gfx
}  // namespace esp
