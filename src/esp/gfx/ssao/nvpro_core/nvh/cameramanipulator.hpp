/*
 * Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * SPDX-FileCopyrightText: Copyright (c) 2018-2021 NVIDIA CORPORATION
 * SPDX-License-Identifier: Apache-2.0
 */
//--------------------------------------------------------------------

#pragma once

#pragma warning(disable : 4201)
#include <array>
#include <string>
#include "../nvmath/nvmath.h"

namespace nvh {
/**
  \class nvh::CameraManipulator

  nvh::CameraManipulator is a camera manipulator help class
  It allow to simply do
  - Orbit        (LMB)
  - Pan          (LMB + CTRL  | MMB)
  - Dolly        (LMB + SHIFT | RMB)
  - Look Around  (LMB + ALT   | LMB + CTRL + SHIFT)

  In a various ways:
  - examiner(orbit around object)
  - walk (look up or down but stays on a plane)
  - fly ( go toward the interest point)

  Do use the camera manipulator, you need to do the following
  - Call setWindowSize() at creation of the application and when the window size
  change
  - Call setLookat() at creation to initialize the camera look position
  - Call setMousePosition() on application mouse down
  - Call mouseMove() on application mouse move

  Retrieve the camera matrix by calling getMatrix()

  See: appbase_vkpp.hpp

  Note: There is a singleton `CameraManip` which can be use across the entire
  application

  \code{.cpp}
  // Retrieve/set camera information
  CameraManip.getLookat(eye, center, up);
  CameraManip.setLookat(eye, center, nvmath::vec3f(m_upVector == 0, m_upVector
  == 1, m_upVector == 2)); CameraManip.getFov(); CameraManip.setSpeed(navSpeed);
  CameraManip.setMode(navMode == 0 ? nvh::CameraManipulator::Examine :
  nvh::CameraManipulator::Fly);
  // On mouse down, keep mouse coordinates
  CameraManip.setMousePosition(x, y);
  // On mouse move and mouse button down
  if(m_inputs.lmb || m_inputs.rmb || m_inputs.mmb)
  {
  CameraManip.mouseMove(x, y, m_inputs);
  }
  // Wheel changes the FOV
  CameraManip.wheel(delta > 0 ? 1 : -1, m_inputs);
  // Retrieve the matrix to push to the shader
  m_ubo.view = CameraManip.getMatrix();
  \endcode

*/

class CameraManipulator {
 public:
  // clang-format off
    enum Modes { Examine, Fly, Walk};
    enum Actions { NoAction, Orbit, Dolly, Pan, LookAround };
    struct Inputs {bool lmb=false; bool mmb=false; bool rmb=false;
                   bool shift=false; bool ctrl=false; bool alt=false;};
  // clang-format on

  struct Camera {
    nvmath::vec3f eye = nvmath::vec3f(10, 10, 10);
    nvmath::vec3f ctr = nvmath::vec3f(0, 0, 0);
    nvmath::vec3f up = nvmath::vec3f(0, 1, 0);
    float fov = 60.0f;

    bool operator!=(const Camera& rhr) const {
      return (eye != rhr.eye) || (ctr != rhr.ctr) || (up != rhr.up) ||
             (fov != rhr.fov);
    }
    bool operator==(const Camera& rhr) const {
      return (eye == rhr.eye) && (ctr == rhr.ctr) && (up == rhr.up) &&
             (fov == rhr.fov);
    }
  };

 public:
  // Main function to call from the application
  // On application mouse move, call this function with the current mouse
  // position, mouse button presses and keyboard modifier. The camera matrix
  // will be updated and can be retrieved calling getMatrix
  Actions mouseMove(int x, int y, const Inputs& inputs);

  // Set the camera to look at the interest point
  // instantSet = true will not interpolate to the new position
  void setLookat(const nvmath::vec3f& eye,
                 const nvmath::vec3f& center,
                 const nvmath::vec3f& up,
                 bool instantSet = true);

  // This should be called in an application loop to update the camera matrix if
  // this one is animated: new position, key movement
  void updateAnim();

  // To call when the size of the window change.  This allows to do nicer
  // movement according to the window size.
  void setWindowSize(int w, int h);

  // Setting the current mouse position, to call on mouse button down. Allow to
  // compute properly the deltas
  void setMousePosition(int x, int y);

  Camera getCamera() const { return m_current; }
  void setCamera(Camera camera, bool instantSet = true);

  // Retrieve the position, interest and up vector of the camera
  void getLookat(nvmath::vec3f& eye,
                 nvmath::vec3f& center,
                 nvmath::vec3f& up) const;
  nvmath::vec3f getEye() const { return m_current.eye; }
  nvmath::vec3f getCenter() const { return m_current.ctr; }
  nvmath::vec3f getUp() const { return m_current.up; }

  // Set the manipulator mode, from Examiner, to walk, to fly, ...
  void setMode(Modes mode) { m_mode = mode; }

  // Retrieve the current manipulator mode
  Modes getMode() const { return m_mode; }

  // Retrieving the transformation matrix of the camera
  const nvmath::mat4f& getMatrix() const { return m_matrix; }

  // Set the position, interest from the matrix.
  // instantSet = true will not interpolate to the new position
  // centerDistance is the distance of the center from the eye
  void setMatrix(const nvmath::mat4f& mat_,
                 bool instantSet = true,
                 float centerDistance = 1.f);

  // Changing the default speed movement
  void setSpeed(float speed) { m_speed = speed; }

  // Retrieving the current speed
  float getSpeed() { return m_speed; }

  // Retrieving the last mouse position
  void getMousePosition(int& x, int& y);

  // Main function which is called to apply a camera motion.
  // It is preferable to
  void motion(int x, int y, int action = 0);

  void keyMotion(float dx, float dy, int action);

  // To call when the mouse wheel change
  void wheel(int value, const Inputs& inputs);

  // Retrieve the screen dimension
  int getWidth() const { return m_width; }
  int getHeight() const { return m_height; }

  // Field of view
  void setFov(float _fov);
  float getFov() { return m_current.fov; }

  void setClipPlanes(nvmath::vec2f clip) { m_clipPlanes = clip; }
  const nvmath::vec2f& getClipPlanes() const { return m_clipPlanes; }

  // Animation duration
  double getAnimationDuration() const { return m_duration; }
  void setAnimationDuration(double val) { m_duration = val; }
  bool isAnimated() { return m_anim_done == false; }

  // Returning a default help string
  const std::string& getHelp();

  // Fitting the camera position and interest to see the bounding box
  void fit(const nvmath::vec3f& boxMin,
           const nvmath::vec3f& boxMax,
           bool instantFit = true,
           bool tight = false,
           float aspect = 1.0f);

 protected:
  CameraManipulator();

 private:
  // Update the internal matrix.
  void update() {
    m_matrix = nvmath::look_at(m_current.eye, m_current.ctr, m_current.up);
  }

  // Do panning: movement parallels to the screen
  void pan(float dx, float dy);
  // Do orbiting: rotation around the center of interest. If invert, the
  // interest orbit around the camera position
  void orbit(float dx, float dy, bool invert = false);
  // Do dolly: movement toward the interest.
  void dolly(float dx, float dy);

  double getSystemTime();

  nvmath::vec3f computeBezier(float t,
                              nvmath::vec3f& p0,
                              nvmath::vec3f& p1,
                              nvmath::vec3f& p2);
  void findBezierPoints();

 protected:
  nvmath::mat4f m_matrix = nvmath::mat4f(1);

  Camera m_current;   // Current camera position
  Camera m_goal;      // Wish camera position
  Camera m_snapshot;  // Current camera the moment a set look-at is done

  // Animation
  std::array<nvmath::vec3f, 3> m_bezier;
  double m_start_time = 0;
  double m_duration = 0.5;
  bool m_anim_done{true};
  nvmath::vec3f m_key_vec{0, 0, 0};

  // Screen
  int m_width = 1;
  int m_height = 1;

  // Other
  float m_speed = 3.f;
  nvmath::vec2f m_mouse = nvmath::vec2f(0.f, 0.f);
  nvmath::vec2f m_clipPlanes = nvmath::vec2f(0.001f, 100000000.f);

  bool m_button = false;  // Button pressed
  bool m_moving = false;  // Mouse is moving
  float m_tbsize = 0.8f;  // Trackball size;

  Modes m_mode = Examine;

 public:
  // Factory.
  static CameraManipulator& Singleton() {
    static CameraManipulator manipulator;
    return manipulator;
  }
};

// Global Manipulator

}  // namespace nvh

#define CameraManip nvh::CameraManipulator::Singleton()
