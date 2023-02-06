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

#include "cameramanipulator.hpp"
#include <chrono>
#include <iostream>
#include "../nvp/nvpwindow.hpp"

namespace nvh {

inline float sign(float s) {
  return (s < 0.f) ? -1.f : 1.f;
}

//--------------------------------------------------------------------------------------------------
//
//
CameraManipulator::CameraManipulator() {
  update();
}

//--------------------------------------------------------------------------------------------------
// Set the new camera as a goal
//
void CameraManipulator::setCamera(Camera camera, bool instantSet /*=true*/) {
  m_anim_done = true;

  if (instantSet) {
    m_current = camera;
    update();
  } else if (camera != m_current) {
    m_goal = camera;
    m_snapshot = m_current;
    m_anim_done = false;
    m_start_time = getSystemTime();
    findBezierPoints();
  }
}

//--------------------------------------------------------------------------------------------------
// Creates a viewing matrix derived from an eye point, a reference point
// indicating the center of the scene, and an up vector
//
void CameraManipulator::setLookat(const nvmath::vec3f& eye,
                                  const nvmath::vec3f& center,
                                  const nvmath::vec3f& up,
                                  bool instantSet) {
  Camera camera{eye, center, up, m_current.fov};
  setCamera(camera, instantSet);
}

//--------------------------------------------------------------------------------------------------
// Modify the position of the camera over time
// - The camera can be updated through keys. A key set a direction which is
// added to both
//   eye and center, until the key is released
// - A new position of the camera is defined and the camera will reach that
// position
//   over time.
void CameraManipulator::updateAnim() {
  auto elapse = static_cast<float>(getSystemTime() - m_start_time) / 1000.f;

  // Key animation
  if (m_key_vec != nvmath::vec3f(0, 0, 0)) {
    m_current.eye += m_key_vec * elapse;
    m_current.ctr += m_key_vec * elapse;
    update();
    m_start_time = getSystemTime();
    return;
  }

  // Camera moving to new position
  if (m_anim_done)
    return;

  float t = std::min(elapse / float(m_duration), 1.0f);
  // Evaluate polynomial (smoother step from Perlin)
  t = t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
  if (t >= 1.0f) {
    m_current = m_goal;
    m_anim_done = true;
    return;
  }

  // Interpolate camera position and interest
  // The distance of the camera between the interest is preserved to
  // create a nicer interpolation
  nvmath::vec3f vpos, vint, vup;
  m_current.ctr = nvmath::lerp(t, m_snapshot.ctr, m_goal.ctr);
  m_current.up = nvmath::lerp(t, m_snapshot.up, m_goal.up);
  m_current.eye = computeBezier(t, m_bezier[0], m_bezier[1], m_bezier[2]);
  m_current.fov = nvmath::lerp(t, m_snapshot.fov, m_goal.fov);

  update();
}

//-----------------------------------------------------------------------------
// Get the current camera information
//   position  camera position
//   interest  camera interesting point (look at position)
//   up        camera up vector
//
void CameraManipulator::getLookat(nvmath::vec3f& eye,
                                  nvmath::vec3f& center,
                                  nvmath::vec3f& up) const {
  eye = m_current.eye;
  center = m_current.ctr;
  up = m_current.up;
}

//--------------------------------------------------------------------------------------------------
//
void CameraManipulator::setMatrix(const nvmath::mat4f& matrix,
                                  bool instantSet,
                                  float centerDistance) {
  Camera camera;
  matrix.get_translation(camera.eye);
  auto rotMat = matrix.get_rot_mat3();
  camera.ctr = {0, 0, -centerDistance};
  camera.ctr = camera.eye + (rotMat * camera.ctr);
  camera.up = {0, 1, 0};
  camera.fov = m_current.fov;

  m_anim_done = instantSet;

  if (instantSet) {
    m_current = camera;
  } else {
    m_goal = camera;
    m_snapshot = m_current;
    m_start_time = getSystemTime();
    findBezierPoints();
  }
  update();
}

//--------------------------------------------------------------------------------------------------
//
//
void CameraManipulator::setMousePosition(int x, int y) {
  m_mouse = {static_cast<float>(x), static_cast<float>(y)};
}

//--------------------------------------------------------------------------------------------------
//
//
void CameraManipulator::getMousePosition(int& x, int& y) {
  x = static_cast<int>(m_mouse[0]);
  y = static_cast<int>(m_mouse[1]);
}

//--------------------------------------------------------------------------------------------------
//
//
void CameraManipulator::setWindowSize(int w, int h) {
  m_width = w;
  m_height = h;
}

//--------------------------------------------------------------------------------------------------
//
// Low level function for when the camera move.
//
void CameraManipulator::motion(int x, int y, int action) {
  float dx = float(x - m_mouse[0]) / float(m_width);
  float dy = float(y - m_mouse[1]) / float(m_height);

  switch (action) {
    case Orbit:
      orbit(dx, dy, false);
      break;
    case CameraManipulator::Dolly:
      dolly(dx, dy);
      break;
    case CameraManipulator::Pan:
      pan(dx, dy);
      break;
    case CameraManipulator::LookAround:
      orbit(dx, -dy, true);
      break;
  }

  // Resetting animation
  m_anim_done = true;

  update();

  m_mouse[0] = static_cast<float>(x);
  m_mouse[1] = static_cast<float>(y);
}

//
// Function for when the camera move with keys.
//
void CameraManipulator::keyMotion(float dx, float dy, int action) {
  if (action == NoAction) {
    m_key_vec = {0, 0, 0};
    return;
  }

  auto d = nvmath::normalize(m_current.ctr - m_current.eye);
  dx *= m_speed * 2.f;
  dy *= m_speed * 2.f;

  nvmath::vec3f key_vec;
  if (action == Dolly) {
    key_vec = d * dx;
    if (m_mode == Walk) {
      if (m_current.up.y > m_current.up.z)
        key_vec.y = 0;
      else
        key_vec.z = 0;
    }
  } else if (action == Pan) {
    auto r = nvmath::cross(d, m_current.up);
    key_vec = r * dx + m_current.up * dy;
  }

  m_key_vec += key_vec;

  // Resetting animation
  m_start_time = getSystemTime();
}

//--------------------------------------------------------------------------------------------------
// To call when the mouse is moving
// It find the appropriate camera operator, based on the mouse button pressed
// and the keyboard modifiers (shift, ctrl, alt)
//
// Returns the action that was activated
//
CameraManipulator::Actions CameraManipulator::mouseMove(int x,
                                                        int y,
                                                        const Inputs& inputs) {
  if (!inputs.lmb && !inputs.rmb && !inputs.mmb) {
    setMousePosition(x, y);
    return NoAction;  // no mouse button pressed
  }

  Actions curAction = NoAction;
  if (inputs.lmb) {
    if (((inputs.ctrl) && (inputs.shift)) || inputs.alt)
      curAction = m_mode == Examine ? LookAround : Orbit;
    else if (inputs.shift)
      curAction = Dolly;
    else if (inputs.ctrl)
      curAction = Pan;
    else
      curAction = m_mode == Examine ? Orbit : LookAround;
  } else if (inputs.mmb)
    curAction = Pan;
  else if (inputs.rmb)
    curAction = Dolly;

  if (curAction != NoAction)
    motion(x, y, curAction);

  return curAction;
}

//--------------------------------------------------------------------------------------------------
// Trigger a dolly when the wheel change
//
void CameraManipulator::wheel(int value, const Inputs& inputs) {
  float fval(static_cast<float>(value));
  float dx = (fval * fabsf(fval)) / static_cast<float>(m_width);

  if (inputs.shift) {
    setFov(m_current.fov + fval);
  } else {
    dolly(dx * m_speed, dx * m_speed);
    update();
  }
}

// Set and clamp FOV between 0.01 and 179 degrees
void CameraManipulator::setFov(float _fov) {
  m_current.fov = std::min(std::max(_fov, 0.01f), 179.0f);
}

nvmath::vec3f CameraManipulator::computeBezier(float t,
                                               nvmath::vec3f& p0,
                                               nvmath::vec3f& p1,
                                               nvmath::vec3f& p2) {
  float u = 1.f - t;
  float tt = t * t;
  float uu = u * u;

  nvmath::vec3f p = uu * p0;  // first term
  p += 2 * u * t * p1;        // second term
  p += tt * p2;               // third term

  return p;
}

void CameraManipulator::findBezierPoints() {
  nvmath::vec3f p0 = m_current.eye;
  nvmath::vec3f p2 = m_goal.eye;
  nvmath::vec3f p1, pc;

  // point of interest
  nvmath::vec3f pi = (m_goal.ctr + m_current.ctr) * 0.5f;

  nvmath::vec3f p02 = (p0 + p2) * 0.5f;                       // mid p0-p2
  float radius = (length(p0 - pi) + length(p2 - pi)) * 0.5f;  // Radius for p1
  nvmath::vec3f p02pi(p02 - pi);  // Vector from interest to mid point
  p02pi.normalize();
  p02pi *= radius;
  pc = pi + p02pi;                        // Calculated point to go through
  p1 = 2.f * pc - p0 * 0.5f - p2 * 0.5f;  // Computing p1 for t=0.5
  p1.y = p02.y;  // Clamping the P1 to be in the same height as p0-p2

  m_bezier[0] = p0;
  m_bezier[1] = p1;
  m_bezier[2] = p2;
}

//--------------------------------------------------------------------------------------------------
// Pan the camera perpendicularly to the light of sight.
//
void CameraManipulator::pan(float dx, float dy) {
  if (m_mode == Fly) {
    dx *= -1;
    dy *= -1;
  }

  nvmath::vec3f z(m_current.eye - m_current.ctr);
  float length = static_cast<float>(nvmath::length(z)) / 0.785f;  // 45 degrees
  z = nvmath::normalize(z);
  nvmath::vec3f x = nvmath::cross(m_current.up, z);
  x = nvmath::normalize(x);
  nvmath::vec3f y = nvmath::cross(z, x);
  y = nvmath::normalize(y);
  x *= -dx * length;
  y *= dy * length;

  m_current.eye += x + y;
  m_current.ctr += x + y;
}

//--------------------------------------------------------------------------------------------------
// Orbit the camera around the center of interest. If 'invert' is true,
// then the camera stays in place and the interest orbit around the camera.
//
void CameraManipulator::orbit(float dx, float dy, bool invert) {
  if (dx == 0 && dy == 0)
    return;

  // Full width will do a full turn
  dx *= nv_two_pi;
  dy *= nv_two_pi;

  // Get the camera
  nvmath::vec3f origin(invert ? m_current.eye : m_current.ctr);
  nvmath::vec3f position(invert ? m_current.ctr : m_current.eye);

  // Get the length of sight
  nvmath::vec3f centerToEye(position - origin);
  float radius = nvmath::length(centerToEye);
  centerToEye = nvmath::normalize(centerToEye);

  nvmath::mat4f rot_x, rot_y;

  // Find the rotation around the UP axis (Y)
  nvmath::vec3f axe_z(nvmath::normalize(centerToEye));
  rot_y = nvmath::mat4f().as_rot(-dx, m_current.up);

  // Apply the (Y) rotation to the eye-center vector
  nvmath::vec4f vect_tmp =
      rot_y * nvmath::vec4f(centerToEye.x, centerToEye.y, centerToEye.z, 0);
  centerToEye = nvmath::vec3f(vect_tmp.x, vect_tmp.y, vect_tmp.z);

  // Find the rotation around the X vector: cross between eye-center and up (X)
  nvmath::vec3f axe_x = nvmath::cross(m_current.up, axe_z);
  axe_x = nvmath::normalize(axe_x);
  rot_x = nvmath::mat4f().as_rot(-dy, axe_x);

  // Apply the (X) rotation to the eye-center vector
  vect_tmp =
      rot_x * nvmath::vec4f(centerToEye.x, centerToEye.y, centerToEye.z, 0);
  nvmath::vec3f vect_rot(vect_tmp.x, vect_tmp.y, vect_tmp.z);
  if (sign(vect_rot.x) == sign(centerToEye.x))
    centerToEye = vect_rot;

  // Make the vector as long as it was originally
  centerToEye *= radius;

  // Finding the new position
  nvmath::vec3f newPosition = centerToEye + origin;

  if (!invert) {
    m_current.eye = newPosition;  // Normal: change the position of the camera
  } else {
    m_current.ctr = newPosition;  // Inverted: change the interest point
  }
}

//--------------------------------------------------------------------------------------------------
// Move the camera toward the interest point, but don't cross it
//
void CameraManipulator::dolly(float dx, float dy) {
  nvmath::vec3f z = m_current.ctr - m_current.eye;
  float length = static_cast<float>(nvmath::length(z));

  // We are at the point of interest, and don't know any direction, so do
  // nothing!
  if (length < 0.000001f)
    return;

  // Use the larger movement.
  float dd;
  if (m_mode != Examine)
    dd = -dy;
  else
    dd = fabs(dx) > fabs(dy) ? dx : -dy;
  float factor = m_speed * dd;

  // Adjust speed based on distance.
  if (m_mode == Examine) {
    // Don't move over the point of interest.
    if (factor >= 1.0f)
      return;

    z *= factor;
  } else {
    // Normalize the Z vector and make it faster
    z *= factor / length * 10.0f;
  }

  // Not going up
  if (m_mode == Walk) {
    if (m_current.up.y > m_current.up.z)
      z.y = 0;
    else
      z.z = 0;
  }

  m_current.eye += z;

  // In fly mode, the interest moves with us.
  if (m_mode != Examine)
    m_current.ctr += z;
}

//--------------------------------------------------------------------------------------------------
// Return the time in fraction of milliseconds
//
double CameraManipulator::getSystemTime() {
  auto now(std::chrono::system_clock::now());
  auto duration = now.time_since_epoch();
  return std::chrono::duration_cast<std::chrono::microseconds>(duration)
             .count() /
         1000.0;
}

//--------------------------------------------------------------------------------------------------
// Return a string which can be included in help dialogs
//
const std::string& CameraManipulator::getHelp() {
  static std::string helpText =
      "LMB: rotate around the target\n"
      "RMB: Dolly in/out\n"
      "MMB: Pan along view plane\n"
      "LMB + Shift: Dolly in/out\n"
      "LMB + Ctrl: Pan\n"
      "LMB + Alt: Look aroundPan\n"
      "Mouse wheel: Dolly in/out\n"
      "Mouse wheel + Shift: Zoom in/out\n";
  return helpText;
}

//--------------------------------------------------------------------------------------------------
// Move the camera closer or further from the center of the the bounding box, to
// see it completely
//
// boxMin - lower corner of the bounding box
// boxMax - upper corner of the bounding box
// instantFit - true: set the new position, false: will animate to new position.
// tight - true: fit exactly the corner, false: fit to radius (larger view, will
// not get closer or further away) aspect - aspect ratio of the window.
//
void CameraManipulator::fit(const nvmath::vec3f& boxMin,
                            const nvmath::vec3f& boxMax,
                            bool instantFit /*= true*/,
                            bool tight /*=false*/,
                            float aspect /*=1.0f*/) {
  const nvmath::vec3f boxHalfSize = (boxMax - boxMin) * .5f;
  const nvmath::vec3f boxCenter = boxMin + boxHalfSize;

  float offset = 0;
  float yfov = m_current.fov;
  float xfov = m_current.fov * aspect;

  if (!tight) {
    // Using the bounding sphere
    float radius = nvmath::length(boxHalfSize);
    if (aspect > 1.f)
      offset = radius / sin(nv_to_rad * yfov * 0.5f);
    else
      offset = radius / sin(nv_to_rad * xfov * 0.5f);
  } else {
    nvmath::mat4f mView =
        nvmath::look_at(m_current.eye, boxCenter, m_current.up);
    mView.set_translate({0, 0, 0});  // Keep only rotation

    for (int i = 0; i < 8; i++) {
      nvmath::vec3f vct(i & 1 ? boxHalfSize.x : -boxHalfSize.x,
                        i & 2 ? boxHalfSize.y : -boxHalfSize.y,
                        i & 4 ? boxHalfSize.z : -boxHalfSize.z);
      vct = nvmath::vec3f(mView * vct);

      if (vct.z < 0)  // Take only points in front of the center
      {
        // Keep the largest offset to see that vertex
        offset = std::max(
            fabs(vct.y) / tan(nv_to_rad * yfov * 0.5f) + fabs(vct.z), offset);
        offset = std::max(
            fabs(vct.x) / tan(nv_to_rad * xfov * 0.5f) + fabs(vct.z), offset);
      }
    }
  }

  // Re-position the camera
  auto viewDir = nvmath::normalize(m_current.eye - m_current.ctr);
  auto veye = boxCenter + viewDir * offset;
  setLookat(veye, boxCenter, m_current.up, instantFit);
}

}  // namespace nvh
