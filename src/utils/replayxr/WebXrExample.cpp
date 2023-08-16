/*
    This file is part of Magnum.

    Original authors — credit is appreciated but not required:

        2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019,
        2020, 2021, 2022 — Vladimír Vondruš <mosra@centrum.cz>
        2019 — Jonathan Hale <squareys@googlemail.com>

    This is free and unencumbered software released into the public domain.

    Anyone is free to copy, modify, publish, use, compile, sell, or distribute
    this software, either in source code form or as a compiled binary, for any
    purpose, commercial or non-commercial, and by any means.

    In jurisdictions that recognize copyright laws, the author or authors of
    this software dedicate any and all copyright interest in the software to
    the public domain. We make this dedication for the benefit of the public
    at large and to the detriment of our heirs and successors. We intend this
    dedication to be an overt act of relinquishment in perpetuity of all
    present and future rights to this software under copyright law.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
    THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
    IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
    CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <Corrade/Containers/Array.h>
#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/Context.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Framebuffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/Magnum.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/Math/Range.h>
#include <Magnum/Math/Vector3.h>
#include <Magnum/Math/Vector4.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/Platform/EmscriptenApplication.h>
#include <Magnum/Primitives/Cube.h>
#include <Magnum/Primitives/UVSphere.h>
#include <Magnum/Shaders/PhongGL.h>
#include <Magnum/Trade/MeshData.h>

#include <esp/core/Logging.h>
#include <esp/io/Json.h>
#include <esp/sensor/CameraSensor.h>
#include <esp/sim/AbstractReplayRenderer.h>
#include <esp/sim/BatchReplayRenderer.h>
#include <esp/sim/ClassicReplayRenderer.h>

#include <emscripten.h>
#include <emscripten/html5.h>
#include "webxr.h"

#include <math.h>
#include <chrono>

using namespace Magnum;
using namespace Magnum::Math::Literals;
typedef std::chrono::high_resolution_clock Clock;

// TODO: These are hardcoded until we integrate a module to provide keyframes
// (from disk or network).
//       Must match definition in driver.js.
const std::string REPLAY_FILE = "data/short_replay_lights.json";
// const std::string REPLAY_FILE = "data/floorplanner.gfx_replay.json";

class WebXrExample : public Platform::Application {
 public:
  explicit WebXrExample(const Arguments& arguments);
  ~WebXrExample();

  /* Callbacks for WebXR */
  void onError(int error);
  void drawWebXRFrame(WebXRView* views);
  void sessionStart();
  void sessionEnd();

 private:
  void drawEvent() override;
  void keyPressEvent(KeyEvent& e) override;
  void mousePressEvent(MouseEvent& event) override;

  GL::Mesh _cubeMesh;
  Matrix4 _cubeTransformations[4]{
      Matrix4::translation({0.0f, 0.0f, -3.0f}) * Matrix4::rotationY(45.0_degf),
      Matrix4::translation({5.0f, 0.0f, 0.0f}) * Matrix4::rotationY(45.0_degf),
      Matrix4::translation({-10.0f, 0.0f, 0.0f}) *
          Matrix4::rotationY(45.0_degf),
      Matrix4::translation({0.0f, 0.0f, 7.0f}) * Matrix4::rotationY(45.0_degf)};
  Color3 _cubeColors[4]{0xffff00_rgbf, 0xff0000_rgbf, 0x0000ff_rgbf,
                        0x00ffff_rgbf};

  GL::Mesh _controllerMesh;
  Matrix4 _controllerTransformations[2];
  Color3 _controllerColors[2]{{0.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 0.0f}};

  Shaders::PhongGL _shader;
  Matrix4 _projectionMatrices[2];
  Matrix4 _viewMatrices[2];
  Range2Di _viewports[2];

  bool _inXR = false;

  std::shared_ptr<esp::sim::AbstractReplayRenderer> _replayRenderer;
  esp::logging::LoggingContext _loggingContext;
  std::chrono::time_point<std::chrono::high_resolution_clock> t1 = Clock::now();
};

WebXrExample::WebXrExample(const Arguments& arguments)
    : Platform::Application(
          arguments,
          Configuration{},
          // Blitting on multisampled framebuffers causes GL_INVALID_OPERATION
          // error. This is disabled for now. Reenable for the batch replay
          // renderer because we don't blit onto the presentation FBO.
          GLConfiguration{}.setSampleCount(0)),
      _loggingContext() {
  webxr_init(
      WEBXR_SESSION_MODE_IMMERSIVE_VR,
      /* Frame callback */
      [](void* userData, int, float[16], WebXRView* views) {
        static_cast<WebXrExample*>(userData)->drawWebXRFrame(views);
      },
      /* Session end callback */
      [](void* userData) {
        static_cast<WebXrExample*>(userData)->sessionStart();
      },
      /* Session end callback */
      [](void* userData) {
        static_cast<WebXrExample*>(userData)->sessionEnd();
      },
      /* Error callback */
      [](void* userData, int error) {
        static_cast<WebXrExample*>(userData)->onError(error);
      },
      /* userData */
      this);

  redraw();
}

WebXrExample::~WebXrExample() {
  if (_inXR)
    webxr_request_exit();
  if (_replayRenderer) {
    _replayRenderer->close();
    _replayRenderer.reset();
  }
}

void WebXrExample::drawWebXRFrame(WebXRView* views) {
  int viewIndex = 0;
  for (WebXRView view : {views[0], views[1]}) {
    _viewports[viewIndex] =
        Range2Di::fromSize({view.viewport[0], view.viewport[1]},
                           {view.viewport[2], view.viewport[3]});
    _viewMatrices[viewIndex] = Matrix4::from(view.viewMatrix);
    _projectionMatrices[viewIndex] = Matrix4::from(view.projectionMatrix);

    ++viewIndex;
  }

  WebXRInputSource sources[2];
  int sourcesCount = 0;
  webxr_get_input_sources(sources, 5, &sourcesCount);

  for (int i = 0; i < sourcesCount; ++i) {
    webxr_get_input_pose(&sources[i], _controllerTransformations[i].data());
  }

  drawEvent();
}

void WebXrExample::drawEvent() {
  auto t2 = Clock::now();
  float dt =
      std::chrono::duration_cast<std::chrono::duration<float>>(t2 - t1).count();

  if (!_inXR) {
    /* Single view */
    _projectionMatrices[0] = Matrix4::perspectiveProjection(
        90.0_degf, Vector2(windowSize()).aspectRatio(), 0.01f, 100.0f);
    _viewMatrices[0] = Matrix4{};
    _viewports[0] = Range2Di{{}, framebufferSize()};

    /* Set some default transformation for the controllers so that they don't
       block view */
    _controllerTransformations[0] = Matrix4::translation({-0.5f, -0.4f, -1.0f});
    _controllerTransformations[1] = Matrix4::translation({0.5f, -0.4f, -1.0f});
  }

  // TODO: Browsers use the default framebuffer (ID=0) for presentation, which
  // is why this works.
  //       WebXR uses an internal framebuffer instead (ID!=0). The commented
  //       code below attempts (unsuccessfully) to find it. The presentation
  //       framebuffer is bound by the JS script immediately before calling this
  //       function.
  // int framebufferId;
  // glGetIntegerv(GL_FRAMEBUFFER_BINDING, &framebufferId);
  // GL::Framebuffer framebuffer = GL::Framebuffer::wrap(framebufferId,
  // _viewports[0]); Utility::Warning{} << std::to_string(framebufferId);

  auto& framebuffer = Mn::GL::defaultFramebuffer;
  framebuffer.clear(GL::FramebufferClear::Color | GL::FramebufferClear::Depth);

  // TODO: Initialization is done here so that the correct framebuffer size can
  // be retrieved.
  //       This should be done outside of the browser-driven render loop.
  // TODO: Wait for assets to be pre-loaded before rendering.
  //       Right now, we are only initializing for '_inXR' to give enough time
  //       to assets to be loaded. It will fail if activated too early. To be
  //       fixed with proper async handling of pre-loading (driver.js).
  if (!_replayRenderer && _inXR) {
    auto pinholeCameraSpec = esp::sensor::CameraSensorSpec::create();
    pinholeCameraSpec->sensorSubType = esp::sensor::SensorSubType::Pinhole;
    pinholeCameraSpec->sensorType = esp::sensor::SensorType::Color;
    pinholeCameraSpec->position = {1.0f, 1.0f, 1.0f};
    pinholeCameraSpec->resolution = {
        _viewports[0].sizeY(), _viewports[0].sizeX()};  // Flipped convention
    pinholeCameraSpec->channels = 4;
    pinholeCameraSpec->hfov = 90.0_degf;
    pinholeCameraSpec->near = 0.01f;
    pinholeCameraSpec->far = 100.0f;
    // TODO: Sensor name constant
    pinholeCameraSpec->uuid = "sensor";
    esp::sim::ReplayRendererConfiguration replayConfig{};
    replayConfig.numEnvironments = 1;
    replayConfig.standalone = 0;
    replayConfig.sensorSpecifications = {pinholeCameraSpec};
    // TODO: BatchReplayRenderer fails:
    //       GL_INVALID_OPERATION: It is undefined behaviour to use a uniform
    //       buffer that is too small.
    //_replayRenderer =
    // std::make_shared<esp::sim::BatchReplayRenderer>(std::move(replayConfig));
    // TODO: PBR shaders don't compile on WebGL.
    //       They are bypassed directly in the resource manager until this is
    //       fixed.
    _replayRenderer = std::make_shared<esp::sim::ClassicReplayRenderer>(
        std::move(replayConfig));

    auto d = esp::io::parseJsonFile(REPLAY_FILE);
    std::vector<esp::gfx::replay::Keyframe> keyframes{};
    esp::io::readMember(d, "keyframes", keyframes);
    _replayRenderer->setEnvironmentKeyframe_temp(0, std::move(keyframes[0]));
  }
  if (_replayRenderer) {
    const int viewCount = _inXR ? 2 : 1;
    for (int eye = 0; eye < viewCount; ++eye) {
      framebuffer.setViewport(_viewports[eye]);
      //_viewMatrices[eye] = Matrix4::translation({1.0f, 1.0f, 1.0f}) *
      // Matrix4::rotationX(Rad(dt)); // Debug animation.
      _replayRenderer->setSensorTransform(0, "sensor",
                                          _viewMatrices[eye].inverted());
      // TODO: This works in browser, but not in VR. See comments above.
      _replayRenderer->render(framebuffer);
    }
  }

  /* No need to call redraw() on WebXR, the webxr callback already does this */
}

void WebXrExample::sessionStart() {
  if (_inXR)
    return;
  _inXR = true;

  if (_replayRenderer) {
    _replayRenderer->close();
    _replayRenderer.reset();
  }

  Debug{} << "Entered VR";
}

void WebXrExample::sessionEnd() {
  _inXR = false;

  if (_replayRenderer) {
    _replayRenderer->close();
    _replayRenderer.reset();
  }

  Debug{} << "Exited VR";
  redraw();
}

void WebXrExample::onError(int error) {
  switch (error) {
    case WEBXR_ERR_API_UNSUPPORTED:
      Error{} << "WebXR unsupported in this browser.";
      break;
    case WEBXR_ERR_GL_INCAPABLE:
      Error{} << "GL context cannot be used to render to WebXR";
      break;
    case WEBXR_ERR_SESSION_UNSUPPORTED:
      Error{} << "VR not supported on this device";
      break;
    default:
      Error{} << "Unknown WebXR error with code" << error;
  }
}

void WebXrExample::keyPressEvent(KeyEvent& e) {
  if (e.key() == KeyEvent::Key::Esc && _inXR) {
    webxr_request_exit();
  }
}

void WebXrExample::mousePressEvent(MouseEvent& event) {
  if (event.button() != MouseEvent::Button::Left)
    return;
  /* Request rendering to the XR device */
  webxr_request_session();
  event.setAccepted();
}

MAGNUM_APPLICATION_MAIN(WebXrExample)
