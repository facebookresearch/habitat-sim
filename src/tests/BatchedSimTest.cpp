// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/batched_sim/BatchedSimAssert.h"
#include "esp/batched_sim/BatchedSimulator.h"
#include "esp/batched_sim/BpsSceneMapping.h"
#include "esp/batched_sim/GlmUtils.h"
#include "esp/batched_sim/ColumnGrid.h"
#include "esp/core/logging.h"

#include <cuda_runtime.h>
#include <gtest/gtest.h>
#include <bps3D.hpp>
#include <glm/gtx/transform.hpp>

// FIXME
// #define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

// for key_press
#include <sys/ioctl.h>
#include <termios.h>

#include <sstream>

namespace Mn = Magnum;

namespace esp {
namespace batched_sim {

namespace {
template <typename T>
static std::vector<T> copyToHost(const T* dev_ptr,
                                 uint32_t width,
                                 uint32_t height,
                                 uint32_t num_channels) {
  uint64_t num_pixels = width * height * num_channels;

  std::vector<T> buffer(num_pixels);

  cudaMemcpy(buffer.data(), dev_ptr, sizeof(T) * num_pixels,
             cudaMemcpyDeviceToHost);

  return buffer;
}

void saveFrame(const char* fname,
               const float* dev_ptr,
               uint32_t width,
               uint32_t height,
               uint32_t num_channels) {
  auto buffer = copyToHost(dev_ptr, width, height, num_channels);

  std::vector<uint8_t> sdr_buffer(buffer.size());
  for (unsigned i = 0; i < buffer.size(); i++) {
    float v = buffer[i];
    if (v < 0)
      v = 0;
    if (v > 1)
      v = 1;
    sdr_buffer[i] = v * 255;
  }

  stbi_write_bmp(fname, width, height, num_channels, sdr_buffer.data());
}

void saveFrame(const char* fname,
               const char* fname2,
               const uint8_t* dev_ptr,
               uint32_t width,
               uint32_t height,
               uint32_t num_channels) {
  auto buffer = copyToHost(dev_ptr, width, height, num_channels);

  if (fname) {
    stbi_write_bmp(fname, width, height, num_channels, buffer.data());
  }
  if (fname2) {
    stbi_write_bmp(fname2, width, height, num_channels, buffer.data());
  }
}

// Hacky way to get keypresses at the terminal. Linux only.
// https://stackoverflow.com/a/67038432
int key_press() { // not working: ¹ (251), num lock (-144), caps lock (-20), windows key (-91), kontext menu key (-93)
    struct termios term;
    tcgetattr(0, &term);
    while(true) {
        term.c_lflag &= ~(ICANON|ECHO); // turn off line buffering and echoing
        tcsetattr(0, TCSANOW, &term);
        int nbbytes;
        ioctl(0, FIONREAD, &nbbytes); // 0 is STDIN
        while(!nbbytes) {
            sleep(0.01);
            fflush(stdout);
            ioctl(0, FIONREAD, &nbbytes); // 0 is STDIN
        }
        int key = (int)getchar();
        if(key==27||key==194||key==195) { // escape, 194/195 is escape for °ß´äöüÄÖÜ
            key = (int)getchar();
            if(key==91) { // [ following escape
                key = (int)getchar(); // get code of next char after \e[
                if(key==49) { // F5-F8
                    key = 62+(int)getchar(); // 53, 55-57
                    if(key==115) key++; // F5 code is too low by 1
                    getchar(); // take in following ~ (126), but discard code
                } else if(key==50) { // insert or F9-F12
                    key = (int)getchar();
                    if(key==126) { // insert
                        key = 45;
                    } else { // F9-F12
                        key += 71; // 48, 49, 51, 52
                        if(key<121) key++; // F11 and F12 are too low by 1
                        getchar(); // take in following ~ (126), but discard code
                    }
                } else if(key==51||key==53||key==54) { // delete, page up/down
                    getchar(); // take in following ~ (126), but discard code
                }
            } else if(key==79) { // F1-F4
                key = 32+(int)getchar(); // 80-83
            }
            key = -key; // use negative numbers for escaped keys
        }
        term.c_lflag |= (ICANON|ECHO); // turn on line buffering and echoing
        tcsetattr(0, TCSANOW, &term);
        switch(key) {
            case  127: return   8; // backspace
            case  -27: return  27; // escape
            case  -51: return 127; // delete
            case -164: return 132; // ä
            case -182: return 148; // ö
            case -188: return 129; // ü
            case -132: return 142; // Ä
            case -150: return 153; // Ö
            case -156: return 154; // Ü
            case -159: return 225; // ß
            case -181: return 230; // µ
            case -167: return 245; // §
            case -176: return 248; // °
            case -178: return 253; // ²
            case -179: return 252; // ³
            case -180: return 239; // ´
            case  -65: return -38; // up arrow
            case  -66: return -40; // down arrow
            case  -68: return -37; // left arrow
            case  -67: return -39; // right arrow
            case  -53: return -33; // page up
            case  -54: return -34; // page down
            case  -72: return -36; // pos1
            case  -70: return -35; // end
            case    0: continue;
            case    1: continue; // disable Ctrl + a
            case    2: continue; // disable Ctrl + b
            case    3: continue; // disable Ctrl + c (terminates program)
            case    4: continue; // disable Ctrl + d
            case    5: continue; // disable Ctrl + e
            case    6: continue; // disable Ctrl + f
            case    7: continue; // disable Ctrl + g
            case    8: continue; // disable Ctrl + h
            //case    9: continue; // disable Ctrl + i (ascii for tab)
            //case   10: continue; // disable Ctrl + j (ascii for new line)
            case   11: continue; // disable Ctrl + k
            case   12: continue; // disable Ctrl + l
            case   13: continue; // disable Ctrl + m
            case   14: continue; // disable Ctrl + n
            case   15: continue; // disable Ctrl + o
            case   16: continue; // disable Ctrl + p
            case   17: continue; // disable Ctrl + q
            case   18: continue; // disable Ctrl + r
            case   19: continue; // disable Ctrl + s
            case   20: continue; // disable Ctrl + t
            case   21: continue; // disable Ctrl + u
            case   22: continue; // disable Ctrl + v
            case   23: continue; // disable Ctrl + w
            case   24: continue; // disable Ctrl + x
            case   25: continue; // disable Ctrl + y
            case   26: continue; // disable Ctrl + z (terminates program)
            default: return key; // any other ASCII character
        }
    }
}

}  // namespace

class BatchedSimulatorTest : public ::testing::Test {};

TEST_F(BatchedSimulatorTest, basic) {

  esp::logging::LoggingContext loggingContext;

  constexpr bool doOverlapPhysics = false;
  constexpr bool doFreeCam = false;
  bool doTuneRobotCam = false;
  constexpr bool doSaveAllFramesForVideo = false; // see make_video_from_image_files.py
  constexpr bool doPairedDebugEnvs = true;

  const bool forceRandomActions = doFreeCam || doTuneRobotCam;

  BatchedSimulatorConfig config{
      // for video: 2048 x 1024, fov 80
      .numEnvs = 8, .gpuId = 0, .sensor0 = {.width = 768, .height = 768, .hfov = 70},
      .forceRandomActions = forceRandomActions,
      .doAsyncPhysicsStep = doOverlapPhysics,
      .maxEpisodeLength = 2000,
      .doPairedDebugEnvs = doPairedDebugEnvs
      };
  BatchedSimulator bsim(config);

  Mn::Vector3 camPos;
  Mn::Quaternion camRot;
  std::string cameraAttachLinkName;

  if (!doFreeCam) {
    // over-the-shoulder cam

    // cameraAttachLinkName = "torso_lift_link";
    // camPos = {-1.18, 2.5, 0.79};
    // camRot = {{-0.254370272, -0.628166556, -0.228633955}, 0.6988765};

    // closer over-the-shoulder
    cameraAttachLinkName = "torso_lift_link";
    camPos = {-0.536559, 1.16173, 0.568379};
    camRot = {{-0.26714, -0.541109, -0.186449}, 0.775289};

    const auto cameraMat = Mn::Matrix4::from(camRot.toMatrix(), camPos);
    bsim.attachCameraToLink(cameraAttachLinkName, cameraMat);
  } else {
    camPos = {-1.61004, 1.5, 3.5455};
    camRot = {{0, -0.529178, 0}, 0.848511};
    bsim.setCamera(camPos, camRot);    
  }

  float moveSpeed = 1.f;

  const int envIndex = 0;
  //int sphereGreenInstance = bsim.addDebugInstance("sphere_green_wireframe", envIndex);
  //int sphereOrangeInstance = bsim.addDebugInstance("sphere_orange_wireframe", envIndex);
  // int testModelInstance = bsim.addDebugInstance("cube_gray_shaded", envIndex, 
  //   Mn::Matrix4(Mn::Math::IdentityInit), /*persistent*/true);
  // auto& bpsEnv = bsim.getBpsEnvironment(0);

  // esp::batched_sim::ColumnGridSource source;
  // source.load("../data/columngrids/Baked_sc0_staging_00_stage_only.columngrid");

  std::cout << "Open ./latest_env0.bmp in VS Code or another viewer that supports hot-reload." << std::endl;
  if (doFreeCam || doTuneRobotCam) {
    std::cout << "Camera controls: press WASDQE/arrow keys to move/look, +/- to adjust speed, or ESC to quit." << std::endl;
  } else {
    std::cout << "Robot controls: press WASD/arrow keys to move, E to grab/drop, +/- to adjust speed, or ESC to quit." << std::endl;
  }

  constexpr int actionDim = 18;
  std::vector<float> actions(actionDim * config.numEnvs, 0.f);
  if (doFreeCam) {
    for (int b = 0; b < config.numEnvs; b++) {
      actions[b * actionDim + 0] = 1.f; // attempt grip
      //actions[b * actionDim + 1] = (b % 2 == 0) ? 0.1f : -0.1f; // base rotate
      actions[b * actionDim + 2] = 0.05f; // base forward
      int jointForThisEnv = ((b / 2) % 15);
      actions[b * actionDim + 3 + jointForThisEnv] = (b % 2 == 0) ? 0.03f : -0.03f;
    }
  }

  bool doAdvanceSim = false;

  if (doOverlapPhysics) {
    bsim.autoResetOrStartAsyncStepPhysics();
  } else {
    bsim.autoResetOrStepPhysics();
  }

  int frameIdx = 0;
  bool isAutoplay = false;
  int autoplayProgress = -1;

  while (true) {

    if (doOverlapPhysics) {
      bsim.waitAsyncStepPhysics();
      if (doAdvanceSim) {
        bsim.setActions(std::vector<float>(actions));
        bsim.autoResetOrStartAsyncStepPhysics();
        doAdvanceSim = false;
      }
      bsim.startRender();
      bsim.waitForFrame();
    } else {
      if (doAdvanceSim) {
        bsim.setActions(std::vector<float>(actions));
        bsim.autoResetOrStepPhysics();
        doAdvanceSim = false;
      }
      bsim.startRender();
      bsim.waitForFrame();
    }

    uint8_t* base_color_ptr = bsim.getBpsRenderer().getColorPointer();
    float* base_depth_ptr = bsim.getBpsRenderer().getDepthPointer();

    // temp hack copied from BpsWrapper internals
    glm::u32vec2 out_dim(config.sensor0.width, config.sensor0.height);

    for (int b = 0; b < config.numEnvs; b++) {
      std::stringstream ss;
      ss << "./env" << b << "_frame"
        << std::setfill('0') << std::setw(4) << frameIdx << ".bmp";

      saveFrame(doSaveAllFramesForVideo ? ss.str().c_str() : nullptr,
                ("./latest_env" + std::to_string(b) + ".bmp").c_str(),
                base_color_ptr + b * out_dim.x * out_dim.y * 4, out_dim.x,
                out_dim.y, 4);
      // saveFrame(("./out_depth_" + std::to_string(b) + ".bmp").c_str(),
      //           base_depth_ptr + b * out_dim.x * out_dim.y, out_dim.x, out_dim.y,
      //           1);
    }

    int key = 0;
    bool doStartAnimation = false;
    if (!isAutoplay) {
      key = key_press();
    }

    if (key == 27) { // ESC
      break;
    }
    
    if (!(doFreeCam || doTuneRobotCam)) {
      
      static bool doHold = false;
      float targetUpDown = 0.f;
      float targetLeftRight = 0.f;
      float baseYaw = 0.f;
      float baseForward = 0.f;
      float baseUpDown = 0.f;
      int jointPosIdx = -1;
      float jointPlusMinus = 0.f;

      doAdvanceSim = true;

      if (key == -38) { // up
        targetUpDown += 1.f;
      } else if (key == -40) { // down
        targetUpDown -= 1.f;
      } else if (key == -37) { // left
        targetLeftRight += 1.f;
      } else if (key == -39) { // right
        targetLeftRight -= 1.f;
      } else if (key >= '1' && key <= '8') {
        int keyIdx = key - '1';
        jointPosIdx = 7 + keyIdx / 2;
        jointPlusMinus = (keyIdx % 2 == 1) ? -1.f : 1.f;
      } else if (key == 'w') {
        baseForward += 1.f;
      } else if (key == 's') {
        baseForward -= 1.f;
      } else if (key == 'a') {
        baseYaw += float(Mn::Rad(Mn::Deg(15.f)));
      } else if (key == 'd') {
        baseYaw -= float(Mn::Rad(Mn::Deg(15.f)));
      } else if (key == 'e') {
        baseUpDown = 0.1f;
      } else if (key == 'q') {
        baseUpDown = -0.1f;
      }else if (key == '=') { // +
        moveSpeed *= 1.5f;
      } else if (key == '-') {
        moveSpeed /= 1.5f;
      } else if (key == 'g') {
        doHold = !doHold;
      } else if (key == 'c') {
        doTuneRobotCam = !doTuneRobotCam;
      } else if (key == 'r') {
        bsim.reloadSerializeCollection();
      } else if (key == 'u') {
        if (frameIdx > 0) {
          bsim.reverseRobotMovementActions();
          frameIdx -= 2; // undoing two keypresses (the last one, and this 'u')
        }
      }

      for (int b = 0; b < config.numEnvs; b++) {
        float* actionsForEnv = &actions[b * actionDim];

        for (int j = 0; j < actionDim; j++) {
          actionsForEnv[j] = 0.f;
        }

        actionsForEnv[0] = doHold ? 1.f : -1.f;
        actionsForEnv[1] = baseYaw * moveSpeed;
        actionsForEnv[2] = baseForward * moveSpeed;

        // 2 is torso up/down
        // 7 shoulder, + is down
        // 8 twist, + is twist to right
        // 9 elbow, + is down
        // 10 elbow twist, + is twst to right
        // 11 wrist, + is down
        // 12 wrist twise, + is right
        //actionsForEnv[3 + 8] = -targetLeftRight * rotSpeed;
        actionsForEnv[3 + 2] = baseUpDown * moveSpeed;
        actionsForEnv[3 + 11] = -targetUpDown * moveSpeed;
        actionsForEnv[3 + 12] = -targetLeftRight * moveSpeed;

        if (jointPosIdx != -1) {
          actionsForEnv[3 + jointPosIdx] = jointPlusMinus;
        }

        // always have grippers open
        actionsForEnv[3 + 13] = 1.f;
        actionsForEnv[3 + 14] = 1.f;
      }

    } else {
      const float camMoveSpeed = 0.2f * moveSpeed;
      const float camRotateSpeed = 20.f * moveSpeed;
      // free camera with spacebar to advance sim
      if (key == -38) { // up
        camRot = camRot * Mn::Quaternion::rotation(Mn::Deg(camRotateSpeed), Mn::Vector3(1.f, 0.f, 0.f));
      } else if (key == -40) { // down
        camRot = camRot * Mn::Quaternion::rotation(Mn::Deg(-camRotateSpeed), Mn::Vector3(1.f, 0.f, 0.f));
      } else if (key == -37) { // left
        camRot = Mn::Quaternion::rotation(Mn::Deg(camRotateSpeed), Mn::Vector3(0.f, 1.f, 0.f)) * camRot;
      } else if (key == -39) { // right
        camRot = Mn::Quaternion::rotation(Mn::Deg(-camRotateSpeed), Mn::Vector3(0.f, 1.f, 0.f)) * camRot;
      } else if (key == '=') { // +
        moveSpeed *= 1.5f;
      } else if (key == '-') {
        moveSpeed /= 1.5f;
      } else if (key == 'w') {
        camPos += camRot.transformVector(Mn::Vector3(0.f, 0.f, -1.f)) * camMoveSpeed;
      } else if (key == 's') {
        camPos += camRot.transformVector(Mn::Vector3(0.f, 0.f, 1.f)) * camMoveSpeed;
      } else if (key == 'a') {
        camPos += camRot.transformVector(Mn::Vector3(-1.f, 0.f, 0.f)) * camMoveSpeed;
      } else if (key == 'd') {
        camPos += camRot.transformVector(Mn::Vector3(1.f, 0.f, 0.f)) * camMoveSpeed;      
      } else if (key == 'e') {
        camPos.y() += camMoveSpeed;
      } else if (key == 'q') {
        camPos.y() += -camMoveSpeed;
      } else if (key == ' ') {
        doAdvanceSim = true;
      } else if (key == 't') {
        doStartAnimation = true;
      } else if (key == 'u') {
        if (frameIdx > 0) {
          bsim.reverseRobotMovementActions();
          frameIdx -= 2; // undoing two keypresses (the last one, and this 'u')
        }
      } else if (key == 'c') {
        doTuneRobotCam = !doTuneRobotCam;
      } else if (key == 'r') {
        bsim.reloadSerializeCollection();
      }

      if (doFreeCam) {
        bsim.setCamera(camPos, camRot);
      } else {
        const auto cameraMat = Mn::Matrix4::from(camRot.toMatrix(), camPos);
        ESP_DEBUG() << "camPos: " << camPos << ", camRot: " << camRot;
        bsim.attachCameraToLink(cameraAttachLinkName, cameraMat);
      }
    }

    if (doStartAnimation) {
      isAutoplay = true;
      autoplayProgress = 0;
    }
    if (isAutoplay) {

      constexpr int animDuration = 50;
      constexpr int renderInc = (250 + 450) / animDuration;
      int prevProgress = autoplayProgress;
      autoplayProgress++;
      bsim.debugRenderColumnGrids(prevProgress * renderInc, autoplayProgress * renderInc);

      // move camera slightly
      camPos.y() -= 0.01f;
      bsim.setCamera(camPos, camRot);

      if (autoplayProgress >= animDuration) {
        isAutoplay = false;
      }
    }

    #if 0 // test columnGrid collision
    {
      constexpr float sphereDist = 1.f;
      constexpr float sphereRadius = 0.1f; // todo: get from ColumnGrid
      Mn::Vector3 spherePos = camPos + camRot.transformVector(Mn::Vector3(0.f, 0.f, -sphereDist));

      esp::batched_sim::ColumnGridSource::QueryCacheValue queryCache = 0;
      bool contact = source.contactTest(spherePos, &queryCache);

      Mn::Matrix4 mat = Mn::Matrix4::translation(spherePos)
        * Mn::Matrix4::scaling(Mn::Vector3(sphereRadius, sphereRadius, sphereRadius));
      
      int instanceToShow = contact ? sphereOrangeInstance : sphereGreenInstance;
      int instanceToHide = contact ? sphereGreenInstance : sphereOrangeInstance;

      CORRADE_INTERNAL_ASSERT(instanceToShow != -1);
      bpsEnv.updateInstanceTransform(instanceToShow, toGlmMat4x3(mat));
      bpsEnv.updateInstanceTransform(instanceToHide, 
        toGlmMat4x3(Mn::Matrix4::translation({1000.f, 1000.f, 1000.f})));
    }
    #endif

    frameIdx++;
    if (frameIdx % 20 == 0) {
      ESP_DEBUG() << "batched_sim stats: " << bsim.getRecentStatsAndReset();          
    }
  }

  bsim.close();
  bsim.close(); // try a redundant close
}

}
}