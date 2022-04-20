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

float calcLerpFraction(float x, float src0, float src1) {
  BATCHED_SIM_ASSERT(src0 < src1);
  return (x - src0) / (src1 - src0);
}

}  // namespace

class BatchedSimulatorTest : public ::testing::Test {};

TEST_F(BatchedSimulatorTest, basic) {

  esp::logging::LoggingContext loggingContext;

  constexpr bool doOverlapPhysics = false;
  constexpr bool doFreeCam = false;
  bool doTuneRobotCam = false;
  constexpr bool doSaveAllFramesForVideo = false; // see make_video_from_image_files.py
  constexpr bool includeDebugSensor = true;
  constexpr bool includeDepth = false;
  constexpr bool includeColor = true;
  constexpr float cameraHfov = 70.f;

  const bool forceRandomActions = doFreeCam || doTuneRobotCam;

  EpisodeGeneratorConfig generatorConfig{
    .numEpisodes = 100,
    .seed = 0,
    .numStageVariations = 12,
    .numObjectVariations = 6,
    .minNontargetObjects = 27,
    .maxNontargetObjects = 32,
    .useFixedRobotStartPos = true,
    .useFixedRobotStartYaw = true,
    .useFixedRobotJointStartPositions = true
  };


  BatchedSimulatorConfig config{
      .numEnvs = 1, .gpuId = 0, 
      .includeDepth = includeDepth,
      .includeColor = includeColor,
      .sensor0 = {.width = 768, .height = 768},
      .numDebugEnvs = 1,
      .debugSensor {.width = 768, .height = 768},
      .forceRandomActions = forceRandomActions,
      .doAsyncPhysicsStep = doOverlapPhysics,
      .numSubsteps = 1,
      .enableRobotCollision = true,
      .enableHeldObjectCollision = true,
      .doProceduralEpisodeSet = true,
      //.episodeSetFilepath = "../data/generated.episode_set.json",
      .episodeGeneratorConfig = generatorConfig,
      .collectionFilepath = "../data/replicacad_composite.collection.json"
      };
  BatchedSimulator bsim(config);

  Mn::Vector3 camPos;
  Mn::Quaternion camRot;
  std::string cameraAttachLinkName;

  bsim.enableDebugSensor(true);

  if (!doFreeCam) {
    // over-the-shoulder cam
    // cameraAttachLinkName = "torso_lift_link";
    // camPos = {-1.18, 2.5, 0.79};
    // camRot = {{-0.254370272, -0.628166556, -0.228633955}, 0.6988765};

    // closer over-the-shoulder
    cameraAttachLinkName = "torso_lift_link";
    camPos = {-0.536559, 1.16173, 0.568379};
    camRot = {{-0.26714, -0.541109, -0.186449}, 0.775289};
    bsim.setCamera("sensor0", camPos, camRot, cameraHfov, cameraAttachLinkName);
    bsim.setCamera("debug", camPos, camRot, cameraHfov, cameraAttachLinkName);
  } else {
    if (cameraHfov == 70.f) {
      camPos = {-1.61004, 1.5, 3.5455};
      camRot = {{0, -0.529178, 0}, 0.848511};
    } else if (cameraHfov == 25.f) {
      camRot = {{-0.339537084, -0.484963357, -0.211753935}, 0.777615011};
      camPos = {-4.61396408, 9.11192417, 4.26546907};
    } else {
      CORRADE_INTERNAL_ASSERT_UNREACHABLE();
    }
    bsim.setCamera("sensor0", camPos, camRot, cameraHfov, "");
    bsim.setCamera("debug", camPos, camRot, cameraHfov, "");
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
    std::cout << "Robot controls: press WASD/arrow keys to move, G to grab/drop, +/- to adjust speed, or ESC to quit." << std::endl;
  }

  const auto& actionMap = bsim.getSerializeCollection().robots[0].actionMap;
  const int actionDim = actionMap.numActions;
  std::vector<float> actions(actionDim * config.numEnvs, 0.f);

  int nextEpisode = 0;
  const int numEpisodes = bsim.getNumEpisodes();
  auto getNextEpisode = [&]() {
    auto retVal = nextEpisode;
    nextEpisode = (nextEpisode + 1) % numEpisodes;
    return retVal;
  };

  std::vector<int> resets(config.numEnvs, -1);
  for (int b = 0; b < config.numEnvs; b++) {
    resets[b] = getNextEpisode();
  }

  bool doAdvanceSim = false;

  bsim.reset(std::vector<int>(resets));

  std::fill(resets.begin(), resets.end(), -1);

  std::vector<PythonEnvironmentState> envStates;

  int frameIdx = 0;
  bool isAutoplay = false;
  int autoplayProgress = -1;

  while (true) {

    if (doOverlapPhysics) {
      bsim.startRender();
      if (doAdvanceSim) {
        bsim.startStepPhysicsOrReset(std::vector<float>(actions), std::vector<int>(resets));
        doAdvanceSim = false;
      }
      bsim.waitRender();
      bsim.waitStepPhysicsOrReset();
    } else {
      if (doAdvanceSim) {
        bsim.startStepPhysicsOrReset(std::vector<float>(actions), std::vector<int>(resets));
        bsim.waitStepPhysicsOrReset(); 
        doAdvanceSim = false;
      }
      bsim.startRender();
      bsim.waitRender();
    }

    // make a copy of envStates
    envStates = bsim.getEnvironmentStates();

    for (bool isDebug : {false, true}) {
        
      if (isDebug && !includeDebugSensor) {
        continue;
      }

      uint8_t* base_color_ptr = isDebug
        ? bsim.getDebugBpsRenderer().getColorPointer()
        : bsim.getBpsRenderer().getColorPointer();
      // float* base_depth_ptr = isDebug
      //   ? nullptr
      //   : bsim.getBpsRenderer().getDepthPointer();
      glm::u32vec2 out_dim = isDebug
        ? glm::u32vec2(config.debugSensor.width, config.debugSensor.height)
        : glm::u32vec2(config.sensor0.width, config.sensor0.height);

      const int numEnvs = isDebug ? config.numDebugEnvs : config.numEnvs;

      for (int b = 0; b < numEnvs; b++) {
        std::stringstream ss;
        ss << (isDebug ? "./debugenv" : "./env")
          << b << "_frame"
          << std::setfill('0') << std::setw(4) << frameIdx << ".bmp";

        saveFrame(doSaveAllFramesForVideo ? ss.str().c_str() : nullptr,
                  ((isDebug ? "./latest_debugenv" : "./latest_env")
                  + std::to_string(b) + ".bmp").c_str(),
                  base_color_ptr + b * out_dim.x * out_dim.y * 4, out_dim.x,
                  out_dim.y, 4);
        // saveFrame(("./out_depth_" + std::to_string(b) + ".bmp").c_str(),
        //           base_depth_ptr + b * out_dim.x * out_dim.y, out_dim.x, out_dim.y,
        //           1);
      }
    }

    int key = 0;
    bool doStartAnimation = false;
    bool doReloadAll = false;
    if (!isAutoplay) {
      key = key_press();
    }

    if (key == 27) { // ESC
      break;
    }
    
    if (!(doFreeCam || doTuneRobotCam)) {
      
      bool doGrapRelease = false;
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
        jointPosIdx = 0 + keyIdx / 2;
        jointPlusMinus = (keyIdx % 2 == 1) ? -1.f : 1.f;
      } else if (key == 'w') {
        baseForward += 1.f;
      } else if (key == 's') {
        baseForward += -1.f;
      } else if (key == 'a') {
        baseYaw += 1.f;
      } else if (key == 'd') {
        baseYaw += -1.f;
      } else if (key == 'e') {
        baseUpDown = 0.1f;
      } else if (key == 'q') {
        baseUpDown = -0.1f;
      }else if (key == '=') { // +
        moveSpeed = Mn::Math::min(moveSpeed * 2.f, 1.f);
      } else if (key == '-') {
        moveSpeed = moveSpeed * 0.5f;
      } else if (key == 'g') {
        doGrapRelease = true;
      } else if (key == 'c') {
        doTuneRobotCam = !doTuneRobotCam;
      } else if (key == -116) {  // F5
        bsim.reloadSerializeCollection();
      } else if (key == 'r') {
        doReloadAll = true;
      }

      for (int b = 0; b < config.numEnvs; b++) {
        float* actionsForEnv = &actions[b * actionDim];

        constexpr float defaultAction = 0.f;
        for (int j = 0; j < actionDim; j++) {
          actionsForEnv[j] = defaultAction;
        }

        const auto& envState = envStates[b];

        constexpr float eps = 0.01;
        if (doGrapRelease) {
          bool isHolding = (envState.held_obj_idx != -1);
          actionsForEnv[actionMap.graspRelease.actionIdx] = !isHolding 
            ? actionMap.graspRelease.thresholds[1] + eps
            : actionMap.graspRelease.thresholds[0] - eps;
        } else {
          // do nothing
          actionsForEnv[actionMap.graspRelease.actionIdx] = actionMap.graspRelease.thresholds[0] + eps;
        }
        actionsForEnv[actionMap.baseRotate.actionIdx] = baseYaw * moveSpeed;
        actionsForEnv[actionMap.baseMove.actionIdx] = baseForward * moveSpeed;

        if (jointPosIdx != -1) {
          BATCHED_SIM_ASSERT(jointPosIdx < actionMap.joints.size());
          const auto& jointActionSetup = actionMap.joints[jointPosIdx];
          // actionsForEnv[jointActionSetup.second.actionIdx] = jointPlusMinus > 0.f
          //   ? jointPlusMinus * jointActionSetup.second.stepMax * moveSpeed
          //   : -jointPlusMinus * jointActionSetup.second.stepMin * moveSpeed;
          actionsForEnv[jointActionSetup.second.actionIdx]  = jointPlusMinus * moveSpeed;
        }
        

        // 0-1 I don't know these
        // 2 is torso up/down
        // 3-6 I don't know these
        // 7 shoulder, + is down
        // 8 shoulder twist, + is twist to right
        // 9 elbow, + is down
        // 10 elbow twist, + is twst to right
        // 11 wrist, + is down
        // 12 wrist twist, + is right
        // 13 gripper finger 1
        // 14 gripper finger 2
        //actionsForEnv[3 + 8] = -targetLeftRight * rotSpeed;
        // actionsForEnv[3 + 2] = baseUpDown * moveSpeed;
        // actionsForEnv[3 + 11] = -targetUpDown * moveSpeed;
        // actionsForEnv[3 + 12] = -targetLeftRight * moveSpeed;
        // // always have grippers open
        // actionsForEnv[3 + 13] = 1.f;
        // actionsForEnv[3 + 14] = 1.f;
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
      } else if (key == 'c') {
        doTuneRobotCam = !doTuneRobotCam;
      } else if (key == 'r') {
        bsim.reloadSerializeCollection();
      }

      if (doFreeCam) {
        bsim.setCamera("sensor0", camPos, camRot, cameraHfov, "");
        bsim.setCamera("debug", camPos, camRot, cameraHfov, "");
      } else {
        const auto cameraMat = Mn::Matrix4::from(camRot.toMatrix(), camPos);
        ESP_DEBUG() << "camPos: " << camPos << ", camRot: " << camRot;
        bsim.setCamera("sensor0", camPos, camRot, cameraHfov, cameraAttachLinkName);
        bsim.setCamera("debug", camPos, camRot, cameraHfov, cameraAttachLinkName);
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
      bsim.setCamera("sensor0", camPos, camRot, cameraHfov, "");
      bsim.setCamera("debug", camPos, camRot, cameraHfov, "");

      if (autoplayProgress >= animDuration) {
        isAutoplay = false;
      }
    }

    #if 1
    for (int b = 0; b < config.numEnvs; b++) {
      // end episode on collision?
      if (/*envStates[b].did_collide */ false
          || doReloadAll) {
        resets[b] = getNextEpisode();
      } else {
        resets[b] = -1;
      }
    }
    #endif

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