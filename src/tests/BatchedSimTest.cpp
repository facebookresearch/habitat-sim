// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/batched_sim/BatchedSimulator.h"
#include "esp/batched_sim/BpsSceneMapping.h"
#include "esp/batched_sim/GlmUtils.h"
#include "esp/batched_sim/ColumnGrid.h"

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

using namespace esp::batched_sim;

namespace Mn = Magnum;

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
               const uint8_t* dev_ptr,
               uint32_t width,
               uint32_t height,
               uint32_t num_channels) {
  auto buffer = copyToHost(dev_ptr, width, height, num_channels);

  stbi_write_bmp(fname, width, height, num_channels, buffer.data());
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

  BatchedSimulatorConfig config{
      .numEnvs = 1, .gpuId = 0, .sensor0 = {.width = 768, .height = 768, .hfov = 60}};
  BatchedSimulator bsim(config);

  Mn::Vector3 camPos{-1.61004, 1.5, 3.5455};
  Mn::Quaternion camRot{{0, -0.529178, 0}, 0.848511};

  float rotSpeed = 20.f;
  float moveSpeed = 0.5f;

  const int envIndex = 0;
  int sphereGreenInstance = bsim.addInstance("sphere_green", envIndex);
  int sphereOrangeInstance = bsim.addInstance("sphere_orange", envIndex);
  auto& bpsEnv = bsim.getBpsEnvironment(0);

  esp::batched_sim::ColumnGridSource source;
  source.load("../data/columngrids/Baked_sc0_staging_00_stage_only.columngrid");

  std::cout << "Open ./out_color_0.bmp in VS Code or another viewer that supports hot-reload." << std::endl;
  std::cout << "Press WASDQE/arrow keys to move/look, +/- to adjust speed, or ESC to quit." << std::endl;

  while (true) {
    for (int i = 0; i < 1; i++) {
      bsim.autoResetOrStepPhysics();
      bsim.getRewards();
      bsim.getDones();
    }
    bsim.startRender();
    bsim.waitForFrame();

    uint8_t* base_color_ptr = bsim.getBpsRenderer().getColorPointer();
    float* base_depth_ptr = bsim.getBpsRenderer().getDepthPointer();

    // temp hack copied from BpsWrapper internals
    glm::u32vec2 out_dim(config.sensor0.width, config.sensor0.height);

    for (int b = 0; b < config.numEnvs; b++) {
      saveFrame(("./out_color_" + std::to_string(b) + ".bmp").c_str(),
                base_color_ptr + b * out_dim.x * out_dim.y * 4, out_dim.x,
                out_dim.y, 4);
      // saveFrame(("./out_depth_" + std::to_string(b) + ".bmp").c_str(),
      //           base_depth_ptr + b * out_dim.x * out_dim.y, out_dim.x, out_dim.y,
      //           1);
    }

    int key = key_press();
    if (key == 27) { // ESC
      break;
    } else if (key == -38) { // up
      camRot = camRot * Mn::Quaternion::rotation(Mn::Deg(rotSpeed), Mn::Vector3(1.f, 0.f, 0.f));
    } else if (key == -40) { // down
      camRot = camRot * Mn::Quaternion::rotation(Mn::Deg(-rotSpeed), Mn::Vector3(1.f, 0.f, 0.f));
    } else if (key == -37) { // left
      camRot = Mn::Quaternion::rotation(Mn::Deg(rotSpeed), Mn::Vector3(0.f, 1.f, 0.f)) * camRot;
    } else if (key == -39) { // right
      camRot = Mn::Quaternion::rotation(Mn::Deg(-rotSpeed), Mn::Vector3(0.f, 1.f, 0.f)) * camRot;
    } else if (key == '=') { // +
      rotSpeed *= 1.25f;
      moveSpeed *= 1.5f;
    } else if (key == '-') {
      rotSpeed /= 1.25f;
      moveSpeed /= 1.5f;
    } else if (key == 'w') {
      camPos += camRot.transformVector(Mn::Vector3(0.f, 0.f, -1.f)) * moveSpeed;
    } else if (key == 's') {
      camPos += camRot.transformVector(Mn::Vector3(0.f, 0.f, 1.f)) * moveSpeed;
    } else if (key == 'a') {
      camPos += camRot.transformVector(Mn::Vector3(-1.f, 0.f, 0.f)) * moveSpeed;
    } else if (key == 'd') {
      camPos += camRot.transformVector(Mn::Vector3(1.f, 0.f, 0.f)) * moveSpeed;      
    } else if (key == 'e') {
      camPos.y() += moveSpeed;
    } else if (key == 'q') {
      camPos.y() += -moveSpeed;
    }

    bsim.setCamera(camPos, camRot);    

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

      bpsEnv.updateInstanceTransform(instanceToShow, toGlmMat4x3(mat));
      bpsEnv.updateInstanceTransform(instanceToHide, 
        toGlmMat4x3(Mn::Matrix4::translation({1000.f, 1000.f, 1000.f})));
    }
  }
}
