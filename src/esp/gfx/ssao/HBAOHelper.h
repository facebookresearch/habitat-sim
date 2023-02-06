// todo: figure out licensing and header here. This was adapted from
// https://github.com/nvpro-samples/gl_ssao

// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef HBAOHELPER_H_
#define HBAOHELPER_H_

#include <algorithm>  // hack to avoid nvpro_core compile problems

// todo before PR merge: get rid of all nvpro dependencies
#include "nvpro_core/nvmath/nvmath.h"
#include "nvpro_core/nvmath/nvmath_glsltypes.h"

#include "nvpro_core/nvgl/programmanager_gl.hpp"

#include "common.h"

namespace esp {
namespace gfx {
namespace ssao {

#define AO_RANDOMTEX_SIZE 4
static const int MAX_SAMPLES = 8;
static const int HBAO_RANDOM_SIZE = AO_RANDOMTEX_SIZE;
static const int HBAO_RANDOM_ELEMENTS = HBAO_RANDOM_SIZE * HBAO_RANDOM_SIZE;

class HBAOHelper {
 public:
  struct Config {
    int samples = 1;
    float intensity = 0.732f;  // hand-tuned
    float bias = 0.05f;
    float radius = 1.24f;
    float blurSharpness = 10.0f;
    bool blur = true;  // true;
  };

  struct Projection {
    float nearplane = 0.1f;
    float farplane = 100.0f;
    float fov = 45.0f;
    float orthoheight = 1.0f;
    bool ortho = false;
    nvmath::mat4 matrix;

    void update(int width, int height) {
      float aspect = float(width) / float(height);
      if (ortho) {
        matrix = nvmath::ortho(-orthoheight * 0.5f * aspect,
                               orthoheight * 0.5f * aspect, -orthoheight * 0.5f,
                               orthoheight * 0.5f, nearplane, farplane);
      } else {
        matrix = nvmath::perspective(fov, aspect, nearplane, farplane);
      }
    }
  };

  void init(int width, int height);
  bool initFramebuffers(int width, int height, int samples);
  bool initMisc();
  bool initPrograms();

  void prepareHbaoData(const Projection& projection, int width, int height);

  void drawLinearDepth(const Projection& projection,
                       int width,
                       int height,
                       int sampleIdx,
                       GLuint scene_depthstencil);
  void drawHbaoBlur(const Projection& projection,
                    int width,
                    int height,
                    int sampleIdx,
                    GLuint fbo_scene);
  void drawHbaoClassic(const Projection& projection,
                       int width,
                       int height,
                       int sampleIdx,
                       GLuint scene_depthstencil,
                       GLuint fbo_scene);
  void drawHbaoCacheAware(const Projection& projection,
                          int width,
                          int height,
                          int sampleIdx,
                          GLuint scene_depthstencil,
                          GLuint fbo_scene);

  Config m_config;

 private:
  struct {
    GLuint scene_viewnormal = 0;
    GLuint scene_depthlinear = 0;
    GLuint hbao_result = 0;
    GLuint hbao_blur = 0;
    GLuint hbao_random = 0;
    GLuint hbao_randomview[MAX_SAMPLES] = {0};
    GLuint hbao2_deptharray = 0;
    GLuint hbao2_depthview[HBAO_RANDOM_ELEMENTS] = {0};
    GLuint hbao2_resultarray = 0;
  } textures;

  struct {
    GLuint hbao_ubo = 0;
  } buffers;

  struct {
    // GLuint scene              = 0;
    GLuint depthlinear = 0;
    GLuint viewnormal = 0;
    GLuint hbao_calc = 0;
    GLuint hbao2_deinterleave = 0;
    GLuint hbao2_calc = 0;
  } fbos;

  struct {
    nvgl::ProgramID depth_linearize, depth_linearize_msaa, viewnormal,
        bilateralblur, displaytex,

        hbao_calc, hbao_calc_blur, hbao_blur, hbao_blur2,

        hbao2_deinterleave, hbao2_calc, hbao2_calc_blur, hbao2_reinterleave,
        hbao2_reinterleave_blur;

  } programs;

  HBAOData m_hbaoUbo;
  nvmath::vec4f m_hbaoRandom[HBAO_RANDOM_ELEMENTS * MAX_SAMPLES];
  nvgl::ProgramManager m_progManager;
  GLuint scene_depthstencil = 0;  // not owned
};

}  // namespace ssao
}  // namespace gfx
}  // namespace esp

#endif
