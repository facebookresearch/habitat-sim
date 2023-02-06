// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "HBAOHelper.h"

// todo before PR merge: get rid of all nvpro dependencies
#include "nvpro_core/nvgl/base_gl.hpp"

#include <random>

namespace esp {
namespace gfx {
namespace ssao {

// optimizes blur, by storing depth along with ssao calculation
// avoids accessing two different textures
#define USE_AO_SPECIALBLUR 1

#define AO_LAYERED_OFF 0
#define AO_LAYERED_IMAGE 1
#define AO_LAYERED_GS 2

#define USE_AO_LAYERED_SINGLEPASS AO_LAYERED_GS

static const int NUM_MRT = 8;

using namespace nvmath;

void HBAOHelper::init(int width, int height) {
  constexpr int samples = 1;
  initPrograms();
  initFramebuffers(width, height, samples);
  initMisc();
}

bool HBAOHelper::initPrograms() {
  m_progManager.m_filetype = nvh::ShaderFileManager::FILETYPE_GLSL;
  // m_progManager.addDirectory(std::string("GLSL_" PROJECT_NAME));
  // m_progManager.addDirectory("./");

  m_progManager.addDirectory("./src/esp/gfx/ssao");

  m_progManager.registerInclude("common.h");

  m_progManager.m_prepend = nvgl::ProgramManager::format(
      "#define AO_LAYERED %d\n", USE_AO_LAYERED_SINGLEPASS);

  programs.bilateralblur = m_progManager.createProgram(
      nvgl::ProgramManager::Definition(GL_VERTEX_SHADER,
                                       "fullscreenquad.vert.glsl"),
      nvgl::ProgramManager::Definition(GL_FRAGMENT_SHADER,
                                       "bilateralblur.frag.glsl"));

  programs.depth_linearize = m_progManager.createProgram(
      nvgl::ProgramManager::Definition(GL_VERTEX_SHADER,
                                       "fullscreenquad.vert.glsl"),
      nvgl::ProgramManager::Definition(GL_FRAGMENT_SHADER,
                                       "#define DEPTHLINEARIZE_MSAA 0\n",
                                       "depthlinearize.frag.glsl"));

  programs.depth_linearize_msaa = m_progManager.createProgram(
      nvgl::ProgramManager::Definition(GL_VERTEX_SHADER,
                                       "fullscreenquad.vert.glsl"),
      nvgl::ProgramManager::Definition(GL_FRAGMENT_SHADER,
                                       "#define DEPTHLINEARIZE_MSAA 1\n",
                                       "depthlinearize.frag.glsl"));

  programs.viewnormal = m_progManager.createProgram(
      nvgl::ProgramManager::Definition(GL_VERTEX_SHADER,
                                       "fullscreenquad.vert.glsl"),
      nvgl::ProgramManager::Definition(GL_FRAGMENT_SHADER,
                                       "viewnormal.frag.glsl"));

  programs.displaytex = m_progManager.createProgram(
      nvgl::ProgramManager::Definition(GL_VERTEX_SHADER,
                                       "fullscreenquad.vert.glsl"),
      nvgl::ProgramManager::Definition(GL_FRAGMENT_SHADER,
                                       "displaytex.frag.glsl"));

  programs.hbao_calc = m_progManager.createProgram(
      nvgl::ProgramManager::Definition(GL_VERTEX_SHADER,
                                       "fullscreenquad.vert.glsl"),
      nvgl::ProgramManager::Definition(
          GL_FRAGMENT_SHADER, "#define AO_DEINTERLEAVED 0\n#define AO_BLUR 0\n",
          "hbao.frag.glsl"));

  programs.hbao_calc_blur = m_progManager.createProgram(
      nvgl::ProgramManager::Definition(GL_VERTEX_SHADER,
                                       "fullscreenquad.vert.glsl"),
      nvgl::ProgramManager::Definition(
          GL_FRAGMENT_SHADER, "#define AO_DEINTERLEAVED 0\n#define AO_BLUR 1\n",
          "hbao.frag.glsl"));

  programs.hbao_blur = m_progManager.createProgram(
      nvgl::ProgramManager::Definition(GL_VERTEX_SHADER,
                                       "fullscreenquad.vert.glsl"),
      nvgl::ProgramManager::Definition(GL_FRAGMENT_SHADER,
                                       "#define AO_BLUR_PRESENT 0\n",
                                       "hbao_blur.frag.glsl"));

  programs.hbao_blur2 = m_progManager.createProgram(
      nvgl::ProgramManager::Definition(
          GL_VERTEX_SHADER,
          "fullscreenquad.vert.glsl"),  // "halfscreenquad.vert.glsl"),
      nvgl::ProgramManager::Definition(GL_FRAGMENT_SHADER,
                                       "#define AO_BLUR_PRESENT 1\n",
                                       "hbao_blur.frag.glsl"));

  programs.hbao2_calc = m_progManager.createProgram(
      nvgl::ProgramManager::Definition(GL_VERTEX_SHADER,
                                       "fullscreenquad.vert.glsl"),
#if USE_AO_LAYERED_SINGLEPASS == AO_LAYERED_GS
      nvgl::ProgramManager::Definition(GL_GEOMETRY_SHADER,
                                       "fullscreenquad.geo.glsl"),
#endif
      nvgl::ProgramManager::Definition(
          GL_FRAGMENT_SHADER, "#define AO_DEINTERLEAVED 1\n#define AO_BLUR 0\n",
          "hbao.frag.glsl"));

  programs.hbao2_calc_blur = m_progManager.createProgram(
      nvgl::ProgramManager::Definition(GL_VERTEX_SHADER,
                                       "fullscreenquad.vert.glsl"),
#if USE_AO_LAYERED_SINGLEPASS == AO_LAYERED_GS
      nvgl::ProgramManager::Definition(GL_GEOMETRY_SHADER,
                                       "fullscreenquad.geo.glsl"),
#endif
      nvgl::ProgramManager::Definition(
          GL_FRAGMENT_SHADER, "#define AO_DEINTERLEAVED 1\n#define AO_BLUR 1\n",
          "hbao.frag.glsl"));

  programs.hbao2_deinterleave = m_progManager.createProgram(
      nvgl::ProgramManager::Definition(GL_VERTEX_SHADER,
                                       "fullscreenquad.vert.glsl"),
      nvgl::ProgramManager::Definition(GL_FRAGMENT_SHADER,
                                       "hbao_deinterleave.frag.glsl"));

  programs.hbao2_reinterleave = m_progManager.createProgram(
      nvgl::ProgramManager::Definition(GL_VERTEX_SHADER,
                                       "fullscreenquad.vert.glsl"),
      nvgl::ProgramManager::Definition(GL_FRAGMENT_SHADER,
                                       "#define AO_BLUR 0\n",
                                       "hbao_reinterleave.frag.glsl"));

  programs.hbao2_reinterleave_blur = m_progManager.createProgram(
      nvgl::ProgramManager::Definition(GL_VERTEX_SHADER,
                                       "fullscreenquad.vert.glsl"),
      nvgl::ProgramManager::Definition(GL_FRAGMENT_SHADER,
                                       "#define AO_BLUR 1\n",
                                       "hbao_reinterleave.frag.glsl"));

  bool validated = m_progManager.areProgramsValid();

  // todo: ESP_CHECK that programs got loaded

  return validated;
}

bool HBAOHelper::initMisc() {
  std::mt19937 rmt;

  float numDir = 8;  // keep in sync to glsl

  signed short hbaoRandomShort[HBAO_RANDOM_ELEMENTS * MAX_SAMPLES * 4];

  for (int i = 0; i < HBAO_RANDOM_ELEMENTS * MAX_SAMPLES; i++) {
    float Rand1 = static_cast<float>(rmt()) / 4294967296.0f;
    float Rand2 = static_cast<float>(rmt()) / 4294967296.0f;

    // Use random rotation angles in [0,2PI/NUM_DIRECTIONS)
    float Angle = 2.f * nv_pi * Rand1 / numDir;
    m_hbaoRandom[i].x = cosf(Angle);
    m_hbaoRandom[i].y = sinf(Angle);
    m_hbaoRandom[i].z = Rand2;
    m_hbaoRandom[i].w = 0;
#define SCALE ((1 << 15))
    hbaoRandomShort[i * 4 + 0] = (signed short)(SCALE * m_hbaoRandom[i].x);
    hbaoRandomShort[i * 4 + 1] = (signed short)(SCALE * m_hbaoRandom[i].y);
    hbaoRandomShort[i * 4 + 2] = (signed short)(SCALE * m_hbaoRandom[i].z);
    hbaoRandomShort[i * 4 + 3] = (signed short)(SCALE * m_hbaoRandom[i].w);
#undef SCALE
  }

  nvgl::newTexture(textures.hbao_random, GL_TEXTURE_2D_ARRAY);
  glBindTexture(GL_TEXTURE_2D_ARRAY, textures.hbao_random);
  glTexStorage3D(GL_TEXTURE_2D_ARRAY, 1, GL_RGBA16_SNORM, HBAO_RANDOM_SIZE,
                 HBAO_RANDOM_SIZE, MAX_SAMPLES);
  glTexSubImage3D(GL_TEXTURE_2D_ARRAY, 0, 0, 0, 0, HBAO_RANDOM_SIZE,
                  HBAO_RANDOM_SIZE, MAX_SAMPLES, GL_RGBA, GL_SHORT,
                  hbaoRandomShort);
  glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glBindTexture(GL_TEXTURE_2D_ARRAY, 0);

  for (int i = 0; i < MAX_SAMPLES; i++) {
    glGenTextures(1, &textures.hbao_randomview[i]);
    glTextureView(textures.hbao_randomview[i], GL_TEXTURE_2D,
                  textures.hbao_random, GL_RGBA16_SNORM, 0, 1, i, 1);
    glBindTexture(GL_TEXTURE_2D, textures.hbao_randomview[i]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glBindTexture(GL_TEXTURE_2D, 0);
  }

  nvgl::newBuffer(buffers.hbao_ubo);
  glNamedBufferStorage(buffers.hbao_ubo, sizeof(HBAOData), NULL,
                       GL_DYNAMIC_STORAGE_BIT);

  return true;
}

bool HBAOHelper::initFramebuffers(int width, int height, int samples) {
  nvgl::newTexture(textures.scene_depthlinear, GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, textures.scene_depthlinear);
  glTexStorage2D(GL_TEXTURE_2D, 1, GL_R32F, width, height);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glBindTexture(GL_TEXTURE_2D, 0);

  nvgl::newFramebuffer(fbos.depthlinear);
  glBindFramebuffer(GL_FRAMEBUFFER, fbos.depthlinear);
  glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                       textures.scene_depthlinear, 0);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  nvgl::newTexture(textures.scene_viewnormal, GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, textures.scene_viewnormal);
  glTexStorage2D(GL_TEXTURE_2D, 1, GL_RGBA8, width, height);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glBindTexture(GL_TEXTURE_2D, 0);

  nvgl::newFramebuffer(fbos.viewnormal);
  glBindFramebuffer(GL_FRAMEBUFFER, fbos.viewnormal);
  glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                       textures.scene_viewnormal, 0);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);

#if USE_AO_SPECIALBLUR
  GLenum formatAO = GL_RG16F;
  GLint swizzle[4] = {GL_RED, GL_GREEN, GL_ZERO, GL_ZERO};
#else
  GLenum formatAO = GL_R8;
  GLint swizzle[4] = {GL_RED, GL_RED, GL_RED, GL_RED};
#endif

  nvgl::newTexture(textures.hbao_result, GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, textures.hbao_result);
  glTexStorage2D(GL_TEXTURE_2D, 1, formatAO, width, height);
  glTexParameteriv(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_RGBA, swizzle);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glBindTexture(GL_TEXTURE_2D, 0);

  nvgl::newTexture(textures.hbao_blur, GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, textures.hbao_blur);
  glTexStorage2D(GL_TEXTURE_2D, 1, formatAO, width, height);
  glTexParameteriv(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_RGBA, swizzle);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glBindTexture(GL_TEXTURE_2D, 0);

  nvgl::newFramebuffer(fbos.hbao_calc);
  glBindFramebuffer(GL_FRAMEBUFFER, fbos.hbao_calc);
  glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                       textures.hbao_result, 0);
  glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, textures.hbao_blur,
                       0);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  // interleaved hbao

  int quarterWidth = ((width + 3) / 4);
  int quarterHeight = ((height + 3) / 4);

  nvgl::newTexture(textures.hbao2_deptharray, GL_TEXTURE_2D_ARRAY);
  glBindTexture(GL_TEXTURE_2D_ARRAY, textures.hbao2_deptharray);
  glTexStorage3D(GL_TEXTURE_2D_ARRAY, 1, GL_R32F, quarterWidth, quarterHeight,
                 HBAO_RANDOM_ELEMENTS);
  glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glBindTexture(GL_TEXTURE_2D_ARRAY, 0);

  for (int i = 0; i < HBAO_RANDOM_ELEMENTS; i++) {
    if (textures.hbao2_depthview[i]) {
      glDeleteTextures(1, &textures.hbao2_depthview[i]);
    }
    glGenTextures(1, &textures.hbao2_depthview[i]);
    glTextureView(textures.hbao2_depthview[i], GL_TEXTURE_2D,
                  textures.hbao2_deptharray, GL_R32F, 0, 1, i, 1);
    glBindTexture(GL_TEXTURE_2D, textures.hbao2_depthview[i]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glBindTexture(GL_TEXTURE_2D, 0);
  }

  nvgl::newTexture(textures.hbao2_resultarray, GL_TEXTURE_2D_ARRAY);
  glBindTexture(GL_TEXTURE_2D_ARRAY, textures.hbao2_resultarray);
  glTexStorage3D(GL_TEXTURE_2D_ARRAY, 1, formatAO, quarterWidth, quarterHeight,
                 HBAO_RANDOM_ELEMENTS);
  glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glBindTexture(GL_TEXTURE_2D_ARRAY, 0);

  GLenum drawbuffers[NUM_MRT];
  for (int layer = 0; layer < NUM_MRT; layer++) {
    drawbuffers[layer] = GL_COLOR_ATTACHMENT0 + layer;
  }

  nvgl::newFramebuffer(fbos.hbao2_deinterleave);
  glBindFramebuffer(GL_FRAMEBUFFER, fbos.hbao2_deinterleave);
  glDrawBuffers(NUM_MRT, drawbuffers);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  nvgl::newFramebuffer(fbos.hbao2_calc);
  glBindFramebuffer(GL_FRAMEBUFFER, fbos.hbao2_calc);
#if USE_AO_LAYERED_SINGLEPASS == AO_LAYERED_IMAGE
  // this fbo will not have any attachments and therefore requires rasterizer to
  // be configured through default parameters
  glFramebufferParameteri(GL_FRAMEBUFFER, GL_FRAMEBUFFER_DEFAULT_WIDTH,
                          quarterWidth);
  glFramebufferParameteri(GL_FRAMEBUFFER, GL_FRAMEBUFFER_DEFAULT_HEIGHT,
                          quarterHeight);
#endif
#if USE_AO_LAYERED_SINGLEPASS == AO_LAYERED_GS
  glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                       textures.hbao2_resultarray, 0);
#endif
  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  return true;
}

void HBAOHelper::prepareHbaoData(const Projection& projection,
                                 int width,
                                 int height) {
  // projection
  const float* P = projection.matrix.get_value();

  float projInfoPerspective[] = {
      2.0f / (P[4 * 0 + 0]),                  // (x) * (R - L)/N
      2.0f / (P[4 * 1 + 1]),                  // (y) * (T - B)/N
      -(1.0f - P[4 * 2 + 0]) / P[4 * 0 + 0],  // L/N
      -(1.0f + P[4 * 2 + 1]) / P[4 * 1 + 1],  // B/N
  };

  float projInfoOrtho[] = {
      2.0f / (P[4 * 0 + 0]),                  // ((x) * R - L)
      2.0f / (P[4 * 1 + 1]),                  // ((y) * T - B)
      -(1.0f + P[4 * 3 + 0]) / P[4 * 0 + 0],  // L
      -(1.0f - P[4 * 3 + 1]) / P[4 * 1 + 1],  // B
  };

  int useOrtho = projection.ortho ? 1 : 0;
  m_hbaoUbo.projOrtho = useOrtho;
  m_hbaoUbo.projInfo = useOrtho ? projInfoOrtho : projInfoPerspective;

  float projScale;
  if (useOrtho) {
    projScale = float(height) / (projInfoOrtho[1]);
  } else {
    projScale = float(height) / (tanf(projection.fov * 0.5f) * 2.0f);
  }

  // radius
  float meters2viewspace = 1.0f;
  float R = m_config.radius * meters2viewspace;
  m_hbaoUbo.R2 = R * R;
  m_hbaoUbo.NegInvR2 = -1.0f / m_hbaoUbo.R2;
  m_hbaoUbo.RadiusToScreen = R * 0.5f * projScale;

  // ao
  m_hbaoUbo.PowExponent = std::max(m_config.intensity, 0.0f);
  m_hbaoUbo.NDotVBias = std::min(std::max(0.0f, m_config.bias), 1.0f);
  m_hbaoUbo.AOMultiplier = 1.0f / (1.0f - m_hbaoUbo.NDotVBias);

  // resolution
  int quarterWidth = ((width + 3) / 4);
  int quarterHeight = ((height + 3) / 4);

  m_hbaoUbo.InvQuarterResolution =
      vec2(1.0f / float(quarterWidth), 1.0f / float(quarterHeight));
  m_hbaoUbo.InvFullResolution = vec2(1.0f / float(width), 1.0f / float(height));

#if USE_AO_LAYERED_SINGLEPASS
  for (int i = 0; i < HBAO_RANDOM_ELEMENTS; i++) {
    m_hbaoUbo.float2Offsets[i] =
        vec4(float(i % 4) + 0.5f, float(i / 4) + 0.5f, 0.0f, 0.0f);
    m_hbaoUbo.jitters[i] = m_hbaoRandom[i];
  }
#endif
}

void HBAOHelper::drawLinearDepth(const Projection& projection,
                                 int width,
                                 int height,
                                 int sampleIdx,
                                 GLuint scene_depthstencil) {
  // NV_PROFILE_GL_SECTION("linearize");
  glBindFramebuffer(GL_FRAMEBUFFER, fbos.depthlinear);

  if (m_config.samples > 1) {
    glUseProgram(m_progManager.get(programs.depth_linearize_msaa));
    glUniform4f(0, projection.nearplane * projection.farplane,
                projection.nearplane - projection.farplane, projection.farplane,
                projection.ortho ? 0.0f : 1.0f);
    glUniform1i(1, sampleIdx);

    nvgl::bindMultiTexture(GL_TEXTURE0, GL_TEXTURE_2D_MULTISAMPLE,
                           scene_depthstencil);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    nvgl::bindMultiTexture(GL_TEXTURE0, GL_TEXTURE_2D_MULTISAMPLE, 0);
  } else {
    glUseProgram(m_progManager.get(programs.depth_linearize));
    glUniform4f(0, projection.nearplane * projection.farplane,
                projection.nearplane - projection.farplane, projection.farplane,
                projection.ortho ? 0.0f : 1.0f);

    nvgl::bindMultiTexture(GL_TEXTURE0, GL_TEXTURE_2D, scene_depthstencil);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    nvgl::bindMultiTexture(GL_TEXTURE0, GL_TEXTURE_2D, 0);
  }
}

void HBAOHelper::drawHbaoBlur(const Projection& projection,
                              int width,
                              int height,
                              int sampleIdx,
                              GLuint fbo_scene) {
  // NV_PROFILE_GL_SECTION("ssaoblur");

  float meters2viewspace = 1.0f;

  glUseProgram(m_progManager.get(USE_AO_SPECIALBLUR ? programs.hbao_blur
                                                    : programs.bilateralblur));
  nvgl::bindMultiTexture(GL_TEXTURE1, GL_TEXTURE_2D,
                         textures.scene_depthlinear);

  glUniform1f(0, m_config.blurSharpness / meters2viewspace);

  glDrawBuffer(GL_COLOR_ATTACHMENT1);

  nvgl::bindMultiTexture(GL_TEXTURE0, GL_TEXTURE_2D, textures.hbao_result);
  glUniform2f(1, 1.0f / float(width), 0);
  glDrawArrays(GL_TRIANGLES, 0, 3);

  // final output to main fbo
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_scene);
  glDisable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_ZERO, GL_SRC_COLOR);
  if (m_config.samples > 1) {
    glEnable(GL_SAMPLE_MASK);
    glSampleMaski(0, 1 << sampleIdx);
  }

#if USE_AO_SPECIALBLUR
  glUseProgram(m_progManager.get(programs.hbao_blur2));
  glUniform1f(0, m_config.blurSharpness / meters2viewspace);
#endif

  nvgl::bindMultiTexture(GL_TEXTURE0, GL_TEXTURE_2D, textures.hbao_blur);
  glUniform2f(1, 0, 1.0f / float(height));
  glDrawArrays(GL_TRIANGLES, 0, 3);
}

void HBAOHelper::drawHbaoClassic(const Projection& projection,
                                 int width,
                                 int height,
                                 int sampleIdx,
                                 GLuint scene_depthstencil,
                                 GLuint fbo_scene) {
  prepareHbaoData(projection, width, height);

  drawLinearDepth(projection, width, height, sampleIdx, scene_depthstencil);

  {
    // NV_PROFILE_GL_SECTION("ssaocalc");

    if (m_config.blur) {
      glBindFramebuffer(GL_FRAMEBUFFER, fbos.hbao_calc);
      glDrawBuffer(GL_COLOR_ATTACHMENT0);
    } else {
      glBindFramebuffer(GL_FRAMEBUFFER, fbo_scene);
      glDisable(GL_DEPTH_TEST);
      glEnable(GL_BLEND);
      glBlendFunc(GL_ZERO, GL_SRC_COLOR);
      if (m_config.samples > 1) {
        glEnable(GL_SAMPLE_MASK);
        glSampleMaski(0, 1 << sampleIdx);
      }
    }

    glUseProgram(m_progManager.get(USE_AO_SPECIALBLUR && m_config.blur
                                       ? programs.hbao_calc_blur
                                       : programs.hbao_calc));

    glBindBufferBase(GL_UNIFORM_BUFFER, 0, buffers.hbao_ubo);
    glNamedBufferSubData(buffers.hbao_ubo, 0, sizeof(HBAOData), &m_hbaoUbo);

    nvgl::bindMultiTexture(GL_TEXTURE0, GL_TEXTURE_2D,
                           textures.scene_depthlinear);
    nvgl::bindMultiTexture(GL_TEXTURE1, GL_TEXTURE_2D,
                           textures.hbao_randomview[sampleIdx]);
    glDrawArrays(GL_TRIANGLES, 0, 3);
  }

  if (m_config.blur) {
    drawHbaoBlur(projection, width, height, sampleIdx, fbo_scene);
  }

  glEnable(GL_DEPTH_TEST);
  glDisable(GL_BLEND);
  glDisable(GL_SAMPLE_MASK);
  glSampleMaski(0, ~0);

  nvgl::bindMultiTexture(GL_TEXTURE0, GL_TEXTURE_2D, 0);
  nvgl::bindMultiTexture(GL_TEXTURE1, GL_TEXTURE_2D, 0);

  glUseProgram(0);
}

void HBAOHelper::drawHbaoCacheAware(const Projection& projection,
                                    int width,
                                    int height,
                                    int sampleIdx,
                                    GLuint scene_depthstencil,
                                    GLuint fbo_scene) {
  // temp debug-visualize the scene_depthstencil buffer
  // {
  //   glBindFramebuffer(GL_FRAMEBUFFER, fbo_scene);

  //   glUseProgram(m_progManager.get(programs.depth_linearize));
  //   glUniform4f(0, projection.nearplane * projection.farplane,
  //   projection.nearplane - projection.farplane,
  //               projection.farplane, projection.ortho ? 0.0f : 1.0f);

  //   nvgl::bindMultiTexture(GL_TEXTURE0, GL_TEXTURE_2D, scene_depthstencil);
  //   glDrawArrays(GL_TRIANGLES, 0, 3);
  //   nvgl::bindMultiTexture(GL_TEXTURE0, GL_TEXTURE_2D, 0);
  //   return;
  // }

  int quarterWidth = ((width + 3) / 4);
  int quarterHeight = ((height + 3) / 4);

  prepareHbaoData(projection, width, height);

  drawLinearDepth(projection, width, height, sampleIdx, scene_depthstencil);

  {
    // NV_PROFILE_GL_SECTION("viewnormal");
    glBindFramebuffer(GL_FRAMEBUFFER, fbos.viewnormal);

    glUseProgram(m_progManager.get(programs.viewnormal));

    glUniform4fv(0, 1, m_hbaoUbo.projInfo.get_value());
    glUniform1i(1, m_hbaoUbo.projOrtho);
    glUniform2fv(2, 1, m_hbaoUbo.InvFullResolution.get_value());

    nvgl::bindMultiTexture(GL_TEXTURE0, GL_TEXTURE_2D,
                           textures.scene_depthlinear);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    nvgl::bindMultiTexture(GL_TEXTURE0, GL_TEXTURE_2D, 0);
  }

  {
    // NV_PROFILE_GL_SECTION("deinterleave");
    glBindFramebuffer(GL_FRAMEBUFFER, fbos.hbao2_deinterleave);
    glViewport(0, 0, quarterWidth, quarterHeight);

    glUseProgram(m_progManager.get(programs.hbao2_deinterleave));
    nvgl::bindMultiTexture(GL_TEXTURE0, GL_TEXTURE_2D,
                           textures.scene_depthlinear);

    for (int i = 0; i < HBAO_RANDOM_ELEMENTS; i += NUM_MRT) {
      glUniform4f(0, float(i % 4) + 0.5f, float(i / 4) + 0.5f,
                  m_hbaoUbo.InvFullResolution.x, m_hbaoUbo.InvFullResolution.y);

      for (int layer = 0; layer < NUM_MRT; layer++) {
        glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + layer,
                             textures.hbao2_depthview[i + layer], 0);
      }
      glDrawArrays(GL_TRIANGLES, 0, 3);
    }
  }

  {
    // NV_PROFILE_GL_SECTION("ssaocalc");

    glBindFramebuffer(GL_FRAMEBUFFER, fbos.hbao2_calc);
    glViewport(0, 0, quarterWidth, quarterHeight);

    glUseProgram(m_progManager.get(USE_AO_SPECIALBLUR && m_config.blur
                                       ? programs.hbao2_calc_blur
                                       : programs.hbao2_calc));
    nvgl::bindMultiTexture(GL_TEXTURE1, GL_TEXTURE_2D,
                           textures.scene_viewnormal);

    glBindBufferBase(GL_UNIFORM_BUFFER, 0, buffers.hbao_ubo);
    glNamedBufferSubData(buffers.hbao_ubo, 0, sizeof(HBAOData), &m_hbaoUbo);

#if USE_AO_LAYERED_SINGLEPASS
    // instead of drawing to each layer individually
    // we draw all layers at once, and use image writes to update the array
    // texture this buys additional performance :)

    nvgl::bindMultiTexture(GL_TEXTURE0, GL_TEXTURE_2D_ARRAY,
                           textures.hbao2_deptharray);
#if USE_AO_LAYERED_SINGLEPASS == AO_LAYERED_IMAGE
    glBindImageTexture(0, textures.hbao2_resultarray, 0, GL_TRUE, 0,
                       GL_WRITE_ONLY, USE_AO_SPECIALBLUR ? GL_RG16F : GL_R8);
#endif
    glDrawArrays(GL_TRIANGLES, 0, 3 * HBAO_RANDOM_ELEMENTS);
#if USE_AO_LAYERED_SINGLEPASS == AO_LAYERED_IMAGE
    glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT |
                    GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
#endif
#else
    for (int i = 0; i < HBAO_RANDOM_ELEMENTS; i++) {
      glUniform2f(0, float(i % 4) + 0.5f, float(i / 4) + 0.5f);
      glUniform4fv(1, 1, m_hbaoRandom[i].get_value());

      nvgl::bindMultiTexture(GL_TEXTURE0, GL_TEXTURE_2D,
                             textures.hbao2_depthview[i]);
      glFramebufferTextureLayer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                                textures.hbao2_resultarray, 0, i);

      glDrawArrays(GL_TRIANGLES, 0, 3);
    }
#endif
  }

  {
    // NV_PROFILE_GL_SECTION("reinterleave");

    if (m_config.blur) {
      glBindFramebuffer(GL_FRAMEBUFFER, fbos.hbao_calc);
      glDrawBuffer(GL_COLOR_ATTACHMENT0);
    } else {
      glBindFramebuffer(GL_FRAMEBUFFER, fbo_scene);
      glDisable(GL_DEPTH_TEST);
      glEnable(GL_BLEND);
      glBlendFunc(GL_ZERO, GL_SRC_COLOR);
      if (m_config.samples > 1) {
        glEnable(GL_SAMPLE_MASK);
        glSampleMaski(0, 1 << sampleIdx);
      }
    }
    glViewport(0, 0, width, height);

    glUseProgram(m_progManager.get(USE_AO_SPECIALBLUR && m_config.blur
                                       ? programs.hbao2_reinterleave_blur
                                       : programs.hbao2_reinterleave));

    nvgl::bindMultiTexture(GL_TEXTURE0, GL_TEXTURE_2D_ARRAY,
                           textures.hbao2_resultarray);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    nvgl::bindMultiTexture(GL_TEXTURE0, GL_TEXTURE_2D_ARRAY, 0);
  }

  if (m_config.blur) {
    drawHbaoBlur(projection, width, height, sampleIdx, fbo_scene);
  }

  glDisable(GL_BLEND);
  glEnable(GL_DEPTH_TEST);
  glDisable(GL_SAMPLE_MASK);
  glSampleMaski(0, ~0);

  nvgl::bindMultiTexture(GL_TEXTURE0, GL_TEXTURE_2D, 0);
  nvgl::bindMultiTexture(GL_TEXTURE1, GL_TEXTURE_2D, 0);

  glUseProgram(0);
}

}  // namespace ssao
}  // namespace gfx
}  // namespace esp
