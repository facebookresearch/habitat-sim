/*
 * Copyright (c) 2014-2021, NVIDIA CORPORATION.  All rights reserved.
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
 * SPDX-FileCopyrightText: Copyright (c) 2014-2021 NVIDIA CORPORATION
 * SPDX-License-Identifier: Apache-2.0
 */

#include "profiler_gl.hpp"
#include <assert.h>

//////////////////////////////////////////////////////////////////////////

namespace nvgl {

void ProfilerGL::resizeQueries() {
  uint32_t timers = getRequiredTimers();
  uint32_t old = (uint32_t)m_queries.size();

  if (old == timers) {
    return;
  }

  m_queries.resize(timers, 0);
  uint32_t add = timers - old;
  glGenQueries(add, &m_queries[old]);
}

void ProfilerGL::init() {
  resizeQueries();
}

void ProfilerGL::deinit() {
  if (m_queries.empty())
    return;

  glDeleteQueries((GLuint)m_queries.size(), m_queries.data());
  m_queries.clear();
}

nvh::Profiler::SectionID ProfilerGL::beginSection(const char* name,
                                                  bool singleShot) {
  nvh::Profiler::gpuTimeProvider_fn fnProvider = [&](SectionID i,
                                                     uint32_t queryFrame,
                                                     double& gpuTime) {
    uint32_t idxBegin = getTimerIdx(i, queryFrame, true);
    uint32_t idxEnd = getTimerIdx(i, queryFrame, false);

    GLint available = 0;
    glGetQueryObjectiv(m_queries[idxEnd], GL_QUERY_RESULT_AVAILABLE,
                       &available);

    if (available) {
      GLuint64 beginTime;
      GLuint64 endTime;
      glGetQueryObjectui64v(m_queries[idxBegin], GL_QUERY_RESULT, &beginTime);
      glGetQueryObjectui64v(m_queries[idxEnd], GL_QUERY_RESULT, &endTime);

      gpuTime = double(endTime - beginTime) / double(1000);

      return true;
    } else {
      return false;
    }
  };

  SectionID slot = Profiler::beginSection(name, "GL ", fnProvider, singleShot);

  if (m_queries.size() != getRequiredTimers()) {
    resizeQueries();
  }

  glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 0, -1, name);

  uint32_t idx = getTimerIdx(slot, getSubFrame(slot), true);
  glQueryCounter(m_queries[idx], GL_TIMESTAMP);

  return slot;
}

void ProfilerGL::endSection(SectionID slot) {
  uint32_t idx = getTimerIdx(slot, getSubFrame(slot), false);
  glQueryCounter(m_queries[idx], GL_TIMESTAMP);
  glPopDebugGroup();
  Profiler::endSection(slot);
}

}  // namespace nvgl
