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

#ifndef NV_PROFILERGL_INCLUDED
#define NV_PROFILERGL_INCLUDED

#include "../nvh/profiler.hpp"
#include "extensions_gl.hpp"

namespace nvgl {

//////////////////////////////////////////////////////////////////////////
/**
    \class nvgl::ProfilerGL

    nvgl::ProfilerGL extends Profiler and uses `glQueryCounter(...
   GL_TIMESTAMP)` to compute the GPU time of a section. `glPushDebugGroup` and
   `glPopDebugGroup` are used within each timed section, so that the section
   names can show up in NSightGraphics, renderdoc or comparable tools.

  */

class ProfilerGL : public nvh::Profiler {
 public:
  // utility class to call begin/end within local scope
  class Section {
   public:
    Section(ProfilerGL& profiler, const char* name, bool singleShot = false)
        : m_profiler(profiler) {
      m_id = profiler.beginSection(name, singleShot);
    }
    ~Section() { m_profiler.endSection(m_id); }

   private:
    SectionID m_id;
    ProfilerGL& m_profiler;
  };

  // recurring, must be within beginFrame/endFrame
  Section timeRecurring(const char* name) {
    return Section(*this, name, false);
  }

  // singleShot, results are available after FRAME_DELAY many endFrame
  Section timeSingle(const char* name) { return Section(*this, name, true); }

  //////////////////////////////////////////////////////////////////////////

  ProfilerGL(nvh::Profiler* masterProfiler = nullptr)
      : Profiler(masterProfiler) {}

  void init();
  void deinit();

  SectionID beginSection(const char* name, bool singleShot = false);
  void endSection(SectionID slot);

 private:
  void resizeQueries();

  std::vector<GLuint> m_queries;
};
}  // namespace nvgl

#endif
