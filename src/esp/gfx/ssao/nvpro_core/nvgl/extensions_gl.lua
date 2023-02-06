--[[
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

HOW TO USE

* look if extension exists in ../GL/glcustom.h
  if not, then merge it from official https://www.khronos.org/registry/OpenGL/api/GL/glext.h

* check out:
  extensions_gl.lua
  extensions_gl.hpp
  extensions_gl.cpp
  ../GL/glsubset.h

* edit the "subsetExtensions" string below, to add the extensions you need

* run this file with a lua interpreter

  e.g get https://github.com/rjpcomputing/luaforwindows/blob/master/files/lua5.1.dll
  and     https://github.com/rjpcomputing/luaforwindows/blob/master/files/lua5.1.exe

  then run "lua.exe extensions_gl.lua" in this directory

* if the extension is still missing, then merge the extension from official glext.h into ../GL/glcustom.h
  and rerun the script

]]

local subsetExtensions =
[[
GL_ARB_debug_output
GL_ARB_shading_language_include
GL_ARB_indirect_parameters
GL_ARB_bindless_texture
GL_ARB_texture_filter_minmax
GL_ARB_texture_float
GL_ARB_cl_event
WGL_ARB_pixel_format
GL_NV_bindless_texture
GL_NV_blend_equation_advanced
GL_NV_clip_space_w_scaling
GL_NV_command_list
GL_NV_compute_shader_derivatives
GL_NV_conservative_raster
GL_NV_conservative_raster_dilate
GL_NV_conservative_raster_pre_snap
GL_NV_conservative_raster_pre_snap_triangles
GL_NV_conservative_raster_underestimation
GL_NV_draw_texture
GL_NV_draw_vulkan_image
GL_NV_fill_rectangle
GL_NV_fragment_coverage_to_color
GL_NV_fragment_shader_barycentric
GL_NV_fragment_shader_interlock
GL_NV_framebuffer_mixed_samples
GL_NV_gpu_multicast
GL_NV_gpu_shader5
GL_NV_internalformat_sample_query
GL_NV_memory_attachment
GL_NV_mesh_shader
GL_NV_path_rendering
GL_NV_representative_fragment_test
GL_NV_sample_locations
GL_NV_sample_mask_override_coverage
GL_NV_scissor_exclusive
GL_NV_shader_atomic_int64
GL_NV_shader_buffer_load
GL_NV_shader_buffer_store
GL_NV_shader_texture_footprint
GL_NV_shader_thread_group
GL_NV_shader_thread_shuffle
GL_NV_shading_rate_image
GL_NV_stereo_view_rendering
GL_NV_texture_multisample
GL_NV_uniform_buffer_unified_memory
GL_NV_vertex_buffer_unified_memory
GL_NV_viewport_swizzle
GL_NVX_gpu_memory_info
GL_EXT_texture_compression_latc
GL_EXT_texture_compression_s3tc
GL_EXT_memory_object
GL_EXT_semaphore
GL_EXT_semaphore_win32
GL_EXT_semaphore_fd
GL_EXT_memory_object_fd
GL_EXT_memory_object_win32
GL_NV_query_resource
GL_NV_query_resource_tag
]]

local function generate(outfilename, header, enablelist)

  local function subsetheader(filename, outfilename, enablelist, prefix)
    local f = io.open(filename,"r")
    local str = f:read("*a")
    f:close()

    str = str:gsub("__gl_glcustom_h_", "__gl_glsubset_h_")

    if (enablelist) then
      local search = ("#ifndef $_[%w_]+\n#define $_[%w_]+ 1\n.-#endif /%* $_[%w_]+ %*/\n\n"):gsub("%$",prefix)
      local num
      str,num = str:gsub(search,
        function(cap)
          local feature = cap:match("#ifndef ("..prefix.."_[%w_]+)")

          local is_version = feature:match(prefix.."_VERSION")
          if (is_version or not enablelist or enablelist[feature]) then
            return cap
          else
            return ""
          end
        end)
    end

    if (outfilename) then
      print(outfilename)

      local f = io.open(outfilename,"w")
      assert(f,"could not open"..outfilename.." for writing (check out?)")
      f:write(str)
      f:close()
    end

    return str
  end

  local function extractFeatureDefs(str, enablelist, prefix, api, apientry)
    local features = {}
    local disables = {}
    for feature,defstr in str:gmatch(("#ifndef ($_[%w_]+)\n#define $_[%w_]+ 1\n(.-)#endif /%* $_"):gsub("%$", prefix)) do
      local is_version = feature:match(prefix.."_VERSION")
      if (is_version or not enablelist or enablelist[feature]) then

        local defs = ""
        for def in defstr:gmatch(("#ifdef $_$EXT_PROTOTYPES(.-)#endif\n"):gsub("%$", prefix)) do
          def = def:gsub(api, ""):gsub(apientry, "")
          defs = defs..def
        end

        table.insert(features, {feature=feature, defs=defs})
      elseif (not is_version) then
        table.insert(disables, feature)
      end
    end

    return features, disables
  end

  local function toTab(str)
    local tab = {}
    for name in str:gmatch("[%w_]+") do
      tab[name] = true
    end
    return tab
  end

  local enablelist = enablelist and toTab(enablelist)

  local parsed = subsetheader("../GL/glcustom.h", "../GL/glsubset.h", enablelist, "GL")

  local gl_features, gl_disables = extractFeatureDefs(parsed, enablelist, "GL", "GLAPI ", "APIENTRY ")

  local availables_header = ""
  local availables_source = ""

  local loaders_header = ""
  local loaders_source = ""
  local loaders_all = ""

  local function process(f, prefix, api, apientry)

    local major,minor = f.feature:match(prefix.."_VERSION_(%d)_(%d)")
    major,minor = tonumber(major), tonumber(minor)

    availables_header = availables_header.."extern int has_"..f.feature..";\n"
    availables_source = availables_source.."int has_"..f.feature.." = 0;\n"

    loaders_header = loaders_header.."int load_"..f.feature.."(nvGLLoaderGetProcFN fnGetProcAddress);\n"

    local defs = "\n"..f.defs:gsub("%s*%*%s*","* ")

    local func      = ""
    local initfunc  = ""
    local success   = ""
    local variable  = ""

    for const,ret,name,arg in defs:gmatch("\n([%w_]-%s*)([%w_%*]+)%s+([%w_]+)%s*(%b());") do

      local prot = "PFN"..string.upper(name).."PROC"
      local mangled = "pfn_"..name

      local exec = ret ~= "void" and "return " or ""
      const = const == "\n" and "" or const

      exec = exec..mangled.."("
      if (arg ~= "(void)") then
        for a in arg:sub(2,-2):gmatch("([^,%%]+),?") do
          local am = a:gsub("%b[]",""):match("([%w_]+)$")
          assert(am,a)
          exec = exec..am..","
        end
        exec = exec:sub(1,-2)..");"
      else
        exec = exec..");"
      end

      variable = variable.."static "..prot.." "..mangled.." = 0;\n"

      func = func..api..const..ret.." "..apientry..name..arg.."\n{\n"
      func = func.."  assert("..mangled..");\n"
      func = func.."  "..exec.."\n"
      func = func.."}\n"

      initfunc  = initfunc.."  "..mangled..' = ('..prot..')fnGetProcAddress("'..name..'");\n'
      success  = success.."  success = success && ("..mangled..' != 0);\n'
    end

    local test
    if (major and minor) then
      test = "has_version("..major..","..minor..")"
    else
      test = 'has_extension("'..f.feature..'")'
    end

    -- already defined in win32 opengl32.dll
    local skip_functions =  (prefix == "GL" and major == 1 and minor < 2) or
                            (prefix == "WGL" and major == 1 and minor == 0)

    loaders_source = loaders_source.."/* /////////////////////////////////// */\n"
    loaders_source = loaders_source.."/* "..f.feature.." */\n\n"
    loaders_source = loaders_source..variable.."\n"
    --loaders_source = loaders_source..(skip_functions and "#ifndef WIN32\n" or "")
    loaders_source = loaders_source..func.."\n"
    --loaders_source = loaders_source..(skip_functions and "#endif\n" or "")
    loaders_source = loaders_source.."int load_"..f.feature.."(nvGLLoaderGetProcFN fnGetProcAddress)\n{\n"
    loaders_source = loaders_source..initfunc
    loaders_source = loaders_source.."  int success = "..test..";\n"
    loaders_source = loaders_source..success
    loaders_source = loaders_source.."  return success;\n}\n\n"

    print("output written", f.feature)

    loaders_all = loaders_all.."  has_"..f.feature.." = load_"..f.feature.."(fnGetProcAddress);\n"
  end

  for i,f in ipairs(gl_features) do
    process(f, "GL", "GLAPI ", "APIENTRY ")
  end

  local disables = ""

  for i,f in ipairs(gl_disables) do
    disables = disables.."#define "..f.." 0\n"
  end

  local fheader = io.open(outfilename..".hpp", "w")
  local fsource = io.open(outfilename..".cpp", "w")
  assert(fheader, "could not open "..outfilename..".hpp for writing (check out?)")
  assert(fsource, "could not open "..outfilename..".cpp for writing (check out?)")

  local structs = ""

  if (enablelist and enablelist.GL_NV_command_list) then
    structs = structs..[[
/* GL_NV_command_list */
typedef struct {
  GLuint  header;
} TerminateSequenceCommandNV;

typedef struct {
  GLuint  header;
} NOPCommandNV;

typedef  struct {
  GLuint  header;
  GLuint  count;
  GLuint  firstIndex;
  GLuint  baseVertex;
} DrawElementsCommandNV;

typedef  struct {
  GLuint  header;
  GLuint  count;
  GLuint  first;
} DrawArraysCommandNV;

typedef  struct {
  GLuint  header;
  GLenum  mode;
  GLuint  count;
  GLuint  instanceCount;
  GLuint  firstIndex;
  GLuint  baseVertex;
  GLuint  baseInstance;
} DrawElementsInstancedCommandNV;

typedef  struct {
  GLuint  header;
  GLenum  mode;
  GLuint  count;
  GLuint  instanceCount;
  GLuint  first;
  GLuint  baseInstance;
} DrawArraysInstancedCommandNV;

typedef struct {
  GLuint  header;
  GLuint  addressLo;
  GLuint  addressHi;
  GLuint  typeSizeInByte;
} ElementAddressCommandNV;

typedef struct {
  GLuint  header;
  GLuint  index;
  GLuint  addressLo;
  GLuint  addressHi;
} AttributeAddressCommandNV;

typedef struct {
  GLuint    header;
  GLushort  index;
  GLushort  stage;
  GLuint    addressLo;
  GLuint    addressHi;
} UniformAddressCommandNV;

typedef struct {
  GLuint  header;
  float   red;
  float   green;
  float   blue;
  float   alpha;
} BlendColorCommandNV;

typedef struct {
  GLuint  header;
  GLuint  frontStencilRef;
  GLuint  backStencilRef;
} StencilRefCommandNV;

typedef struct {
  GLuint  header;
  float   lineWidth;
} LineWidthCommandNV;

typedef struct {
  GLuint  header;
  float   scale;
  float   bias;
} PolygonOffsetCommandNV;

typedef struct {
  GLuint  header;
  float   alphaRef;
} AlphaRefCommandNV;

typedef struct {
  GLuint  header;
  GLuint  x;
  GLuint  y;
  GLuint  width;
  GLuint  height;
} ViewportCommandNV;  /* only ViewportIndex 0 */

typedef struct {
  GLuint  header;
  GLuint  x;
  GLuint  y;
  GLuint  width;
  GLuint  height;
} ScissorCommandNV;   /* only ViewportIndex 0 */

typedef struct {
  GLuint  header;
  GLuint  frontFace;  /* 0 for CW, 1 for CCW */
} FrontFaceCommandNV;

]]
  end

fheader:write(header..[[

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

/**
  # OpenGL Extension loader

  Provides a subset of OpenGL Extensions generated by `extensions_gl.lua`.
  The framework uses a sanitized OpenGL header that is mostly core functionality.
  `GL/glsubset.h` is generated using a enablelist by the lua script
  from `GL/glcustom.h` (which is mostly based on `glcorearb.h` with some
  additional extensions and features)
*/

#pragma once

#define GL_GLEXT_PROTOTYPES
#include <GL/glsubset.h>

typedef void* (* nvGLLoaderGetProcFN)(const char* name);

/* structs */
]]..structs..[[

/* loader */

void load_GL(nvGLLoaderGetProcFN fnGetProcAddress);

/* availables */
]]..availables_header..[[

/* loaders */
]]..loaders_header..[[

]])

fsource:write(header..[[

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

#include <assert.h>
#include <cstring>
#include "]]..outfilename..[[.hpp"

/* availables */
]]..availables_source..[[

/* tests */
static int has_extension(const char* name)
{
  GLint numExtensions = 0;
  glGetIntegerv(GL_NUM_EXTENSIONS, &numExtensions);
  for (int i = 0; i < numExtensions; i++) {
    const char* ext = (const char*)glGetStringi(GL_EXTENSIONS, i);
    if (strcmp(name, ext) == 0) {
      return true;
    }
  }

  return false;
}
static int has_version(int major, int minor)
{
  GLint ctx_major;
  GLint ctx_minor;

  glGetIntegerv(GL_MAJOR_VERSION, &ctx_major);
  glGetIntegerv(GL_MINOR_VERSION, &ctx_minor);

  return (ctx_major >= major && (ctx_major > major || ctx_minor >= minor));
}

/* loaders */

void load_GL(nvGLLoaderGetProcFN fnGetProcAddress)
{
]]..loaders_all..[[
}

/* loaders */
]]..loaders_source..[[

]])


end

do
  generate("extensions_gl",
    "/* auto generated by extensions_gl.lua */",
    subsetExtensions)
end
