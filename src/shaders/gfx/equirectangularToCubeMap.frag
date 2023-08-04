// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

// Our implementation is based on:
// https://github.com/KhronosGroup/glTF-Sample-Viewer

precision highp float;

// -------------- input ---------------------
in highp vec2 textureCoordinates;

// ------------ uniforms --------------------
uniform highp sampler2D uEquirectangularTexture;
uniform uint uCubeSideIndex;

//------------- output ----------------------
#ifdef EXPLICIT_ATTRIB_LOCATION
layout(location = OUTPUT_ATTRIBUTE_LOCATION_COLOR)
#endif
    out highp vec4 fragmentColor;

//------------- shader ----------------------
const float INV_PI = 1.0f / 3.14159265359f;

// Given the texture coordinates (u,v) (u, v belongs to [0, 1]),
// unproject it to a light direction.
// This is based on:
// https://en.wikipedia.org/wiki/Cube_mapping
// We use uvToXYZFlipped version, not the uvToXYZ.
// But leave it here as a reference.
vec3 uvToXYZ(vec2 uv) {
  // convert range 0 to 1 to -1 to 1
  uv = 2.0f * uv - 1.0f;
  // Do NOT add break here!
  // Otherwise it gives you warning
  // "Unreachable statement in switch body"
  switch (uCubeSideIndex) {
    case 0u:
      return vec3(1.0f, uv.y, -uv.x);

    case 1u:
      return vec3(-1.0f, uv.y, uv.x);

    case 2u:
      return vec3(uv.x, -1.0f, uv.y);

    case 3u:
      return vec3(uv.x, 1.0f, -uv.y);

    case 4u:
      return vec3(uv.x, uv.y, 1.0f);

    case 5u:
      return vec3(-uv.x, +uv.y, -1.0f);
  }
}
// The effect of uvToXYZFlipped is equivalant to
// apply uvToXYZ on the "flipped (up side down)"
// equirectangular image.
// This is rather important for the cube mapping skybox
vec3 uvToXYZFlipped(vec2 uv) {
  // make it up  side down:
  uv.y = 1.0 - uv.y;
  // convert range 0 to 1 to -1 to 1
  uv = vec2(2.0f) * uv - vec2(1.0f);
  // Do NOT add break here!
  // Otherwise it gives you warning
  // "Unreachable statement in switch body"
  switch (uCubeSideIndex) {
    case 0u:
      return vec3(1.0f, uv.y, -uv.x);

    case 1u:
      return vec3(-1.0f, uv.y, uv.x);

    case 2u:
      // CAREFUL: py and ny are also switched
      return vec3(uv.x, 1.0f, -uv.y);

    case 3u:
      return vec3(uv.x, -1.0f, uv.y);

    case 4u:
      return vec3(uv.x, uv.y, 1.0f);

    case 5u:
      return vec3(-uv.x, +uv.y, -1.0f);
  }
}

// Give a light direction, convert it to the spherical coordinates
vec2 dirToUV(vec3 dir) {
  return vec2(0.5f + 0.5f * atan(dir.z, dir.x) * INV_PI,
              1.0f - acos(dir.y) * INV_PI);
}

vec3 equirectangularToCubeMap() {
  // regular uv --> light dir -->
  // --> transform the dir to the spherical space -->
  // new uv = (theta, phi) --> transform each to [0, 1]
  vec3 direction = normalize(uvToXYZFlipped(textureCoordinates));
  vec2 equirectangularTextureUV = dirToUV(direction);

  return texture(uEquirectangularTexture, equirectangularTextureUV).rgb;
}

void main(void) {
  fragmentColor = vec4(equirectangularToCubeMap(), 1.0f);
}
