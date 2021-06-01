// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

// Our implementation is based on:
// https://github.com/KhronosGroup/glTF-Sample-Viewer/tree/6386b1d8b1a0af257b280aea92d6cd39456e9689/source/shaders

precision highp float;

// -------------- input ---------------------
in highp vec2 textureCoordinates;

// ------------ uniforms --------------------
uniform highp sampler2D EquirectangularTexture;
uniform uint CubeSideIndex;

//------------- output ----------------------
#ifdef EXPLICIT_ATTRIB_LOCATION
layout(location = OUTPUT_ATTRIBUTE_LOCATION_COLOR)
#endif
out highp vec4 fragmentColor;

//------------- shader ----------------------
const float INV_PI = 1.0f / 3.14159265359f;

vec3 uvToXYZ(vec2 uv) {
  // convert range 0 to 1 to -1 to 1
  uv = 2.0f * uv - 1.0f;
  // Do NOT add break here!
  // Otherwise it gives you warning
  // "Unreachable statement in switch body"
  switch (CubeSideIndex) {
    case 0:
		return vec3( 1.0f,  uv.y, -uv.x);

    case 1:
		return vec3(-1.0f,  uv.y,  uv.x);

    case 2:
		return vec3( uv.x, -1.0f,  uv.y);

    case 3:
		return vec3( uv.x,  1.0f, -uv.y);

    case 4:
		return vec3( uv.x,  uv.y,  1.0f);

    case 5:
    return vec3(-uv.x, +uv.y, -1.0f);
  }
}

vec2 dirToUV(vec3 dir) {
	return vec2(
		0.5f + 0.5f * atan(dir.z, dir.x) * INV_PI,
		1.0f - acos(dir.y) * INV_PI);
}

vec3 equirectangularToCubeMap() {
	vec3 direction = normalize(uvToXYZ(textureCoordinates));
	vec2 equirectangularTextureUV = dirToUV(direction);

	return  texture(EquirectangularTexture,
                  equirectangularTextureUV).rgb;
}

void main(void)
{
	fragmentColor = vec4(equirectangularToCubeMap(), 1.0f);
}
