// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

export async function isWebXRSupported() {
  return await navigator.xr.isSessionSupported("immersive-vr");
}

// GL stuff
let inds = null;
let tex = null;

// Pass in an html element with a webgl context
export function initGL(gl) {
  if (gl === null) {
    return null;
  }
  const vertSrc =
    "attribute vec3 p;attribute vec2 u;varying highp vec2 v;void main(){gl_Position=vec4(p,1);v=u;}";
  const fragSrc =
    "uniform sampler2D t;varying highp vec2 v;void main(){gl_FragColor=texture2D(t,v);}";

  // Compiles a GLSL shader. Returns null on failure
  function compileShader(gl, src, type) {
    const shader = gl.createShader(type);
    gl.shaderSource(shader, src);
    gl.compileShader(shader);

    if (gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
      return shader;
    } else {
      console.error("GLSL shader error: " + gl.getShaderInfoLog(shader));
      return null;
    }
  }
  const vert = compileShader(gl, vertSrc, gl.VERTEX_SHADER);
  const frag = compileShader(gl, fragSrc, gl.FRAGMENT_SHADER);
  if (vert === null || frag === null) {
    console.log("Failed to compile shaders");
    return null;
  }

  const program = gl.createProgram();

  gl.attachShader(program, vert);
  gl.attachShader(program, frag);

  gl.linkProgram(program);

  if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
    console.error("GLSL program error:" + gl.getProgramInfoLog(program));
    return null;
  }

  gl.useProgram(program);

  // Vertices
  const verts = [
    -1.0,
    1.0,
    0.0,
    -1.0,
    -1.0,
    0.0,
    1.0,
    -1.0,
    0.0,
    1.0,
    1.0,
    0.0
  ];
  const vertBuf = gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER, vertBuf);
  gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(verts), gl.STATIC_DRAW);
  // Position attribute
  const posAttrib = gl.getAttribLocation(program, "p");
  gl.vertexAttribPointer(posAttrib, 3, gl.FLOAT, false, 0, 0);
  gl.enableVertexAttribArray(posAttrib);

  // UVs
  const UVs = [0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 1.0, 1.0];
  const uvBuf = gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER, uvBuf);
  gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(UVs), gl.STATIC_DRAW);
  // UV attribute
  const uvAttrib = gl.getAttribLocation(program, "u");
  gl.vertexAttribPointer(uvAttrib, 2, gl.FLOAT, false, 0, 0);
  gl.enableVertexAttribArray(uvAttrib);

  // Indices
  inds = [3, 2, 1, 3, 1, 0];
  const indBuf = gl.createBuffer();
  gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, indBuf);
  gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, new Uint16Array(inds), gl.STATIC_DRAW);

  // Texture uniform
  const texUni = gl.getUniformLocation(program, "t");
  gl.uniform1i(texUni, 0);
}

export function drawTextureData(gl, texRes, texData) {
  if (tex === null) {
    tex = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, tex);

    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
  }

  gl.bindTexture(gl.TEXTURE_2D, tex);
  gl.texImage2D(
    gl.TEXTURE_2D,
    0, // level
    gl.RGBA, // internal format
    texRes[1], // width
    texRes[0], // height
    0, // border
    gl.RGBA, // format
    gl.UNSIGNED_BYTE, // type
    new Uint8Array(texData) // data
  );

  gl.activeTexture(gl.TEXTURE0);

  gl.drawElements(gl.TRIANGLES, inds.length, gl.UNSIGNED_SHORT, 0);
}
