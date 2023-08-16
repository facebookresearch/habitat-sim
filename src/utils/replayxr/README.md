# WebXR Replay Renderer Prototype

This is a prototype application that uses the replay renderer on WebGL/WebXR.

Based on [Magnum WebXR example](https://magnum.graphics/showcase/webxr/).

See [current state](#current-state) for support status.

# Setup

## Emscripten

Emscripten is required to build this application. It does not require any other Javascript dependency.

1. Download and install [emscripten](https://emscripten.org/docs/getting_started/downloads.html) (you need version 3.1.20).

   In the `emsdk` repository:
   ```bash
   git pull
   ./emsdk install 3.1.20
   ```
2. Activate your emsdk environment
   ```bash
   ./emsdk activate 3.1.20
   source ./emsdk_env.sh
   ```

## Serving

### Launching Server

1. Generate Certificates
   
   VR headsets will *ignore* WebXR experiences that are not served by HTTPS. Use the following command to generate a certificate:

   `openssl req -new -x509 -keyout cert.pem -out localhost.pem -days 365 -nodes`

   Then, change the path to the certificate in `launch_server.py`.

2. Run server using: `python src/utils/replayxr/launch_server.py`

3. Finally, open a **chromium-based** browser at: `http://0.0.0.0:4443/build_js/utils/replayxr/index.html`

Note that the Quest Pro headset uses aggressive caching. It's often a good idea to clear all browser data before loading a new build.

### Remote Browser on LAN

TODO

### Linting

The web build now exports compilation commands (`build_js/compile_commands.json`). You can copy it to the root of the repository to load symbols in the IDE.

If using `clangd`, you can create a `.clangd` file in the root as such:

```
CompileFlags:
  CompilationDatabase: build_js/ # Search directory for compile_commands.json
```

### Debugging:

`spector.js` is a simple browser-based graphics debugger that helps demystifying nondescript WebGL errors.

# Current State

This application works in-browser on the Classic replay renderer, using ReplicaCAD. Here are the ongoing issues:

* VR:
  * Rendering doesn't work. This is because WebXR uses an internal framebuffer rather than the default framebuffer (ID=0). Magnum cannot currently pick up that framebuffer. See `ReplayXr::drawEvent()` for details.
* HSSD (FloorPlanner):
  * Does not work. Various GL errors need to be investigated.
* ReplicaCAD:
  * Some assets have multiple texture slots. The following warning is generated: `MeshTools::compile(): ignoring Trade::MeshAttribute::TextureCoordinates 1 as its biding slot is already occupied by Trade::MeshAttribute::TextureCoordinates 0`
* Batch replay renderer
  * Does not work because of the following error: `GL_INVALID_OPERATION: It is undefined behaviour to use a uniform buffer that is too small.`
* Classic replay renderer:
  * PBR shaders fail to be compiled. See `ResourceManager::createDrawable()`.
  * MSAA must be disabled because WebGL cannot blit into multisampled framebuffers. See `ReplayXr` constructor.
  * The sensor render targets don't persist state. See `ClassicReplayRenderer::doRender()`.
* Assimp does not work on WebGL. Therefore, COLLADA files (.dae) cannot be imported. See `BatchPlayerImplementation::isSupportedRenderAsset()`.
* In WebGL, resources must be "pre-loaded" before being available for file system query. This is currently hardcoded. See `driver.js`.