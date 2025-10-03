# Building Habitat-Sim for UV/pip on WSL2

This branch contains patches to enable building Habitat-Sim wheels for use with UV/pip-based workflows on WSL2, without requiring conda.

## Features

This branch adds:

1. **Build Environment Variables** (`setup.py`)
   - `HSIM_NO_SUBMODULE_UPDATE=1`: Skip git submodule updates
   - `HSIM_SKIP_CMAKE_BUILD=1`: Reuse existing build artifacts

2. **WSL2 GPU Support** (`src/esp/gfx/WindowlessContext.cpp`)
   - `HSIM_DISABLE_CUDA_DEVICE=1`: Bypass CUDA device enumeration
   - Enables D3D12 backend rendering on WSL2

3. **OpenEXR Patches** (documented in `OPENEXR_PATCHES.md`)
   - Required `<cstdint>` includes for modern C++ compilers

## Prerequisites

### System Dependencies (Ubuntu/Debian/WSL2)

```bash
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    ninja-build \
    ccache \
    libjpeg-dev \
    libglm-dev \
    libegl1-mesa-dev \
    libgl1-mesa-dev \
    libgles2-mesa-dev \
    mesa-common-dev \
    libnvidia-gl-570  # For WSL2 GPU support
```

### GPU Access (WSL2)

```bash
sudo chmod 666 /dev/dri/renderD128
```

## Build Instructions

### Step 1: Clone and Checkout

```bash
git clone https://github.com/killerapp/habitat-sim.git
cd habitat-sim
git checkout feature/uv-wsl2-support
git submodule update --init --recursive
```

### Step 2: Apply OpenEXR Patches

See `OPENEXR_PATCHES.md` for detailed instructions. Add `#include <cstdint>` to three header files in the OpenEXR submodule.

### Step 3: Configure CMake

```bash
cmake -S . -B build \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_MAKE_PROGRAM=/usr/bin/ninja \
    -DBUILD_WITH_BULLET=ON \
    -GNinja
```

### Step 4: Build

```bash
# Single-threaded to avoid memory issues on WSL2
cmake --build build --config RelWithDebInfo -- -j1
```

### Step 5: Create Wheel

```bash
# Install build dependencies
pip install build

# Create wheel, reusing existing build artifacts
HSIM_NO_SUBMODULE_UPDATE=1 HSIM_SKIP_CMAKE_BUILD=1 \
    python -m build --wheel --outdir dist
```

This creates: `dist/habitat_sim-0.2.2-cp38-cp38-linux_x86_64.whl`

### Step 6: Install

```bash
# With pip
pip install dist/habitat_sim-0.2.2-cp38-cp38-linux_x86_64.whl

# With UV
uv pip install dist/habitat_sim-0.2.2-cp38-cp38-linux_x86_64.whl
```

## Usage on WSL2

When running code that uses Habitat-Sim on WSL2, set these environment variables:

```bash
export HSIM_DISABLE_CUDA_DEVICE=1
export GALLIUM_DRIVER=d3d12
```

Or prefix commands:

```bash
HSIM_DISABLE_CUDA_DEVICE=1 GALLIUM_DRIVER=d3d12 python your_script.py
```

## Testing

```bash
# Basic import test
python -c "import habitat_sim; print(habitat_sim.__version__)"

# With GPU rendering
HSIM_DISABLE_CUDA_DEVICE=1 GALLIUM_DRIVER=d3d12 \
    python -c "import habitat_sim; print('GPU rendering enabled')"
```

## Integration with TBP Monty

This wheel was developed for use with the Thousand Brains Project Monty repository:

- **TBP Fork**: https://github.com/killerapp/tbp
- **Documentation**: See `UV_PROJECT.md` in the tbp repository for full integration instructions

## Environment Variables Reference

### Build-Time Variables

- **HSIM_NO_SUBMODULE_UPDATE=1**
  - Skips `git submodule update --init --recursive`
  - Use when submodules have local patches (like OpenEXR)

- **HSIM_SKIP_CMAKE_BUILD=1**
  - Reuses existing `build/` directory artifacts
  - Use when packaging a wheel from an already-built source tree

### Runtime Variables

- **HSIM_DISABLE_CUDA_DEVICE=1**
  - Skips CUDA device enumeration in EGL context
  - Required for WSL2 D3D12 backend rendering

- **GALLIUM_DRIVER=d3d12**
  - Selects D3D12 backend for Mesa rendering
  - Enables GPU acceleration on WSL2 with NVIDIA drivers

## Known Limitations

- Python 3.8 required (due to other dependencies in TBP Monty)
- Tested only on WSL2 with NVIDIA GPU
- Build is single-threaded to avoid memory pressure
- OpenEXR patches must be applied manually

## Related Files

- `OPENEXR_PATCHES.md`: Detailed OpenEXR submodule patching instructions
- `setup.py`: Build script with new environment variable support
- `src/esp/gfx/WindowlessContext.cpp`: WSL2 GPU compatibility patch

## Upstream Compatibility

These changes are designed to be non-intrusive and backward-compatible:

- Environment variables are opt-in
- Default behavior unchanged
- No breaking changes to the build system
- Can be merged upstream if desired

## Credits

Developed as part of the UV/pip installation support effort for the Thousand Brains Project Monty framework.

## License

Same as upstream Habitat-Sim (MIT License)
