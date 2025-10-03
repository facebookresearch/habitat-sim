# Local Fork Context

## Purpose

This fork exists to build and publish **habitat-sim wheels** for the **Thousand Brains Project (TBP) Monty** (`/home/vaskin/projects/tbp/`).

## Problem

**Habitat-sim** is officially conda-only and not available as wheels on PyPI. TBP Monty is a sensorimotor learning system that uses habitat-sim for 3D environment simulation. This project demonstrates that TBP can run entirely with UV/pip (no conda) by building custom habitat-sim wheels.

## Solution

This fork (`feature/uv-wsl2-support` branch) provides:

1. **UV/pip-compatible wheels** - Build habitat-sim as a wheel for UV/pip workflows
2. **WSL2 GPU support** - Environment variables to bypass CUDA device enumeration, enabling D3D12 backend
3. **Build flexibility** - Environment variables to control submodule updates and CMake builds
4. **Documentation** - Complete guides for building and integrating with TBP

## About TBP Monty

**Thousand Brains Project Monty** is an open-source sensorimotor learning system following neocortex principles. Named after Vernon Mountcastle, it implements learning through active sensing and motor interaction with 3D environments.

- **Project**: https://thousandbrainsproject.org
- **Docs**: https://thousandbrainsproject.readme.io/
- **Repo**: https://github.com/thousandbrainsproject/tbp.monty

Monty uses habitat-sim for realistic 3D object recognition tasks with vision and touch sensors.

## Built Wheels

Pre-built wheels are stored in `dist/` directory:

```bash
# Habitat-sim wheel for Python 3.8 (WSL2/Linux)
dist/habitat_sim-0.2.2-cp38-cp38-linux_x86_64.whl

# Magnum bindings (editable install from build directory)
build/deps/magnum-bindings/src/python/
```

TBP uses these via `pyproject.toml` configuration:
```toml
[tool.uv.sources]
habitat-sim = { path = "../habitat-sim/dist/habitat_sim-0.2.2-cp38-cp38-linux_x86_64.whl" }
magnum = { path = "../habitat-sim/build/deps/magnum-bindings/src/python", editable = true }
```

## Key Features (feature/uv-wsl2-support branch)

### Build Environment Variables
- `HSIM_NO_SUBMODULE_UPDATE=1` - Skip git submodule updates (preserve local patches)
- `HSIM_SKIP_CMAKE_BUILD=1` - Reuse existing CMake build (faster wheel packaging)

### WSL2 GPU Compatibility
- `HSIM_DISABLE_CUDA_DEVICE=1` - Bypass CUDA device enumeration
- Enables D3D12 backend rendering on WSL2 with NVIDIA RTX 4090

### Documentation
- `UV_WSL2_BUILD.md` - Complete build guide
- `OPENEXR_PATCHES.md` - Required OpenEXR submodule patches
- Cross-references TBP UV setup documentation

## Repository Structure

**Upstream**: `facebookresearch/habitat-sim` (v0.2.2)
**Fork**: `killerapp/habitat-sim`
**Branch**: `feature/uv-wsl2-support` (5 commits ahead)

### Commits on feature branch
1. `b0519a23` - feat: Add build environment variables for UV/pip workflow
2. `d3f83f76` - feat: Add HSIM_DISABLE_CUDA_DEVICE env var for WSL2 compatibility
3. `b22caa0e` - docs: Document required OpenEXR submodule patches
4. `47d87f24` - docs: Add comprehensive UV/WSL2 build guide
5. `d9ad079b` - chore: Add UV workflow files

## Related Projects

### TBP Monty (Main Project)
- **Path**: `/home/vaskin/projects/tbp/`
- **Fork**: https://github.com/killerapp/tbp
- **Purpose**: Sensorimotor learning system using this habitat-sim fork
- **Documentation**: See `../tbp/UV_PROJECT.md` for complete UV setup guide

### Habitat-Sim (This Fork)
- **Path**: `/home/vaskin/projects/habitat-sim/`
- **Fork**: https://github.com/killerapp/habitat-sim
- **Purpose**: Build UV/pip-compatible wheels for TBP

## Quick Start (For TBP Users)

If TBP is already set up with UV sources configured:

```bash
cd /home/vaskin/projects/tbp
uv sync --extra dev  # Automatically installs habitat-sim wheel from this fork
```

See TBP's `UV_PROJECT.md` for full installation instructions.
