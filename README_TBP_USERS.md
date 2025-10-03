# habitat-sim Wheels for TBP Monty Users

This repository provides **experimental pip/UV-compatible wheels** for [habitat-sim](https://github.com/facebookresearch/habitat-sim), specifically built to support the [Thousand Brains Project (TBP) Monty](https://github.com/thousandbrainsproject/tbp.monty) sensorimotor learning system.

## What is This?

**TBP Monty** is a sensorimotor learning system that simulates how the neocortex learns through active sensing. Monty uses habitat-sim to create realistic 3D environments for object recognition experiments with vision and touch sensors.

**The Problem**: Habitat-sim is officially only available through conda, which conflicts with modern Python workflows using UV or pip.

**The Solution**: This fork builds habitat-sim as standard Python wheels and publishes them to PyPI, allowing TBP users to run Monty entirely with UV/pip.

## Installation for TBP Monty

### Quick Start (Recommended for TBP Users)

If you're setting up TBP Monty with UV (the modern, fast package manager):

```bash
# Clone TBP Monty
git clone https://github.com/thousandbrainsproject/tbp.monty.git
cd tbp.monty

# Install with UV (includes habitat-sim automatically)
uv sync --extra dev

# Verify installation
uv run python -c "import habitat_sim; print('Habitat-Sim:', habitat_sim.__version__)"
```

**Note**: The TBP fork at [github.com/killerapp/tbp](https://github.com/killerapp/tbp) already configures this wheel for you. For the official TBP repo, follow the manual installation below.

### Manual Installation (Official TBP Repo)

If using the official TBP repository without UV configuration:

```bash
# Option 1: With UV
uv pip install habitat-sim-uv-wheels-experimental

# Option 2: With pip
pip install habitat-sim-uv-wheels-experimental

# Option 3: Add to pyproject.toml
[project]
dependencies = [
    "habitat-sim-uv-wheels-experimental>=0.2.2",
]
```

### System Requirements

**Operating System**: Linux (manylinux2014+ compatible) or WSL2

**Python Version**: 3.8+ (currently wheels available for 3.8, more versions coming)

**For WSL2 GPU Support** (recommended for faster rendering):

```bash
# Grant GPU access
sudo chmod 666 /dev/dri/renderD128

# Set environment variables for D3D12 backend
export HSIM_DISABLE_CUDA_DEVICE=1
export GALLIUM_DRIVER=d3d12

# Run Monty experiments with GPU
HSIM_DISABLE_CUDA_DEVICE=1 GALLIUM_DRIVER=d3d12 uv run monty -e <experiment_name>
```

## Running TBP Monty Experiments

Once installed, you can run Monty experiments using habitat-sim for 3D object recognition:

```bash
# Basic experiment (with habitat environments)
uv run monty -e surf_agent_1lm_10distobj_base

# With GPU acceleration on WSL2
HSIM_DISABLE_CUDA_DEVICE=1 GALLIUM_DRIVER=d3d12 \
    uv run monty -e surf_agent_1lm_10distobj_base

# Run tests
uv run pytest tests/unit/habitat_sim_test.py

# With GPU for tests
HSIM_DISABLE_CUDA_DEVICE=1 GALLIUM_DRIVER=d3d12 \
    uv run pytest tests/unit/habitat_sim_test.py
```

## What's Different from Official Habitat-Sim?

This experimental build includes:

1. **PyPI Distribution**: Available as `habitat-sim-uv-wheels-experimental` on PyPI
2. **UV/Pip Compatible**: Standard wheel format, no conda required
3. **WSL2 GPU Support**: Environment variable `HSIM_DISABLE_CUDA_DEVICE=1` to bypass CUDA device enumeration
4. **Build Controls**: Environment variables for reproducible builds
5. **TBP Integration**: Tested specifically with Monty experiments

**Code is identical** to official habitat-sim v0.2.2 - only the packaging and build system differ.

## For TBP Contributors

If you're contributing to TBP Monty and need to modify habitat-sim:

### Development Setup

```bash
# Clone this fork
git clone https://github.com/killerapp/habitat-sim.git
cd habitat-sim
git checkout feature/uv-wsl2-support

# See UV_WSL2_BUILD.md for complete build instructions
cat UV_WSL2_BUILD.md
```

### Building Custom Wheels

```bash
# Apply OpenEXR patches (documented in OPENEXR_PATCHES.md)
# See OPENEXR_PATCHES.md for details

# Configure build
cmake -S . -B build \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_MAKE_PROGRAM=/usr/bin/ninja \
    -DBUILD_WITH_BULLET=ON \
    -GNinja

# Build (single-threaded to avoid memory issues)
cmake --build build --config RelWithDebInfo -- -j1

# Create wheel (reusing existing build)
HSIM_NO_SUBMODULE_UPDATE=1 HSIM_SKIP_CMAKE_BUILD=1 \
    uv build --wheel --out-dir dist

# Install locally for testing
uv pip install dist/*.whl
```

### Testing with TBP

```bash
# Link your custom build to TBP
cd ../tbp.monty

# Update pyproject.toml to use local wheel
[tool.uv.sources]
habitat-sim-uv-wheels-experimental = { path = "../habitat-sim/dist/habitat_sim_uv_wheels_experimental-0.2.2-cp38-cp38-manylinux_2_17_x86_64.manylinux2014_x86_64.whl" }

# Sync and test
uv sync --extra dev
uv run pytest tests/unit/habitat_sim_test.py
```

## Available Python Versions

Currently available wheels:
- **Python 3.8**: ✅ Available (Linux/WSL2)
- **Python 3.9**: 🚧 Coming soon
- **Python 3.10**: 🚧 Coming soon
- **Python 3.11+**: 🚧 Coming soon

If you need a specific Python version, please [open an issue](https://github.com/killerapp/habitat-sim/issues).

## Experimental Status

⚠️ **This is an experimental build**. While it works well with TBP Monty, consider:

- **Not official**: This is a community-maintained fork, not endorsed by Meta/FAIR
- **Limited platforms**: Currently only Linux/WSL2 x86_64
- **WSL2 specific**: Some features (like GPU support) require WSL2-specific environment variables
- **Testing**: Thoroughly tested with TBP Monty, but your mileage may vary

**For production or official research**, consider using the [official conda-based installation](https://github.com/facebookresearch/habitat-sim).

## Troubleshooting for TBP Users

### Import Error: `No module named 'habitat_sim'`

**Solution**: The package name on PyPI is different from the import name.

```bash
# Install the PyPI package
pip install habitat-sim-uv-wheels-experimental

# But import as usual
python -c "import habitat_sim"  # This works!
```

### GPU Not Working on WSL2

**Solution**: Set environment variables before running:

```bash
export HSIM_DISABLE_CUDA_DEVICE=1
export GALLIUM_DRIVER=d3d12
sudo chmod 666 /dev/dri/renderD128

# Then run Monty
uv run monty -e <experiment_name>
```

### Monty Experiments Failing with Rendering Errors

**Solution**: Make sure you're using the GPU environment variables (see above). Some Monty experiments require hardware rendering.

### "Magnum not found" Error

**Solution**: Magnum is bundled separately. If using local builds, make sure to install magnum:

```bash
# Magnum is built as part of habitat-sim
# For published wheels, it's included automatically
# For local builds:
uv pip install ../habitat-sim/build/deps/magnum-bindings/src/python
```

## More Information

### TBP Monty Resources
- **Documentation**: https://thousandbrainsproject.readme.io/
- **Repository**: https://github.com/thousandbrainsproject/tbp.monty
- **API Docs**: https://api-monty.thousandbrains.org
- **Community**: https://thousandbrains.discourse.group/

### This Fork
- **Repository**: https://github.com/killerapp/habitat-sim
- **Branch**: `feature/uv-wsl2-support`
- **PyPI Package**: https://pypi.org/project/habitat-sim-uv-wheels-experimental/
- **Build Guide**: [UV_WSL2_BUILD.md](UV_WSL2_BUILD.md)
- **OpenEXR Patches**: [OPENEXR_PATCHES.md](OPENEXR_PATCHES.md)

### Original Habitat-Sim
- **Repository**: https://github.com/facebookresearch/habitat-sim
- **Documentation**: https://aihabitat.org/docs/habitat-sim/

## Contributing

Found a bug with TBP integration? Have a feature request?

1. For **TBP Monty issues**: Open an issue at [tbp.monty](https://github.com/thousandbrainsproject/tbp.monty/issues)
2. For **habitat-sim wheel issues**: Open an issue at [this fork](https://github.com/killerapp/habitat-sim/issues)
3. For **official habitat-sim**: See [upstream repository](https://github.com/facebookresearch/habitat-sim)

## License

Same as official habitat-sim: [MIT License](LICENSE)

This is an unofficial fork - all credit for habitat-sim goes to the original FAIR team.

---

**Built with ❤️ for the Thousand Brains Project community**
