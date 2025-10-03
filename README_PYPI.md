# habitat-sim Wheels for TBP Monty

**Experimental pip/UV-compatible wheels** for [habitat-sim](https://github.com/facebookresearch/habitat-sim), built specifically for the [Thousand Brains Project (TBP) Monty](https://github.com/thousandbrainsproject/tbp.monty) sensorimotor learning system.

⚠️ **Unofficial community build** - Not affiliated with Meta/FAIR. For production use, see [official habitat-sim](https://github.com/facebookresearch/habitat-sim).

## What is This?

[**TBP Monty**](https://thousandbrainsproject.org) is a sensorimotor learning system that learns through active sensing (like your brain!). It uses habitat-sim for realistic 3D environments in object recognition experiments.

**Problem**: Official habitat-sim requires conda, incompatible with modern UV/pip workflows.

**Solution**: Standard Python wheels on PyPI - install with pip or uv, no conda needed.

## Installation

```bash
# With pip
pip install habitat-sim-uv-wheels-experimental

# With uv (recommended - much faster!)
uv pip install habitat-sim-uv-wheels-experimental

# Import as usual
python -c "import habitat_sim; print(habitat_sim.__version__)"
```

### For TBP Monty Users

```bash
# Clone TBP Monty
git clone https://github.com/thousandbrainsproject/tbp.monty.git
cd tbp.monty

# Install everything with UV (includes habitat-sim)
uv sync --extra dev

# Run an experiment
uv run monty -e surf_agent_1lm_10distobj_base
```

See [full TBP installation guide](https://github.com/killerapp/habitat-sim/blob/feature/uv-wsl2-support/README_TBP_USERS.md).

## WSL2 GPU Support

For hardware-accelerated rendering on WSL2:

```bash
# Enable D3D12 backend (NVIDIA RTX recommended)
export HSIM_DISABLE_CUDA_DEVICE=1
export GALLIUM_DRIVER=d3d12
sudo chmod 666 /dev/dri/renderD128

# Run with GPU
HSIM_DISABLE_CUDA_DEVICE=1 GALLIUM_DRIVER=d3d12 python your_script.py
```

## Platform Support

- **Linux**: ✅ manylinux2014+ (Ubuntu 20.04+, Debian 11+, etc.)
- **WSL2**: ✅ With D3D12 GPU support
- **macOS**: ❌ Not yet available
- **Windows**: ❌ Use WSL2

**Python Versions**:
- Python 3.8: ✅ Available
- Python 3.9+: 🚧 Coming soon ([request here](https://github.com/killerapp/habitat-sim/issues))

## What's Different from Official?

This build:
- ✅ Available on PyPI (no conda required)
- ✅ WSL2 GPU support via `HSIM_DISABLE_CUDA_DEVICE=1`
- ✅ Tested with TBP Monty experiments
- ✅ Same code as official v0.2.2 (only packaging differs)

## Quick Example

```python
import habitat_sim

# Load a 3D scene
backend_cfg = habitat_sim.SimulatorConfiguration()
backend_cfg.scene_id = "path/to/scene.glb"

# Create agent configuration
agent_cfg = habitat_sim.agent.AgentConfiguration()

# Initialize simulator
cfg = habitat_sim.Configuration(backend_cfg, [agent_cfg])
sim = habitat_sim.Simulator(cfg)

# Get an observation
obs = sim.get_sensor_observations()
rgb = obs["color_sensor"]
print(f"Image shape: {rgb.shape}")
```

For TBP Monty examples, see [TBP documentation](https://thousandbrainsproject.readme.io/).

## Troubleshooting

### Import Error after Install

**Problem**: `pip install habitat-sim-uv-wheels-experimental` succeeded but `import habitat_sim` fails.

**Solution**: The PyPI package name is different from the import name (this is normal). Just `import habitat_sim`.

### Rendering Errors on WSL2

**Problem**: Crashes or black screens when rendering.

**Solution**: Set WSL2 environment variables (see "WSL2 GPU Support" above).

### Missing Magnum Module

**Problem**: `ModuleNotFoundError: No module named 'magnum'`

**Solution**: Magnum should be installed automatically. Try reinstalling:
```bash
pip install --force-reinstall habitat-sim-uv-wheels-experimental
```

## Links

**TBP Monty**:
- Documentation: https://thousandbrainsproject.readme.io/
- Repository: https://github.com/thousandbrainsproject/tbp.monty
- Community: https://thousandbrains.discourse.group/

**This Fork**:
- Repository: https://github.com/killerapp/habitat-sim
- Build Guide: [UV_WSL2_BUILD.md](https://github.com/killerapp/habitat-sim/blob/feature/uv-wsl2-support/UV_WSL2_BUILD.md)
- TBP Instructions: [README_TBP_USERS.md](https://github.com/killerapp/habitat-sim/blob/feature/uv-wsl2-support/README_TBP_USERS.md)

**Original habitat-sim**:
- Repository: https://github.com/facebookresearch/habitat-sim
- Official Docs: https://aihabitat.org/docs/habitat-sim/

## Contributing

- **TBP Monty issues**: https://github.com/thousandbrainsproject/tbp.monty/issues
- **Wheel build issues**: https://github.com/killerapp/habitat-sim/issues
- **Official habitat-sim**: https://github.com/facebookresearch/habitat-sim/issues

## License

MIT License - Same as official habitat-sim.

**Credits**: All credit for habitat-sim goes to the original FAIR team at Meta AI. This is just alternative packaging.

---

**Built for the [Thousand Brains Project](https://thousandbrainsproject.org) community** 🧠
