module purge
module load cuda/10.0
module load cudnn/v7.6-cuda.10.0
module load NCCL/2.4.7-1-cuda.10.0

source activate sim2real-13

export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu/nvidia-opengl:${LD_LIBRARY_PATH}"
export GLOG_minloglevel=2
export MAGNUM_LOG=quiet
export PYTHONPATH="/private/home/akadian/sim2real/habitat-sim:${PYTHONPATH}"
export PHYSICS_CONFIG="/private/home/akadian/sim2real/habitat-sim/data/default.phys_scene_config.json"


# python examples/example.py --enable_physics --max_frames 1 --save_png

python examples/clutter_nav.py
# python examples/test_recompute_navmesh.py