module purge

source activate sim2real-12
conda install numpy

export PYTHONPATH=$(pwd)
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu/nvidia-opengl:${LD_LIBRARY_PATH}

conda install numpy
./build.sh --headless --bullet -j20

