module purge

export PYTHONPATH=$(pwd)
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu/nvidia-opengl:${LD_LIBRARY_PATH}

source activate sim2real-13

# conda install numpy
./build.sh --headless --bullet -j20

# ./build.sh --headless --bullet -j20
