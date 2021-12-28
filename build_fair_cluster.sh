module load anaconda3
# module load cuda/10.2
# module load cudnn/v7.6.5.32-cuda.10.2
module load cuda/11.0
module load cudnn/v8.0.3.33-cuda.11.0

# source activate py36habcuda102
source activate py38habcuda113
source /private/home/eundersander/downloads/1.2.189.0/setup-env.sh

export PYTHONPATH=/private/home/eundersander/projects/habitat-sim-symlink

./build.sh --build-type Release --build-temp "build_release" --bullet --build-tests --headless
