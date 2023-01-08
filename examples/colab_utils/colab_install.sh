#!/bin/bash

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

set -e
shopt -s extglob
shopt -s globstar

#Checks if in Google Colab and does not run installation if it is not
python -c 'import google.colab' 2>/dev/null || exit
#Don't run again if it's already installed
[ -f /content/habitat_sim_installed ] && echo "Habitat is already installed. Aborting..." >&2 && exit
trap 'catch $? $LINENO' EXIT # Installs trap now
catch() {
  if [ "$1" != "0" ]; then
    echo "An error occured during the installation of Habitat-sim or Habitat-Lab." >&2
  fi
}
#Don't change the colab versions for these libraries
PYTHON_VERSION="$( python -c 'import sys; print(".".join(map(str, sys.version_info[:2])))' )"
PIL_VERSION="$(python -c 'import PIL; print(PIL.__version__)')"
CFFI_VERSION="$(python -c 'import cffi; print(cffi.__version__)')"
NUMPY_VERSION="$(python -c 'import numpy as np; print(np.__version__)')"
SCIPY_VERSION="$(python -c 'import scipy; print(scipy.__version__)')"
NUMBA_VERSION="$(python -c 'import numba; print(numba.__version__)')"
#Install Miniconda
cd /content/
wget -c https://repo.continuum.io/miniconda/Miniconda3-latest-Linux-x86_64.sh && bash Miniconda3-latest-Linux-x86_64.sh -bfp /usr/local

#Adds the conda libraries directly to the colab path.
ln -s "/usr/local/lib/python${PYTHON_VERSION}/dist-packages" "/usr/local/lib/python${PYTHON_VERSION}/site-packages"

##Install Habitat-Sim and Magnum binaries
conda config --set pip_interop_enabled True
NIGHTLY="${NIGHTLY:-false}" #setting the ENV $NIGHTLY to true will install the nightly version from conda
CHANNEL="${CHANNEL:-aihabitat}"
if ${NIGHTLY}; then
  CHANNEL="${CHANNEL}-nightly"
fi
conda install -S -y --prefix /usr/local -c "${CHANNEL}" -c conda-forge habitat-sim headless withbullet "python=${PYTHON_VERSION}" "numpy=${NUMPY_VERSION}" "pillow=${PIL_VERSION}" "cffi=${CFFI_VERSION}" "scipy=${SCIPY_VERSION}" "numba=${NUMBA_VERSION}"

#Shallow GIT clone for speed
git clone https://github.com/facebookresearch/habitat-lab --depth 1
git clone https://github.com/facebookresearch/habitat-sim --depth 1

#Install Requirements.
cd /content/habitat-lab/
set +e
pip install -r ./habitat-lab/requirements.txt
reqs=(./habitat-baselines/habitat_baselines/**/requirements.txt)
pip install "${reqs[@]/#/-r}"
set -e
pip install -e habitat-lab
cd /content/habitat-sim/

#Download Assets
python src_python/habitat_sim/utils/datasets_download.py --uids ci_test_assets --replace --data-path data/

#symlink assets appear in habitat-api folder
ln -s /content/habitat-sim/data /content/habitat-lab/.

touch /content/habitat_sim_installed
