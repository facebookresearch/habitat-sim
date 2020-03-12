#!/bin/bash

set -ex

# Anaconda
wget -q https://repo.continuum.io/miniconda/Miniconda2-latest-Linux-x86_64.sh
chmod +x  Miniconda2-latest-Linux-x86_64.sh
./Miniconda2-latest-Linux-x86_64.sh -b -p /opt/conda
rm Miniconda2-latest-Linux-x86_64.sh
export PATH=/opt/conda/bin:$PATH
conda install -y conda-build anaconda-client git ninja
conda remove -y --force patchelf
