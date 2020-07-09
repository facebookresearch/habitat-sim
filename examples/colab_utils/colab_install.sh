#!/bin/bash
#Don't run again if it's already installed
[ -f /content/habitat_sim_installed ] && exit

#Install Miniconda
cd /content/
wget -c https://repo.continuum.io/miniconda/Miniconda3-latest-Linux-x86_64.sh && bash Miniconda3-latest-Linux-x86_64.sh -bfp /usr/local

#Adds the conda libraries directly to the colab path.
!ln -s /usr/local/lib/python3.6/dist-packages /usr/local/lib/python3.6/site-packages

##Install Habitat-Sim and Magnum binaries
conda config --set default_threads 4 #Enables multithread conda installation
conda install -y --prefix /usr/local -c aihabitat -c conda-forge habitat-sim headless withbullet python=3.6

#Shallow GIT clone for speed
git clone https://github.com/facebookresearch/habitat-api --depth 1
git clone https://github.com/facebookresearch/habitat-sim --depth 1

#Install Requirements.
python3.6 -m pip install -r /content/habitat-api/requirements.txt
cd /content/habitat-api/
python3.6 setup.py develop --all
cd /content/habitat-sim/
rm -rf habitat_sim/ # Deletes the habitat_sim folder so it doesn't interfere with import path

#Download Assets
wget -c http://dl.fbaipublicfiles.com/habitat/habitat-test-scenes.zip && unzip -o habitat-test-scenes.zip
wget -c http://dl.fbaipublicfiles.com/habitat/objects_v0.1.zip && unzip -o objects_v0.1.zip -d data/objects/
wget -c http://dl.fbaipublicfiles.com/habitat/locobot_merged.zip && unzip -o locobot_merged.zip -d data/objects

touch /content/habitat_sim_installed
