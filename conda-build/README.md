### Building for macOS

Running ```python macos_matrix_builder.py``` with python >= 3.6 will start the build process by setting all the environment variables and making a call to conda build. Make sure the meta.yaml file in conda-build/habitat-sim/ is configured correctly accoridng to https://docs.conda.io/projects/conda-build/en/latest/resources/define-metadata.html

Once the package is built, make sure you're logged in to anaconda cloud and then run ```anaconda upload <path to the tarball file that conda build created>```. For exmaple ```anaconda upload hsim-macos/osx-64/habitat-sim-1.3.2-py3.6_osx.tar.bz2```. This will upload the package to anaconda cloud for everyone to download.

To then download the package, run ```conda install -c aihabitat -c conda-forge habitat-sim```.


### Building for Linux

The process is almost the same for linux; there is a corresponding python script for starting things off, however we use a docker container to do the builds. There is a dockerfile in this directory that you can use to create a container.

```docker build -t mycontainer -f Dockerfile .```

That will create your docker container. Now run

```docker run -it --ipc=host --rm -v $(pwd)/../:/remote mycontainer bash```

From there you will have a shell within your linux container from which you can run ```python linux_matrix_build.py``` again with python >=3.6 (so you might have to create a conda environment with this version of python), which will kick off the build process. After this has finished, upload it to anaconda cloud in the same way described in the macOS section.

