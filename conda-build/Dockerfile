FROM nvidia/cudagl:10.0-devel-centos7

ENV LC_ALL en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US.UTF-8
ARG AIHABITAT_CONDA_CHN
ARG AIHABITAT_CONDA_CHN_PWD

RUN yum install -y wget curl perl cmake util-linux xz bzip2 git patch which unzip python3
RUN yum install -y yum-utils centos-release-scl
RUN yum-config-manager --enable rhel-server-rhscl-7-rpms
RUN yum install -y devtoolset-7-gcc devtoolset-7-gcc-c++ devtoolset-7-gcc-gfortran devtoolset-7-binutils
ENV PATH=/opt/rh/devtoolset-7/root/usr/bin:$PATH
ENV LD_LIBRARY_PATH=/opt/rh/devtoolset-7/root/usr/lib64:/opt/rh/devtoolset-7/root/usr/lib:$LD_LIBRARY_PATH

RUN yum install -y autoconf aclocal automake make
RUN yum install -y mesa-libEGL-devel mesa-libGL-devel

# Install patchelf
ADD ./common/install_patchelf.sh install_patchelf.sh
RUN bash ./install_patchelf.sh && rm install_patchelf.sh

# Install CUDA (we don't build with cuda support for now)
# ADD ./common/install_cuda.sh install_cuda.sh
# RUN bash ./install_cuda.sh 9.2 10.0 10.1 && rm install_cuda.sh

# switch shell sh (default in Linux) to bash
SHELL ["/bin/bash", "-c"]

# Install Anaconda
ENV PATH /opt/conda/bin:$PATH
ADD ./common/install_conda.sh install_conda.sh
RUN bash ./install_conda.sh && rm install_conda.sh

RUN conda init bash && conda create --name py39 python=3.9 -y
RUN source ~/.bashrc && conda activate py39 && conda install -y anaconda-client git gitpython ninja conda-build
RUN conda config --set anaconda_upload yes
