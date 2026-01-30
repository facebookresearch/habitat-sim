FROM nvidia/cuda:12.1.0-devel-ubuntu22.04

ENV LC_ALL en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US.UTF-8
ENV DEBIAN_FRONTEND=noninteractive
ARG AIHABITAT_CONDA_CHN
ARG AIHABITAT_CONDA_CHN_PWD

RUN apt-get update && apt-get install -y \
    wget \
    curl \
    perl \
    cmake \
    xz-utils \
    bzip2 \
    git \
    patch \
    unzip \
    python3 \
    autoconf \
    automake \
    make \
    libegl1-mesa-dev \
    libgl1-mesa-dev \
    locales \
    && rm -rf /var/lib/apt/lists/*

# Set locale
RUN locale-gen en_US.UTF-8

# Install patchelf
ADD ./common/install_patchelf.sh install_patchelf.sh
RUN bash ./install_patchelf.sh && rm install_patchelf.sh

# switch shell sh (default in Linux) to bash
SHELL ["/bin/bash", "-c"]

# Install Anaconda
ENV PATH /opt/conda/bin:$PATH
ADD ./common/install_conda.sh install_conda.sh
RUN bash ./install_conda.sh && rm install_conda.sh

RUN conda init bash && conda create --name py312 python=3.12 -y
RUN source ~/.bashrc && conda activate py312 && conda install -y anaconda-client git gitpython ninja conda-build
RUN conda config --set anaconda_upload yes
