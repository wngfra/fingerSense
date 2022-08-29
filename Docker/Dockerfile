# Copyright (c) 2020 wngfra
# Use of this source code is governed by the Apache-2.0 license, see LICENSE

ARG BASE_IMAGE=nvcr.io/nvidia/pytorch:22.04-py3
FROM $BASE_IMAGE

RUN conda update conda --yes && \
    conda install -c conda-forge --quiet --yes \
    bidict \
    pandas \
    scipy \
    seaborn \
    scikit-learn && \
    conda clean -a -q -y

RUN pip3 install -U --no-cache-dir \
    scikit-fda

WORKDIR /workspace