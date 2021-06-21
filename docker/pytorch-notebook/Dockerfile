# Copyright (c) 2020 wngfra
# Use of this source code is governed by the Apache-2.0 license, see LICENSE

ARG BASE_IMAGE=nvcr.io/nvidia/pytorch:21.05-py3
FROM $BASE_IMAGE

# Install Jupyter Notebook, Lab, and Hub
# Generate a notebook server config
# Cleanup temporary files
# Correct permissions
# Do all this in a single RUN command to avoid duplicating all of the
# files across image layers when the permissions change
RUN conda install -c conda-forge --quiet --yes \
    notebook \
    jupyterhub \
    jupyterlab && \
    conda clean --all -f -y && \
    npm cache clean --force && \
    jupyter notebook --generate-config && \
    jupyter lab clean

RUN pip3 install --no-cache-dir --yes \
    bidict \
    matplotlib \
    scikit-fda

EXPOSE 8888

WORKDIR "{HOME}"
