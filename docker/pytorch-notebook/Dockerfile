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
    'notebook=6.4.0' \
    'jupyterhub=1.4.1' \
    'jupyterlab=3.0.16' && \
    conda clean --all -f -y && \
    npm cache clean --force && \
    jupyter notebook --generate-config && \
    jupyter lab clean

EXPOSE 8888

WORKDIR "${HOME}"