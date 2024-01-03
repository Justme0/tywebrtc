#!/usr/bin/env bash

# doc
# https://github.com/naivesystems/analyze/wiki/%E5%A6%82%E4%BD%95%E6%A3%80%E6%9F%A5%E4%BD%BF%E7%94%A8-Bazel-%E6%9E%84%E5%BB%BA%E7%9A%84%E9%A1%B9%E7%9B%AE

set -ex

mkdir -p output

# if use podman
# podman run --rm -v /usr/bin:/root/.local/bin:O \
#     -v $PWD:/src:O \
#     -v $PWD/.naivesystems:/config:Z \
#     -v $PWD/output:/output:Z \
#     -w /src/ \
#     -it ghcr.io/naivesystems/analyze:latest /bin/bash

# if use docker
docker run --platform linux/amd64 -v $(pwd):/src -v $(pwd)/.naivesystems:/config -v $(pwd)/output:/output ghcr.io/naivesystems/analyze:latest /opt/naivesystems/misra_analyzer -show_results