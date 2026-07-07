#!/usr/bin/env bash
set -euo pipefail

MODEL_TRAIN_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="${IMAGE:-${AI_TOOLCHAIN_IMAGE:-ai_toolchain_ubuntu_22_s100_s600_cpu:v3.7.0}}"

if (($#)); then
  printf -v DOCKER_CMD "%q " "$@"
else
  DOCKER_CMD="bash"
fi

TTY_ARGS=(-i)
if [[ -t 0 && -t 1 ]]; then
  TTY_ARGS=(-it)
fi

docker run --rm "${TTY_ARGS[@]}" \
  --network host \
  --shm-size=15g \
  -v "${MODEL_TRAIN_DIR}:/data" \
  --workdir /data/perception_detector \
  --entrypoint /bin/bash \
  "${IMAGE}" \
  -lc "${DOCKER_CMD}"
