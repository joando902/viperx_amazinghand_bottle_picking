#!/bin/bash

### --- config --- ###
DOCKER_USERNAME=hrcnthu
IMAGE_NAME=viperx-300s
IMAGE_TAG=default
CONTAINER_SERVICE=dev

IMAGE_FULL_NAME="${DOCKER_USERNAME}/${IMAGE_NAME}:${IMAGE_TAG}"


# 1. env setup
export COMPOSE_BAKE=true
export DISPLAY=:1
xhost +local:docker
cd docker

# 3. cmd-line args
usage() {
  echo "usage: $0 [mode]"
  echo "mode:"
  echo "- build              Build the colcon ws"
  echo "- dev                Entry the env without any service"
  echo "- rviz-ee            To control ViperX by publishing end effector pose via RViz."
  exit 1
}

# 4. Check and validate mode argument
[[ $# -lt 1 || ! "$1" =~ ^(build|dev|rviz-ee)$ ]] && usage

# 5. Startup the container
# Reuse the development container exactly as-is so packages installed inside
# it (for example Ultralytics) are not lost to an unnecessary Compose recreate.
if [[ "$1" == "dev" ]] && docker container inspect dev >/dev/null 2>&1; then
  docker start dev >/dev/null
  echo "Started existing 'dev' container without recreating it."
else
  docker compose -f 'cpu.compose.yml' up "$1" -d
fi
# docker compose -f 'cpu.compose.yml' up --build "$1" -d
