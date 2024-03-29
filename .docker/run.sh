#!/bin/bash

#!/usr/bin/env bash

#
# Copyright (C) 2023 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#

# Runs a docker container with the image created by build.bash
# Requires:
#   docker
#   an X server
# Optional:
#   nvidia-docker
#   A joystick mounted to /dev/input/js0 or /dev/input/js1

# Default to NVIDIA
# Note that on all non-Debian/non-Ubuntu platforms, dpkg won't exist so they'll always choose
# --runtime=nvidia.  If you are running one of those platforms and --runtime=nvidia doesn't work
# for you, change the else statement.
if [[ -x "$(command -v dpkg)" ]] && dpkg --compare-versions "$(docker version --format '{{.Server.Version}}')" gt "19.3"; then
  DOCKER_OPTS="--gpus=all"
else
  DOCKER_OPTS="--runtime=nvidia"
fi

for arg in "$@"
do
    if [ "$arg" == "--no-nvidia" ]
    then
        DOCKER_OPTS=""
    fi
done

IMG="3d-surface-planner:latest"

ARGS=("$@")
WORKSPACES=("${ARGS[@]:1}")

# Make sure processes in the container can connect to the x server
# Necessary so gazebo can create a context for OpenGL rendering (even headless)
XAUTH=$HOME/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist $DISPLAY | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        touch $XAUTH
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

# Share your vim settings.
VIMRC=~/.vimrc
if [ -f $VIMRC ]
then
  DOCKER_OPTS="$DOCKER_OPTS -v $VIMRC:/home/developer/.vimrc:ro"
fi

# Share your custom terminal setup commands
GITCONFIG=~/.gitconfig
DOCKER_OPTS="$DOCKER_OPTS -v $GITCONFIG:/home/developer/.gitconfig:ro"

for WS_DIR in ${WORKSPACES[@]}
do
  WS_DIRNAME=$(basename $WS_DIR)
  if [ ! -d $WS_DIR/src ]
  then
    echo "Other! $WS_DIR"
    DOCKER_OPTS="$DOCKER_OPTS -v $WS_DIR:/home/developer/other/$WS_DIRNAME"
  else
    echo "Workspace! $WS_DIR"
    DOCKER_OPTS="$DOCKER_OPTS -v $WS_DIR:/home/developer/workspaces/$WS_DIRNAME"
  fi
done

# Synchronize ROS 2 and Gazebo transport envhumblement variables with the host
# Default to isolated communication
ROS_ENVS=(
  "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-"0"}"
  "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-"1"}"
)
GZ_ENVS=(
  "GZ_IP=${GZ_IP:-"127.0.0.1"}"
  "GZ_RELAY=${GZ_RELAY:-"127.0.0.1"}"
)
DOCKER_OPTS="${DOCKER_OPTS} ${ROS_ENVS[*]/#/"-e "} ${GZ_ENVS[*]/#/"-e "}"

# move dir to /path/to/rustremoval/docker 
current_dir=$(pwd)

if [[ $current_dir != */.docker ]]; then
    current_dir="$current_dir/.docker"
    echo "Added '/.docker' to the current directory"
fi

# Get the name of the directory containing the project
directory_name=$(basename $(dirname $current_dir))

# Mount extra volumes if needed.
# E.g.:
# -v "/opt/sublime_text:/opt/sublime_text" \

# Relax X server permissions so that local X connections work; this is necessary
# when running under XWayland
xhost + local:

# --ipc=host and --network=host are needed for no-NVIDIA Dockerfile to work
docker run -itd \
  -e DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e XAUTHORITY=$XAUTH \
  -v "$XAUTH:$XAUTH" \
  -v "/dev/shm:/dev/shm" \
  -v "/tmp/.X11-unix:/tmp/.X11-unix" \
  -v "/etc/localtime:/etc/localtime:ro" \
  -v "$current_dir/..:/home/developer/$directory_name" \
  -v "/dev/input:/dev/input" \
  --name ${IMG//[:\/]/_} \
  --privileged \
  --security-opt seccomp=unconfined \
  --ipc=host \
  --network=host \
  $DOCKER_OPTS \
  $IMG

# Put X server permissions back to what they were.  If this script is killed
# uncleanly, then this may not run.
xhost - local: