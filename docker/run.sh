#!/bin/bash
PROG_NAME=$(basename $0)
RUN_DIR=$(dirname $(readlink -f $0))
PKG_DIR=$(dirname ${RUN_DIR})

ROS_DOMAIN_ID="0"
DOCKER_IMAGE="shikishimatasakilab/pmod-ros2:amd64-torch1.7"

function usage_exit {
  cat <<_EOS_ 1>&2
  Usage: build-docker.sh [OPTIONS...]
  OPTIONS:
    -h, --help              Show this help
    -i, --ros-domain-id ID  ROS_DOMAIN_ID (Default: ${ROS_DOMAIN_ID})
_EOS_
  exit 1
}

while (( $# > 0 )); do
  if [[ $1 == "-h" ]] || [[ $1 == "--help" ]]; then
    usage_exit
  elif [[ $1 == "-i" ]] || [[ $1 == "--ros-domain-id" ]]; then
    if [[ $2 == -* ]]; then
      echo "Invalid parameter"
      usage_exit
    else
      ROS_DOMAIN_ID=$2
    fi
    shift 2
  else
    echo "Invalid parameter: $1"
    usage_exit
  fi
done

DOCKER_VOLUME="${DOCKER_VOLUME} -v ${PKG_DIR}:/workspace/src/pmod_ros:rw"
DOCKER_ENV="${DOCKER_ENV} -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

docker run \
    -it \
    --rm \
    --gpus all \
    --net host \
    ${DOCKER_VOLUME} \
    ${DOCKER_ENV} \
    --name "pmod-ros" \
    ${DOCKER_IMAGE} \
    bash
