#!/bin/bash
if [ $# -eq 0 ]
then
  # default image and tag
  image_name="3d-surface-planner:latest"
elif [ $# -eq 1 ]
then
  image_name=$1
fi
# check if ~/.ssh/id_rsa exist
if [ ! -f ~/.ssh/id_rsa ]; then
    echo "SSH private key (id_rsa) not found in ~/.ssh/ directory."
    exit 1
fi

# read ~/.ssh/id_rsa and save it into ssh_private_key_content
ssh_private_key_content=$(cat ~/.ssh/id_rsa)

# Get path to current directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# check if Dockerfile exist
if [ ! -f $DIR/Dockerfile ]; then
    echo "Please change directory to /xxx/3d-surface-planner/.docker!"
    exit 1
fi

user_id=$(id -u)

echo "Building..."

docker build --rm -t \
  $image_name \
  --build-arg user_id=$user_id \
  --build-arg host_name=$USER \
  --build-arg SSH_PRIVATE_KEY="$ssh_private_key_content" \
  --build-arg BUILD_DATE=$(date -u +'%Y-%m-%dT%H:%M:%SZ') \
  -f $DIR/Dockerfile .

echo "Done!"