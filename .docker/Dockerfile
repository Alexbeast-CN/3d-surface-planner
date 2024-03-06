FROM nvidia/opengl:1.0-glvnd-devel-ubuntu22.04

# This avoids keyboard interaction when asked for geographic area
ARG DEBIAN_FRONTEND=noninteractive

# Change apt source to ustc
RUN mv /etc/apt/sources.list /etc/apt/sources_backup.list && \
  echo "deb http://mirrors.ustc.edu.cn/ubuntu/ jammy main restricted universe multiverse" >> /etc/apt/sources.list && \
  echo "deb http://mirrors.ustc.edu.cn/ubuntu/ jammy-updates main restricted universe multiverse" >> /etc/apt/sources.list && \
  echo "deb http://mirrors.ustc.edu.cn/ubuntu/ jammy-backports main restricted universe multiverse " >> /etc/apt/sources.list && \
  echo "deb http://mirrors.ustc.edu.cn/ubuntu/ jammy-security main restricted universe multiverse" >> /etc/apt/sources.list


# Install some basic utilities
RUN apt-get update && \
  apt-get install -y \
  build-essential \
  curl \
  doxygen \
  git \
  gnupg2 \
  libbluetooth-dev \
  libcwiid-dev \
  libgoogle-glog-dev \
  libspnav-dev \
  libusb-dev \
  locales \
  lsb-release \
  mercurial \
  python3-dbg \
  python3-empy \
  python3-pip \
  python3-psutil \
  python3-venv \
  software-properties-common \
  sudo \
  tzdata \
  vim \
  rsync \
  ufw \
  gdb \
  gdbserver \
  wget \
  openssh-client \
  clang-format-11 \
  cpplint \
  cppcheck \
  flake8 \
  libxml2-utils \
  tmux \
  openssh-server \
  bash-completion \
  && apt-get clean

# Set Locale for ROS 2
RUN locale-gen en_US en_US.UTF-8 && \
  update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
  export LANG=en_US.UTF-8

# Add ROS2 key
COPY ../ros.key  /usr/share/keyrings/ros-archive-keyring.gpg
RUN curl -s https://gitee.com/ohhuo/rosdistro/raw/master/ros.asc | apt-key add -

# Set up sources.list
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu jammy main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null


RUN export DEBIAN_FRONTEND=noninteractive

# Install ROS 2, Gazebo garden, and build tools
# https://colcon.readthedocs.io/en/released/user/installation.html
RUN /bin/sh -c 'wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg' \
  && /bin/sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null' \
  && apt-get update \
  && apt-get install -y \
  python3-rosdep \
  python3-vcstool \
  python3-argcomplete \
  python3-colcon-common-extensions \
  python3-colcon-mixin \
  ros-humble-desktop \
  ros-humble-ros2-control\
  ros-humble-ros2-controllers\
  ros-humble-xacro\
  ros-humble-robot-localization\
  ros-humble-joint-state-broadcaster\
  ros-humble-joint-state-publisher\
  ros-humble-joint-state-publisher-gui\
  ros-humble-twist-mux\
  ros-humble-bondcpp \
  ros-humble-test-msgs \
  gz-garden \
  ros-humble-ros-gzgarden \
  libopencv-dev \
  && apt-get clean

# install python packages
RUN pip install \
  pre-commit \
  open3d \
  numpy==1.24

# Initialize rosdep
RUN pip install rosdepc && rosdepc init && rosdepc update

# Set up environment variables
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1
ENV XAUTHORITY=/tmp/.docker.xauth

# Add a user with the same user_id as the user outside the container
# Requires a docker build argument `user_id`
ARG user_id
ENV USERNAME developer
RUN useradd -U --uid $user_id -ms /bin/bash $USERNAME \
  && echo "$USERNAME:1" | chpasswd \
  && adduser $USERNAME sudo \
  && echo "$USERNAME ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USERNAME

# Setup SSH key to be the same as the host user
ARG SSH_PRIVATE_KEY

RUN mkdir -p /home/$USERNAME/.ssh && \
  echo "$SSH_PRIVATE_KEY" > /home/$USERNAME/.ssh/id_rsa && \
  chown $USERNAME:sudo /home/$USERNAME/.ssh/id_rsa && \
  chmod 600 /home/$USERNAME/.ssh/id_rsa

# Create a custom script to use for Git SSH commands
RUN echo -e '#!/bin/sh\nexec /usr/bin/ssh -o "StrictHostKeyChecking=no" -i /home/$USERNAME/.ssh/id_rsa "$@"' > /usr/local/bin/git-ssh
RUN chmod +x /usr/local/bin/git-ssh

# Set the custom script as the GIT_SSH_COMMAND
ENV GIT_SSH_COMMAND="/usr/local/bin/git-ssh"

# setup bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc && \
  echo "export PATH="$PATH:/home/developer/.local/bin"" >> /home/$USERNAME/.bashrc

# Commands below run as the developer user
USER $USERNAME

# When running a container start in the developer's home folder
WORKDIR /home/$USERNAME/3d-surface-planner
