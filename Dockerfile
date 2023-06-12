# FROM ubuntu:latest

# Install basic packages
# RUN apt update && apt install locales
# RUN locale-gen en_US en_US.UTF-8
# RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
# RUN export LANG=en_US.UTF-8
# RUN apt update

# RUN apt install -y openssl ca-certificates python3 python3-pip netcat iputils-ping iproute2 git

# RUN apt install -y software-properties-common
# RUN add-apt-repository universe
# RUN apt update && apt install curl -y
# RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
# RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# RUN useradd --create-home --home-dir /home/ubuntu --shell /bin/bash --user-group --groups adm,sudo ubuntu && \
#     echo ubuntu:ubuntu | chpasswd && \
#     echo "ubuntu ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# RUN apt update
# RUN apt -y upgrade
# RUN apt install -y ros-humble-desktop


# # Set up certificates
# ARG cert_location=/usr/local/share/ca-certificates
# RUN mkdir -p ${cert_location}
# # Get certificate from "github.com"
# RUN openssl s_client -showcerts -connect github.com:443 </dev/null 2>/dev/null|openssl x509 -outform PEM > ${cert_location}/github.crt
# # Update certificates
# RUN update-ca-certificates

# RUN mkdir workspace
# WORKDIR /workspace/
# # RUN pip3 install -r sailbot/sailbot/requirements.txt
# # RUN pip3 install -r sailbot/sailbot/requirements-dev.txt
# ENTRYPOINT /bin/bash



# Copyright 2020-2022 Tiryoh<tiryoh@gmail.com>
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

FROM dorowu/ubuntu-desktop-lxde-vnc:focal
LABEL maintainer="Tiryoh<tiryoh@gmail.com>"

RUN sed -i.bak -r 's!(deb|deb-src) \S+!\1 http://archive.ubuntu.com/ubuntu/!' /etc/apt/sources.list
RUN wget -q -O - https://dl.google.com/linux/linux_signing_key.pub | apt-key add - 
RUN apt-get update -q && \
    apt-get upgrade -yq && \
    DEBIAN_FRONTEND=noninteractive apt-get install -yq --no-install-recommends wget curl git build-essential vim sudo lsb-release locales bash-completion tzdata gosu terminator && \
    rm -rf /var/lib/apt/lists/*
RUN rm /etc/apt/apt.conf.d/docker-clean
RUN useradd --create-home --home-dir /home/ubuntu --shell /bin/bash --user-group --groups adm,sudo ubuntu && \
    echo ubuntu:ubuntu | chpasswd && \
    echo "ubuntu ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

ARG ROS_DISTRO=foxy
ARG INSTALL_PACKAGE=desktop

RUN apt-get update -q && \
    apt-get install -y curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update -q && \
    apt-get install -y ros-${ROS_DISTRO}-${INSTALL_PACKAGE} \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-rosdep python3-vcstool \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs && \
    rosdep init && \
    rm -rf /var/lib/apt/lists/*

RUN gosu ubuntu rosdep update && \
    grep -F "source /opt/ros/${ROS_DISTRO}/setup.bash" /home/ubuntu/.bashrc || echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/ubuntu/.bashrc && \
    grep -F "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" /home/ubuntu/.bashrc || echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/ubuntu/.bashrc && \
    sudo chown ubuntu:ubuntu /home/ubuntu/.bashrc

RUN apt update
RUN apt install -y pip
COPY requirements.txt /workspace/
COPY requirements-dev.txt /workspace/

RUN pip install --no-cache-dir -r /workspace/requirements.txt
RUN pip install --no-cache-dir -r /workspace/requirements-dev.txt

# temporally fix to enable resize uri
# https://github.com/fcwu/docker-ubuntu-vnc-desktop/pull/247
RUN sed -i "s#location ~ .*/(api/.*|websockify) {#location ~ .*/(api/.*|websockify|resize) {#" /etc/nginx/sites-enabled/default

ENV USER ubuntu
ENV IS_DOCKER True

RUN mkdir workspace
WORKDIR /workspace/