FROM ubuntu:jammy

# Set the locale
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8

# Set the timezone
ENV ROS_VERSION=2
ENV ROS_DISTRO=humble
ENV ROS_PYTHON_VERSION=3
ENV TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Setup the sources
RUN apt-get update && apt-get install -y software-properties-common curl && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

    # Install ROS 2 packages
    RUN apt-get update && apt-get upgrade -y && \
        apt-get install -y ros-humble-desktop 

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    nano \
    iputils-ping \
    wget \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    dos2unix \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro humble

# Environment setup
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
RUN echo 'export PYTHONPATH=$PYTHONPATH:/workspace/install/sailbot/lib' >> ~/.bashrc
RUN echo '#!/usr/bin/env bash' > /ros_entrypoint.sh
RUN echo 'source /opt/ros/humble/setup.bash' >> /ros_entrypoint.sh
RUN echo 'exec "$@"' >> /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

RUN apt update
RUN apt install -y pip
COPY requirements-dev.txt /workspace/
COPY requirements-pi.txt /workspace/

ENV ROS_DOMAIN_ID 0

ENV USER ubuntu

RUN apt-get install -y net-tools iproute2


RUN apt-get install -y ros-humble-camera-ros
RUN apt-get install -y pkg-config python3-yaml python3-ply python3-jinja2 openssl libyaml-dev libssl-dev libudev-dev libatomic1 meson

RUN git clone https://github.com/christianrauch/camera_ros.git /camera_ws/src/camera_ros
RUN pip install colcon-meson
RUN /bin/bash -c "cd /camera_ws/ && \
                  source /opt/ros/humble/setup.bash && \
                  colcon build"

# Install the GPIO library if running on the Pi (assumed that Pi is only aarch64 cpu used)
RUN uname -m > /tmp/arch.txt
RUN cat /tmp/arch.txt | grep -q 'aarch64' && pip install --no-cache-dir -r /workspace/requirements-pi.txt || echo "Not a Raspberry Pi, skipping extra dependency installation"
RUN cat /tmp/arch.txt | grep -q 'aarch64' && pip install RPi.GPIO || echo "Not a Raspberry Pi, skipping extra dependency installation"

RUN pip install --no-cache-dir -r /workspace/requirements-dev.txt

WORKDIR /workspace/