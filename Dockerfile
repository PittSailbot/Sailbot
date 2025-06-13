# Use official ROS image as base to reduce build time and image size
FROM ros:humble-ros-base-jammy

# Set environment variables at the top
ENV ROS_VERSION=2 \
    ROS_DISTRO=humble \
    ROS_PYTHON_VERSION=3 \
    TZ=Europe/Berlin \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    ROS_DOMAIN_ID=0 \
    DEBIAN_FRONTEND=noninteractive

# Configure locale and timezone in a single RUN layer
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales tzdata && \
    locale-gen en_US.UTF-8 && \
    ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone && \
    rm -rf /var/lib/apt/lists/*

# Install core ROS desktop and development tools in a single layer
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
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
    python3-pip \
    python3-venv \
    net-tools \
    iproute2 \
    python3-libgpiod \
    && rm -rf /var/lib/apt/lists/*

# Bootstrap rosdep (skip if already initialized)
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        rosdep init; \
    fi && \
    rosdep update --rosdistro $ROS_DISTRO

# Set up environment and entrypoint
RUN echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> ~/.bashrc && \
    echo 'export PYTHONPATH=$PYTHONPATH:/workspace/install/sailbot/lib' >> ~/.bashrc && \
    echo '#!/usr/bin/env bash\nsource /opt/ros/$ROS_DISTRO/setup.bash\nexec "$@"' > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

# Install additional ROS packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-geographic-msgs \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements files first to leverage Docker cache
COPY requirements-dev.txt requirements-pi.txt /workspace/

# Conditional Pi-specific installations
RUN ARCH=$(uname -m) && \
    if [ "$ARCH" = "aarch64" ]; then \
        pip install --no-cache-dir -r /workspace/requirements-pi.txt RPi.GPIO; \
    else \
        echo "Not a Raspberry Pi, skipping extra dependency installation"; \
    fi

# Install Python dependencies
RUN pip install --no-cache-dir \
    -r /workspace/requirements-dev.txt \
    build \
    adafruit-blinka \
    adafruit-circuitpython-gps

# Clean up apt cache and temporary files
RUN apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

WORKDIR /workspace

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
# PRE-AI CHAT Dockerfile
# FROM ubuntu:jammy

# # Set the locale
# RUN apt-get update && apt-get install -y locales && \
#     locale-gen en_US en_US.UTF-8 && \
#     update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
#     export LANG=en_US.UTF-8

# # Set the timezone
# ENV ROS_VERSION=2
# ENV ROS_DISTRO=humble
# ENV ROS_PYTHON_VERSION=3
# ENV TZ=Europe/Berlin
# RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# # Setup the sources
# RUN apt-get update && apt-get install -y software-properties-common curl && \
#     add-apt-repository universe && \
#     curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
#     echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

#     # Install ROS 2 packages
#     RUN apt-get update && apt-get upgrade -y && \
#         apt-get install -y ros-humble-desktop 

# # install bootstrap tools
# RUN apt-get update && apt-get install --no-install-recommends -y \
#     build-essential \
#     git \
#     nano \
#     iputils-ping \
#     wget \
#     python3-colcon-common-extensions \
#     python3-colcon-mixin \
#     python3-rosdep \
#     python3-vcstool \
#     dos2unix \
#     && rm -rf /var/lib/apt/lists/*

# # bootstrap rosdep
# RUN rosdep init && \
#   rosdep update --rosdistro humble

# # Environment setup
# RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
# RUN echo 'export PYTHONPATH=$PYTHONPATH:/workspace/install/sailbot/lib' >> ~/.bashrc
# RUN echo '#!/usr/bin/env bash' > /ros_entrypoint.sh
# RUN echo 'source /opt/ros/humble/setup.bash' >> /ros_entrypoint.sh
# RUN echo 'exec "$@"' >> /ros_entrypoint.sh
# RUN chmod +x /ros_entrypoint.sh

# # Install ROS packages
# RUN apt-get update && apt-get install -y ros-humble-geographic-msgs

# RUN apt-get update && apt-get install -y pip
# COPY requirements-dev.txt /workspace/
# COPY requirements-pi.txt /workspace/

# ENV ROS_DOMAIN_ID 0

# RUN apt-get install -y net-tools iproute2


# # RUN apt-get install -y ros-humble-camera-ros
# # RUN apt-get update && apt-get install -y pkg-config python3-yaml python3-ply python3-jinja2 openssl libyaml-dev libssl-dev libudev-dev libatomic1 meson

# # RUN git clone https://github.com/christianrauch/camera_ros.git /camera_ws/src/camera_ros
# # RUN pip install colcon-meson
# # RUN /bin/bash -c "cd /camera_ws/ && \
# #                   source /opt/ros/humble/setup.bash && \
# #                   colcon build"

# # Install the GPIO library if running on the Pi (assumed that Pi is only aarch64 cpu used)
# RUN uname -m > /tmp/arch.txt
# RUN cat /tmp/arch.txt | grep -q 'aarch64' && pip install --no-cache-dir -r /workspace/requirements-pi.txt || echo "Not a Raspberry Pi, skipping extra dependency installation"
# RUN cat /tmp/arch.txt | grep -q 'aarch64' && pip install RPi.GPIO || echo "Not a Raspberry Pi, skipping extra dependency installation"

# RUN pip install --no-cache-dir -r /workspace/requirements-dev.txt

# # Install ZephyrRTOS
# # RUN wget https://apt.kitware.com/kitware-archive.sh \
# #     sudo bash kitware-archive.sh
# # RUN sudo apt install -y --no-install-recommends git cmake ninja-build gperf \
# #     ccache dfu-util device-tree-compiler wget \
# #     python3-dev python3-pip python3-setuptools python3-tk python3-wheel xz-utils file \
# #     make gcc gcc-multilib g++-multilib libsdl2-dev libmagic1
# # RUN west init ~/zephyrproject \
# #     cd ~/zephyrproject \
# #     west update
# # RUN west zephyr-export
# # RUN west packages pip --install

# # WORKDIR ~
# # USER root
# # RUN sudo apt-get install libusb-1.0-0-dev
# # RUN git clone https://github.com/mvp/uhubctl
# # WORKDIR ~/uhubctl
# # RUN cd ~/uhubctl && make
# # RUN sudo make install

# # GPS requirement
# WORKDIR /libgpiod/
# RUN sudo apt-get install -y python3-pip python3
# RUN pip install build
# RUN pip install adafruit-blinka
# RUN sudo apt install -y python3-libgpiod
# RUN pip install adafruit-circuitpython-gps

# # Dev Container PlatformIO Requirement
# #RUN apt-get update && apt-get install -y software-properties-common && \
#     #add-apt-repository universe && \
#     #apt-get update
# RUN apt-get update
# RUN apt-get install -y python3-venv

# RUN apt-get update && apt-get upgrade -y

# WORKDIR /workspace/