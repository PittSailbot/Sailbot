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

# Install uv
COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /usr/local/bin/

# Copy project files to leverage Docker cache
COPY pyproject.toml uv.lock /workspace/

# Install Python dependencies into system Python (not /workspace/.venv)
# so they are still available when VS Code bind-mounts /workspace.
RUN ARCH=$(uname -m) && \
    if [ "$ARCH" = "aarch64" ]; then \
        uv export --frozen --no-dev --group pi --no-emit-project --format requirements.txt --directory /workspace --output-file /tmp/requirements.txt; \
    else \
        uv export --frozen --no-dev --no-emit-project --format requirements.txt --directory /workspace --output-file /tmp/requirements.txt; \
    fi && \
    uv pip install --system --break-system-packages -r /tmp/requirements.txt && \
    rm -f /tmp/requirements.txt

# Create .uv-venv in /workspace for dev container consistency
# This ensures the venv exists and has all dependencies with catkin_pkg
RUN cd /workspace && \
    uv venv .uv-venv && \
    uv pip install -p .uv-venv --break-system-packages \
        catkin_pkg \
        rosdistro && \
    .uv-venv/bin/pip install -e . --no-deps 2>/dev/null || true


# Clean up apt cache and temporary files
RUN apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

WORKDIR /workspace

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]