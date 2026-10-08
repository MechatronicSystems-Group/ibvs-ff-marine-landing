FROM ubuntu:22.04

# Prevent interactive prompts during install
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Africa/Johannesburg

# ----------
# ROS2 Setup
# ----------

RUN apt-get update && apt-get install -y locales

RUN locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Enable Universe repo and install prerequisites
RUN apt-get install -y software-properties-common \
    && add-apt-repository universe \
    && apt-get update && apt-get install -y curl

# Add ROS 2 apt source
RUN ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | python3 -c "import sys, json; print(json.load(sys.stdin)['tag_name'])") \
    && curl -L -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" \
    && dpkg -i /tmp/ros2-apt-source.deb \
    && rm /tmp/ros2-apt-source.deb
# Install ROS 2 Humble
RUN apt-get update && apt-get install -y ros-humble-ros-base ros-dev-tools

# Source ROS 2 for all future RUN commands and at runtime
ENV ROS_DISTRO=humble
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Handle GZ Harmonic sim compatibility
RUN apt-get install -y lsb-release gnupg
RUN curl https://packages.osrfoundation.org/gazebo.gpg \
        --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && apt-get install -y ros-humble-ros-gzharmonic

# ----------------------------
# Install PX4-autopilot 1.16.0
# ----------------------------

RUN git clone https://github.com/PX4/PX4-Autopilot.git /root/PX4-Autopilot \
    && cd /root/PX4-Autopilot \
    && git checkout v1.16.0 \
    && make submodulesclean

# Install PX4 dev. tools
RUN cd /root/PX4-Autopilot \
    && bash ./Tools/setup/ubuntu.sh --no-nuttx

# Compile PX4
RUN cd /root/PX4-Autopilot \
    && make px4_sitl

# -------------------------------
# Install custom content into PX4
# -------------------------------

COPY project_files/gz_px4_custom /root/ws_x500/project_files/gz_px4_custom/
RUN cd /root/ws_x500/project_files/gz_px4_custom/gz_plugins/barge_controller \
    && rm -rf build \
    && mkdir -p build \
    && cd build \
    && cmake .. \
    && make

RUN bash /root/ws_x500/project_files/gz_px4_custom/install.sh

# -----------------------
# Install MicroXRCE-Agent
# -----------------------

RUN git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /root/Micro-XRCE-DDS-Agent \
    && cd /root/Micro-XRCE-DDS-Agent \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make \
    && make install \
    && ldconfig /usr/local/lib/

# --------------
# Install OpenCV
# --------------

# Install correct opencv version (this can take a very long time)
ARG OPENCV_VERSION=4.10.0

# Install build dependencies for opencv
RUN apt-get update && apt-get install -y \
    build-essential \
    unzip \
    pkg-config \
    libgtk2.0-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libtbb-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libv4l-dev \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*

# Remove any existing OpenCV packages
RUN apt-get purge -y "*libopencv*" || true

# Download source
WORKDIR /opt

RUN curl -L https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.zip -o opencv.zip && \
    curl -L https://github.com/opencv/opencv_contrib/archive/${OPENCV_VERSION}.zip -o opencv_contrib.zip && \
    unzip opencv.zip && \
    unzip opencv_contrib.zip && \
    rm opencv.zip opencv_contrib.zip

# Build
WORKDIR /opt/opencv-${OPENCV_VERSION}
RUN mkdir build

WORKDIR /opt/opencv-${OPENCV_VERSION}/build

ARG ENABLE_NEON_FLAG=""
RUN if [ "$(uname -m)" = "aarch64" ] || [ "$(uname -m)" = "armv7l" ]; then \
        ENABLE_NEON_FLAG="-D ENABLE_NEON=ON"; \
    fi && \
    cmake \
        -D OPENCV_GENERATE_PKGCONFIG=ON \
        -D OPENCV_EXTRA_MODULES_PATH=/opt/opencv_contrib-${OPENCV_VERSION}/modules \
        -D WITH_GSTREAMER=ON \
        -D WITH_LIBV4L=ON \
        $ENABLE_NEON_FLAG \
        -D BUILD_opencv_python3=ON \
        -D BUILD_TESTS=OFF \
        -D BUILD_PERF_TESTS=OFF \
        -D BUILD_EXAMPLES=OFF \
        -D CMAKE_BUILD_TYPE=RELEASE \
        -D CMAKE_INSTALL_PREFIX=/usr/local \
        ..

RUN make -j$(($(nproc)-1))
RUN make install
RUN ldconfig

RUN pkg-config --modversion opencv4
    
# ---------------------------------------
# Additional project requirements & setup
# ---------------------------------------

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    libboost-all-dev \
    tmux

# Project direct dependencies
RUN pip install pymap3d scipy pyserial numpy==1.25.0
RUN pip3 install --upgrade "scipy>=1.10.0" # upgrade scipy

# Source ROS2
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /root/ws_x500/install/local_setup.bash" >> /root/.bashrc

WORKDIR /root/ws_x500