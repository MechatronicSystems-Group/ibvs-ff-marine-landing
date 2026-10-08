FROM ros:humble

# Prevent interactive prompts during install
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Africa/Johannesburg

# ROS2 config
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US.UTF-8 &&  update-locale LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Dependencies for... something. image_transport I think(?)
RUN apt-get update && apt-get install -y \
	python3-colcon-common-extensions \
	libboost-all-dev

# -----------------------
# Install MicroXRCE-Agent
# -----------------------

# Micro-XRCE-Agent 
RUN mkdir /home/docker && cd /home/docker
WORKDIR /home/docker
RUN git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
WORKDIR /home/docker/Micro-XRCE-DDS-Agent
RUN mkdir build
WORKDIR /home/docker/Micro-XRCE-DDS-Agent/build
RUN cmake .. && make && sudo make install && ldconfig /usr/local/lib
WORKDIR /

# ---------------------------------------
# Build from source rpi fork of libcamera
# ---------------------------------------

# Install dependencies for libcamera 
RUN apt-get update && apt-get install -y \
    python3-pip \
    git \
    python3-jinja2 \
    ninja-build \
    libboost-dev \
    libgnutls28-dev \
    openssl \
    libtiff-dev \
    pybind11-dev \
    qtbase5-dev \
    libqt5core5a \
    libqt5widgets5 \
    cmake \
    python3-yaml \
    python3-ply \
    libglib2.0-dev \
    libgstreamer-plugins-base1.0-dev \
 && rm -rf /var/lib/apt/lists/*

RUN pip3 install --user meson
ENV PATH=$PATH:/root/.local/bin/
RUN ldconfig

# install raspberry-pi fork of libcamera
WORKDIR /home/docker
RUN git clone https://github.com/raspberrypi/libcamera.git
WORKDIR /home/docker/libcamera
RUN meson setup build --buildtype=release -Dgstreamer=disabled -Dpycamera=enabled
RUN ninja -C build install

# install rpicam-apps from source, as well as dependencies for opencv
WORKDIR /home/docker

RUN apt-get update && apt-get install -y \
    build-essential \
    unzip \
    pkg-config \
    cmake \
    libboost-program-options-dev \
    libdrm-dev \
    libexif-dev \
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

RUN git clone https://github.com/raspberrypi/rpicam-apps.git
WORKDIR /home/docker/rpicam-apps
RUN meson setup build \
    -Denable_libav=disabled \
    -Denable_drm=enabled \
    -Denable_egl=disabled \
    -Denable_qt=disabled \
    -Denable_opencv=disabled \
    -Denable_tflite=disabled \
    -Denable_hailo=disabled
RUN meson compile -C build
RUN meson install -C build
RUN ldconfig

# ------------------------------
# Install camera_ros from source
# ------------------------------

WORKDIR /home/docker
RUN mkdir -p camera_ws/src
WORKDIR /home/docker/camera_ws/src
RUN git clone https://github.com/christianrauch/camera_ros.git
WORKDIR /home/docker/camera_ws
RUN apt-get update && rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO --skip-keys=libcamera
RUN . /opt/ros/humble/setup.sh && colcon build --event-handlers=console_direct+

# --------------
# Install OpenCV
# --------------

# Install correct opencv version (this can take a very long time)
ARG OPENCV_VERSION=4.10.0

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

# -------------------
# Additional packages
# -------------------

# Install tmux for single shell docker-ing
RUN sudo apt-get install tmux nano -y

# Project direct dependencies
RUN pip install pymap3d scipy pyserial numpy==1.25.0

# Source ROS2
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /home/docker/camera_ws/install/local_setup.bash" >> /root/.bashrc
RUN echo "source /home/docker/ws_x500/install/local_setup.bash" >> /root/.bashrc

RUN mkdir /home/docker/ws_x500
WORKDIR /home/docker/ws_x500
