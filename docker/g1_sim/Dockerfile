FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

# Build tools, G1 controller dependencies, MuJoCo GUI/runtime libraries,
# and the ROS 2 message packages used by the modified simulator.
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    pkg-config \
    python3-dev \
    python3-pip \
    libboost-all-dev \
    libeigen3-dev \
    libfmt-dev \
    libgl1-mesa-dev \
    libglfw3-dev \
    libspdlog-dev \
    libyaml-cpp-dev \
    libx11-6 \
    libxcursor1 \
    libxi6 \
    libxinerama1 \
    libxrandr2 \
    ros-humble-nav-msgs \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

# CycloneDDS 0.10.x is required by unitree_sdk2_python.
RUN git clone --depth 1 \
        --branch releases/0.10.x \
        https://github.com/eclipse-cyclonedds/cyclonedds.git \
        /opt/cyclonedds-src \
    && cmake -S /opt/cyclonedds-src -B /opt/cyclonedds-src/build \
        -DCMAKE_INSTALL_PREFIX=/opt/cyclonedds \
        -DBUILD_EXAMPLES=OFF \
    && cmake --build /opt/cyclonedds-src/build -j"$(nproc)" \
    && cmake --install /opt/cyclonedds-src/build

ENV CYCLONEDDS_HOME=/opt/cyclonedds
ENV CMAKE_PREFIX_PATH=/opt/cyclonedds
ENV LD_LIBRARY_PATH=/opt/cyclonedds/lib:/usr/local/lib

# Use the same unitree_sdk2 revision already tested on the laptop.
ARG UNITREE_SDK2_COMMIT=9754cd153af3da471b0fe5f3aa535e426fb11db3

RUN git clone https://github.com/unitreerobotics/unitree_sdk2.git /opt/unitree_sdk2 \
    && cd /opt/unitree_sdk2 \
    && git checkout ${UNITREE_SDK2_COMMIT} \
    && cmake -S . -B build \
        -DBUILD_EXAMPLES=OFF \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
    && cmake --build build -j"$(nproc)" \
    && cmake --install build

# Use the same unitree_sdk2_python revision already tested on the laptop.
ARG UNITREE_SDK2_PYTHON_COMMIT=814556d15970dd2ecf1c9984e845ca02ab07e206

RUN python3 -m pip install --no-cache-dir --upgrade pip setuptools wheel \
    && git clone https://github.com/unitreerobotics/unitree_sdk2_python.git \
        /opt/unitree_sdk2_python \
    && cd /opt/unitree_sdk2_python \
    && git checkout ${UNITREE_SDK2_PYTHON_COMMIT} \
    && CYCLONEDDS_HOME=/opt/cyclonedds \
        python3 -m pip install --no-cache-dir -e .

# Python dependencies for the modified MuJoCo simulator and JT128 publisher.
# MuJoCo-LiDAR itself will be mounted from Helene's repository so that her
# modified source remains outside the disposable container.
RUN python3 -m pip install --no-cache-dir \
    "numpy<2" \
    scipy \
    "mujoco==3.3.6" \
    "warp-lang>=1.11.0" \
    pynput \
    matplotlib \
    pygame \
    onnxruntime \
    "etils[epath]"

WORKDIR /workspace

CMD ["bash"]
