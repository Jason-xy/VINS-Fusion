FROM jasonxxxyyy/sky-explorer:runtime-cuda11.4-ros2-arm64

# Install ncnn
RUN git clone https://github.com/Tencent/ncnn.git && \
    cd ncnn && \
    git submodule update --init && \
    apt-get update && \
    apt-get install -y \
    libprotobuf-dev \
    protobuf-compiler \
    libvulkan-dev \
    vulkan-utils && \
    apt-get clean && \
    mkdir build && \
    cd build && \
    cmake .. \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DCMAKE_BUILD_TYPE=Release \
        -DNCNN_VULKAN=ON && \
    make -j$(nproc) && \
    make install && \
    rm -rf ncnn

# Install backword-cpp
RUN git clone https://github.com/bombela/backward-cpp.git && \
    cd backward-cpp && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    rm -rf backward-cpp && \
    apt-get update && \
    apt-get install -y \
    libdw-dev \
    binutils-dev \
    libunwind-dev && \
    apt-get clean