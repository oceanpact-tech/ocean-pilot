# === STAGE 1: Builder (Heavy compilation environment) ===
FROM ubuntu:26.04 AS builder

# Install build dependencies
RUN apt-get update && apt-get install -y \
    git \
    cmake \
    build-essential \
    python3 \
    python3-pip \
    libssl-dev \
    && rm -rf /var/lib/apt/lists/*

# Build and install Boost to /usr/local
WORKDIR /build/boost
RUN git clone --depth 1 --recursive https://github.com/boostorg/boost.git . \
    && ./bootstrap.sh \
    && ./b2 install -j$(nproc)

# Build and install MAVSDK to /usr/local
WORKDIR /build/mavsdk
RUN git clone --depth 1 --recursive https://github.com/mavlink/MAVSDK.git . \
    && cd cpp \
    && cmake -DCMAKE_BUILD_TYPE=Release -Bbuild -S. \
    && cmake --build build -j$(nproc) --target install


# === STAGE 2: Final (Clean development environment) ===
FROM ubuntu:26.04

# Install essential tools for compiling your project
RUN apt-get update && apt-get install -y \
    cmake \
    build-essential \
    python3 \
    libssl-dev \
    && rm -rf /var/lib/apt/lists/*

# Copy headers and compiled libraries from the builder stage
COPY --from=builder /usr/local/include /usr/local/include
COPY --from=builder /usr/local/lib /usr/local/lib

# Update dynamic linker run-time bindings to find the new .so files
RUN ldconfig

# Set the working directory for the project volume
WORKDIR /root/app

# Default command to keep the container interactive
CMD ["/bin/bash"]