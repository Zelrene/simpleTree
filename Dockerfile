FROM ubuntu:18.04

ENV DEBIAN_FRONTEND=noninteractive

# =========================
# 1. Base toolchain
# =========================
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    wget \
    curl \
    software-properties-common

# =========================
# 2. Qt5 (core migration target)
# =========================
RUN apt-get install -y \
    qtbase5-dev \
    qttools5-dev \
    qttools5-dev-tools \
    qt5-qmake \
    libqt5widgets5 \
    libqt5gui5 \
    libqt5core5a

# =========================
# 3. VTK (keep stable, NOT VTK9 yet)
# =========================
RUN apt-get install -y \
    libvtk6-dev \
    libvtk6-qt-dev

# =========================
# 4. PCL (fixed to 1.8-compatible ecosystem)
# =========================
RUN apt-get install -y \
    libpcl-dev \
    libeigen3-dev \
    libboost-all-dev

# =========================
# 5. OpenGL + GUI dependencies (VTK Qt rendering)
# =========================
RUN apt-get install -y \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    mesa-utils

# =========================
# 6. Optional debugging tools
# =========================
RUN apt-get install -y \
    gdb \
    valgrind

# =========================
# 7. Workspace setup
# =========================
WORKDIR /workspace

# =========================
# 8. Default build behavior helper
# =========================
CMD ["/bin/bash"]