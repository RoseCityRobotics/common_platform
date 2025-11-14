# Installing ONNX Runtime on Raspberry Pi 5 (Ubuntu 24 Server)

This guide provides instructions for installing ONNX Runtime C++ library on Raspberry Pi 5 running Ubuntu 24 Server (ARM64).

## Prerequisites

```bash
# Update system packages
sudo apt update
sudo apt upgrade -y

# Install build dependencies
sudo apt install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    libssl-dev \
    python3-dev \
    python3-pip
```

## Method 1: Install Pre-built Binaries (Recommended)

ONNX Runtime provides pre-built binaries for ARM64. This is the fastest method.

### Step 1: Download ONNX Runtime

```bash
# Create installation directory
sudo mkdir -p /opt/onnxruntime
cd /tmp

# Download the latest ONNX Runtime release for Linux ARM64
# Check https://github.com/microsoft/onnxruntime/releases for the latest version
ONNXRUNTIME_VERSION="1.18.1"  # Update this to the latest version
wget https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/onnxruntime-linux-aarch64-${ONNXRUNTIME_VERSION}.tgz

# Extract
tar -xzf onnxruntime-linux-aarch64-${ONNXRUNTIME_VERSION}.tgz
```

### Step 2: Install to System Directory

```bash
# Copy files to /opt/onnxruntime
sudo cp -r onnxruntime-linux-aarch64-${ONNXRUNTIME_VERSION}/* /opt/onnxruntime/

# Create symlinks for easier access
sudo ln -sf /opt/onnxruntime/lib/libonnxruntime.so /usr/lib/aarch64-linux-gnu/libonnxruntime.so
sudo ln -sf /opt/onnxruntime/include /usr/local/include/onnxruntime

# Update library cache
sudo ldconfig
```

### Step 3: Verify Installation

```bash
# Check if library is found
ldconfig -p | grep onnxruntime

# Check if headers are accessible
ls /opt/onnxruntime/include/onnxruntime_cxx_api.h
```

## Method 2: Build from Source (If Pre-built Not Available)

If pre-built binaries are not available or you need a custom build:

### Step 1: Clone ONNX Runtime Repository

```bash
cd ~
git clone --recursive https://github.com/microsoft/onnxruntime.git
cd onnxruntime
```

### Step 2: Configure Build

```bash
# Create build directory
mkdir build && cd build

# Configure CMake
# Note: This will take a long time on Raspberry Pi (1-2 hours)
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/opt/onnxruntime \
    -Donnxruntime_BUILD_SHARED_LIB=ON \
    -Donnxruntime_TARGET_DEVICE=CPU \
    -Donnxruntime_BUILD_UNIT_TESTS=OFF \
    -Donnxruntime_BUILD_BENCHMARKS=OFF
```

### Step 3: Build and Install

```bash
# Build (this will take 1-2 hours on Raspberry Pi 5)
make -j$(nproc)

# Install
sudo make install

# Update library cache
sudo ldconfig
```

## Method 3: Using vcpkg (Alternative Package Manager)

If you prefer using vcpkg:

```bash
# Install vcpkg
cd ~
git clone https://github.com/Microsoft/vcpkg.git
cd vcpkg
./bootstrap-vcpkg.sh

# Install ONNX Runtime
./vcpkg install onnxruntime[core]:arm64-linux

# Note: You'll need to configure CMake to use vcpkg toolchain
```

## Verify Installation for ROS2 Node

After installation, verify that the ROS2 node can find ONNX Runtime:

```bash
# Check library location
find /usr -name "libonnxruntime.so" 2>/dev/null
find /opt -name "libonnxruntime.so" 2>/dev/null
find /usr/local -name "libonnxruntime.so" 2>/dev/null

# Check header location
find /usr -name "onnxruntime_cxx_api.h" 2>/dev/null
find /opt -name "onnxruntime_cxx_api.h" 2>/dev/null
find /usr/local -name "onnxruntime_cxx_api.h" 2>/dev/null
```

## Troubleshooting

### Issue: Library Not Found During Build

If CMake can't find ONNX Runtime, you can specify the paths explicitly:

```bash
# In your CMakeLists.txt or when building:
cmake .. \
    -DONNXRUNTIME_LIBRARY=/opt/onnxruntime/lib/libonnxruntime.so \
    -DONNXRUNTIME_INCLUDE_DIR=/opt/onnxruntime/include
```

Or set environment variables:

```bash
export ONNXRUNTIME_LIBRARY=/opt/onnxruntime/lib/libonnxruntime.so
export ONNXRUNTIME_INCLUDE_DIR=/opt/onnxruntime/include
```

### Issue: Runtime Library Not Found

If the node can't find the library at runtime:

```bash
# Add to /etc/ld.so.conf.d/onnxruntime.conf
echo "/opt/onnxruntime/lib" | sudo tee /etc/ld.so.conf.d/onnxruntime.conf
sudo ldconfig
```

Or set LD_LIBRARY_PATH:

```bash
export LD_LIBRARY_PATH=/opt/onnxruntime/lib:$LD_LIBRARY_PATH
```

### Issue: Performance is Slow

ONNX Runtime on CPU can be slow for large models. Consider:

1. **Use optimized build**: Ensure you're using the Release build
2. **Thread configuration**: The node is configured to use 4 threads for intra-op and 2 for inter-op
3. **Model optimization**: Use ONNX Simplifier to optimize the model before deployment
4. **Quantization**: Consider quantizing the model to INT8 for faster inference

## Testing Installation

Create a simple test program to verify ONNX Runtime works:

```cpp
// test_onnx.cpp
#include <onnxruntime_cxx_api.h>
#include <iostream>

int main() {
    try {
        Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "test");
        std::cout << "ONNX Runtime initialized successfully!" << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
```

Compile and run:

```bash
g++ -o test_onnx test_onnx.cpp -lonnxruntime -I/opt/onnxruntime/include -L/opt/onnxruntime/lib
export LD_LIBRARY_PATH=/opt/onnxruntime/lib:$LD_LIBRARY_PATH
./test_onnx
```

## Additional Resources

- **ONNX Runtime Documentation**: https://onnxruntime.ai/docs/
- **GitHub Releases**: https://github.com/microsoft/onnxruntime/releases
- **Build Instructions**: https://onnxruntime.ai/docs/build/inferencing.html

## Notes for Raspberry Pi 5

- **Memory**: ONNX Runtime can be memory-intensive. Ensure you have sufficient RAM (4GB+ recommended)
- **CPU**: Raspberry Pi 5's Cortex-A76 cores are much faster than Pi 4, but still expect inference times of 50-200ms for transformer models
- **Thermal**: Monitor CPU temperature during long inference sessions. Consider using a heatsink or fan
- **Power**: Ensure adequate power supply (official Raspberry Pi 5 power supply recommended)

